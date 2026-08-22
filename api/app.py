import asyncio
from pathlib import Path
from threading import Lock
from typing import Literal
from uuid import uuid4

from fastapi import BackgroundTasks, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, model_validator

from api.runtime import RunSettings, runtime
from routing.parameters import ROUTING_PARAMETER_DEFINITIONS, resolve_routing_parameters
from scene.compiler import compile_scene
from scene.models import GeoBounds, SceneModel
from scene.osm_importer import fetch_osm_scene
from scene.terrain import attach_terrain
from utils import config


class StartRequest(BaseModel):
    seed: int = 2025
    node_count: int = Field(default=8, ge=2, le=50)
    duration_seconds: float = Field(default=20, gt=0, le=3600)
    playback_speed: float = Field(default=1, gt=0, le=100)
    uav_speed_mps: float = Field(default=10, gt=0, le=100)
    uav_min_altitude_m: float | None = Field(default=None, ge=0, le=10000)
    uav_max_altitude_m: float | None = Field(default=None, gt=0, le=10000)
    initial_energy_j: float = Field(default=20000, gt=0, le=1e9)
    traffic_pattern: str = Field(default="Poisson", pattern="^(Uniform|Poisson)$")
    packet_arrival_rate: float = Field(default=5, gt=0, le=1000)
    routing: str = "Greedy"
    routing_parameters: dict[str, float] = Field(default_factory=dict)
    mac: str = "CSMA_CA"
    mobility: str = "GaussMarkov3D"
    channel_mode: Literal["online", "offline"] = "online"
    samples_per_source: int = Field(default=100000, ge=100, le=10000000)
    sionna_max_depth: int = Field(default=4, ge=0, le=32)
    sionna_frequency_samples: int = Field(default=32, ge=1, le=4096)
    sionna_los: bool = True
    sionna_specular_reflection: bool = True
    sionna_diffuse_reflection: bool = False
    sionna_refraction: bool = False
    sionna_diffraction: bool = False
    sionna_edge_diffraction: bool = False
    channel_snapshot_interval_ms: float = Field(default=100, gt=0, le=60000)
    channel_snapshot_displacement_m: float = Field(default=1, gt=0, le=1000)

    @model_validator(mode="after")
    def validate_altitude_range(self):
        if (
            self.uav_min_altitude_m is not None
            and self.uav_max_altitude_m is not None
            and self.uav_min_altitude_m >= self.uav_max_altitude_m
        ):
            raise ValueError("Maximum UAV altitude must be greater than minimum UAV altitude")
        return self


class OsmImportRequest(BaseModel):
    name: str = "OSM Scene"
    bounds: GeoBounds


class SceneBuildState(BaseModel):
    id: str
    status: Literal["queued", "running", "completed", "failed"] = "queued"
    progress: int = Field(default=0, ge=0, le=100)
    stage: str = "Queued"
    error: str | None = None
    scene: SceneModel | None = None


app = FastAPI(title="UavNetSim v2")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
    allow_methods=["*"],
    allow_headers=["*"],
)
scene_builds: dict[str, SceneBuildState] = {}
scene_build_lock = Lock()


def _activate_scene(scene, progress=None):
    output = config.PROJECT_ROOT / "artifacts" / "scene"
    compile_scene(scene, output, progress)
    config.SIONNA_SCENE_PATH = str(output / "scene.xml")
    config.MAP_LENGTH = scene.size_x
    config.MAP_WIDTH = scene.size_y
    terrain_height = max((point.z for point in scene.terrain.vertices), default=0.0) if scene.terrain else 0.0
    config.MAP_HEIGHT = terrain_height + config.AIRSPACE_HEIGHT_ABOVE_TERRAIN
    return scene


def _update_scene_build(build_id, **changes):
    with scene_build_lock:
        scene_builds[build_id] = scene_builds[build_id].model_copy(update=changes)


def _scene_build_in_progress():
    with scene_build_lock:
        return any(build.status in {"queued", "running"} for build in scene_builds.values())


def _run_scene_build(build_id, request):
    try:
        _update_scene_build(build_id, status="running", progress=2, stage="Starting scene build")

        def import_progress(value, stage):
            _update_scene_build(build_id, progress=5 + round(value * 25), stage=stage)

        scene = fetch_osm_scene(request.bounds, request.name, import_progress)

        def terrain_progress(value, stage):
            _update_scene_build(build_id, progress=30 + round(value * 25), stage=stage)

        scene = attach_terrain(scene, terrain_progress)

        def compile_progress(value, stage):
            _update_scene_build(build_id, progress=55 + round(value * 43), stage=stage)

        _activate_scene(scene, compile_progress)
        _update_scene_build(
            build_id,
            status="completed",
            progress=100,
            stage="Scene ready",
            scene=scene,
        )
    except Exception as error:
        _update_scene_build(build_id, status="failed", stage="Scene build failed", error=str(error))


def _scene_path():
    path = config.PROJECT_ROOT / "artifacts" / "scene" / "scene.json"
    if not path.is_file():
        default_scene = config.PROJECT_ROOT / "scenarios" / "default_scene.json"
        scene = SceneModel.model_validate_json(default_scene.read_text(encoding="utf-8"))
        _activate_scene(scene)
    return path


@app.get("/api/options")
def options():
    return {
        "routing": ["Greedy", "DSDV", "GRAD", "OPAR", "QRouting", "QFANET", "QGeo", "QMR", "Baseline_DRL"],
        "routing_parameters": ROUTING_PARAMETER_DEFINITIONS,
        "mac": ["CSMA_CA", "Pure_Aloha", "TDMA"],
        "mobility": ["GaussMarkov3D", "RandomWalk3D", "RandomWaypoint3D"],
        "traffic_pattern": ["Uniform", "Poisson"],
        "channel_mode": ["online", "offline"],
    }


@app.get("/api/scene", response_model=SceneModel)
def get_scene():
    return SceneModel.model_validate_json(_scene_path().read_text(encoding="utf-8"))


@app.post("/api/scene/import", response_model=SceneModel)
def import_scene(scene: SceneModel):
    if runtime.status in {"running", "paused", "starting", "preparing", "stopping"}:
        raise HTTPException(409, "Stop the simulation before changing the scene")
    if _scene_build_in_progress():
        raise HTTPException(409, "Wait for the current scene build to finish")
    return _activate_scene(scene)


@app.post("/api/scene/osm", response_model=SceneBuildState, status_code=202)
def import_osm(request: OsmImportRequest, background_tasks: BackgroundTasks):
    if runtime.status in {"running", "paused", "starting", "preparing", "stopping"}:
        raise HTTPException(409, "Stop the simulation before changing the scene")
    with scene_build_lock:
        if any(build.status in {"queued", "running"} for build in scene_builds.values()):
            raise HTTPException(409, "Another scene is already being built")
        build = SceneBuildState(id=uuid4().hex)
        scene_builds[build.id] = build
    background_tasks.add_task(_run_scene_build, build.id, request)
    return build


@app.get("/api/scene/osm/{build_id}", response_model=SceneBuildState)
def scene_build_state(build_id: str):
    with scene_build_lock:
        build = scene_builds.get(build_id)
    if build is None:
        raise HTTPException(404, "Scene build not found")
    return build


@app.post("/api/simulation/start")
def start_simulation(request: StartRequest):
    if _scene_build_in_progress():
        raise HTTPException(409, "Wait for the current scene build to finish")
    try:
        settings = request.model_dump()
        settings["routing_parameters"] = resolve_routing_parameters(
            request.routing,
            request.routing_parameters,
        )
        runtime.start(RunSettings(**settings))
    except (RuntimeError, ValueError) as error:
        raise HTTPException(409, str(error)) from error
    return runtime.state()


@app.post("/api/simulation/pause")
def pause_simulation():
    try:
        runtime.pause()
    except RuntimeError as error:
        raise HTTPException(409, str(error)) from error
    return runtime.state()


@app.post("/api/simulation/resume")
def resume_simulation():
    try:
        runtime.resume()
    except RuntimeError as error:
        raise HTTPException(409, str(error)) from error
    return runtime.state()


@app.post("/api/simulation/stop")
def stop_simulation():
    runtime.stop()
    return runtime.state()


@app.get("/api/simulation/state")
def simulation_state():
    return runtime.state()


@app.get("/api/events")
def events(after: int = 0):
    return runtime.event_bus.since(after)


@app.websocket("/api/ws")
async def event_stream(websocket: WebSocket):
    await websocket.accept()
    event_bus = runtime.event_bus
    run_id = runtime.run_id
    sequence = 0
    try:
        while True:
            if run_id != runtime.run_id:
                event_bus = runtime.event_bus
                run_id = runtime.run_id
                sequence = 0
            events = event_bus.since(sequence)
            if events:
                sequence = events[-1]["sequence"]
                await websocket.send_json({
                    "events": events,
                    "state": runtime.state(),
                })
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        return


frontend = config.PROJECT_ROOT / "frontend" / "dist"
if frontend.is_dir():
    app.mount("/assets", StaticFiles(directory=frontend / "assets"), name="assets")

    @app.get("/{path:path}")
    def frontend_app(path: str):
        candidate = frontend / path
        if path and candidate.is_file():
            return FileResponse(candidate)
        return FileResponse(frontend / "index.html")
