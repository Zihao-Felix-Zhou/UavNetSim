import asyncio
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from api.runtime import RunSettings, runtime
from routing.parameters import ROUTING_PARAMETER_DEFINITIONS, resolve_routing_parameters
from scene.compiler import compile_scene
from scene.models import GeoBounds, SceneModel
from scene.osm_importer import fetch_osm_scene
from utils import config


class StartRequest(BaseModel):
    seed: int = 2025
    node_count: int = Field(default=8, ge=2, le=50)
    duration_seconds: float = Field(default=20, gt=0, le=3600)
    playback_speed: float = Field(default=1, gt=0, le=100)
    uav_speed_mps: float = Field(default=10, gt=0, le=100)
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


class OsmImportRequest(BaseModel):
    name: str = "OSM Scene"
    bounds: GeoBounds


app = FastAPI(title="UavNetSim v2")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
    allow_methods=["*"],
    allow_headers=["*"],
)


def _activate_scene(scene):
    output = config.PROJECT_ROOT / "artifacts" / "scene"
    compile_scene(scene, output)
    config.SIONNA_SCENE_PATH = str(output / "scene.xml")
    config.MAP_LENGTH = scene.size_x
    config.MAP_WIDTH = scene.size_y
    return scene


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
    return _activate_scene(scene)


@app.post("/api/scene/osm", response_model=SceneModel)
async def import_osm(request: OsmImportRequest):
    if runtime.status in {"running", "paused", "starting", "preparing", "stopping"}:
        raise HTTPException(409, "Stop the simulation before changing the scene")
    try:
        scene = await asyncio.to_thread(fetch_osm_scene, request.bounds, request.name)
        return _activate_scene(scene)
    except Exception as error:
        raise HTTPException(502, str(error)) from error


@app.post("/api/simulation/start")
def start_simulation(request: StartRequest):
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
