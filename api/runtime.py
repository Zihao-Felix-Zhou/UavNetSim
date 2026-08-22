import threading
import time
from dataclasses import dataclass

import simpy

from mobility.trajectory import TraceBuildCancelled, generate_trajectory
from phy.offline_trace import build_or_load_channel_trace
from scene.airspace import Airspace
from simulator.simulator import Simulator
from telemetry import EventBus
from utils import config


@dataclass(slots=True)
class RunSettings:
    seed: int
    node_count: int
    duration_seconds: float
    playback_speed: float
    uav_speed_mps: float
    uav_min_altitude_m: float | None
    uav_max_altitude_m: float | None
    initial_energy_j: float
    traffic_pattern: str
    packet_arrival_rate: float
    routing: str
    routing_parameters: dict[str, float]
    mac: str
    mobility: str
    channel_mode: str
    samples_per_source: int
    sionna_max_depth: int
    sionna_frequency_samples: int
    sionna_los: bool
    sionna_specular_reflection: bool
    sionna_diffuse_reflection: bool
    sionna_refraction: bool
    sionna_diffraction: bool
    sionna_edge_diffraction: bool
    channel_snapshot_interval_ms: float
    channel_snapshot_displacement_m: float


class SimulationRuntime:
    def __init__(self):
        self.event_bus = EventBus()
        self.simulator = None
        self._thread = None
        self._pause = threading.Event()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.status = "idle"
        self.error = None
        self.settings = None
        self.preparation = None
        self.run_id = 0

    def start(self, settings):
        with self._lock:
            if self.status in {"starting", "preparing", "running", "paused", "stopping"}:
                raise RuntimeError("A simulation is already active")
            self.run_id += 1
            self.event_bus = EventBus()
            self.settings = settings
            self.preparation = None
            self.error = None
            self._pause.clear()
            self._stop.clear()
            self.status = "starting"
            self._thread = threading.Thread(target=self._run, name="uavnetsim", daemon=True)
            self._thread.start()

    def _run(self):
        environment = simpy.Environment()
        simulator = None
        final_status = None
        try:
            config.ROUTING_PROTOCOL = self.settings.routing
            config.MAC_PROTOCOL = self.settings.mac
            config.MOBILITY_MODEL = self.settings.mobility
            config.CHANNEL_MODE = self.settings.channel_mode
            config.NUMBER_OF_DRONES = self.settings.node_count
            config.UAV_SPEED = self.settings.uav_speed_mps
            config.UAV_MIN_ALTITUDE = self.settings.uav_min_altitude_m
            config.UAV_MAX_ALTITUDE = self.settings.uav_max_altitude_m
            config.INITIAL_ENERGY = self.settings.initial_energy_j
            config.TRAFFIC_PATTERN = self.settings.traffic_pattern
            config.PACKET_ARRIVAL_RATE = self.settings.packet_arrival_rate
            config.MAX_TTL = self.settings.node_count + 1
            config.ROUTING_PROTOCOL_PARAMETERS = self.settings.routing_parameters.copy()
            config.SIONNA_SAMPLES_PER_SOURCE = self.settings.samples_per_source
            config.SIONNA_MAX_DEPTH = self.settings.sionna_max_depth
            config.SIONNA_FREQUENCY_SAMPLES = self.settings.sionna_frequency_samples
            config.SIONNA_LOS = self.settings.sionna_los
            config.SIONNA_SPECULAR_REFLECTION = self.settings.sionna_specular_reflection
            config.SIONNA_DIFFUSE_REFLECTION = self.settings.sionna_diffuse_reflection
            config.SIONNA_REFRACTION = self.settings.sionna_refraction
            config.SIONNA_DIFFRACTION = self.settings.sionna_diffraction
            config.SIONNA_EDGE_DIFFRACTION = self.settings.sionna_edge_diffraction
            config.CHANNEL_SNAPSHOT_INTERVAL = self.settings.channel_snapshot_interval_ms * 1e3
            config.CHANNEL_SNAPSHOT_DISPLACEMENT = self.settings.channel_snapshot_displacement_m
            duration_us = self.settings.duration_seconds * 1e6
            trajectory_trace = None
            channel_trace = None
            if config.CHANNEL_MODE == "offline":
                self.status = "preparing"
                self.preparation = {
                    "phase": "trajectory",
                    "completed": 0,
                    "total": 0,
                    "cache_hit": False,
                }
                self.event_bus.publish("channel_trajectory_started", 0)
                scene_path = config.PROJECT_ROOT / "artifacts" / "scene" / "scene.json"
                airspace = Airspace.from_file(
                    scene_path,
                    max_height=config.MAP_HEIGHT,
                    building_clearance=config.UAV_BUILDING_CLEARANCE,
                    boundary_clearance=config.UAV_BOUNDARY_CLEARANCE,
                    min_flight_height=config.UAV_MIN_ALTITUDE,
                    max_flight_height=config.UAV_MAX_ALTITUDE,
                )
                config.MAP_LENGTH = airspace.size_x
                config.MAP_WIDTH = airspace.size_y
                config.MAP_HEIGHT = airspace.max_height
                trajectory_trace = generate_trajectory(
                    seed=self.settings.seed,
                    node_count=self.settings.node_count,
                    duration_us=duration_us,
                    drone_speed=self.settings.uav_speed_mps,
                    airspace=airspace,
                    stop_event=self._stop,
                )
                self.event_bus.publish(
                    "channel_trajectory_ready", 0, samples=len(trajectory_trace.times_us)
                )

                def update_trace_progress(completed, total, cache_hit):
                    self.preparation = {
                        "phase": "channel_trace",
                        "completed": completed,
                        "total": total,
                        "cache_hit": cache_hit,
                    }

                channel_trace = build_or_load_channel_trace(
                    trajectory_trace,
                    self.event_bus,
                    stop_event=self._stop,
                    progress=update_trace_progress,
                )
                if self._stop.is_set():
                    raise TraceBuildCancelled()
            simulator = Simulator(
                seed=self.settings.seed,
                env=environment,
                n_drones=self.settings.node_count,
                total_simulation_time=duration_us,
                event_bus=self.event_bus,
                drone_speed=self.settings.uav_speed_mps,
                trajectory_trace=trajectory_trace,
                channel_trace=channel_trace,
            )
            self.simulator = simulator
            self.preparation = None
            self.status = "running"
            wall_started = time.perf_counter()
            while environment.peek() <= duration_us and not self._stop.is_set():
                while self._pause.is_set() and not self._stop.is_set():
                    time.sleep(0.02)
                    wall_started = time.perf_counter() - environment.now / 1e6 / self.settings.playback_speed
                target = wall_started + environment.peek() / 1e6 / self.settings.playback_speed
                delay = target - time.perf_counter()
                if delay > 0:
                    time.sleep(min(delay, 0.02))
                    continue
                environment.step()
            if self._stop.is_set():
                final_status = "stopped"
                self.event_bus.publish("simulation_stopped", environment.now)
            else:
                final_status = "completed"
        except TraceBuildCancelled:
            final_status = "stopped"
            self.event_bus.publish("simulation_stopped", environment.now)
        except BaseException as error:
            self.error = f"{type(error).__name__}: {error}"
            final_status = "failed"
            self.event_bus.publish("simulation_failed", environment.now, error=self.error)
        finally:
            if simulator is not None:
                simulator.close()
            self.status = final_status or "failed"
            self.event_bus.publish(
                "runtime_status",
                environment.now,
                status=self.status,
            )

    def pause(self):
        if self.status != "running":
            raise RuntimeError("The simulation is not running")
        self._pause.set()
        self.status = "paused"
        sim_time = self.simulator.env.now if self.simulator else 0
        self.event_bus.publish("simulation_paused", sim_time)

    def resume(self):
        if self.status != "paused":
            raise RuntimeError("The simulation is not paused")
        self._pause.clear()
        self.status = "running"
        sim_time = self.simulator.env.now if self.simulator else 0
        self.event_bus.publish("simulation_resumed", sim_time)

    def stop(self):
        if self.status not in {"running", "paused", "starting", "preparing"}:
            return
        self._stop.set()
        self._pause.clear()
        self.status = "stopping"
        sim_time = self.simulator.env.now if self.simulator else 0
        self.event_bus.publish("simulation_stopping", sim_time)

    def state(self):
        latest = self.event_bus.latest("simulation_state")
        state = latest["data"].copy() if latest else {
            "nodes": [],
            "metrics": {},
        }
        state["sim_time_us"] = latest["sim_time_us"] if latest else 0.0
        state.update({
            "status": self.status,
            "error": self.error,
            "sequence": self.event_bus.sequence,
            "run_id": self.run_id,
            "channel_mode": self.settings.channel_mode if self.settings else config.CHANNEL_MODE,
            "preparation": self.preparation,
        })
        return state


runtime = SimulationRuntime()
