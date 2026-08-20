import math
import random
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import simpy

from energy.energy_model import EnergyModel
from mobility import start_coords
from mobility.gauss_markov_3d import GaussMarkov3D
from mobility.random_walk_3d import RandomWalk3D
from mobility.random_waypoint_3d import RandomWaypoint3D
from utils import config


class TraceBuildCancelled(RuntimeError):
    pass


@dataclass(slots=True)
class TrajectoryTrace:
    times_us: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    speeds: np.ndarray


class _SilentEventBus:
    def publish(self, event_type, sim_time_us, **data):
        return None


class _MobilityDrone:
    def __init__(self, simulator, identifier, position, speed, mobility_class):
        self.simulator = simulator
        self.env = simulator.env
        self.identifier = identifier
        self._coords = [float(value) for value in position]
        self.start_coords = tuple(self._coords)
        self.rng_drone = random.Random(identifier + simulator.seed)
        self.direction = self.rng_drone.uniform(0, 2 * np.pi)
        self.pitch = self.rng_drone.uniform(-0.05, 0.05)
        self.speed = float(speed)
        self.velocity = [
            self.speed * math.cos(self.direction) * math.cos(self.pitch),
            self.speed * math.sin(self.direction) * math.cos(self.pitch),
            self.speed * math.sin(self.pitch),
        ]
        self.direction_mean = self.direction
        self.pitch_mean = self.pitch
        self.velocity_mean = self.speed
        self.residual_energy = config.INITIAL_ENERGY
        self.sleep = False
        self.mobility_model = mobility_class(self)
        self.energy_model = EnergyModel(self)

    @property
    def coords(self):
        return self._coords

    def move_to(self, position, velocity):
        resolved_position, resolved_velocity, collision = self.simulator.airspace.resolve_motion(
            self._coords, position, velocity
        )
        self._coords = resolved_position
        self.velocity = resolved_velocity
        return collision


def _mobility_class(name):
    classes = {
        "GAUSSMARKOV3D": GaussMarkov3D,
        "RANDOMWALK3D": RandomWalk3D,
        "RANDOMWAYPOINT3D": RandomWaypoint3D,
    }
    normalized = name.replace("-", "").replace("_", "").upper()
    try:
        return classes[normalized]
    except KeyError as error:
        raise ValueError(f"Unsupported mobility model: {name}") from error


def generate_trajectory(seed, node_count, duration_us, drone_speed, airspace, stop_event=None):
    environment = simpy.Environment()
    simulator = SimpleNamespace(
        env=environment,
        seed=seed,
        airspace=airspace,
        event_bus=_SilentEventBus(),
    )
    positions = start_coords.get_random_start_point_3d(seed, node_count, airspace)
    mobility_class = _mobility_class(config.MOBILITY_MODEL)
    drones = []
    for identifier in range(node_count):
        speed = (
            random.Random(seed + identifier).randint(5, 60)
            if config.HETEROGENEOUS else drone_speed
        )
        drones.append(_MobilityDrone(
            simulator, identifier, positions[identifier], speed, mobility_class
        ))

    sample_interval_us = 100000.0
    times = np.arange(0.0, duration_us + sample_interval_us, sample_interval_us)
    times = times[times <= duration_us]
    if not len(times) or times[-1] < duration_us:
        times = np.append(times, float(duration_us))
    position_samples = []
    velocity_samples = []
    speed_samples = []
    for sample_time in times:
        if stop_event is not None and stop_event.is_set():
            raise TraceBuildCancelled()
        while environment.peek() <= sample_time:
            environment.step()
        position_samples.append([list(drone.coords) for drone in drones])
        velocity_samples.append([list(drone.velocity) for drone in drones])
        speed_samples.append([float(drone.speed) for drone in drones])
    return TrajectoryTrace(
        times_us=np.asarray(times, dtype=float),
        positions=np.asarray(position_samples, dtype=float),
        velocities=np.asarray(velocity_samples, dtype=float),
        speeds=np.asarray(speed_samples, dtype=float),
    )


class TraceMobility3D:
    def __init__(self, drone, trace):
        self.my_drone = drone
        self.trace = trace
        self.position_update_interval = 100000.0
        self.my_drone.simulator.env.process(self.mobility_update())

    def _apply(self, index):
        identifier = self.my_drone.identifier
        self.my_drone._coords = self.trace.positions[index, identifier].astype(float).tolist()
        self.my_drone.velocity = self.trace.velocities[index, identifier].astype(float).tolist()
        self.my_drone.speed = float(self.trace.speeds[index, identifier])

    def mobility_update(self):
        self._apply(0)
        for index in range(1, len(self.trace.times_us)):
            interval_us = float(self.trace.times_us[index] - self.trace.times_us[index - 1])
            yield self.my_drone.simulator.env.timeout(interval_us)
            energy = interval_us / 1e6 * self.my_drone.energy_model.power_consumption(
                self.my_drone.speed
            )
            self.my_drone.residual_energy = max(0.0, self.my_drone.residual_energy - energy)
            if self.my_drone.sleep:
                break
            self._apply(index)
