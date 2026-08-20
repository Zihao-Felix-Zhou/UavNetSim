import random
from pathlib import Path

import simpy

from entities.drone import Drone
from mobility import start_coords
from phy.channel import Channel
from scene.airspace import Airspace
from simulator.metrics import Metrics
from telemetry import EventBus
from utils import config


class Simulator:
    """Discrete-event UAV network simulator."""

    def __init__(self,
                 seed,
                 env,
                 n_drones,
                 total_simulation_time=config.SIM_TIME,
                 event_bus=None,
                 action_queue=None,
                 obs_queue=None,
                 drone_speed=config.UAV_SPEED,
                 trajectory_trace=None,
                 channel_trace=None):
        self.env = env
        self.seed = seed
        self.total_simulation_time = total_simulation_time
        self.n_drones = n_drones
        self.drone_speed = drone_speed
        self.trajectory_trace = trajectory_trace
        self.event_bus = event_bus or EventBus()
        self.metrics = Metrics(self)
        self.action_queue = action_queue
        self.obs_queue = obs_queue
        scene_path = Path(config.SIONNA_SCENE_PATH).with_name("scene.json")
        self.airspace = Airspace.from_file(
            scene_path,
            max_height=config.MAP_HEIGHT,
            building_clearance=config.UAV_BUILDING_CLEARANCE,
            boundary_clearance=config.UAV_BOUNDARY_CLEARANCE,
        )
        config.MAP_LENGTH = self.airspace.size_x
        config.MAP_WIDTH = self.airspace.size_y
        self.channel_states = {i: simpy.Resource(env, capacity=1) for i in range(n_drones)}
        self.channel = Channel(self.env, self, channel_trace)

        config.reset_runtime_ids()
        start_position = start_coords.get_random_start_point_3d(seed, n_drones, self.airspace)
        self.drones = []
        for identifier in range(n_drones):
            speed = random.Random(seed + identifier).randint(5, 60) if config.HETEROGENEOUS else self.drone_speed
            drone = Drone(
                env=env,
                node_id=identifier,
                coords=start_position[identifier],
                speed=speed,
                inbox=self.channel.create_inbox_for_receiver(identifier),
                simulator=self,
            )
            self.drones.append(drone)

        self.event_bus.publish(
            "simulation_initialized",
            self.env.now,
            seed=seed,
            node_count=n_drones,
            duration_us=total_simulation_time,
            routing=config.ROUTING_PROTOCOL,
            mac=config.MAC_PROTOCOL,
            mobility=config.MOBILITY_MODEL,
            uav_speed_mps=drone_speed,
            initial_energy_j=config.INITIAL_ENERGY,
            traffic_pattern=config.TRAFFIC_PATTERN,
            packet_arrival_rate=config.PACKET_ARRIVAL_RATE,
            routing_parameters=config.ROUTING_PROTOCOL_PARAMETERS.copy(),
            sionna={
                "channel_mode": config.CHANNEL_MODE,
                "max_depth": config.SIONNA_MAX_DEPTH,
                "samples_per_source": config.SIONNA_SAMPLES_PER_SOURCE,
                "frequency_samples": config.SIONNA_FREQUENCY_SAMPLES,
                "los": config.SIONNA_LOS,
                "specular_reflection": config.SIONNA_SPECULAR_REFLECTION,
                "diffuse_reflection": config.SIONNA_DIFFUSE_REFLECTION,
                "refraction": config.SIONNA_REFRACTION,
                "diffraction": config.SIONNA_DIFFRACTION,
                "edge_diffraction": config.SIONNA_EDGE_DIFFRACTION,
                "snapshot_interval_us": config.CHANNEL_SNAPSHOT_INTERVAL,
                "snapshot_displacement_m": config.CHANNEL_SNAPSHOT_DISPLACEMENT,
            },
        )
        self.env.process(self.publish_state())
        self.env.process(self.finish())

    def publish_state(self):
        while True:
            self.event_bus.publish(
                "simulation_state",
                self.env.now,
                duration_us=float(self.total_simulation_time),
                nodes=self.node_snapshot(),
                metrics=self.metrics.snapshot(),
            )
            yield self.env.timeout(100000)

    def finish(self):
        yield self.env.timeout(self.total_simulation_time)
        self.event_bus.publish(
            "simulation_finished",
            self.env.now,
            metrics=self.metrics.snapshot(),
        )

    def node_snapshot(self):
        return [
            {
                "id": drone.identifier,
                "position": [float(value) for value in drone.coords],
                "velocity": [float(value) for value in drone.velocity],
                "energy_j": float(drone.residual_energy),
                "queue_size": drone.transmitting_queue.qsize(),
                "sleeping": drone.sleep,
            }
            for drone in self.drones
        ]

    def snapshot(self):
        return {
            "sim_time_us": float(self.env.now),
            "duration_us": float(self.total_simulation_time),
            "nodes": self.node_snapshot(),
            "metrics": self.metrics.snapshot(),
        }

    def close(self):
        self.channel.close()
