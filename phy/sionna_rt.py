import math
import multiprocessing
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from phy.sionna_worker import run_worker
from utils import config


SPEED_OF_LIGHT_M_S = 299_792_458.0


def free_space_path_gain(distance_m, frequency_hz):
    distance_m = float(distance_m)
    frequency_hz = float(frequency_hz)
    if distance_m <= 0 or frequency_hz <= 0:
        raise ValueError("Distance and frequency must be positive")
    wavelength = SPEED_OF_LIGHT_M_S / frequency_hz
    return (wavelength / (4 * math.pi * distance_m)) ** 2


class SionnaWorkerClient:
    def __init__(self):
        scene_path = Path(config.SIONNA_SCENE_PATH)
        if not scene_path.is_file():
            raise FileNotFoundError(f"Sionna scene does not exist: {scene_path}")
        context = multiprocessing.get_context("spawn")
        parent_connection, child_connection = context.Pipe()
        self._process = context.Process(target=run_worker, args=(child_connection,), daemon=True)
        self._process.start()
        child_connection.close()
        self._connection = parent_connection
        self._connection.send({
            "type": "configure",
            "scene_path": str(scene_path.resolve()),
            "frequency_hz": config.CARRIER_FREQUENCY,
        })
        self._receive()

    def _receive(self):
        response = self._connection.recv()
        if not response["ok"]:
            raise RuntimeError(response["error"])
        return response.get("result")

    def solve(self, transmitter_positions, receiver_positions):
        self._connection.send({
            "type": "snapshot",
            "transmitters": [
                {"id": int(identifier), "position": [float(value) for value in position]}
                for identifier, position in transmitter_positions.items()
            ],
            "receivers": [
                {"id": int(identifier), "position": [float(value) for value in position]}
                for identifier, position in receiver_positions.items()
            ],
            "max_depth": config.SIONNA_MAX_DEPTH,
            "samples_per_source": config.SIONNA_SAMPLES_PER_SOURCE,
            "bandwidth_hz": config.BANDWIDTH,
            "frequency_samples": config.SIONNA_FREQUENCY_SAMPLES,
            "seed": config.SIONNA_SEED,
            "los": config.SIONNA_LOS,
            "specular_reflection": config.SIONNA_SPECULAR_REFLECTION,
            "diffuse_reflection": config.SIONNA_DIFFUSE_REFLECTION,
            "refraction": config.SIONNA_REFRACTION,
            "diffraction": config.SIONNA_DIFFRACTION,
            "edge_diffraction": config.SIONNA_EDGE_DIFFRACTION,
        })
        return self._receive()

    def close(self):
        if self._process is None:
            return
        process = self._process
        if process.is_alive():
            try:
                self._connection.send({"type": "stop"})
            except (BrokenPipeError, EOFError, OSError):
                pass
            process.join(timeout=5)
        self._connection.close()
        process.close()
        self._process = None


@dataclass(slots=True)
class LinkSnapshot:
    gain: float
    sim_time_us: float
    transmitter_position: tuple[float, float, float]
    receiver_position: tuple[float, float, float]


class OnlineSionnaRtChannelModel:
    def __init__(self, event_bus, client=None):
        self.event_bus = event_bus
        self._client = client or SionnaWorkerClient()
        self._links = {}

    @staticmethod
    def _positions(drones):
        return {
            drone.identifier: tuple(float(value) for value in drone.coords)
            for drone in drones
        }

    @staticmethod
    def _unique(identifiers):
        return list(dict.fromkeys(int(identifier) for identifier in identifiers))

    def _is_stale(self, snapshot, sim_time_us, transmitter_position, receiver_position):
        return (
            snapshot is None
            or sim_time_us - snapshot.sim_time_us >= config.CHANNEL_SNAPSHOT_INTERVAL
            or math.dist(snapshot.transmitter_position, transmitter_position)
            >= config.CHANNEL_SNAPSHOT_DISPLACEMENT
            or math.dist(snapshot.receiver_position, receiver_position)
            >= config.CHANNEL_SNAPSHOT_DISPLACEMENT
        )

    def gains(self, sim_time_us, drones, transmitter_ids, receiver_ids):
        transmitter_ids = self._unique(transmitter_ids)
        receiver_ids = self._unique(receiver_ids)
        positions = self._positions(drones)
        requested_pairs = [
            (transmitter_id, receiver_id)
            for transmitter_id in transmitter_ids
            for receiver_id in receiver_ids
            if transmitter_id != receiver_id
        ]
        stale_pairs = [
            pair for pair in requested_pairs
            if self._is_stale(
                self._links.get(pair),
                sim_time_us,
                positions[pair[0]],
                positions[pair[1]],
            )
        ]
        if stale_pairs:
            stale_transmitters = self._unique(pair[0] for pair in stale_pairs)
            stale_receivers = self._unique(pair[1] for pair in stale_pairs)
            result = self._client.solve(
                {identifier: positions[identifier] for identifier in stale_transmitters},
                {identifier: positions[identifier] for identifier in stale_receivers},
            )
            solved_gains = np.asarray(result["gains"], dtype=float)
            for tx_index, transmitter_id in enumerate(result["transmitter_ids"]):
                for rx_index, receiver_id in enumerate(result["receiver_ids"]):
                    if transmitter_id == receiver_id:
                        continue
                    self._links[(transmitter_id, receiver_id)] = LinkSnapshot(
                        gain=float(solved_gains[tx_index, rx_index]),
                        sim_time_us=float(sim_time_us),
                        transmitter_position=positions[transmitter_id],
                        receiver_position=positions[receiver_id],
                    )
            self.event_bus.publish(
                "channel_snapshot",
                sim_time_us,
                mode="online",
                solve_time_ms=result["solve_time_ms"],
                transmitter_count=len(stale_transmitters),
                receiver_count=len(stale_receivers),
                link_count=len(stale_pairs),
            )
        return {pair: self._links[pair].gain for pair in requested_pairs}

    def close(self):
        self._client.close()


class HybridSionnaRtChannelModel(OnlineSionnaRtChannelModel):
    def __init__(self, event_bus, airspace, client=None):
        self.event_bus = event_bus
        self.airspace = airspace
        self._client = client
        self._links = {}

    def _solve(self, transmitter_positions, receiver_positions):
        if self._client is None:
            self._client = SionnaWorkerClient()
        return self._client.solve(transmitter_positions, receiver_positions)

    def gains(self, sim_time_us, drones, transmitter_ids, receiver_ids):
        transmitter_ids = self._unique(transmitter_ids)
        receiver_ids = self._unique(receiver_ids)
        positions = self._positions(drones)
        requested_pairs = [
            (transmitter_id, receiver_id)
            for transmitter_id in transmitter_ids
            for receiver_id in receiver_ids
            if transmitter_id != receiver_id
        ]
        stale_pairs = [
            pair for pair in requested_pairs
            if self._is_stale(
                self._links.get(pair),
                sim_time_us,
                positions[pair[0]],
                positions[pair[1]],
            )
        ]
        if not stale_pairs:
            return {pair: self._links[pair].gain for pair in requested_pairs}

        los_pairs = []
        nlos_pairs = []
        for pair in stale_pairs:
            collection = (
                los_pairs
                if self.airspace.has_line_of_sight(positions[pair[0]], positions[pair[1]])
                else nlos_pairs
            )
            collection.append(pair)

        for transmitter_id, receiver_id in los_pairs:
            self._links[(transmitter_id, receiver_id)] = LinkSnapshot(
                gain=free_space_path_gain(
                    math.dist(positions[transmitter_id], positions[receiver_id]),
                    config.CARRIER_FREQUENCY,
                ),
                sim_time_us=float(sim_time_us),
                transmitter_position=positions[transmitter_id],
                receiver_position=positions[receiver_id],
            )

        solve_time_ms = 0.0
        rt_matrix_link_count = 0
        if nlos_pairs:
            nlos_pair_set = set(nlos_pairs)
            nlos_transmitters = self._unique(pair[0] for pair in nlos_pairs)
            nlos_receivers = self._unique(pair[1] for pair in nlos_pairs)
            result = self._solve(
                {identifier: positions[identifier] for identifier in nlos_transmitters},
                {identifier: positions[identifier] for identifier in nlos_receivers},
            )
            solved_gains = np.asarray(result["gains"], dtype=float)
            for tx_index, transmitter_id in enumerate(result["transmitter_ids"]):
                for rx_index, receiver_id in enumerate(result["receiver_ids"]):
                    pair = (int(transmitter_id), int(receiver_id))
                    if pair not in nlos_pair_set:
                        continue
                    self._links[pair] = LinkSnapshot(
                        gain=float(solved_gains[tx_index, rx_index]),
                        sim_time_us=float(sim_time_us),
                        transmitter_position=positions[pair[0]],
                        receiver_position=positions[pair[1]],
                    )
            solve_time_ms = float(result["solve_time_ms"])
            rt_matrix_link_count = sum(
                transmitter_id != receiver_id
                for transmitter_id in nlos_transmitters
                for receiver_id in nlos_receivers
            )

        self.event_bus.publish(
            "channel_snapshot",
            sim_time_us,
            mode="hybrid",
            solve_time_ms=solve_time_ms,
            transmitter_count=len(self._unique(pair[0] for pair in stale_pairs)),
            receiver_count=len(self._unique(pair[1] for pair in stale_pairs)),
            link_count=len(stale_pairs),
            los_link_count=len(los_pairs),
            nlos_link_count=len(nlos_pairs),
            rt_matrix_link_count=rt_matrix_link_count,
        )
        return {pair: self._links[pair].gain for pair in requested_pairs}

    def close(self):
        if self._client is not None:
            self._client.close()


class OfflineSionnaRtChannelModel:
    def __init__(self, event_bus, channel_trace):
        self.event_bus = event_bus
        self.trace = channel_trace
        self._published_index = None

    def gains(self, sim_time_us, drones, transmitter_ids, receiver_ids):
        index = int(np.searchsorted(self.trace.channel_times_us, sim_time_us, side="right") - 1)
        index = max(0, min(index, len(self.trace.channel_times_us) - 1))
        if index != self._published_index:
            self._published_index = index
            self.event_bus.publish(
                "channel_snapshot",
                sim_time_us,
                mode="offline",
                solve_time_ms=0.0,
                trace_time_us=float(self.trace.channel_times_us[index]),
                trace_index=index,
            )
        matrix = self.trace.channel_gains[index]
        return {
            (int(transmitter_id), int(receiver_id)): float(matrix[transmitter_id, receiver_id])
            for transmitter_id in transmitter_ids
            for receiver_id in receiver_ids
            if transmitter_id != receiver_id
        }

    def close(self):
        return
