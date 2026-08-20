import hashlib
import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from mobility.trajectory import TraceBuildCancelled
from phy.sionna_rt import SionnaWorkerClient
from utils import config


@dataclass(slots=True)
class ChannelTrace:
    channel_times_us: np.ndarray
    channel_gains: np.ndarray
    cache_key: str


def _positions_at(trajectory, sim_time_us):
    index = int(np.searchsorted(trajectory.times_us, sim_time_us, side="right") - 1)
    return trajectory.positions[max(0, index)]


def channel_snapshot_positions(trajectory):
    interval = float(config.CHANNEL_SNAPSHOT_INTERVAL)
    duration = float(trajectory.times_us[-1])
    interval_times = np.arange(0.0, duration + interval, interval)
    interval_times = interval_times[interval_times <= duration]
    candidates = np.unique(np.concatenate((trajectory.times_us, interval_times, [duration])))
    selected_times = [float(candidates[0])]
    selected_positions = [_positions_at(trajectory, candidates[0]).copy()]
    for candidate in candidates[1:]:
        positions = _positions_at(trajectory, candidate)
        elapsed = candidate - selected_times[-1]
        displacement = np.linalg.norm(positions - selected_positions[-1], axis=1).max()
        if (
            elapsed >= config.CHANNEL_SNAPSHOT_INTERVAL
            or displacement >= config.CHANNEL_SNAPSHOT_DISPLACEMENT
        ):
            selected_times.append(float(candidate))
            selected_positions.append(positions.copy())
    return np.asarray(selected_times, dtype=float), np.asarray(selected_positions, dtype=float)


def _cache_key(times_us, positions):
    scene_path = Path(config.SIONNA_SCENE_PATH)
    settings = {
        "version": 1,
        "scene_sha256": hashlib.sha256(scene_path.read_bytes()).hexdigest(),
        "carrier_frequency": config.CARRIER_FREQUENCY,
        "bandwidth": config.BANDWIDTH,
        "max_depth": config.SIONNA_MAX_DEPTH,
        "samples_per_source": config.SIONNA_SAMPLES_PER_SOURCE,
        "frequency_samples": config.SIONNA_FREQUENCY_SAMPLES,
        "seed": config.SIONNA_SEED,
        "los": config.SIONNA_LOS,
        "specular_reflection": config.SIONNA_SPECULAR_REFLECTION,
        "diffuse_reflection": config.SIONNA_DIFFUSE_REFLECTION,
        "refraction": config.SIONNA_REFRACTION,
        "diffraction": config.SIONNA_DIFFRACTION,
        "edge_diffraction": config.SIONNA_EDGE_DIFFRACTION,
    }
    digest = hashlib.sha256(json.dumps(settings, sort_keys=True).encode("utf-8"))
    digest.update(times_us.tobytes())
    digest.update(positions.tobytes())
    return digest.hexdigest()


def _load(path, cache_key):
    with np.load(path, allow_pickle=False) as data:
        return ChannelTrace(
            channel_times_us=np.asarray(data["channel_times_us"], dtype=float),
            channel_gains=np.asarray(data["channel_gains"], dtype=float),
            cache_key=cache_key,
        )


def _save(path, trace):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, suffix=".npz", delete=False) as handle:
        temporary_path = Path(handle.name)
        np.savez_compressed(
            handle,
            channel_times_us=trace.channel_times_us,
            channel_gains=trace.channel_gains,
        )
    os.replace(temporary_path, path)


def build_or_load_channel_trace(trajectory, event_bus, stop_event=None, progress=None):
    times_us, positions = channel_snapshot_positions(trajectory)
    cache_key = _cache_key(times_us, positions)
    cache_path = config.PROJECT_ROOT / "artifacts" / "channel_traces" / f"{cache_key}.npz"
    total = len(times_us)
    if cache_path.is_file():
        if progress is not None:
            progress(total, total, True)
        event_bus.publish(
            "channel_trace_ready", 0, cache_hit=True, snapshots=total, cache_key=cache_key
        )
        return _load(cache_path, cache_key)

    if progress is not None:
        progress(0, total, False)
    event_bus.publish(
        "channel_trace_started", 0, cache_hit=False, snapshots=total, cache_key=cache_key
    )
    client = SionnaWorkerClient()
    matrices = []
    solved_positions = {}
    identifiers = list(range(positions.shape[1]))
    try:
        for index, snapshot_positions in enumerate(positions):
            if stop_event is not None and stop_event.is_set():
                raise TraceBuildCancelled()
            position_key = snapshot_positions.tobytes()
            matrix = solved_positions.get(position_key)
            if matrix is None:
                event_bus.publish(
                    "channel_trace_snapshot_started",
                    0,
                    completed=index,
                    total=total,
                )
                result = client.solve(
                    {identifier: snapshot_positions[identifier] for identifier in identifiers},
                    {identifier: snapshot_positions[identifier] for identifier in identifiers},
                )
                matrix = np.asarray(result["gains"], dtype=float)
                np.fill_diagonal(matrix, 0.0)
                solved_positions[position_key] = matrix
                solve_time_ms = result["solve_time_ms"]
            else:
                solve_time_ms = 0.0
            matrices.append(matrix)
            if progress is not None:
                progress(index + 1, total, False)
            event_bus.publish(
                "channel_trace_snapshot_ready",
                0,
                completed=index + 1,
                total=total,
                solve_time_ms=solve_time_ms,
            )
    finally:
        client.close()
    trace = ChannelTrace(
        channel_times_us=times_us,
        channel_gains=np.asarray(matrices, dtype=float),
        cache_key=cache_key,
    )
    _save(cache_path, trace)
    event_bus.publish(
        "channel_trace_ready", 0, cache_hit=False, snapshots=total, cache_key=cache_key
    )
    return trace
