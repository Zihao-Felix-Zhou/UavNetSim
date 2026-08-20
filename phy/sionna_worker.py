import os
import time

import numpy as np


def _remove_devices(scene):
    for name in list(scene.transmitters):
        scene.remove(name)
    for name in list(scene.receivers):
        scene.remove(name)


def _configure_scene(scene_path, frequency_hz):
    from sionna.rt import PlanarArray, load_scene

    scene = load_scene(scene_path)
    scene.frequency = float(frequency_hz)
    antenna = PlanarArray(
        num_rows=1,
        num_cols=1,
        vertical_spacing=0.5,
        horizontal_spacing=0.5,
        pattern="iso",
        polarization="V",
    )
    scene.tx_array = antenna
    scene.rx_array = antenna
    return scene


def _solve_snapshot(scene, request):
    from sionna.rt import PathSolver, Receiver, Transmitter

    _remove_devices(scene)
    transmitters = request["transmitters"]
    receivers = request["receivers"]
    for item in transmitters:
        scene.add(Transmitter(name=f"tx-{item['id']}", position=item["position"]))
    for item in receivers:
        scene.add(Receiver(name=f"rx-{item['id']}", position=item["position"]))
    started = time.perf_counter()
    paths = PathSolver()(
        scene,
        max_depth=request["max_depth"],
        samples_per_src=request["samples_per_source"],
        seed=request["seed"],
        los=request["los"],
        specular_reflection=request["specular_reflection"],
        diffuse_reflection=request["diffuse_reflection"],
        refraction=request["refraction"],
        diffraction=request["diffraction"],
        edge_diffraction=request["edge_diffraction"],
        synthetic_array=True,
    )
    frequencies = np.linspace(
        -request["bandwidth_hz"] / 2,
        request["bandwidth_hz"] / 2,
        request["frequency_samples"],
    )
    frequency_response = paths.cfr(
        frequencies=frequencies,
        normalize_delays=False,
        normalize=False,
        out_type="numpy",
    )
    gains_rx_tx = np.mean(np.abs(frequency_response) ** 2, axis=(1, 3, 4, 5))
    return {
        "gains": gains_rx_tx.T.tolist(),
        "transmitter_ids": [item["id"] for item in transmitters],
        "receiver_ids": [item["id"] for item in receivers],
        "solve_time_ms": (time.perf_counter() - started) * 1000,
    }


def run_worker(connection):
    scene = None
    try:
        while True:
            request = connection.recv()
            request_type = request["type"]
            if request_type == "configure":
                scene = _configure_scene(request["scene_path"], request["frequency_hz"])
                connection.send({"ok": True})
            elif request_type == "snapshot":
                if scene is None:
                    raise RuntimeError("Sionna scene is not configured")
                connection.send({"ok": True, "result": _solve_snapshot(scene, request)})
            elif request_type == "stop":
                connection.close()
                os._exit(0)
    except BaseException as error:
        try:
            connection.send({"ok": False, "error": f"{type(error).__name__}: {error}"})
        finally:
            connection.close()
            os._exit(1)
