import argparse
import json
from pathlib import Path

import simpy
import uvicorn

from scene.compiler import compile_scene
from scene.models import GeoBounds, SceneModel
from scene.osm_importer import fetch_osm_scene
from simulator.simulator import Simulator
from utils import config


def run_simulation(arguments):
    config.ROUTING_PROTOCOL = arguments.routing
    config.MAC_PROTOCOL = arguments.mac
    config.MOBILITY_MODEL = arguments.mobility
    config.NUMBER_OF_DRONES = arguments.nodes
    config.MAX_TTL = arguments.nodes + 1
    config.SIONNA_SAMPLES_PER_SOURCE = arguments.samples
    environment = simpy.Environment()
    simulator = Simulator(
        seed=arguments.seed,
        env=environment,
        n_drones=arguments.nodes,
        total_simulation_time=arguments.duration * 1e6,
        drone_speed=arguments.uav_speed,
    )
    try:
        environment.run(until=arguments.duration * 1e6)
        simulator.metrics.print_metrics()
    finally:
        simulator.close()


def compile_scene_file(arguments):
    scene = SceneModel.model_validate_json(Path(arguments.input).read_text(encoding="utf-8"))
    print(compile_scene(scene, arguments.output))


def import_osm(arguments):
    bounds = GeoBounds(south=arguments.south, west=arguments.west,
                       north=arguments.north, east=arguments.east)
    scene = fetch_osm_scene(bounds, arguments.name)
    compile_scene(scene, arguments.output)
    print(json.dumps(scene.model_dump(), indent=2))


def parser():
    root = argparse.ArgumentParser(prog="uavnetsim")
    commands = root.add_subparsers(dest="command", required=True)

    run = commands.add_parser("run")
    run.add_argument("--nodes", type=int, default=10)
    run.add_argument("--duration", type=float, default=20)
    run.add_argument("--seed", type=int, default=2025)
    run.add_argument("--routing", default="Greedy")
    run.add_argument("--mac", default="CSMA_CA")
    run.add_argument("--mobility", default="GaussMarkov3D")
    run.add_argument("--uav-speed", type=float, default=config.UAV_SPEED)
    run.add_argument("--samples", type=int, default=100000)
    run.set_defaults(handler=run_simulation)

    serve = commands.add_parser("serve")
    serve.add_argument("--host", default="127.0.0.1")
    serve.add_argument("--port", type=int, default=8000)
    serve.set_defaults(handler=lambda args: uvicorn.run("api.app:app", host=args.host, port=args.port))

    compile_command = commands.add_parser("compile-scene")
    compile_command.add_argument("input")
    compile_command.add_argument("--output", default="artifacts/scene")
    compile_command.set_defaults(handler=compile_scene_file)

    osm = commands.add_parser("import-osm")
    osm.add_argument("--south", type=float, required=True)
    osm.add_argument("--west", type=float, required=True)
    osm.add_argument("--north", type=float, required=True)
    osm.add_argument("--east", type=float, required=True)
    osm.add_argument("--name", default="OSM Scene")
    osm.add_argument("--output", default="artifacts/scene")
    osm.set_defaults(handler=import_osm)
    return root


def main():
    arguments = parser().parse_args()
    arguments.handler(arguments)


if __name__ == "__main__":
    main()
