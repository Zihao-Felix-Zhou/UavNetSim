import math
import random
from dataclasses import dataclass
from pathlib import Path

from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree
from shapely.validation import make_valid

from scene.models import SceneModel
from scene.terrain import terrain_height


@dataclass(frozen=True, slots=True)
class BuildingCollision:
    building_id: str
    kind: str
    path_fraction: float


@dataclass(frozen=True, slots=True)
class BuildingVolume:
    building_id: str
    footprint: Polygon
    top: float


class Airspace:
    def __init__(self, scene, max_height, building_clearance=1.0, boundary_clearance=1.0,
                 height_above_terrain=120.0):
        self.scene = scene
        self.size_x = float(scene.size_x)
        self.size_y = float(scene.size_y)
        terrain_peak = max((point.z for point in scene.terrain.vertices), default=0.0) if scene.terrain else 0.0
        self.max_height = max(float(max_height), terrain_peak + float(height_above_terrain))
        self.building_clearance = float(building_clearance)
        self.boundary_clearance = float(boundary_clearance)
        self.terrain = scene.terrain
        self.buildings = self._building_volumes(scene)
        self._footprints = [building.footprint for building in self.buildings]
        self._tree = STRtree(self._footprints) if self._footprints else None

    @classmethod
    def from_file(cls, scene_path, max_height, building_clearance=1.0, boundary_clearance=1.0):
        scene = SceneModel.model_validate_json(Path(scene_path).read_text(encoding="utf-8"))
        return cls(scene, max_height, building_clearance, boundary_clearance)

    def _building_volumes(self, scene):
        volumes = []
        for feature in scene.features:
            if feature.category != "building":
                continue
            polygon = make_valid(Polygon((point.x, point.y) for point in feature.footprint))
            polygons = polygon.geoms if polygon.geom_type == "MultiPolygon" else [polygon]
            for part in polygons:
                if part.geom_type != "Polygon" or part.area == 0:
                    continue
                base_height = sum(point.z for point in feature.footprint) / len(feature.footprint)
                volumes.append(BuildingVolume(
                    building_id=feature.id,
                    footprint=part.buffer(self.building_clearance, join_style="mitre"),
                    top=float(feature.height) + base_height + self.building_clearance,
                ))
        return volumes

    def _candidate_indices(self, geometry):
        if self._tree is None:
            return []
        return self._tree.query(geometry, predicate="intersects")

    def building_at(self, position):
        point = Point(float(position[0]), float(position[1]))
        altitude = float(position[2])
        for index in self._candidate_indices(point):
            building = self.buildings[int(index)]
            if altitude <= building.top and building.footprint.covers(point):
                return building.building_id
        return None

    def ground_height(self, x, y):
        return terrain_height(self.terrain, float(x), float(y), self.size_x, self.size_y)

    def position_is_free(self, position):
        x, y, z = (float(value) for value in position)
        margin = self.boundary_clearance
        inside_bounds = (
            margin <= x <= self.size_x - margin
            and margin <= y <= self.size_y - margin
            and self.ground_height(x, y) + margin <= z <= self.max_height - margin
        )
        return inside_bounds and self.building_at(position) is None

    @staticmethod
    def _geometry_intervals(intersection, line, line_length):
        if intersection.is_empty:
            return []
        if intersection.geom_type == "Point":
            fraction = line.project(intersection) / line_length
            return [(fraction, fraction)]
        if intersection.geom_type == "LineString":
            fractions = [line.project(Point(coordinate)) / line_length for coordinate in intersection.coords]
            return [(min(fractions), max(fractions))]
        intervals = []
        for geometry in intersection.geoms:
            intervals.extend(Airspace._geometry_intervals(geometry, line, line_length))
        return intervals

    @staticmethod
    def _volume_collision(interval, start_z, end_z, building_top):
        interval_start, interval_end = interval
        delta_z = end_z - start_z
        altitude_at_start = start_z + delta_z * interval_start
        altitude_at_end = start_z + delta_z * interval_end
        if altitude_at_start <= building_top:
            return interval_start, "side"
        if altitude_at_end > building_top or delta_z >= 0:
            return None
        roof_fraction = (building_top - start_z) / delta_z
        return max(interval_start, roof_fraction), "roof"

    def path_collision(self, start, end):
        start = [float(value) for value in start]
        end = [float(value) for value in end]
        start_xy = Point(start[0], start[1])
        end_xy = Point(end[0], end[1])
        line_length = start_xy.distance(end_xy)
        geometry = LineString([start_xy, end_xy]) if line_length > 1e-9 else start_xy
        earliest = self._terrain_collision(start, end)
        for index in self._candidate_indices(geometry):
            building = self.buildings[int(index)]
            if line_length <= 1e-9:
                intervals = [(0.0, 1.0)] if building.footprint.covers(start_xy) else []
            else:
                intersection = geometry.intersection(building.footprint)
                intervals = self._geometry_intervals(intersection, geometry, line_length)
            for interval in intervals:
                result = self._volume_collision(interval, start[2], end[2], building.top)
                if result is None:
                    continue
                fraction, kind = result
                collision = BuildingCollision(building.building_id, kind, fraction)
                if earliest is None or collision.path_fraction < earliest.path_fraction:
                    earliest = collision
        return earliest

    def _terrain_collision(self, start, end):
        if self.terrain is None:
            return None
        horizontal_distance = math.dist(start[:2], end[:2])
        step_length = max(1.0, self.terrain.resolution_m / 2.0)
        steps = max(1, math.ceil(horizontal_distance / step_length))
        for step in range(1, steps + 1):
            fraction = step / steps
            x = start[0] + (end[0] - start[0]) * fraction
            y = start[1] + (end[1] - start[1]) * fraction
            z = start[2] + (end[2] - start[2]) * fraction
            if z <= self.ground_height(x, y) + self.boundary_clearance:
                return BuildingCollision("terrain", "terrain", fraction)
        return None

    def path_is_free(self, start, end):
        return self.position_is_free(end) and self.path_collision(start, end) is None

    def resolve_motion(self, start, end, velocity):
        resolved_end = [float(value) for value in end]
        reflected = [float(value) for value in velocity]
        limits = (
            (self.boundary_clearance, self.size_x - self.boundary_clearance),
            (self.boundary_clearance, self.size_y - self.boundary_clearance),
            (self.boundary_clearance, self.max_height - self.boundary_clearance),
        )
        for axis, (minimum, maximum) in enumerate(limits):
            clamped = max(minimum, min(resolved_end[axis], maximum))
            if clamped != resolved_end[axis]:
                reflected[axis] = -reflected[axis]
                resolved_end[axis] = clamped
        collision = self.path_collision(start, resolved_end)
        if collision is None:
            return resolved_end, reflected, None
        if collision.kind == "roof" or (collision.kind == "terrain" and reflected[2] < 0):
            reflected[2] = -reflected[2]
        else:
            reflected[0] = -reflected[0]
            reflected[1] = -reflected[1]
        return [float(value) for value in start], reflected, collision

    def random_free_position(self, rng, existing=(), min_separation=0.0, visible_from=None,
                             minimum_distance=0.0, attempts=10000):
        margin = self.boundary_clearance
        for _ in range(attempts):
            x = rng.uniform(margin, self.size_x - margin)
            y = rng.uniform(margin, self.size_y - margin)
            minimum_z = self.ground_height(x, y) + margin
            maximum_z = self.max_height - margin
            if minimum_z > maximum_z:
                continue
            position = [
                x,
                y,
                rng.uniform(minimum_z, maximum_z),
            ]
            if not self.position_is_free(position):
                continue
            if any(math.dist(position, other) < min_separation for other in existing):
                continue
            if visible_from is not None:
                if math.dist(position, visible_from) < minimum_distance:
                    continue
                if self.path_collision(visible_from, position) is not None:
                    continue
            return position
        raise RuntimeError("Unable to place a UAV in the free airspace")

    def random_positions(self, seed, count, minimum_separation):
        positions = []
        for identifier in range(count):
            rng = random.Random(seed + identifier)
            positions.append(self.random_free_position(
                rng,
                existing=positions,
                min_separation=minimum_separation,
            ))
        return positions

    def validate_path(self, path):
        for position in path:
            if not self.position_is_free(position):
                raise ValueError(f"Flight path position is outside free airspace: {position}")
        for start, end in zip(path, path[1:]):
            collision = self.path_collision(start, end)
            if collision is not None:
                raise ValueError(f"Flight path intersects {collision.building_id}")
