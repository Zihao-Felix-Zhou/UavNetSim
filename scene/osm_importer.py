import json
import urllib.error
import urllib.parse
import urllib.request

from scene.coordinates import lat_lon_to_enu
from scene.models import GeoAnchor, GeoBounds, SceneFeature, SceneModel


OVERPASS_URLS = (
    "https://overpass-api.de/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
)
RETRYABLE_HTTP_ERRORS = {429, 502, 503, 504}
MATERIALS = {
    "brick": "itu_brick",
    "glass": "itu_glass",
    "metal": "itu_metal",
    "steel": "itu_metal",
    "wood": "itu_wood",
}
TERRAIN_TAGS = {
    ("natural", "grassland"),
    ("natural", "heath"),
    ("landuse", "grass"),
    ("landuse", "meadow"),
    ("landuse", "recreation_ground"),
    ("leisure", "garden"),
    ("leisure", "park"),
}
WATER_TAGS = {
    ("natural", "water"),
    ("landuse", "basin"),
    ("landuse", "reservoir"),
    ("waterway", "riverbank"),
}


def _height(tags):
    raw_height = tags.get("height")
    if raw_height:
        try:
            return max(1.0, float(raw_height.split()[0]))
        except ValueError:
            pass
    levels = tags.get("building:levels")
    if levels:
        try:
            return max(1.0, float(levels) * 3.2)
        except ValueError:
            pass
    building_type = tags.get("building", "")
    if building_type in {"office", "commercial", "apartments"}:
        return 18.0
    if building_type in {"industrial", "warehouse"}:
        return 10.0
    return 8.0


def _material(tags):
    raw = tags.get("building:material", tags.get("material", "")).lower()
    return MATERIALS.get(raw, "itu_concrete")


def _tagged(tags, choices):
    return any(tags.get(key) == value for key, value in choices)


def _feature_kind(tags):
    if tags.get("building"):
        return "building", _material(tags), _height(tags)
    if tags.get("highway"):
        return "road", "itu_concrete", 0.0
    if _tagged(tags, WATER_TAGS):
        return "water", "itu_wet_ground", 0.0
    if _tagged(tags, TERRAIN_TAGS):
        return "terrain", "itu_medium_dry_ground", 0.0
    return None


def _query(bounds: GeoBounds):
    box = f"{bounds.south},{bounds.west},{bounds.north},{bounds.east}"
    return f"""[out:json][timeout:30];
(
  way[\"building\"]({box});
  relation[\"building\"]({box});
  way[\"highway\"]({box});
  way[\"natural\"~\"^(grassland|heath|water)$\"]({box});
  relation[\"natural\"~\"^(grassland|heath|water)$\"]({box});
  way[\"landuse\"~\"^(grass|meadow|recreation_ground|basin|reservoir)$\"]({box});
  relation[\"landuse\"~\"^(grass|meadow|recreation_ground|basin|reservoir)$\"]({box});
  way[\"leisure\"~\"^(garden|park)$\"]({box});
  relation[\"leisure\"~\"^(garden|park)$\"]({box});
  way[\"waterway\"=\"riverbank\"]({box});
  relation[\"waterway\"=\"riverbank\"]({box});
);
out body;
>;
out skel qt;"""


def _join_outer_ways(paths):
    remaining = [path[:] for path in paths if len(path) >= 2]
    rings = []
    while remaining:
        ring = remaining.pop(0)
        joined = True
        while ring[0] != ring[-1] and joined:
            joined = False
            for index, path in enumerate(remaining):
                if ring[-1] == path[0]:
                    ring.extend(path[1:])
                elif ring[-1] == path[-1]:
                    ring.extend(reversed(path[:-1]))
                elif ring[0] == path[-1]:
                    ring = path[:-1] + ring
                elif ring[0] == path[0]:
                    ring = list(reversed(path[1:])) + ring
                else:
                    continue
                remaining.pop(index)
                joined = True
                break
        if len(ring) >= 4 and ring[0] == ring[-1]:
            rings.append(ring)
    return rings


def fetch_osm_scene(bounds: GeoBounds, name="OSM Scene", progress=None):
    progress = progress or (lambda _value, _stage: None)
    request_data = urllib.parse.urlencode({"data": _query(bounds)}).encode()
    failures = []
    for index, endpoint in enumerate(OVERPASS_URLS):
        progress(0.05 + index * 0.05, "Downloading OpenStreetMap data")
        request = urllib.request.Request(
            endpoint,
            data=request_data,
            headers={
                "Accept": "application/json",
                "Content-Type": "application/x-www-form-urlencoded; charset=UTF-8",
                "User-Agent": "UavNetSim-v2/2.0 research-simulator",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=45) as response:
                payload = json.loads(response.read().decode("utf-8"))
            progress(0.55, "Parsing buildings and land cover")
            return parse_osm_scene(payload, bounds, name, progress)
        except urllib.error.HTTPError as error:
            if error.code not in RETRYABLE_HTTP_ERRORS:
                raise RuntimeError(f"Overpass API rejected the request with HTTP {error.code}") from error
            failures.append(f"{endpoint}: HTTP {error.code}")
        except (urllib.error.URLError, TimeoutError) as error:
            failures.append(f"{endpoint}: {error.reason if hasattr(error, 'reason') else error}")

    details = "; ".join(failures)
    raise RuntimeError(
        "Unable to reach an Overpass API server. Check the network, proxy, or firewall settings. "
        f"Attempts: {details}"
    )


def parse_osm_scene(payload, bounds: GeoBounds, name="OSM Scene", progress=None):
    progress = progress or (lambda _value, _stage: None)
    anchor = GeoAnchor(latitude=bounds.south, longitude=bounds.west)
    elements = payload.get("elements", [])
    nodes = {
        element["id"]: (element["lat"], element["lon"])
        for element in elements
        if element.get("type") == "node"
    }
    ways = {
        element["id"]: element.get("nodes", [])
        for element in elements
        if element.get("type") == "way"
    }
    features = []
    relation_way_ids = {
        member["ref"]
        for element in elements
        if element.get("type") == "relation" and _feature_kind(element.get("tags", {}))
        for member in element.get("members", [])
        if member.get("type") == "way" and member.get("role") in {"", "outer"}
    }
    for element_index, element in enumerate(elements):
        if element_index % max(1, len(elements) // 20) == 0:
            progress(0.55 + 0.35 * element_index / max(1, len(elements)), "Parsing buildings and land cover")
        if element.get("type") != "way" or not element.get("tags"):
            continue
        tags = element["tags"]
        kind = _feature_kind(tags)
        if not kind or element["id"] in relation_way_ids:
            continue
        category, material, height = kind
        node_ids = ways[element["id"]]
        points = [lat_lon_to_enu(*nodes[node_id], anchor) for node_id in node_ids if node_id in nodes]
        minimum_points = 2 if category == "road" else 3
        if len(points) >= minimum_points:
            features.append(SceneFeature(
                id=f"osm-{category}-{element['id']}",
                category=category,
                footprint=points,
                height=height,
                material=material,
                source="openstreetmap",
            ))
    for relation in elements:
        if relation.get("type") != "relation":
            continue
        tags = relation.get("tags", {})
        kind = _feature_kind(tags)
        if not kind or kind[0] == "road":
            continue
        category, material, height = kind
        outer_paths = [
            ways.get(member["ref"], [])
            for member in relation.get("members", [])
            if member.get("type") == "way" and member.get("role") in {"", "outer"}
        ]
        for index, ring in enumerate(_join_outer_ways(outer_paths)):
            points = [lat_lon_to_enu(*nodes[node_id], anchor) for node_id in ring if node_id in nodes]
            if len(points) >= 3:
                features.append(SceneFeature(
                    id=f"osm-{category}-relation-{relation['id']}-{index}",
                    category=category,
                    footprint=points,
                    height=height,
                    material=material,
                    source="openstreetmap",
                ))
    progress(0.95, "Finalizing geospatial model")
    northeast = lat_lon_to_enu(bounds.north, bounds.east, anchor)
    return SceneModel(
        name=name,
        anchor=anchor,
        bounds=bounds,
        size_x=northeast.x,
        size_y=northeast.y,
        features=features,
    )
