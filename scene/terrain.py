import math
import urllib.error
import urllib.request
from io import BytesIO

import numpy as np
from PIL import Image

from scene.models import EnuPoint, TerrainMesh


TERRAIN_ZOOM = 14
TILE_SIZE = 256
TARGET_GRID_SPACING_M = 12.0
MAX_GRID_SIZE = 129
TERRAIN_TILE_URLS = (
    "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png",
    "https://elevation-tiles-prod.s3.amazonaws.com/terrarium/{z}/{x}/{y}.png",
)


def _global_pixel(latitude, longitude, zoom=TERRAIN_ZOOM):
    latitude = max(-85.05112878, min(85.05112878, latitude))
    scale = TILE_SIZE * (2 ** zoom)
    x = (longitude + 180.0) / 360.0 * scale
    y = (1.0 - math.asinh(math.tan(math.radians(latitude))) / math.pi) / 2.0 * scale
    return x, y


def _download_tile(x, y, zoom=TERRAIN_ZOOM):
    errors = []
    for template in TERRAIN_TILE_URLS:
        url = template.format(z=zoom, x=x, y=y)
        request = urllib.request.Request(url, headers={"User-Agent": "UavNetSim/2.0 research-simulator"})
        try:
            with urllib.request.urlopen(request, timeout=45) as response:
                image = Image.open(BytesIO(response.read())).convert("RGBA")
                return np.asarray(image, dtype=np.float64)
        except (urllib.error.HTTPError, urllib.error.URLError, TimeoutError) as error:
            errors.append(f"{url}: {error}")
    raise RuntimeError("Unable to download terrain elevation tile. " + "; ".join(errors))


def _tile_keys(bounds, zoom=TERRAIN_ZOOM):
    west_x, north_y = _global_pixel(bounds.north, bounds.west, zoom)
    east_x, south_y = _global_pixel(bounds.south, bounds.east, zoom)
    minimum_x = math.floor(min(west_x, east_x))
    maximum_x = math.ceil(max(west_x, east_x)) + 1
    minimum_y = math.floor(min(north_y, south_y))
    maximum_y = math.ceil(max(north_y, south_y)) + 1
    return [
        (tile_x, tile_y)
        for tile_y in range(minimum_y // TILE_SIZE, maximum_y // TILE_SIZE + 1)
        for tile_x in range(minimum_x // TILE_SIZE, maximum_x // TILE_SIZE + 1)
    ]


def _sample_elevation(tiles, latitude, longitude, zoom=TERRAIN_ZOOM):
    global_x, global_y = _global_pixel(latitude, longitude, zoom)
    pixel_x = math.floor(global_x)
    pixel_y = math.floor(global_y)
    fraction_x = global_x - pixel_x
    fraction_y = global_y - pixel_y
    samples = []
    for offset_y in (0, 1):
        row = []
        for offset_x in (0, 1):
            x = pixel_x + offset_x
            y = pixel_y + offset_y
            tile = tiles[(x // TILE_SIZE, y // TILE_SIZE)]
            red, green, blue, alpha = tile[y % TILE_SIZE, x % TILE_SIZE]
            if alpha == 0:
                raise RuntimeError("Terrain elevation tile contains missing data")
            row.append(red * 256.0 + green + blue / 256.0 - 32768.0)
        samples.append(row)
    north = samples[0][0] * (1.0 - fraction_x) + samples[0][1] * fraction_x
    south = samples[1][0] * (1.0 - fraction_x) + samples[1][1] * fraction_x
    return north * (1.0 - fraction_y) + south * fraction_y


def terrain_height(terrain, x, y, size_x, size_y):
    if terrain is None:
        return 0.0
    column = max(0.0, min(terrain.columns - 1.0, x / size_x * (terrain.columns - 1)))
    row = max(0.0, min(terrain.rows - 1.0, y / size_y * (terrain.rows - 1)))
    left = min(terrain.columns - 2, int(math.floor(column)))
    lower = min(terrain.rows - 2, int(math.floor(row)))
    fraction_x = column - left
    fraction_y = row - lower
    base = lower * terrain.columns + left
    southwest = terrain.vertices[base].z
    southeast = terrain.vertices[base + 1].z
    northwest = terrain.vertices[base + terrain.columns].z
    northeast = terrain.vertices[base + terrain.columns + 1].z
    south = southwest * (1.0 - fraction_x) + southeast * fraction_x
    north = northwest * (1.0 - fraction_x) + northeast * fraction_x
    return south * (1.0 - fraction_y) + north * fraction_y


def attach_terrain(scene, progress=None):
    if scene.bounds is None:
        raise ValueError("Geographic bounds are required to build terrain")
    progress = progress or (lambda _value, _stage: None)
    keys = _tile_keys(scene.bounds)
    tiles = {}
    for index, key in enumerate(keys):
        progress(0.05 + 0.4 * index / max(1, len(keys)), f"Downloading terrain tile {index + 1}/{len(keys)}")
        tiles[key] = _download_tile(*key)

    columns = min(MAX_GRID_SIZE, max(17, math.ceil(scene.size_x / TARGET_GRID_SPACING_M) + 1))
    rows = min(MAX_GRID_SIZE, max(17, math.ceil(scene.size_y / TARGET_GRID_SPACING_M) + 1))
    elevations = np.empty((rows, columns), dtype=np.float64)
    for row in range(rows):
        latitude = scene.bounds.south + (scene.bounds.north - scene.bounds.south) * row / (rows - 1)
        for column in range(columns):
            longitude = scene.bounds.west + (scene.bounds.east - scene.bounds.west) * column / (columns - 1)
            elevations[row, column] = _sample_elevation(tiles, latitude, longitude)
        progress(0.45 + 0.3 * (row + 1) / rows, "Sampling terrain elevation")

    elevation_offset = float(np.min(elevations))
    elevations -= elevation_offset
    vertices = [
        EnuPoint(
            x=round(scene.size_x * column / (columns - 1), 3),
            y=round(scene.size_y * row / (rows - 1), 3),
            z=round(float(elevations[row, column]), 3),
        )
        for row in range(rows)
        for column in range(columns)
    ]
    faces = []
    for row in range(rows - 1):
        for column in range(columns - 1):
            southwest = row * columns + column
            southeast = southwest + 1
            northwest = southwest + columns
            northeast = northwest + 1
            faces.extend(((southwest, southeast, northeast), (southwest, northeast, northwest)))
    terrain = TerrainMesh(
        rows=rows,
        columns=columns,
        vertices=vertices,
        faces=faces,
        elevation_offset_m=round(elevation_offset, 3),
        resolution_m=max(scene.size_x / (columns - 1), scene.size_y / (rows - 1)),
        source="Mapzen Terrain Tiles (SRTM and global DEM sources)",
    )

    progress(0.8, "Draping scene objects onto terrain")
    features = []
    for feature in scene.features:
        footprint = [
            point.model_copy(update={"z": round(terrain_height(terrain, point.x, point.y, scene.size_x, scene.size_y), 3)})
            for point in feature.footprint
        ]
        if feature.category in {"building", "water"}:
            base_height = float(np.median([point.z for point in footprint]))
            footprint = [point.model_copy(update={"z": round(base_height, 3)}) for point in footprint]
        features.append(feature.model_copy(update={"footprint": footprint}))
    progress(1.0, "Terrain elevation ready")
    return scene.model_copy(update={"terrain": terrain, "features": features})
