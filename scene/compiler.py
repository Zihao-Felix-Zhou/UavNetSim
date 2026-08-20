import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from shapely.geometry import Polygon
from shapely.validation import make_valid

from scene.models import SceneModel


ITU_TYPES = {
    "itu_concrete": "concrete",
    "itu_brick": "brick",
    "itu_glass": "glass",
    "itu_metal": "metal",
    "itu_wood": "wood",
    "itu_medium_dry_ground": "medium_dry_ground",
    "itu_wet_ground": "wet_ground",
}


def _ground(scene: SceneModel):
    if scene.terrain is not None:
        vertices = np.asarray([(point.x, point.y, point.z) for point in scene.terrain.vertices], dtype=float)
        faces = np.asarray(scene.terrain.faces, dtype=int)
        return trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    margin = 50.0
    vertices = np.array([
        [-margin, -margin, 0.0],
        [scene.size_x + margin, -margin, 0.0],
        [scene.size_x + margin, scene.size_y + margin, 0.0],
        [-margin, scene.size_y + margin, 0.0],
    ])
    return trimesh.Trimesh(vertices=vertices, faces=[[0, 1, 2], [0, 2, 3]], process=False)


def _polygon(feature):
    points = [(point.x, point.y) for point in feature.footprint]
    polygon = make_valid(Polygon(points))
    if polygon.geom_type == "MultiPolygon":
        polygon = max(polygon.geoms, key=lambda item: item.area)
    if polygon.geom_type != "Polygon" or polygon.area < 0.5:
        raise ValueError(f"Feature {feature.id} has an invalid footprint")
    return polygon


def _volume_mesh(feature):
    mesh = trimesh.creation.extrude_polygon(_polygon(feature), max(feature.height, 1.0))
    mesh.apply_translation((0.0, 0.0, float(np.mean([point.z for point in feature.footprint]))))
    return mesh


def _surface_mesh(feature):
    mesh = trimesh.creation.extrude_polygon(_polygon(feature), 0.08)
    mesh.apply_translation((0.0, 0.0, float(np.mean([point.z for point in feature.footprint])) + 0.04))
    return mesh


def _write_xml(meshes, materials, output_directory):
    root = ET.Element("scene", {"version": "3.0.0"})
    ET.SubElement(root, "integrator", {"type": "path"})
    emitter = ET.SubElement(root, "emitter", {"type": "constant"})
    ET.SubElement(emitter, "rgb", {"name": "radiance", "value": "0.7 0.7 0.7"})
    for material in sorted(materials):
        bsdf = ET.SubElement(root, "bsdf", {"type": "itu-radio-material", "id": material})
        ET.SubElement(bsdf, "string", {"name": "type", "value": ITU_TYPES[material]})
    ground_material = ET.SubElement(root, "bsdf", {"type": "itu-radio-material", "id": "ground-material"})
    ET.SubElement(ground_material, "string", {"name": "type", "value": "medium_dry_ground"})
    for filename, material, shape_id in meshes:
        shape = ET.SubElement(root, "shape", {"type": "ply", "id": shape_id})
        ET.SubElement(shape, "string", {"name": "filename", "value": filename})
        ET.SubElement(shape, "boolean", {"name": "face_normals", "value": "true"})
        ET.SubElement(shape, "ref", {"id": material})
    xml_path = output_directory / "scene.xml"
    ET.ElementTree(root).write(xml_path, encoding="utf-8", xml_declaration=True)
    return xml_path


def compile_scene(scene: SceneModel, output_directory, progress=None):
    progress = progress or (lambda _value, _stage: None)
    output_directory = Path(output_directory)
    mesh_directory = output_directory / "meshes"
    mesh_directory.mkdir(parents=True, exist_ok=True)
    for mesh_path in mesh_directory.glob("*.ply"):
        mesh_path.unlink()
    meshes = []
    materials = set()
    progress(0.02, "Creating ground mesh")
    _ground(scene).export(mesh_directory / "ground.ply", file_type="ply")
    meshes.append(("meshes/ground.ply", "ground-material", "mesh-ground"))
    compiled_features = [
        feature for feature in scene.features
        if feature.category in {"building", "water"}
        or (feature.category == "terrain" and scene.terrain is None)
    ]
    for index, feature in enumerate(compiled_features):
        material = feature.material if feature.material in ITU_TYPES else "itu_concrete"
        mesh_path = mesh_directory / f"{feature.category}-{index}.ply"
        mesh = _volume_mesh(feature) if feature.category == "building" else _surface_mesh(feature)
        mesh.export(mesh_path, file_type="ply")
        materials.add(material)
        meshes.append((f"meshes/{mesh_path.name}", material, f"mesh-{feature.category}-{index}"))
        progress(
            0.05 + 0.85 * (index + 1) / max(1, len(compiled_features)),
            f"Meshing {feature.category} features",
        )
    progress(0.93, "Writing Sionna scene")
    scene_path = output_directory / "scene.json"
    scene_path.write_text(scene.model_dump_json(indent=2), encoding="utf-8")
    xml_path = _write_xml(meshes, materials, output_directory)
    manifest = {
        "scene": scene_path.name,
        "mitsuba": xml_path.name,
        "building_count": sum(feature.category == "building" for feature in scene.features),
        "terrain_count": sum(feature.category == "terrain" for feature in scene.features),
        "water_count": sum(feature.category == "water" for feature in scene.features),
        "terrain_mesh": {
            "rows": scene.terrain.rows,
            "columns": scene.terrain.columns,
            "elevation_offset_m": scene.terrain.elevation_offset_m,
            "resolution_m": scene.terrain.resolution_m,
            "source": scene.terrain.source,
        } if scene.terrain else None,
    }
    (output_directory / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    progress(1.0, "Sionna scene ready")
    return xml_path
