"""Export only the live ship-grid environment as a portable physics USD bundle.

Call export_environment(sim, grid) through ./ctl eval; do not run scene.py.
Geometry is read from the compiled live model, not reconstructed from defaults.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt


ROOT = Path(__file__).resolve().parent
MATERIALS = {
    "deck": ((0.40, 0.46, 0.47), 0.40, 0.80, "base", 3.0),
    "flange": ((0.52, 0.60, 0.59), 0.50, 0.72, "base", 8.0),
    "web": ((0.18, 0.25, 0.23), 0.50, 0.72, "upright", 8.0),
    "cut": ((0.68, 0.76, 0.75), 0.75, 0.40, None, 1.0),
    "surround": ((0.22, 0.25, 0.28), 0.0, 0.95, None, 1.0),
}


def _surface(stage, kind):
    color, metallic, roughness, texture, repeat = MATERIALS[kind]
    path = f"/ShipGrid/Looks/{kind}"
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    if texture:
        reader = UsdShade.Shader.Define(stage, path + "/UV")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
        reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
        transform = UsdShade.Shader.Define(stage, path + "/Repeat")
        transform.CreateIdAttr("UsdTransform2d")
        transform.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
        transform.CreateInput("scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(repeat, repeat))
        transform.CreateOutput("result", Sdf.ValueTypeNames.Float2)
        image = UsdShade.Shader.Define(stage, path + "/Texture")
        image.CreateIdAttr("UsdUVTexture")
        image.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(f"textures/steel_{texture}.png"))
        image.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        image.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        image.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        image.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(transform.ConnectableAPI(), "result")
        image.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*color, 1.0))
        image.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        shader.GetInput("diffuseColor").ConnectToSource(image.ConnectableAPI(), "rgb")
    return material


def _box(stage, part, materials, physics):
    path = "/ShipGrid/Parts/" + part["name"]
    frame = UsdGeom.Xform.Define(stage, path)
    frame.AddTranslateOp().Set(Gf.Vec3d(*part["center_m"]))
    x, y, z, w = part["quaternion_xyzw"]
    frame.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
    hx, hy, hz = [value / 2 for value in part["size_m"]]
    points = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
              (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
    # Outward-facing quads, with independent texture coordinates on each face.
    faces = [0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7]
    mesh = UsdGeom.Mesh.Define(stage, path + "/Visual")
    mesh.CreatePointsAttr(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr([4] * 6)
    mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr([Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, hy, hz)])
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*MATERIALS[part["kind"]][0])])
    uv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                UsdGeom.Tokens.faceVarying)
    uv.Set(Vt.Vec2fArray([(0, 0), (1, 0), (1, 1), (0, 1)] * 6))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials[part["kind"]])
    if part["collision"]:
        # Analytic boxes are exact, cheap colliders. No hull spans an entire cell.
        cube = UsdGeom.Cube.Define(stage, path + "/Collision")
        cube.CreateSizeAttr(1.0)
        cube.AddScaleOp().Set(Gf.Vec3f(*part["size_m"]))
        cube.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
            physics, UsdShade.Tokens.weakerThanDescendants, "physics")


def export_environment(sim, grid, destination=None):
    """Snapshot static geometry and export ship_grid.usda plus its dependencies."""
    destination = Path(destination or ROOT / "assets/export/ship-grid").resolve()
    if not destination.is_relative_to(ROOT):
        raise ValueError("Keep the source export inside this twin; copy the resulting bundle afterwards")
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "textures").mkdir(exist_ok=True)
    poses = sim.model.shape_transform.numpy()
    scales = sim.model.shape_scale.numpy()
    bodies = sim.model.shape_body.numpy()
    parts = []
    entity = sim.scene.by_name[grid.name]
    for kind, indices in entity.parts.items():
        for index in indices:
            if bodies[index] != -1:
                raise ValueError("Expected a fixed world shape in the ship grid")
            parts.append({"name": str(sim.model.shape_label[index]), "kind": kind,
                          "center_m": poses[index, :3].astype(float).tolist(),
                          "quaternion_xyzw": poses[index, 3:].astype(float).tolist(),
                          "size_m": (scales[index] * 2).astype(float).tolist(),
                          "collision": kind != "cut"})
    # The original yard surround is visual; keep that property in the transfer.
    yard_index = next(i for i, label in enumerate(sim.model.shape_label) if str(label) == "yard_floor")
    parts.append({"name": "yard_floor", "kind": "surround",
                  "center_m": poses[yard_index, :3].astype(float).tolist(),
                  "quaternion_xyzw": poses[yard_index, 3:].astype(float).tolist(),
                  "size_m": (scales[yard_index] * 2).astype(float).tolist(), "collision": False})
    for kind in ("base", "upright"):
        shutil.copy2(ROOT / f"assets/assembled/steel_mill_scale_{kind}.png",
                     destination / f"textures/steel_{kind}.png")

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/ShipGrid").GetPrim()
    stage.SetDefaultPrim(root)
    root.SetDocumentation("Static ship-plate grid, exported without robots. Origin: deck top at grid centre.")
    root.SetCustomDataByKey("environmentType", "ship-grid")
    UsdGeom.Scope.Define(stage, "/ShipGrid/Looks")
    UsdGeom.Xform.Define(stage, "/ShipGrid/Parts")
    materials = {kind: _surface(stage, kind) for kind in MATERIALS}
    physics = UsdShade.Material.Define(stage, "/ShipGrid/Looks/SteelContact")
    contact = UsdPhysics.MaterialAPI.Apply(physics.GetPrim())
    friction = grid.friction if grid.friction is not None else 0.65
    contact.CreateStaticFrictionAttr(friction)
    contact.CreateDynamicFrictionAttr(friction)
    contact.CreateRestitutionAttr(0.0)
    for part in parts:
        _box(stage, part, materials, physics)
    asset = destination / "ship_grid.usda"
    if not stage.GetRootLayer().Export(str(asset)):
        raise OSError(f"Could not export {asset}")
    manifest = {
        "asset": asset.name, "default_prim": "/ShipGrid", "units": "metres", "up_axis": "Z",
        "origin": "grid centre, deck top", "robots_included": False,
        "source": "robosimstudio/examples/live/twins/unitree_b2_z1",
        "source_sha256": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                          for name in ("scene.py", "ship_grid.py")},
        "grid": {"rows": grid.rows, "columns": grid.columns, "clear_size_m": grid.clear_size,
                 "web_height_m": grid.web_height, "thickness_m": grid.thickness,
                 "flange_width_m": grid.flange_width, "deck_size_m": [value * 2 for value in grid.deck_half]},
        "friction": friction, "parts": parts,
        "limitations": ["Static fixture, no thermal or welding process simulation",
                        "Visual surround only; no infinite ground plane outside the steel deck",
                        "Textures and UV mapping use USD Preview Surface; renderer appearance can differ"],
    }
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"asset": str(asset), "visual_parts": len(parts),
                      "static_box_colliders": sum(p["collision"] for p in parts),
                      "robots_included": False}, indent=2), flush=True)
    return str(asset)
