"""Portable OpenUSD export from the same hall geometry and compiled source MJCF.

Run with the RoboSimStudio interpreter; this does not execute scene.py. Robot
body/joint frames, inertias and mesh coordinates come from MuJoCo compilation.
USD is an editable scene, not a baked image. Isaac opens it with timeline paused.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import zipfile
from pathlib import Path

import mujoco as mj
import numpy as np
from camera_views import VIEWS
from hall_geometry import ROOT, Hall
from prepare_assets import ARM_JOINTS
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, UsdUtils, Vt
from robot_initial import apply_initial_pose
from scipy.spatial.transform import Rotation as R
from steel_surface import add_uv, make_steel_maps


def ident(value):
    return __import__("re").sub(r"[^a-zA-Z0-9_]", "_", value)


def quat(q, double=False):
    cls = Gf.Quatd if double else Gf.Quatf
    return cls(float(q[3]), *map(float, q[:3]))


def xyzw(q):
    return [*q[1:], q[0]]


def pose(prim, position, rotation=(0, 0, 0, 1), scale=None):
    x = UsdGeom.Xformable(prim)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, position)))
    x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat(rotation, double=True))
    if scale is not None:
        x.AddScaleOp().Set(Gf.Vec3f(*map(float, scale)))


def custom(prim, name, value, kind=Sdf.ValueTypeNames.String):
    prim.CreateAttribute("welding:" + name, kind, custom=True).Set(value)


def mesh(stage, path, vertices, faces):
    x = UsdGeom.Mesh.Define(stage, path)
    x.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(vertices, np.float32)))
    x.CreateFaceVertexCountsAttr([len(face) for face in faces])
    x.CreateFaceVertexIndicesAttr(np.asarray(faces, np.int32).flatten().tolist())
    x.CreateSubdivisionSchemeAttr("none")
    x.CreateDoubleSidedAttr(True)
    x.CreateExtentAttr(UsdGeom.Mesh.ComputeExtent(x.GetPointsAttr().Get()))
    return x


class Exporter:
    def __init__(self, output):
        self.hall = Hall()
        self.out = output.resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        self.path = self.out / "industrial_welding.usdc"
        if self.path.exists():
            self.path.unlink()
        self.stage = Usd.Stage.CreateNew(str(self.path))
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(self.stage, 1.0)
        world = UsdGeom.Xform.Define(self.stage, "/World")
        self.stage.SetDefaultPrim(world.GetPrim())
        self.stage.SetTimeCodesPerSecond(60)
        self.stage.SetFramesPerSecond(60)
        self.stage.SetStartTimeCode(0)
        self.stage.SetEndTimeCode(0)
        self.stage.GetRootLayer().customLayerData = {
            "description": f"{self.hall.config['hall']['size_m']} m B2 + E05L-Pro industrial welding hall",
            "source": "RoboSimStudio industrial_welding config + source MJCF",
            "scope": "scene migration; controllers and dynamic PhysX validation pending",
            "sourceModelLicenseText": (ROOT / "assets/assembled/source/LICENSE").read_text(),
            "config_sha256": hashlib.sha256((ROOT / "config.json").read_bytes()).hexdigest(),
        }
        self.steel_maps = make_steel_maps(self.out / "textures")
        self.materials = {}
        self.contact_materials = {}
        self.count = {
            "fixtures": 0,
            "fixture_parts": 0,
            "surface_preparation_patches": 0,
            "fixture_colliders": 0,
            "robot_bodies": 0,
            "robot_geoms": 0,
            "robot_colliders": 0,
            "revolute_joints": 0,
            "fixed_joints": 0,
            "seams": 0,
        }
        self.joint_records = []
        self.body_records = []

    def material(self, color, role):
        key = (tuple(np.round(color[:3], 5)), role)
        if key not in self.materials:
            m = UsdShade.Material.Define(self.stage, f"/World/Looks/material_{len(self.materials):03}")
            s = UsdShade.Shader.Define(self.stage, str(m.GetPath()) + "/PreviewSurface")
            s.CreateIdAttr("UsdPreviewSurface")
            c = np.asarray(color[:3])
            c = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
            s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*map(float, c)))
            s.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(
                self.hall.config.get("appearance", {}).get("steel_metallic", 0.78)
                if role in ("steel", "ramp", "machined")
                else 0.05
            )
            s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6 if role == "steel" else 0.8)
            if role in ("steel", "ramp", "machined"):
                s.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).Set(Gf.Vec3f(0, 0, 1))
                reader = UsdShade.Shader.Define(self.stage, str(m.GetPath()) + "/UV")
                reader.CreateIdAttr("UsdPrimvarReader_float2")
                reader.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
                reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
                for name, target in (("albedo", "diffuseColor"), ("roughness", "roughness"), ("normal", "normal")):
                    tex = UsdShade.Shader.Define(self.stage, str(m.GetPath()) + "/" + name)
                    tex.CreateIdAttr("UsdUVTexture")
                    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
                        Sdf.AssetPath("textures/" + self.steel_maps[name].name)
                    )
                    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(
                        "sRGB" if name == "albedo" else "raw"
                    )
                    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
                    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
                    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
                    output = "r" if name == "roughness" else "rgb"
                    kind = Sdf.ValueTypeNames.Float if output == "r" else Sdf.ValueTypeNames.Float3
                    tex.CreateOutput(output, kind)
                    if name == "albedo":
                        factor = c / 0.233
                        tex.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*map(float, factor), 1))
                    elif name == "normal":
                        tex.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2, 2, 2, 1))
                        tex.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1, -1, -1, 0))
                    elif role == "machined":
                        tex.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(0.65, 0.65, 0.65, 1))
                    s.GetInput(target).ConnectToSource(tex.ConnectableAPI(), output)
            m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
            self.materials[key] = m
        return self.materials[key]

    def finish(self, prim, color, role):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.material(color, role))
        UsdGeom.Gprim(prim).CreateDisplayColorAttr([Gf.Vec3f(*map(float, color[:3]))])

    def collider(self, prim, convex=False, friction=0.75):
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        key = round(float(friction), 5)
        if key not in self.contact_materials:
            material = UsdShade.Material.Define(self.stage, f"/World/Looks/contact_{len(self.contact_materials)}")
            physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
            physics.CreateStaticFrictionAttr(key)
            physics.CreateDynamicFrictionAttr(key)
            physics.CreateRestitutionAttr(0.0)
            self.contact_materials[key] = material
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.contact_materials[key], materialPurpose="physics")
        if convex:
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")

    def fixtures(self):
        for name, o in self.hall.objects.items():
            path = "/World/Fixtures/" + ident(name)
            obj = UsdGeom.Xform.Define(self.stage, path)
            pose(obj.GetPrim(), o["pose"][:3], o["pose"][3:])
            custom(obj.GetPrim(), "workpieceId", name)
            custom(obj.GetPrim(), "kind", o["kind"])
            for p in o["parts"]:
                partpath = path + "/" + ident(p["name"])
                if p["type"] == "box":
                    if p["role"] in ("steel", "ramp", "machined"):
                        vertices = [[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)]
                        faces = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
                        shape = mesh(self.stage, partpath, vertices, faces)
                        add_uv(shape, p["size"], self.hall.config["appearance"]["steel_uv_repeats_per_m"])
                    else:
                        shape = UsdGeom.Cube.Define(self.stage, partpath)
                        shape.CreateSizeAttr(1)
                    pose(shape.GetPrim(), p["center"], p["quat"], p["size"])
                else:
                    shape = mesh(self.stage, partpath, p["vertices"], p["faces"])
                    if p["role"] in ("steel", "ramp", "machined"):
                        add_uv(shape, repeats_per_m=self.hall.config["appearance"]["steel_uv_repeats_per_m"])
                self.finish(shape.GetPrim(), p["color"], p["role"])
                custom(shape.GetPrim(), "role", p["role"])
                if p["collision"]:
                    self.collider(shape.GetPrim(), p["type"] != "box")
                    if p["type"] == "box" and shape.GetPrim().IsA(UsdGeom.Mesh):
                        UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr("boundingCube")
                    self.count["fixture_colliders"] += 1
                self.count["fixture_parts"] += 1
            self.count["fixtures"] += 1
        physics = UsdPhysics.Scene.Define(self.stage, "/World/PhysicsScene")
        physics.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        physics.CreateGravityMagnitudeAttr(9.81)

    def robot(self):
        # Read the already prepared, self-contained source asset. Do not rewrite
        # the live twin or import/execute its scene.py.
        from robot_assets import prepare_robot

        modelpath, _, _ = prepare_robot(self.hall.config["robot"])
        m = mj.MjModel.from_xml_path(str(modelpath))
        d = mj.MjData(m)
        self.initial_pose = apply_initial_pose(m, d, self.hall.config["robot"])
        self.robot_model, self.robot_data = m, d
        robot = UsdGeom.Xform.Define(self.stage, "/World/Robot")
        sx, sy, yaw = self.hall.config["robot"]["start_xy_yaw"]
        pose(robot.GetPrim(), [sx, sy, 0], R.from_euler("z", yaw).as_quat())
        # Floating PhysX articulation root is the actual root rigid body.

        custom(robot.GetPrim(), "model", "Unitree B2 no lidar/guard + E05L-Pro + source welding torch")
        custom(robot.GetPrim(), "dynamicsStatus", "USD physics authored; PhysX controller/gait/welding not validated")
        custom(robot.GetPrim(), "sourceMassKg", float(m.body_mass.sum()), Sdf.ValueTypeNames.Double)
        paths = {}
        rotations = {}
        for b in range(1, m.nbody):
            name = mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, b)
            path = "/World/Robot/" + ident(name)
            paths[b] = path
            rotations[b] = R.from_matrix(d.xmat[b].reshape(3, 3))
            body = UsdGeom.Xform.Define(self.stage, path).GetPrim()
            pose(body, d.xpos[b], xyzw(d.xquat[b]))
            UsdPhysics.RigidBodyAPI.Apply(body).CreateRigidBodyEnabledAttr(True)
            if int(m.body_parentid[b]) == 0:
                UsdPhysics.ArticulationRootAPI.Apply(body)
            mass = UsdPhysics.MassAPI.Apply(body)
            mass.CreateMassAttr(float(m.body_mass[b]))
            mass.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, m.body_ipos[b])))
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, m.body_inertia[b])))
            mass.CreatePrincipalAxesAttr(quat(xyzw(m.body_iquat[b])))
            self.body_records.append(
                {
                    "name": name,
                    "path": path,
                    "position_m": d.xpos[b].tolist(),
                    "quat_xyzw": xyzw(d.xquat[b]),
                    "mass_kg": float(m.body_mass[b]),
                }
            )
            self.count["robot_bodies"] += 1
        for g in range(m.ngeom):
            b = int(m.geom_bodyid[g])
            if b == 0:
                continue
            name = mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, g) or f"geom_{g}"
            path = paths[b] + "/" + ident(name)
            t = int(m.geom_type[g])
            size = m.geom_size[g]
            if t == mj.mjtGeom.mjGEOM_MESH:
                i = int(m.geom_dataid[g])
                a, n = int(m.mesh_vertadr[i]), int(m.mesh_vertnum[i])
                f, nf = int(m.mesh_faceadr[i]), int(m.mesh_facenum[i])
                shape = mesh(self.stage, path, m.mesh_vert[a : a + n], m.mesh_face[f : f + nf])
            elif t == mj.mjtGeom.mjGEOM_BOX:
                shape = UsdGeom.Cube.Define(self.stage, path)
                shape.CreateSizeAttr(1)
            elif t == mj.mjtGeom.mjGEOM_SPHERE:
                shape = UsdGeom.Sphere.Define(self.stage, path)
                shape.CreateRadiusAttr(float(size[0]))
            elif t in (mj.mjtGeom.mjGEOM_CYLINDER, mj.mjtGeom.mjGEOM_CAPSULE):
                cls = UsdGeom.Cylinder if t == mj.mjtGeom.mjGEOM_CYLINDER else UsdGeom.Capsule
                shape = cls.Define(self.stage, path)
                shape.CreateRadiusAttr(float(size[0]))
                shape.CreateHeightAttr(float(2 * size[1]))
                shape.CreateAxisAttr("Z")
            else:
                raise ValueError(f"Unsupported source geom {name}: {t}")
            prim = shape.GetPrim()
            pose(prim, m.geom_pos[g], xyzw(m.geom_quat[g]), 2 * size if t == mj.mjtGeom.mjGEOM_BOX else None)
            color = m.mat_rgba[m.geom_matid[g]] if m.geom_matid[g] >= 0 else m.geom_rgba[g]
            self.finish(prim, color, "robot")
            collides = bool(m.geom_contype[g] or m.geom_conaffinity[g])
            collision_only = int(m.geom_group[g]) == 3
            custom(prim, "sourceGeomId", g, Sdf.ValueTypeNames.Int)
            custom(prim, "sourceGroup", int(m.geom_group[g]), Sdf.ValueTypeNames.Int)
            if collides:
                self.collider(prim, t == mj.mjtGeom.mjGEOM_MESH, m.geom_friction[g, 0])
                self.count["robot_colliders"] += 1
            if collision_only:
                UsdGeom.Imageable(prim).CreatePurposeAttr("guide")
                UsdGeom.Imageable(prim).CreateVisibilityAttr("invisible")
            self.count["robot_geoms"] += 1
        for b in range(1, m.nbody):
            parent = int(m.body_parentid[b])
            if parent == 0:
                continue  # Source free base remains free; no world fixed joint.
            n = int(m.body_jntnum[b])
            if n > 1:
                raise ValueError("Multiple joints per body need an explicit converter")
            j = int(m.body_jntadr[b]) if n else None
            name = mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, j) if n else "fixed_" + paths[b].rsplit("/", 1)[1]
            path = "/World/Robot/Joints/" + ident(name)
            if n:
                assert m.jnt_type[j] == mj.mjtJoint.mjJNT_HINGE
                joint = UsdPhysics.RevoluteJoint.Define(self.stage, path)
                joint.CreateAxisAttr("Z")
                joint.CreateLowerLimitAttr(float(np.rad2deg(m.jnt_range[j, 0])))
                joint.CreateUpperLimitAttr(float(np.rad2deg(m.jnt_range[j, 1])))
                origin = d.xanchor[j]
                # Align a local joint Z axis to the compiled source hinge axis.
                axis_local = m.jnt_axis[j]
                z_to_axis = Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*map(float, axis_local))).GetQuat()
                align = R.from_quat([*z_to_axis.GetImaginary(), z_to_axis.GetReal()])
                frame1 = rotations[b] * align
                angle = float(d.qpos[m.jnt_qposadr[j]])
                frame0 = frame1 * R.from_euler("z", -angle)
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
                drive.CreateTypeAttr("force")
                drive.CreateTargetPositionAttr(float(np.rad2deg(angle)))
                # Isaac/PhysX reads this schema to preserve multi-turn initial
                # coordinates. It is intentionally authored by token so the
                # standalone usd-core exporter need not import Kit plugins.
                joint.GetPrim().AddAppliedSchema("PhysicsJointStateAPI:angular")
                joint.GetPrim().CreateAttribute(
                    "state:angular:physics:position", Sdf.ValueTypeNames.Float, custom=False
                ).Set(float(np.rad2deg(angle)))
                joint.GetPrim().CreateAttribute(
                    "state:angular:physics:velocity", Sdf.ValueTypeNames.Float, custom=False
                ).Set(0.0)
                # USD angular drive coefficients are per degree.
                arm = name in ARM_JOINTS
                drive.CreateStiffnessAttr((50000 if arm else 1800) * math.pi / 180)
                drive.CreateDampingAttr((400 if arm else 100) * math.pi / 180)
                if not arm:
                    drive.CreateMaxForceAttr(320 if "calf" in name else 200)
                custom(
                    joint.GetPrim(), "sourceRangeRad", Gf.Vec2d(*map(float, m.jnt_range[j])), Sdf.ValueTypeNames.Double2
                )
                self.joint_records.append(
                    {
                        "name": name,
                        "path": path,
                        "q_rad": angle,
                        "limits_rad": m.jnt_range[j].tolist(),
                        "body0": paths[parent],
                        "body1": paths[b],
                    }
                )
                self.count["revolute_joints"] += 1
            else:
                joint = UsdPhysics.FixedJoint.Define(self.stage, path)
                origin = d.xpos[b]
                frame0 = frame1 = rotations[b]
                self.count["fixed_joints"] += 1
            joint.CreateBody0Rel().SetTargets([paths[parent]])
            joint.CreateBody1Rel().SetTargets([paths[b]])
            joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float, rotations[parent].inv().apply(origin - d.xpos[parent]))))
            joint.CreateLocalPos1Attr(Gf.Vec3f(*map(float, rotations[b].inv().apply(origin - d.xpos[b]))))
            joint.CreateLocalRot0Attr(quat((rotations[parent].inv() * frame0).as_quat()))
            joint.CreateLocalRot1Attr(quat((rotations[b].inv() * frame1).as_quat()))
            joint.CreateCollisionEnabledAttr(False)
        for s in range(m.nsite):
            name = mj.mj_id2name(m, mj.mjtObj.mjOBJ_SITE, s)
            if name == "welding_tcp":
                p = UsdGeom.Xform.Define(self.stage, paths[int(m.site_bodyid[s])] + "/welding_tcp")
                pose(p.GetPrim(), m.site_pos[s], xyzw(m.site_quat[s]))
                custom(p.GetPrim(), "axisConvention", "+Z nozzle outlet, +X travel tangent")
        # Preserve source non-adjacent collision exclusions in standard USD.
        for i in range(m.nexclude):
            signature = int(m.exclude_signature[i])
            a, b = signature >> 16, signature & 65535
            if a in paths and b in paths:
                UsdPhysics.FilteredPairsAPI.Apply(
                    self.stage.GetPrimAtPath(paths[a])
                ).CreateFilteredPairsRel().AddTarget(paths[b])

    def seams(self):
        for name in self.hall.objects:
            scope = UsdGeom.Xform.Define(self.stage, f"/World/Fixtures/{ident(name)}/WeldSeams")
            scope.CreateVisibilityAttr("invisible")
        for s in self.hall.seams:
            path = f"/World/Fixtures/{ident(s['workpiece'])}/WeldSeams/{ident(s['id'])}"
            curve = UsdGeom.BasisCurves.Define(self.stage, path)
            curve.CreateTypeAttr("linear")
            curve.CreateWrapAttr("nonperiodic")
            curve.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.array(s["local_points_m"], np.float32)))
            curve.CreateCurveVertexCountsAttr([len(s["local_points_m"])])
            curve.CreateWidthsAttr([0.004])
            curve.SetWidthsInterpolation("constant")
            curve.CreateDisplayColorAttr([Gf.Vec3f(1, 0.4, 0.04)])
            custom(curve.GetPrim(), "seamId", s["id"])
            custom(curve.GetPrim(), "access", s["access"])
            custom(curve.GetPrim(), "dataFile", "metadata/seams.json")
            self.count["seams"] += 1

    def surface_preparation(self):
        # Visual finish only: thin planar ground-metal strips on the real
        # adjacent surfaces, never collision plates or completed weld beads.
        width = self.hall.config.get("appearance", {}).get("preparation_band_width_m", 0.014)
        for seam in self.hall.seams:
            if seam["workpiece"] != "A08":
                continue
            points = np.array(seam["local_points_m"])
            tangent = points[-1] - points[0]
            length = np.linalg.norm(tangent)
            tangent /= length
            normals = np.array(seam["local_surface_normals"])[0]
            for side, normal in enumerate(normals):
                across = normals[1 - side] - np.dot(normals[1 - side], normal) * normal
                across -= np.dot(across, tangent) * tangent
                across /= np.linalg.norm(across)
                a = points[0] + tangent * 0.005 + normal * 0.00015
                b = points[-1] - tangent * 0.005 + normal * 0.00015
                vertices = [a, b, b + width * across, a + width * across]
                path = f"/World/Fixtures/A08/SurfacePreparation/{ident(seam['id'])}_{side}"
                shape = mesh(self.stage, path, vertices, [[0, 1, 2, 3]])
                add_uv(shape)
                self.finish(shape.GetPrim(), (0.70, 0.72, 0.73), "machined")
                custom(shape.GetPrim(), "role", "ground metal surface finish; no collision or weld completion")
                self.count["surface_preparation_patches"] += 1

    def lighting_cameras(self):
        settings = self.hall.config["lighting"]
        dome = UsdLux.DomeLight.Define(self.stage, "/World/Lights/Ambient")
        dome.CreateIntensityAttr(settings["dome_intensity"])
        dome.CreateColorAttr(Gf.Vec3f(*settings["dome_color"]))
        UsdLux.ShadowAPI.Apply(dome.GetPrim()).CreateShadowEnableAttr(False)
        sun = UsdLux.DistantLight.Define(self.stage, "/World/Lights/Key")
        sun.CreateIntensityAttr(settings["key_intensity"])
        sun.CreateAngleAttr(settings["key_angle_deg"])
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-25, -35, -25))
        for i, xy in enumerate(settings["fill_positions_xy_m"]):
            light = UsdLux.RectLight.Define(self.stage, f"/World/Lights/Softbox_{i}")
            light.CreateWidthAttr(settings["fill_size_m"][0])
            light.CreateHeightAttr(settings["fill_size_m"][1])
            light.CreateIntensityAttr(settings["fill_intensity"])
            light.CreateColorAttr(Gf.Vec3f(1.0, 0.985, 0.96))
            # Tilt broad sources toward the task fronts, so vertical plate
            # faces and internal corners receive fill as well as the floor.
            eye = Gf.Vec3d(float(xy[0]), float(xy[1]), settings["fill_height_m"])
            target = Gf.Vec3d(float(xy[0]), float(xy[1]) + settings.get("fill_target_offset_y_m", 2.0), 0.7)
            matrix = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse()
            UsdGeom.Xformable(light).AddTransformOp().Set(matrix)
        for name, (eye, target, fov, size) in VIEWS.items():
            camera = UsdGeom.Camera.Define(self.stage, "/World/Cameras/" + name)
            matrix = (
                Gf.Matrix4d()
                .SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 1, 0) if name == "top" else Gf.Vec3d(0, 0, 1))
                .GetInverse()
            )
            UsdGeom.Xformable(camera).AddTransformOp().Set(matrix)
            camera.CreateVerticalApertureAttr(24)
            camera.CreateHorizontalApertureAttr(24 * size[0] / size[1])
            camera.CreateFocalLengthAttr(12 / math.tan(math.radians(fov) / 2))
            camera.CreateClippingRangeAttr(Gf.Vec2f(0.03, 100))
        self.stage.GetRootLayer().customLayerData = {
            **self.stage.GetRootLayer().customLayerData,
            "defaultCamera": "/World/Cameras/overview",
        }

    def save(self):
        self.fixtures()
        self.robot()
        self.seams()
        self.surface_preparation()
        self.lighting_cameras()
        self.stage.GetRootLayer().Save()
        metadata = self.out / "metadata"
        metadata.mkdir(exist_ok=True)
        for name in [
            "seams.json",
            "stances.json",
            "navigation.json",
            "map.yaml",
            "map.pgm",
            "map_inflated.pgm",
            "map_preview.png",
            "scene_geometry.json",
            "asset_manifest.json",
            "validation.json",
            "witness_replay.json",
            "geometry_audit.json",
            "layout_validation.json",
            "summary.json",
            "initial_pose_validation.json",
            "butt_joint_validation.json",
        ]:
            src = ROOT / "exports" / name
            if src.exists():
                shutil.copy2(src, metadata / name)
        material_manifest = {
            "source": "procedural steel_surface.py, seed 418; no external images or reference photo edits",
            "status": "visual approximation, not measured optical properties",
            "uv_repeats_per_m": self.hall.config["appearance"]["steel_uv_repeats_per_m"],
            "maps": [
                {
                    "role": name,
                    "path": "textures/" + path.name,
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "color_space": "sRGB" if name == "albedo" else "raw",
                }
                for name, path in self.steel_maps.items()
            ],
        }
        (metadata / "material_manifest.json").write_text(json.dumps(material_manifest, indent=2))
        shutil.copy2(ROOT / "config.json", metadata / "source_config.json")
        shutil.copy2(ROOT / "assets/assembled/source/LICENSE", metadata / "ROBOT_SOURCE_LICENSE.txt")
        manifest = {
            "stage": self.path.name,
            "stage_sha256": hashlib.sha256(self.path.read_bytes()).hexdigest(),
            "units": "m",
            "up_axis": "Z",
            "counts": self.count,
            "robot_mass_kg": sum(b["mass_kg"] for b in self.body_records),
            "bodies": self.body_records,
            "initial_pose": self.initial_pose,
            "joints": self.joint_records,
            "source_config_sha256": hashlib.sha256((ROOT / "config.json").read_bytes()).hexdigest(),
            "omitted": ["Viser panel", "TCP animation", "welding controller", "navigation and gait controllers"],
            "runtime": "Isaac Sim opens paused; dynamic behavior requires separate PhysX validation",
        }
        (self.out / "manifest.json").write_text(json.dumps(manifest, indent=2))
        # USDZ is a geometry/material/physics package; sidecar task/map JSON stays
        # in the ZIP bundle next to the ordinary, editable USD stage.
        usdz = self.out / "industrial_welding.usdz"
        if usdz.exists():
            usdz.unlink()
        assert UsdUtils.CreateNewUsdzPackage(Sdf.AssetPath(str(self.path)), str(usdz))
        self.validate()
        print(
            json.dumps({"output": str(self.out), "counts": self.count, "mass_kg": manifest["robot_mass_kg"]}, indent=2)
        )

    def validate(self):
        stage = Usd.Stage.Open(str(self.path))
        layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(self.path))
        assert not unresolved, unresolved
        _, packaged_assets, packaged_unresolved = UsdUtils.ComputeAllDependencies(
            str(self.out / "industrial_welding.usdz")
        )
        assert not packaged_unresolved, packaged_unresolved
        assert UsdGeom.GetStageMetersPerUnit(stage) == 1
        assert UsdGeom.GetStageUpAxis(stage) == "Z"
        assert len([p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]) == 18
        assert self.count["fixture_parts"] == sum(len(o["parts"]) for o in self.hall.objects.values())
        compliance = UsdUtils.ComplianceChecker(arkit=False, skipARKitRootLayerCheck=True)
        compliance.CheckCompliance(str(self.path))
        assert not compliance.GetErrors() and not compliance.GetFailedChecks()
        # Pose and anchor checks use USD transforms independently of authoring.
        cache = UsdGeom.XformCache()
        robot_matrix = cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/Robot"))
        max_body = max_anchor = max_joint_angle = 0.0
        for b in self.body_records:
            mat = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(b["path"]))
            expected = robot_matrix.Transform(Gf.Vec3d(*b["position_m"]))
            max_body = max(max_body, float((mat.ExtractTranslation() - expected).GetLength()))
        for p in stage.Traverse():
            if not p.IsA(UsdPhysics.Joint):
                continue
            j = UsdPhysics.Joint(p)
            b0 = stage.GetPrimAtPath(j.GetBody0Rel().GetTargets()[0])
            b1 = stage.GetPrimAtPath(j.GetBody1Rel().GetTargets()[0])
            a = cache.GetLocalToWorldTransform(b0).Transform(Gf.Vec3d(j.GetLocalPos0Attr().Get()))
            b = cache.GetLocalToWorldTransform(b1).Transform(Gf.Vec3d(j.GetLocalPos1Attr().Get()))
            max_anchor = max(max_anchor, float((a - b).GetLength()))
            if p.IsA(UsdPhysics.RevoluteJoint):
                frames = []
                for body, attr in [(b0, j.GetLocalRot0Attr()), (b1, j.GetLocalRot1Attr())]:
                    q = attr.Get()
                    local = R.from_quat([*q.GetImaginary(), q.GetReal()])
                    qworld = cache.GetLocalToWorldTransform(body).ExtractRotationQuat()
                    frames.append(R.from_quat([*qworld.GetImaginary(), qworld.GetReal()]) * local)
                relative = (frames[0].inv() * frames[1]).as_rotvec()
                target = next(x["q_rad"] for x in self.joint_records if x["path"] == str(p.GetPath()))
                err = abs(math.atan2(math.sin(relative[2] - target), math.cos(relative[2] - target)))
                max_joint_angle = max(max_joint_angle, err)
        assert max_body < 1e-6 and max_anchor < 1e-5 and max_joint_angle < 1e-5
        max_fixture_vertex = max_robot_vertex = 0.0
        world_parts = {p["name"]: p for p in self.hall.world_parts()}
        signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        for name, obj in self.hall.objects.items():
            for part in obj["parts"]:
                prim = stage.GetPrimAtPath(f"/World/Fixtures/{ident(name)}/{ident(part['name'])}")
                matrix = cache.GetLocalToWorldTransform(prim)
                source = world_parts[part["name"]]
                if part["type"] == "box":
                    expected = (
                        R.from_quat(source["quat"]).apply(signs * np.array(source["size"]) / 2) + source["center"]
                    )
                    local = signs / 2
                else:
                    expected = np.array(source["vertices"])
                    local = np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get())
                actual = np.array([matrix.Transform(Gf.Vec3d(*map(float, v))) for v in local])
                max_fixture_vertex = max(max_fixture_vertex, float(np.linalg.norm(actual - expected, axis=1).max()))
        m, d = self.robot_model, self.robot_data
        for prim in stage.Traverse():
            source_id = prim.GetAttribute("welding:sourceGeomId")
            if not source_id or not prim.IsA(UsdGeom.Mesh):
                continue
            g = source_id.Get()
            i = int(m.geom_dataid[g])
            a, n = int(m.mesh_vertadr[i]), int(m.mesh_vertnum[i])
            expected = R.from_matrix(d.geom_xmat[g].reshape(3, 3)).apply(m.mesh_vert[a : a + n]) + d.geom_xpos[g]
            expected = np.array([robot_matrix.Transform(Gf.Vec3d(*map(float, v))) for v in expected])
            matrix = cache.GetLocalToWorldTransform(prim)
            actual = np.array(
                [matrix.Transform(Gf.Vec3d(*map(float, v))) for v in UsdGeom.Mesh(prim).GetPointsAttr().Get()]
            )
            max_robot_vertex = max(max_robot_vertex, float(np.linalg.norm(actual - expected, axis=1).max()))
        site = mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, "welding_tcp")
        tcp = cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/Robot/e05l_welding_torch/welding_tcp"))
        expected_tcp = robot_matrix.Transform(Gf.Vec3d(*map(float, d.site_xpos[site])))
        tcp_error = float((tcp.ExtractTranslation() - expected_tcp).GetLength())
        assert max_fixture_vertex < 2e-6 and max_robot_vertex < 2e-6 and tcp_error < 2e-6
        bbox = (
            UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
            .ComputeWorldBound(stage.GetPrimAtPath("/World/Fixtures/hall"))
            .ComputeAlignedRange()
        )
        report = {
            "passed": True,
            "unresolved_assets": list(unresolved),
            "layers": len(layers),
            "external_assets": len(assets),
            "external_asset_scope": "three relative PBR maps in textures/; no network or source robot mesh dependencies",
            "packaged_assets": len(packaged_assets),
            "packaged_unresolved_assets": list(packaged_unresolved),
            "compliance_errors": compliance.GetErrors(),
            "compliance_failed_checks": compliance.GetFailedChecks(),
            "compliance_warnings": compliance.GetWarnings(),
            "max_body_position_error_m": max_body,
            "max_fixture_vertex_error_m": max_fixture_vertex,
            "max_robot_mesh_vertex_error_m": max_robot_vertex,
            "tcp_position_error_m": tcp_error,
            "max_joint_anchor_error_m": max_anchor,
            "max_joint_coordinate_error_rad": max_joint_angle,
            "hall_bounds_m": [list(bbox.GetMin()), list(bbox.GetMax())],
            "counts": self.count,
            "usdz_openable": bool(Usd.Stage.Open(str(self.out / "industrial_welding.usdz"))),
            "physics_scope": "USD schema/poses/anchors checked; no PhysX dynamics or controller equivalence claim",
        }
        (self.out / "usd_validation.json").write_text(json.dumps(report, indent=2))


def bundle(output):
    dest = output.parent / "industrial_welding_usd.zip"
    with zipfile.ZipFile(dest, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for p in sorted(output.rglob("*")):
            if p.is_file():
                z.write(p, Path("industrial_welding_usd") / p.relative_to(output))
    print("Bundle:", dest)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "exports/usd")
    args = parser.parse_args()
    Exporter(args.output).save()
    bundle(args.output)
