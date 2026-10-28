"""Compose the official Unitree B2 MJCF and Z1 URDF without ROS dependencies."""

from __future__ import annotations

import copy
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
B2 = ROOT / "assets/unitree_mujoco/unitree_robots/b2"
Z1 = ROOT / "assets/unitree_ros/robots/z1_description"
OUT = ROOT / "assets/assembled"
LEG_HOME = {f"{leg}_{joint}_joint": angle for leg in ("FL", "FR", "RL", "RR")
            for joint, angle in zip(("hip", "thigh", "calf"), (0.0, 0.75, -1.50), strict=True)}
ARM_HOME = (0.0, 1.30, -1.60, 0.30, 0.0, 0.0)
BASE_HEIGHT = 2.0 * 0.35 * math.cos(0.75) + 0.032


def steel_texture(kind="base"):
    """A deterministic mill-scale material, not a modification of the reference photo."""
    output = OUT / f"steel_mill_scale_{kind}.png"
    if not output.exists():
        import imageio.v3 as iio

        OUT.mkdir(parents=True, exist_ok=True)
        rng = np.random.default_rng(21)
        fine = rng.normal(size=(512, 512))
        coarse = np.repeat(np.repeat(rng.normal(size=(128, 128)), 4, axis=0), 4, axis=1)
        value = np.clip(0.80 + 0.06 * fine + 0.05 * coarse, 0.45, 1.0)
        tint = (0.70, 0.78, 0.77) if kind == "base" else (0.20, 0.28, 0.26)
        rgb = np.stack(tuple(value * channel for channel in tint), axis=-1)
        iio.imwrite(output, np.uint8(np.clip(rgb, 0.0, 1.0) * 255))
    return output


def numbers(values):
    return " ".join(f"{float(value):.10g}" for value in values)


def origin(node):
    if node is None:
        return {}
    result = {"pos": node.get("xyz", "0 0 0")}
    rpy = np.fromstring(node.get("rpy", "0 0 0"), sep=" ")
    xyzw = Rotation.from_euler("xyz", rpy).as_quat()
    result["quat"] = numbers((xyzw[3], *xyzw[:3]))
    return result


def gripper_nodes():
    """Use the optional gripper's actual geometry, inertias and hinge from xacro."""
    properties = {}
    for node in ET.parse(Z1 / "xacro/const.xacro").getroot():
        if node.tag.endswith("property"):
            try:
                properties[node.get("name")] = float(node.get("value"))
            except ValueError:
                pass
    # The source writes this one bound as ${-PI*90/180}.
    properties["Gripper_PositionMin"] = -math.pi / 2
    source = ET.parse(Z1 / "xacro/robot.xacro").getroot()
    conditional = next(node for node in source if node.tag.endswith("if"))
    result = []
    import re

    for node in conditional:
        if node.tag not in ("link", "joint"):
            continue
        text = re.sub(r"\$\{([^}]+)\}", lambda match: str(properties[match[1]]),
                      ET.tostring(node, encoding="unicode"))
        result.append(ET.fromstring(text))
    return result


def visual_meshes(filename, assets):
    """Bake Collada node transforms and retain its per-part source materials."""
    import trimesh

    source = Z1 / filename.removeprefix("package://z1_description/")
    scene = trimesh.load(source, force="scene")
    result = []
    for index, node in enumerate(scene.graph.nodes_geometry):
        transform, geometry = scene.graph[node]
        mesh = scene.geometry[geometry].copy()
        mesh.apply_transform(transform)
        name = f"z1_{source.stem}_{index}"
        output = OUT / f"{name}.obj"
        # The MJCF material carries colour; avoid unused OBJ material sidecars.
        output.write_text(trimesh.exchange.obj.export_obj(mesh, include_texture=False, include_color=False))
        material = getattr(mesh.visual, "material", None)
        color = getattr(material, "diffuse", None)
        if color is None:
            color = (180, 180, 185, 255)
        rgba = np.asarray(color, dtype=float)[:4] / 255.0
        if len(rgba) == 3:
            rgba = np.append(rgba, 1.0)
        ET.SubElement(assets, "mesh", name=name, file=output.name)
        result.append((name, numbers(rgba)))
    return result


def prepare():
    OUT.mkdir(parents=True, exist_ok=True)
    output = OUT / "b2_z1.xml"
    # Versioned cache: changing this module invalidates the generated assembly.
    marker = OUT / "prepared.json"
    stamp = Path(__file__).stat().st_mtime_ns
    if output.exists() and marker.exists() and json.loads(marker.read_text()).get("source_stamp") == stamp:
        return output

    tree = ET.parse(B2 / "b2.xml")
    root = tree.getroot()
    root.set("model", "unitree_b2_z1")
    root.find("compiler").set("meshdir", ".")
    for child in list(root):
        if child.tag in ("sensor", "keyframe", "actuator"):
            root.remove(child)
    assets = root.find("asset")
    for mesh in assets.findall("mesh"):
        mesh.set("file", f"../unitree_mujoco/unitree_robots/b2/assets/{mesh.get('file')}")
    trunk = root.find("worldbody/body")
    trunk.set("pos", numbers((0.0, 0.0, BASE_HEIGHT)))
    # A rigid bolted mounting plate, part of the robot, never attached to the workpiece.
    ET.SubElement(trunk, "geom", name="z1_mount_plate", type="box", size="0.12 0.085 0.01",
                  pos="-0.10 0 0.085", rgba="0.12 0.14 0.17 1", density="2700")

    urdf = ET.parse(Z1 / "xacro/z1.urdf").getroot()
    for node in gripper_nodes():
        urdf.append(node)
    # An arm-only URDF keeps interactive IK independent of the leg and free-base DOFs.
    arm_urdf = ET.Element("robot", name="unitree_z1_with_gripper")
    for node in urdf:
        if node.tag in ("link", "joint"):
            arm_urdf.append(copy.deepcopy(node))
    for mesh in arm_urdf.findall(".//mesh"):
        relative = mesh.get("filename").removeprefix("package://z1_description/")
        mesh.set("filename", f"../unitree_ros/robots/z1_description/{relative}")
    ET.indent(arm_urdf, space="  ")
    ET.ElementTree(arm_urdf).write(OUT / "z1_kinematics.urdf", encoding="unicode")
    links = {link.get("name"): link for link in urdf.findall("link")}
    joints = [joint for joint in urdf.findall("joint") if joint.get("name") != "base_static_joint"]
    actuators = ET.SubElement(root, "actuator")
    for name in LEG_HOME:
        ET.SubElement(actuators, "position", name=f"stand_{name}", joint=name,
                      kp="1800", kv="100", forcerange="-200 200" if "calf" not in name else "-300 300")

    def add_link(name, parent, incoming=None):
        link = links[name]
        transform = origin(incoming.find("origin")) if incoming is not None else {"pos": "-0.10 0 0.095"}
        body = ET.SubElement(parent, "body", name=f"z1_{name}", **transform)
        inertial = link.find("inertial")
        if inertial is not None:
            inertia = inertial.find("inertia")
            ET.SubElement(body, "inertial", mass=inertial.find("mass").get("value"),
                          fullinertia=" ".join(inertia.get(k) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")),
                          pos=inertial.find("origin").get("xyz", "0 0 0"))
        if incoming is not None and incoming.get("type") != "fixed":
            label = f"z1_{incoming.get('name')}"
            limit = incoming.find("limit")
            lower = limit.get("lower", str(-math.pi / 2))
            upper = limit.get("upper", "0")
            ET.SubElement(body, "joint", name=label, type="hinge", axis=incoming.find("axis").get("xyz"),
                          range=f"{lower} {upper}", damping="0.15", armature="0.01")
            effort = limit.get("effort", "30")
            ET.SubElement(actuators, "position", name=f"drive_{label}", joint=label,
                          kp="500" if "Gripper" not in label else "80", kv="25" if "Gripper" not in label else "4",
                          forcerange=f"-{effort} {effort}")
        for visual in link.findall("visual"):
            geometry = visual.find("geometry/mesh")
            if geometry is not None:
                for mesh_name, rgba in visual_meshes(geometry.get("filename"), assets):
                    ET.SubElement(body, "geom", type="mesh", mesh=mesh_name, rgba=rgba,
                                  contype="0", conaffinity="0", density="0", **origin(visual.find("origin")))
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            transform = origin(collision.find("origin"))
            cylinder = geometry.find("cylinder")
            mesh = geometry.find("mesh")
            if cylinder is not None:
                ET.SubElement(body, "geom", type="cylinder", size=f"{cylinder.get('radius')} {float(cylinder.get('length')) / 2}",
                              **{"class": "collision"}, **transform)
            elif mesh is not None:
                source = mesh.get("filename").removeprefix("package://z1_description/")
                label = f"z1_collision_{name}"
                ET.SubElement(assets, "mesh", name=label, file=f"../unitree_ros/robots/z1_description/{source}")
                ET.SubElement(body, "geom", type="mesh", mesh=label, **{"class": "collision"}, **transform)
        for joint in joints:
            if joint.find("parent").get("link") == name:
                add_link(joint.find("child").get("link"), body, joint)

    add_link("link00", trunk)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="unicode")
    sources = []
    for folder in ("unitree_mujoco", "unitree_ros"):
        revision = subprocess.check_output(["git", "-C", str(ROOT / "assets" / folder), "rev-parse", "HEAD"], text=True).strip()
        sources.append({"url": f"https://github.com/unitreerobotics/{folder}", "commit": revision})
    marker.write_text(json.dumps({"source_stamp": stamp, "sources": sources,
                                 "mount_xyz": [-0.10, 0.0, 0.095], "leg_home": LEG_HOME,
                                 "arm_home": ARM_HOME, "base_height": BASE_HEIGHT}, indent=2) + "\n")
    return output
