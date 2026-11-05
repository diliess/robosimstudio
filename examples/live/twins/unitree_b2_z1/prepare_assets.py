"""Adapt the supplied B2 (no lidar/guard) + E05L-Pro assembly for the live scene.

Geometry, inertias, limits, mounting pose and short-nozzle TCP come from
GO2_RL_LAB_ROOT (default: the sibling go2_rl_lab checkout). A portable snapshot
of the required source files is kept under assets/assembled/source/.
"""

from __future__ import annotations

import copy
import hashlib
import json
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "assets/assembled"
SNAPSHOT = OUT / "source"
SOURCE = Path(os.environ.get("GO2_RL_LAB_ROOT", str(ROOT.parents[4] / "go2_rl_lab"))).expanduser().resolve()
MODEL_REL = Path("deploy/deploy_mujoco/assets/b2_e05l_pro_simplified.xml")
FLAT_REL = MODEL_REL.with_name("flat_b2_e05l_pro_simplified.xml")
META_REL = Path("source/go2_rl_lab/go2_rl_lab/assets/urdf/b2_e05l_pro")
URDF_REL = META_REL / "b2_e05l_pro_simplified.urdf"


def _input(relative):
    live = SOURCE / relative
    cached = SNAPSHOT / relative
    if live.is_file():
        return live
    if cached.is_file():
        return cached
    raise FileNotFoundError(f"Missing B2/E05L asset: {live}. Set GO2_RL_LAB_ROOT to the supplied repository.")


def _metadata(name):
    return json.loads(_input(META_REL / name).read_text())


_INTEGRATION = _metadata("integration.json")
_POSE = _metadata("visualization_pose.json")
_TIP = _metadata("welding_tip.json")
DEFAULT_MOUNT_XYZ = tuple(_INTEGRATION["mount_pos"])
DEFAULT_MOUNT_RPY = tuple(_INTEGRATION["mount_rpy"])
ARM_JOINTS = tuple(_POSE["arm_joint_names"])
ARM_HOME = tuple(_POSE["arm_q_rad"])
LEG_HOME = dict(
    zip(
        (f"{leg}_{joint}_joint" for leg in ("FL", "FR", "RL", "RR") for joint in ("hip", "thigh", "calf")),
        _POSE["leg_joint_positions"],
        strict=True,
    )
)
TCP_OFFSET = tuple(_TIP["position_m"])
# Tool +z follows the source short-nozzle outlet normal; +x is perpendicular to world/local +y.
_z = np.asarray(_TIP["outward_axis"], dtype=float)
_z /= np.linalg.norm(_z)
_x = np.cross((0.0, 1.0, 0.0), _z)
_x /= np.linalg.norm(_x)
TCP_QUAT = tuple(Rotation.from_matrix(np.column_stack((_x, np.cross(_z, _x), _z))).as_quat())
ARM_KP, ARM_KD = 50000.0, 400.0  # simulation position servos, not manufacturer motor ratings


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


def _sha(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def _write_xml(tree, path):
    ET.indent(tree, space="  ")
    tree.write(path, encoding="unicode")


def prepare(*, mount_xyz=DEFAULT_MOUNT_XYZ, mount_rpy=DEFAULT_MOUNT_RPY):
    """Keep the supplied combined robot; change only paths, home and simulation drives."""
    OUT.mkdir(parents=True, exist_ok=True)
    output = OUT / "b2_e05l_pro.xml"
    marker = OUT / "prepared_e05l.json"
    tree = ET.parse(_input(MODEL_REL))
    robot = tree.getroot()
    files = {
        MODEL_REL,
        FLAT_REL,
        URDF_REL,
        *(META_REL / n for n in ("integration.json", "visualization_pose.json", "welding_tip.json")),
    }
    if (SOURCE / "LICENSE").is_file() or (SNAPSHOT / "LICENSE").is_file():
        files.add(Path("LICENSE"))
    meshdir = robot.find("compiler").get("meshdir", ".")
    mesh_paths = []
    for mesh in robot.findall("asset/mesh"):
        relative = (SOURCE / MODEL_REL.parent / meshdir / mesh.get("file")).resolve().relative_to(SOURCE)
        path = _input(relative)
        with path.open("rb") as stream:
            if stream.read(80).startswith(b"version https://git-lfs.github.com/spec/v1"):
                raise RuntimeError(f"Git LFS mesh not downloaded: {path}; run git lfs pull in go2_rl_lab")
        files.add(relative)
        mesh_paths.append((mesh, relative))
    signature = {
        "adapter_sha256": _sha(Path(__file__)),
        "mount_xyz": list(mount_xyz),
        "mount_rpy": list(mount_rpy),
        "inputs": {str(rel): _sha(_input(rel)) for rel in sorted(files)},
    }
    if output.is_file() and (OUT / "e05l_kinematics.urdf").is_file() and marker.is_file():
        if json.loads(marker.read_text()).get("signature") == signature:
            return output
    for relative in sorted(files):
        src, dst = _input(relative), SNAPSHOT / relative
        dst.parent.mkdir(parents=True, exist_ok=True)
        if src.resolve() != dst.resolve() and (not dst.is_file() or _sha(dst) != signature["inputs"][str(relative)]):
            shutil.copy2(src, dst)
    for mesh, relative in mesh_paths:
        mesh.set("file", (Path("source") / relative).as_posix())
    robot.find("compiler").set("meshdir", ".")
    for name in ("sensor", "keyframe", "actuator"):
        for node in list(robot.findall(name)):
            robot.remove(node)
    base = robot.find("worldbody/body[@name='base_link']")
    standing = ET.parse(_input(FLAT_REL)).getroot().find("keyframe/key[@name='standing']")
    qpos = np.fromstring(standing.get("qpos"), sep=" ")
    base.set("pos", " ".join(map(str, qpos[:3])))
    base.set("quat", " ".join(map(str, qpos[3:7])))
    mount = base.find("body[@name='e05l_base_link']")
    mount.set("pos", " ".join(map(str, mount_xyz)))
    xyzw = Rotation.from_euler("xyz", mount_rpy).as_quat()
    mount.set("quat", " ".join(map(str, (xyzw[3], *xyzw[:3]))))
    actuators = ET.SubElement(robot, "actuator")
    original = ET.parse(_input(MODEL_REL)).getroot()
    torque_limits = {n.get("joint"): n.get("ctrlrange") for n in original.findall("actuator/motor")}
    for joint in LEG_HOME:
        ET.SubElement(
            actuators,
            "position",
            name=f"stand_{joint}",
            joint=joint,
            kp="1800",
            kv="100",
            forcerange=torque_limits[joint],
        )
    for joint in ARM_JOINTS:
        ET.SubElement(
            actuators,
            "position",
            name=f"drive_{joint}",
            joint=joint,
            kp=str(ARM_KP),
            kv=str(ARM_KD),
            forcelimited="false",
        )
    _write_xml(tree, output)
    # A six-axis kinematic model with the original URDF origins and limits.
    # Mesh-free links avoid an unrelated second geometry import in the IK solver.
    urdf = ET.parse(_input(URDF_REL)).getroot()
    arm = ET.Element("robot", name="e05l_pro_with_welding_torch")
    links = {n.get("name") for n in urdf.findall("link") if n.get("name").startswith("e05l_")}
    for node in urdf:
        if node.tag == "link" and node.get("name") in links:
            link = copy.deepcopy(node)
            for child in list(link):
                if child.tag in ("visual", "collision"):
                    link.remove(child)
            arm.append(link)
        elif node.tag == "joint" and all(node.find(endpoint).get("link") in links for endpoint in ("parent", "child")):
            arm.append(copy.deepcopy(node))
    _write_xml(ET.ElementTree(arm), OUT / "e05l_kinematics.urdf")
    marker.write_text(
        json.dumps(
            {
                "signature": signature,
                "source_root": str(SOURCE),
                "model": str(MODEL_REL),
                "mount_frame": "B2 base_link",
                "tcp": _TIP,
                "source_mass_kg": _INTEGRATION["combined_mjcf_mass_kg"],
                "leg_home": LEG_HOME,
                "arm_home": ARM_HOME,
                "simulation_servos": {
                    "leg_kp": 1800,
                    "leg_kd": 100,
                    "arm_kp": ARM_KP,
                    "arm_kd": ARM_KD,
                    "arm_effort_status": "unlimited simulation servo; source motor ratings unknown",
                },
            },
            indent=2,
        )
        + "\n"
    )
    return output
