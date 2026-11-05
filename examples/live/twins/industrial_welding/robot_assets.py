"""Configurable physical mounting plate and calibrated-source TCP adapter."""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from prepare_assets import TCP_OFFSET, TCP_QUAT, prepare
from scipy.spatial.transform import Rotation as R


def prepare_robot(c):
    src = prepare(mount_xyz=c["mount_xyz_m"], mount_rpy=c["mount_rpy_rad"])
    tree = ET.parse(src)
    root = tree.getroot()
    base = root.find("worldbody/body[@name='base_link']")
    if c.get("mount_plate_enabled", True):
        dx, dy, dz = c["mount_plate_size_m"]
        mx, my, mz = c["mount_xyz_m"]
        # Rigidly bolted part. The coarse source body collider overlaps its seat;
        # this is an intended fixed-assembly interface, not an environment contact.
        mount_rotation = R.from_euler("xyz", c["mount_rpy_rad"])
        plate_center = np.array([mx, my, mz]) + mount_rotation.apply([0, 0, dz / 2])
        plate_quat = mount_rotation.as_quat()
        plate = ET.SubElement(
            base,
            "body",
            name="e05l_mount_plate",
            pos=" ".join(map(str, plate_center)),
            quat=" ".join(map(str, [plate_quat[3], *plate_quat[:3]])),
        )
        mass = 7850 * dx * dy * dz
        ET.SubElement(
            plate,
            "inertial",
            pos="0 0 0",
            mass=str(mass),
            diaginertia=" ".join(str(mass * v / 12) for v in (dy * dy + dz * dz, dx * dx + dz * dz, dx * dx + dy * dy)),
        )
        ET.SubElement(
            plate,
            "geom",
            name="mount_plate_collision_visual",
            type="box",
            size=f"{dx / 2} {dy / 2} {dz / 2}",
            rgba=".16 .19 .20 1",
            friction="0.8 .005 .0001",
        )
    tool = root.find(".//body[@name='e05l_welding_torch']")
    if tool is None:
        raise RuntimeError("Source E05L welding torch body missing")
    source_position = np.fromstring(tool.get("pos", "0 0 0"), sep=" ")
    source_quat = np.fromstring(tool.get("quat", "1 0 0 0"), sep=" ")
    source_rotation = R.from_quat([*source_quat[1:], source_quat[0]])
    physical_position = source_position + source_rotation.apply(c.get("torch_mount_delta_xyz_m", [0, 0, 0]))
    physical_rotation = (source_rotation * R.from_euler("xyz", c.get("torch_mount_delta_rpy_rad", [0, 0, 0]))).as_quat()
    tool.set("pos", " ".join(map(str, physical_position)))
    tool.set("quat", " ".join(map(str, [physical_rotation[3], *physical_rotation[:3]])))
    offset = np.array(TCP_OFFSET) + np.array(c["torch_tcp_delta_m"])
    q = (R.from_quat(TCP_QUAT) * R.from_euler("xyz", c["torch_tcp_delta_rpy_rad"])).as_quat()
    ET.SubElement(
        tool,
        "site",
        name="welding_tcp",
        pos=" ".join(map(str, offset)),
        quat=" ".join(map(str, [q[3], *q[:3]])),
        size=".006",
        rgba="1 .2 0 1",
    )
    out = Path(src).with_name("industrial_b2_e05l_pro.xml")
    ET.indent(tree, space="  ")
    tree.write(out, encoding="unicode")
    if "initial_arm_q_rad" in c or "initial_leg_q_rad" in c:
        import mujoco as mj
        from robot_initial import apply_initial_pose

        model = mj.MjModel.from_xml_path(str(out))
        data = mj.MjData(model)
        apply_initial_pose(model, data, c)
        # The resident importer sets joint coordinates/targets after import;
        # write the matching floating root pose to the MJCF first.
        base.set("pos", " ".join(map(str, data.qpos[:3])))
        base.set("quat", " ".join(map(str, data.qpos[3:7])))
        tree.write(out, encoding="unicode")
    return out, tuple(offset), tuple(q)
