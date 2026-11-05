"""Shared configured initial pose for MJCF, live importer and OpenUSD export.

The transport pose remains a separate navigation/task configuration.
"""

import mujoco as mj
import numpy as np
from prepare_assets import ARM_HOME, ARM_JOINTS, LEG_HOME
from scipy.spatial.transform import Rotation as R


def initial_joint_targets(config):
    legs = config.get("initial_leg_q_rad", list(LEG_HOME.values()))
    arm = config.get("initial_arm_q_rad", config.get("transport_arm_q_rad", ARM_HOME))
    targets = {**dict(zip(LEG_HOME, legs, strict=True)), **dict(zip(ARM_JOINTS, arm, strict=True))}
    if not np.isfinite(list(targets.values())).all():
        raise ValueError("Initial robot joint positions must be finite radians")
    return targets


def apply_initial_pose(model, data, config, *, world=False):
    """Set full articulation pose, align foot bottoms and initialize servo targets."""
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0
    data.qpos[:3] = 0
    rotation = R.from_quat(config.get("initial_base_quat_xyzw", [0, 0, 0, 1]))
    if world:
        x, y, yaw = config["start_xy_yaw"]
        data.qpos[:2] = [x, y]
        rotation = R.from_euler("z", yaw) * rotation
    q = rotation.as_quat()
    data.qpos[3:7] = [q[3], *q[:3]]
    targets = initial_joint_targets(config)
    for name, value in targets.items():
        joint = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
        if joint < 0:
            raise ValueError(f"Unknown configured initial joint: {name}")
        if model.jnt_limited[joint] and not model.jnt_range[joint, 0] <= value <= model.jnt_range[joint, 1]:
            raise ValueError(f"Initial {name}={value} rad is outside source limits {model.jnt_range[joint]}")
        data.qpos[model.jnt_qposadr[joint]] = value
    feet = [
        i for i in range(model.ngeom) if model.geom_type[i] == mj.mjtGeom.mjGEOM_SPHERE and model.geom_size[i, 0] > 0.03
    ]
    if len(feet) != 4:
        raise ValueError(f"Expected four source B2 foot spheres, found {len(feet)}")
    mj.mj_forward(model, data)

    def bottom():
        return data.geom_xpos[feet, 2] - model.geom_size[feet, 0]

    data.qpos[2] += float(config.get("initial_ground_clearance_m", 0)) - float(bottom().min())
    for actuator in range(model.nu):
        joint = int(model.actuator_trnid[actuator, 0])
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, joint)
        if name in targets:
            data.ctrl[actuator] = targets[name]
    mj.mj_forward(model, data)
    return {
        "source": config.get("initial_pose_source", "configured initial joints; foot-aligned base"),
        "arm_joint_names": list(ARM_JOINTS),
        "arm_q_rad": [targets[name] for name in ARM_JOINTS],
        "leg_joint_names": list(LEG_HOME),
        "leg_q_rad": [targets[name] for name in LEG_HOME],
        "base_position_m": data.qpos[:3].tolist(),
        "base_quat_xyzw": [*data.qpos[4:7], data.qpos[3]],
        "foot_bottom_z_m": bottom().tolist(),
        "joint_limits_passed": True,
        "scope": "initial configuration only; no gait or dynamic balance certificate",
    }
