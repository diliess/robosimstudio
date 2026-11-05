"""Parametric industrial welding hall. Edit config.json, then save/reload this file."""

from pathlib import Path

import scene_parts  # noqa: F401 -- registers resident floating B2 importer
from camera_views import VIEWS
from fixtures import FixtureSpec
from hall_geometry import Hall
from prepare_assets import ARM_HOME, ARM_JOINTS, ARM_KD, ARM_KP
from robot_assets import prepare_robot
from scipy.spatial.transform import Rotation as R

import robosimstudio as rss
from robosimstudio.core.render.spec import RenderSpec
from robosimstudio.robots import register_gripper
from robosimstudio.specs import Binding, GripperSpec

ROOT = Path(__file__).resolve().parent
hall = Hall()
config = hall.config
MODEL, TCP_OFFSET, TCP_QUAT = prepare_robot(config["robot"])
hall.export()
cell = rss.register_cell(
    rss.CellSpec(
        name="welding_hall_floor",
        ground_z=-0.05,
        surface_z=0.0,
        center=(0, 0, 0),
        provenance=f"Configured {config['hall']['size_m']} m hall; geometric ground truth, no SLAM",
    )
)
tool = register_gripper(
    GripperSpec(
        name="industrial_e05l_torch",
        finger_names=(),
        tcp_body="e05l_welding_torch",
        tcp_offset=TCP_OFFSET,
        tcp_offset_rot=TCP_QUAT,
        approach_axis=(0, 0, 1),
    )
)
robot = rss.EmbodimentSpec(
    name="unitree_b2_e05l_pro",
    asset=rss.AssetSpec(path=str(MODEL)),
    arms=(
        rss.ArmSpec(
            name="e05l",
            joint_names=ARM_JOINTS,
            ee_body_name="e05l_end_Link",
            base_body_name="e05l_base_link",
            gripper=tool.name,
            home_q=tuple(
                config["robot"].get("initial_arm_q_rad", config["robot"].get("transport_arm_q_rad", ARM_HOME))
            ),
        ),
    ),
    arm_actuation=rss.ActuationSpec(target_ke=ARM_KP, target_kd=ARM_KD),
    urdf="",
    ik_iterations=8,
)
scene = rss.Scene(
    cell=cell.name,
    name="industrial_welding",
    physics=rss.MUJOCO_500HZ,
    binding=Binding(scene=cell.name, embodiment=robot.name, scene_resident=True),
    render=RenderSpec(environment="lab", scenery="", key_intensity=2.3, ambient=(2.4, 2.4, 2.4), quality="preview"),
)
for o in hall.objects.values():
    scene.add(FixtureSpec(name=o["id"], parts=tuple(o["parts"]), xform=tuple(o["pose"])))
x, y, yaw = config["robot"]["start_xy_yaw"]
q = R.from_euler("z", yaw).as_quat()
scene.add(
    rss.StaticSpec(
        name="robot_asset", asset=rss.AssetSpec(path=str(MODEL)), entity="unitree_b2_e05l_import", xform=(x, y, 0, *q)
    )
)
scene.add(rss.robot(robot))

for name, (eye, look, fov, size) in VIEWS.items():
    scene.add(
        rss.CameraSpec(
            name=name,
            width=size[0],
            height=size[1],
            fov_y=fov,
            eye=eye,
            look_at=look,
            up=(0, 1, 0) if name == "top" else (0, 0, 1),
            near=0.03,
            far=100,
        )
    )


def controls(sim, session):
    """Install only scene-local UI, through the repository's routine/page APIs."""
    from welding_ui import install

    install(sim, session, hall, VIEWS)
    yield


routines = {"Welding controls": controls}
