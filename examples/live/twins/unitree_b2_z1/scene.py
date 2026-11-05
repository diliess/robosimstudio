# SPDX-License-Identifier: Apache-2.0
"""B2 (no lidar/guard) + E05L-Pro: a real-to-sim twin, rebuilt on the live page on every save. The method: examples/live/AGENTS.md."""

from pathlib import Path

import scene_parts  # noqa: F401 -- registers this scene's entities
from prepare_assets import (
    ARM_HOME,
    ARM_JOINTS,
    ARM_KD,
    ARM_KP,
    DEFAULT_MOUNT_RPY,
    DEFAULT_MOUNT_XYZ,
    TCP_OFFSET,
    TCP_QUAT,
    prepare,
)
from ship_grid import ShipGridSpec

import robosimstudio as rss
from robosimstudio.core.render.finish import Finish
from robosimstudio.core.render.spec import RenderSpec
from robosimstudio.robots import register_gripper
from robosimstudio.specs import Binding, GripperSpec

ROOT = Path(__file__).resolve().parent
# Mount in B2 base_link: +x forward, +y left, metres/radians.
# Source model default; the study also records (0, 0, 0.075) as a candidate.
ARM_MOUNT_XYZ = DEFAULT_MOUNT_XYZ
ARM_MOUNT_RPY = DEFAULT_MOUNT_RPY
MODEL = prepare(mount_xyz=ARM_MOUNT_XYZ, mount_rpy=ARM_MOUNT_RPY)

# Layout controls. A ring of four shared boundaries defines one cell.
GRID_ROWS = 3
GRID_COLUMNS = 3
CELL_CLEAR_SIZE = 2.0  # metres between the inner faces of adjacent webs
WEB_HEIGHT = 0.220
PLATE_THICKNESS = 0.010
FLANGE_WIDTH = 0.180
ACTIVE_CELL = (GRID_ROWS // 2, GRID_COLUMNS // 2)

grid = ShipGridSpec(
    name="ship_grid",
    rows=GRID_ROWS,
    columns=GRID_COLUMNS,
    clear_size=CELL_CLEAR_SIZE,
    web_height=WEB_HEIGHT,
    thickness=PLATE_THICKNESS,
    flange_width=FLANGE_WIDTH,
    friction=0.65,
)
robot_x, robot_y = grid.cell_center(*ACTIVE_CELL)
cell = rss.register_cell(
    rss.CellSpec(
        name="ship_plate_floor",
        ground_z=-PLATE_THICKNESS,
        surface_z=0.0,
        center=(0.0, 0.0, 0.0),
        provenance="Assumed ship-plate welding grid; dimensions in scene.py",
    )
)

tool = register_gripper(
    GripperSpec(
        name="e05l_fixed_welding_torch",
        finger_names=(),
        tcp_body="e05l_welding_torch",
        tcp_offset=TCP_OFFSET,
        tcp_offset_rot=TCP_QUAT,
        approach_axis=(0.0, 0.0, 1.0),
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
            home_q=ARM_HOME,
        ),
    ),
    arm_actuation=rss.ActuationSpec(target_ke=ARM_KP, target_kd=ARM_KD),
    # Newton IK follows the live floating base; fixed-root PyRoKi used its initial pose.
    urdf="",
    ik_iterations=8,
)

scene = rss.Scene(
    cell=cell.name,
    name="unitree_b2_e05l_pro",
    physics=rss.MUJOCO_500HZ,
    binding=Binding(scene=cell.name, embodiment=robot.name, scene_resident=True),
    render=RenderSpec(environment="lab", scenery="", key_intensity=1.4, ambient=(1.6, 1.6, 1.6), quality="preview"),
)
scene.add(grid)
scene.add(
    rss.StaticSpec(
        name="robot_asset",
        asset=rss.AssetSpec(path=str(MODEL)),
        entity="unitree_b2_e05l_import",
        xform=(robot_x, robot_y, 0.0, 0.0, 0.0, 0.0, 1.0),
    )
)
scene.add(rss.robot(robot))

# An unobstructed fabrication-floor surround; the physical ground is already in the cell.
yard_half = max(*grid.deck_half) + 4.0
scene.add(
    rss.slab(
        name="yard_floor",
        half=(yard_half, yard_half, 0.02),
        at=(0.0, 0.0, -PLATE_THICKNESS - 0.02),
        collide=False,
        color=(0.22, 0.25, 0.28),
        finish=Finish(roughness=0.95),
    )
)

extent = max(*grid.deck_half)
scene.add(
    rss.CameraSpec(
        name="overview",
        width=1600,
        height=1100,
        fov_y=40.0,
        eye=(extent * 2.0, -extent * 2.4, extent * 2.1),
        look_at=(0.0, 0.0, 0.0),
        up=(0.0, 0.0, 1.0),
        near=0.05,
        far=40.0,
    )
)
scene.add(
    rss.CameraSpec(
        name="top",
        width=1400,
        height=1400,
        fov_y=35.0,
        eye=(0.0, 0.0, extent * 3.5),
        look_at=(0.0, 0.0, 0.0),
        up=(0.0, 1.0, 0.0),
        near=0.05,
        far=40.0,
    )
)
scene.add(
    rss.CameraSpec(
        name="center_cell",
        width=1280,
        height=800,
        fov_y=50.0,
        eye=(robot_x + 2.0, robot_y - 2.3, 1.9),
        look_at=(robot_x, robot_y, 0.40),
        up=(0.0, 0.0, 1.0),
        near=0.01,
        far=20.0,
    )
)
scene.add(
    rss.CameraSpec(
        name="part_detail",
        width=1000,
        height=1250,
        fov_y=35.0,
        eye=(robot_x + 1.65, robot_y - 1.65, 0.85),
        look_at=(robot_x + 1.005, robot_y - 1.005, 0.12),
        up=(0.0, 0.0, 1.0),
        near=0.01,
        far=12.0,
    )
)
