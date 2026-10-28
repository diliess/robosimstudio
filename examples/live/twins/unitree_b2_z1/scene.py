# SPDX-License-Identifier: Apache-2.0
"""unitree_b2_z1: a real-to-sim twin, rebuilt on the live page on every save. The method: examples/live/AGENTS.md."""

from pathlib import Path

import robosimstudio as rss
from robosimstudio.core.render.finish import Finish
from robosimstudio.core.render.spec import RenderSpec
from robosimstudio.robots import register_gripper
from robosimstudio.specs import Binding, GripperSpec

import scene_parts  # registers only this scene's import and workpiece entities
from prepare_assets import ARM_HOME, prepare
from ship_grid import ShipGridSpec

ROOT = Path(__file__).resolve().parent
MODEL = prepare()

# Layout controls. A ring of four shared boundaries defines one cell.
GRID_ROWS = 3
GRID_COLUMNS = 3
CELL_CLEAR_SIZE = 2.0  # metres between the inner faces of adjacent webs
WEB_HEIGHT = 0.220
PLATE_THICKNESS = 0.010
FLANGE_WIDTH = 0.180
ACTIVE_CELL = (GRID_ROWS // 2, GRID_COLUMNS // 2)

grid = ShipGridSpec(name="ship_grid", rows=GRID_ROWS, columns=GRID_COLUMNS,
                    clear_size=CELL_CLEAR_SIZE, web_height=WEB_HEIGHT,
                    thickness=PLATE_THICKNESS, flange_width=FLANGE_WIDTH, friction=0.65)
robot_x, robot_y = grid.cell_center(*ACTIVE_CELL)
cell = rss.register_cell(rss.CellSpec(name="ship_plate_floor", ground_z=-PLATE_THICKNESS,
                                     surface_z=0.0, center=(0.0, 0.0, 0.0),
                                     provenance="Assumed ship-plate welding grid; dimensions in scene.py"))

hand = register_gripper(
    GripperSpec(
        name="unitree_z1_native",
        finger_names=("z1_jointGripper",),
        finger_open_q=(-0.6,),
        finger_closed_q=(0.0,),
        linear_travel=False,
        tcp_body="z1_gripperStator",
        tcp_offset=(0.09, 0.0, 0.0),
        tcp_offset_rot=(0.0, 0.7071067811865476, 0.0, 0.7071067811865476),
        approach_axis=(1.0, 0.0, 0.0),
        close_axis=(0.0, 0.0, 1.0),
        finger_bodies=(("z1_gripperStator",), ("z1_gripperMover",)),
    )
)

robot = rss.EmbodimentSpec(
    name="unitree_b2_z1",
    asset=rss.AssetSpec(path=str(MODEL)),
    arms=(rss.ArmSpec(name="z1", joint_names=tuple(f"z1_joint{i}" for i in range(1, 7)),
                      ee_body_name="z1_link06", base_body_name="z1_link00", gripper=hand.name,
                      home_q=ARM_HOME, home_gripper_q=(-0.6,)),),
    arm_actuation=rss.ActuationSpec(target_ke=500, target_kd=25),
    gripper_actuation=rss.ActuationSpec(target_ke=80, target_kd=4),
    urdf=str(MODEL.parent / "z1_kinematics.urdf"),
    ik_iterations=8,
)

scene = rss.Scene(cell=cell.name, name="unitree_b2_z1", physics=rss.MUJOCO_500HZ,
                  binding=Binding(scene=cell.name, embodiment=robot.name, scene_resident=True),
                  render=RenderSpec(environment="lab", scenery="", key_intensity=1.4,
                                    ambient=(1.6, 1.6, 1.6), quality="preview"))
scene.add(grid)
scene.add(rss.StaticSpec(name="robot_asset", asset=rss.AssetSpec(path=str(MODEL)),
                         entity="unitree_b2_z1_import",
                         xform=(robot_x, robot_y, 0.0, 0.0, 0.0, 0.0, 1.0)))
scene.add(rss.robot(robot))

# An unobstructed fabrication-floor surround; the physical ground is already in the cell.
yard_half = max(*grid.deck_half) + 4.0
scene.add(rss.slab(name="yard_floor", half=(yard_half, yard_half, 0.02),
                   at=(0.0, 0.0, -PLATE_THICKNESS - 0.02), collide=False,
                   color=(0.22, 0.25, 0.28), finish=Finish(roughness=0.95)))

extent = max(*grid.deck_half)
scene.add(rss.CameraSpec(name="overview", width=1600, height=1100, fov_y=40.0,
                         eye=(extent * 2.0, -extent * 2.4, extent * 2.1), look_at=(0.0, 0.0, 0.0),
                         up=(0.0, 0.0, 1.0), near=0.05, far=40.0))
scene.add(rss.CameraSpec(name="top", width=1400, height=1400, fov_y=35.0,
                         eye=(0.0, 0.0, extent * 3.5), look_at=(0.0, 0.0, 0.0),
                         up=(0.0, 1.0, 0.0), near=0.05, far=40.0))
scene.add(rss.CameraSpec(name="center_cell", width=1280, height=800, fov_y=50.0,
                         eye=(robot_x + 2.0, robot_y - 2.3, 1.9), look_at=(robot_x, robot_y, 0.40),
                         up=(0.0, 0.0, 1.0), near=0.01, far=20.0))
scene.add(rss.CameraSpec(name="part_detail", width=1000, height=1250, fov_y=35.0,
                         eye=(robot_x + 1.65, robot_y - 1.65, 0.85),
                         look_at=(robot_x + 1.005, robot_y - 1.005, 0.12),
                         up=(0.0, 0.0, 1.0), near=0.01, far=12.0))
