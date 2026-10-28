"""Scene-local extensions for a free B2 base and the inseparable T-shaped part."""

from __future__ import annotations

from dataclasses import replace

import warp as wp

import robosimstudio as rss
from robosimstudio.domains.rigid.entity import EntityRigid
from robosimstudio.domains.static.entity import EntityStatic
from robosimstudio.core.render.finish import Finish
from robosimstudio.paths import resolve_asset

from prepare_assets import ARM_HOME, LEG_HOME, steel_texture


@rss.register_entity("unitree_b2_z1_import")
class B2Z1Import(EntityStatic):
    """Import the robot as a resident articulation, keeping its real free joint."""

    def _add_asset(self, builder):
        from newton.solvers import SolverMuJoCo

        SolverMuJoCo.register_custom_attributes(builder)
        _, frame = self._shape_frame()
        builder.add_mjcf(str(resolve_asset(self.spec.asset)), floating=None, xform=frame,
                         enable_self_collisions=False, collapse_fixed_joints=False,
                         parse_sites=False, verbose=False)
        home = {**LEG_HOME, **{f"z1_joint{i + 1}": q for i, q in enumerate(ARM_HOME)},
                "z1_jointGripper": -0.6}
        for index, label in enumerate(builder.joint_label):
            name = str(label).rsplit("/", 1)[-1]
            if name in home:
                coord = int(builder.joint_q_start[index])
                builder.joint_q[coord] = home[name]
                # The compiler uses coordinate-indexed targets.
                if coord < len(builder.joint_target_q):
                    builder.joint_target_q[coord] = home[name]

    def handle(self, model):
        # The resident embodiment publishes the robot, without a duplicate fixture handle.
        return None


@rss.register_entity("steel_t_workpiece")
class SteelTWorkpiece(EntityRigid):
    """Two exact box colliders sharing one free rigid body, without a weld constraint."""

    def _contact_cfg(self, builder):
        cfg = super()._contact_cfg(builder)
        cfg.density = float(self.spec.density)
        return cfg

    def _add_primitive(self, builder, pose):
        super()._add_primitive(builder, pose)
        self.base_shape = builder.shape_count - 1
        cfg = self._contact_cfg(builder)
        self.upright_shape = builder.add_shape_box(
            self.body, hx=0.150, hy=0.005, hz=0.110,
            xform=wp.transform((0.0, 0.0, 0.115), wp.quat_identity()),
            cfg=cfg, label="workpiece_upright_plate",
        )
        # A display-only cut face. It adds neither contact geometry nor mass.
        visual = cfg.copy()
        visual.density = 0.0
        visual.has_shape_collision = False
        visual.has_particle_collision = False
        self.cut_shape = builder.add_shape_box(
            self.body, hx=0.150, hy=0.005, hz=0.00001,
            xform=wp.transform((0.0, 0.0, 0.225), wp.quat_identity()),
            cfg=visual, label="workpiece_bright_cut_face",
        )

    def apply_display(self, builder):
        super().apply_display(builder)
        builder.shape_color[self.base_shape] = (0.52, 0.60, 0.59)
        builder.shape_color[self.upright_shape] = (0.22, 0.27, 0.26)
        builder.shape_color[self.cut_shape] = (0.70, 0.77, 0.76)
        self.finishes[self.upright_shape] = replace(self.spec.finish, texture=str(steel_texture("upright")))
        self.finishes[self.cut_shape] = Finish(metallic=0.8, roughness=0.35)
