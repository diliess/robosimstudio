"""Scene-local extensions for a free B2 base and the inseparable T-shaped part."""

from __future__ import annotations

from robot_initial import initial_joint_targets

import robosimstudio as rss
from robosimstudio.domains.static.entity import EntityStatic
from robosimstudio.paths import resolve_asset


@rss.register_entity("unitree_b2_e05l_import")
class B2E05LImport(EntityStatic):
    """Import the robot as a resident articulation, keeping its real free joint."""

    def _add_asset(self, builder):
        from newton.solvers import SolverMuJoCo

        SolverMuJoCo.register_custom_attributes(builder)
        _, frame = self._shape_frame()
        builder.add_mjcf(
            str(resolve_asset(self.spec.asset)),
            floating=None,
            xform=frame,
            enable_self_collisions=True,
            collapse_fixed_joints=False,
            parse_sites=False,
            verbose=False,
        )
        from hall_geometry import load_config

        home = initial_joint_targets(load_config()["robot"])
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
