"""Scene-local compound fixtures; physical geometry matches the exported pieces."""

from dataclasses import dataclass

import newton
import numpy as np
import warp as wp
from prepare_assets import steel_texture

import robosimstudio as rss
from robosimstudio.core.render.finish import Finish
from robosimstudio.domains.static.entity import EntityStatic


@dataclass(frozen=True, kw_only=True)
class FixtureSpec(rss.StaticSpec):
    entity: str = "industrial_fixture"
    parts: tuple = ()


@rss.register_entity("industrial_fixture")
class Fixture(EntityStatic):
    def _add_box(self, builder):
        body, frame = self._shape_frame()
        self.part_shapes = []
        for part in self.spec.parts:
            cfg = builder.default_shape_cfg.copy()
            cfg.density = 0
            cfg.mu = 0.75
            cfg.has_shape_collision = part["collision"]
            cfg.has_particle_collision = part["collision"]
            if part["type"] == "box":
                pos = wp.transform(part["center"], wp.quat(*part["quat"]))
                i = builder.add_shape_box(
                    body=body,
                    xform=wp.transform_multiply(frame, pos),
                    hx=part["size"][0] / 2,
                    hy=part["size"][1] / 2,
                    hz=part["size"][2] / 2,
                    cfg=cfg,
                    label=part["name"],
                )
            else:
                mesh = newton.Mesh(
                    vertices=np.asarray(part["vertices"], np.float32),
                    indices=np.asarray(part["faces"], np.int32).flatten(),
                )
                i = builder.add_shape_mesh(body=body, xform=frame, mesh=mesh, cfg=cfg, label=part["name"])
            self.part_shapes.append((i, part))

    def apply_display(self, builder):
        super().apply_display(builder)
        for i, p in self.part_shapes:
            builder.shape_color[i] = tuple(p["color"])
            if p["role"] == "steel" and p["type"] == "box":
                self.finishes[i] = Finish(
                    metallic=0.65, roughness=0.70, texture=str(steel_texture()), uv_scale=(3.0, 3.0)
                )
            elif p["role"] == "steel":
                self.finishes[i] = Finish(metallic=0.65, roughness=0.55)
            elif p["role"] == "floor":
                self.finishes[i] = Finish(metallic=0.0, roughness=0.95)
            elif p["role"] == "paint":
                self.finishes[i] = Finish(roughness=0.9)
            else:
                self.finishes[i] = Finish(metallic=0.15, roughness=0.85)
