"""Scene-local, fixed ship-plate assembly with shared T-section boundaries."""

from __future__ import annotations

from dataclasses import dataclass

import warp as wp

import robosimstudio as rss
from robosimstudio.core.render.finish import Finish
from robosimstudio.domains.static.entity import EntityStatic

from prepare_assets import steel_texture


@dataclass(frozen=True, kw_only=True)
class ShipGridSpec(rss.StaticSpec):
    entity: str = "ship_plate_grid"
    rows: int = 3
    columns: int = 3
    clear_size: float = 2.0
    web_height: float = 0.220
    thickness: float = 0.010
    flange_width: float = 0.180
    deck_margin: float = 0.120

    def __post_init__(self):
        if self.rows < 1 or self.columns < 1:
            raise ValueError("The ship grid needs at least one row and column")
        if not 0 < self.thickness < self.flange_width < self.clear_size:
            raise ValueError("Require 0 < thickness < flange width < cell clear size")
        if self.web_height <= 0 or self.deck_margin < 0:
            raise ValueError("Require positive web height and nonnegative deck margin")

    @property
    def pitch(self):
        return self.clear_size + self.thickness

    @property
    def half_span(self):
        return self.columns * self.pitch / 2, self.rows * self.pitch / 2

    @property
    def deck_half(self):
        hx, hy = self.half_span
        edge = self.flange_width / 2 + self.deck_margin
        return hx + edge, hy + edge

    def cell_center(self, row, column):
        if not 0 <= row < self.rows or not 0 <= column < self.columns:
            raise ValueError("Active cell is outside the grid")
        return ((column - (self.columns - 1) / 2) * self.pitch,
                (row - (self.rows - 1) / 2) * self.pitch)


@rss.register_entity("ship_plate_grid")
class ShipPlateGrid(EntityStatic):
    """A supported assembly: continuous deck, flanges, webs and display-only cut faces."""

    def _add_box(self, builder):
        spec = self.spec
        body, frame = self._shape_frame()
        self.parts = {kind: [] for kind in ("deck", "flange", "web", "cut")}

        def box(label, center, half, kind, *, collide=True):
            cfg = builder.default_shape_cfg.copy()
            cfg.density = 0.0
            cfg.mu = spec.friction if spec.friction is not None else 0.65
            cfg.has_shape_collision = collide
            cfg.has_particle_collision = collide
            pose = wp.transform_multiply(frame, wp.transform(center, wp.quat_identity()))
            index = builder.add_shape_box(body=body, xform=pose,
                                          hx=half[0], hy=half[1], hz=half[2], cfg=cfg,
                                          label=f"{spec.name}_{label}")
            self.parts[kind].append(index)

        hx, hy = spec.half_span
        deck_x, deck_y = spec.deck_half
        t = spec.thickness
        flange = spec.flange_width
        web_z = t + spec.web_height / 2
        top_z = t + spec.web_height
        # Surface z=0 is the deck top. The lower ground plane is at -thickness.
        box("deck", (0.0, 0.0, -t / 2), (deck_x, deck_y, t / 2), "deck")

        # Horizontal ribs are continuous across the assembly. Adjacent cells share them.
        for row in range(spec.rows + 1):
            y = -hy + row * spec.pitch
            box(f"row_{row}_flange", (0.0, y, t / 2), (hx + flange / 2, flange / 2, t / 2), "flange")
            box(f"row_{row}_web", (0.0, y, web_z), (hx + t / 2, t / 2, spec.web_height / 2), "web")
            box(f"row_{row}_cut", (0.0, y, top_z), (hx + t / 2, t / 2, 0.00001), "cut", collide=False)

        # Vertical webs butt against the horizontal webs; their foot flanges stop
        # at the horizontal flange edge, so junctions have no overlapping plates.
        for column in range(spec.columns + 1):
            x = -hx + column * spec.pitch
            for row in range(spec.rows):
                y = -hy + (row + 0.5) * spec.pitch
                tag = f"column_{column}_segment_{row}"
                box(f"{tag}_flange", (x, y, t / 2),
                    (flange / 2, (spec.pitch - flange) / 2, t / 2), "flange")
                box(f"{tag}_web", (x, y, web_z),
                    (t / 2, spec.clear_size / 2, spec.web_height / 2), "web")
                box(f"{tag}_cut", (x, y, top_z), (t / 2, spec.clear_size / 2, 0.00001), "cut", collide=False)

    def apply_display(self, builder):
        super().apply_display(builder)
        colors = {"deck": (0.40, 0.46, 0.47), "flange": (0.52, 0.60, 0.59),
                  "web": (0.18, 0.25, 0.23), "cut": (0.68, 0.76, 0.75)}
        finishes = {
            "deck": Finish(metallic=0.40, roughness=0.80, texture=str(steel_texture()), uv_scale=(3.0, 3.0)),
            "flange": Finish(metallic=0.50, roughness=0.72, texture=str(steel_texture()), uv_scale=(8.0, 8.0)),
            "web": Finish(metallic=0.50, roughness=0.72, texture=str(steel_texture("upright")), uv_scale=(8.0, 8.0)),
            "cut": Finish(metallic=0.75, roughness=0.40),
        }
        for kind, indices in self.parts.items():
            for index in indices:
                builder.shape_color[index] = colors[kind]
                self.finishes[index] = finishes[kind]
