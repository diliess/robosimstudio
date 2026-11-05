"""Expanded hall geometry/layout checks, without executing scene.py."""

from __future__ import annotations

import itertools
import json
from collections import Counter

import numpy as np
from geometry_audit import vertices
from hall_geometry import ROOT, Hall, panel_configs, transform
from scipy.optimize import linprog
from scipy.spatial import ConvexHull
from validate_scene import Audit, write_json


def validate_layout():
    hall = Hall()
    previous = ROOT / "run/before_expansion_20261102/config.json"
    old = Hall(json.loads(previous.read_text())) if previous.exists() else None

    def totals(h):
        return {
            zone: {
                "workpieces": sum(o.get("zone") == zone for o in h.objects.values()),
                "seams": sum(h.objects[s["workpiece"]].get("zone") == zone for s in h.seams),
                "segments": sum(len(s["segments"]) for s in h.seams if h.objects[s["workpiece"]].get("zone") == zone),
                "length_m": sum(s["length_m"] for s in h.seams if h.objects[s["workpiece"]].get("zone") == zone),
            }
            for zone in ("A", "B")
        }

    bounds = {}
    outside = []
    half = np.array(hall.config["hall"]["size_m"]) / 2
    for o in hall.objects.values():
        if o["id"] == "hall":
            continue
        points = np.vstack([transform(vertices(p), o["pose"]) for p in o["parts"] if p["collision"]])
        bounds[o["id"]] = (points.min(0), points.max(0))
        if np.any(points[:, :2].min(0) < -half) or np.any(points[:, :2].max(0) > half):
            outside.append(o["id"])

    # First reject by assembly AABBs. Only nearby inter-assembly pairs need LP.
    intersections = []
    for a, b in itertools.combinations(bounds, 2):
        alo, ahi = bounds[a]
        blo, bhi = bounds[b]
        if np.any(np.minimum(ahi, bhi) - np.maximum(alo, blo) <= 0.00005):
            continue
        for pa in hall.objects[a]["parts"]:
            if not pa["collision"]:
                continue
            av = transform(vertices(pa), hall.objects[a]["pose"])
            ae = ConvexHull(av).equations
            for pb in hall.objects[b]["parts"]:
                if not pb["collision"]:
                    continue
                bv = transform(vertices(pb), hall.objects[b]["pose"])
                if np.any(np.minimum(av.max(0), bv.max(0)) - np.maximum(av.min(0), bv.min(0)) <= 0.00005):
                    continue
                eq = np.vstack([ae, ConvexHull(bv).equations])
                result = linprog(
                    [0, 0, 0, -1],
                    A_ub=np.column_stack([eq[:, :3], np.ones(len(eq))]),
                    b_ub=-eq[:, 3],
                    bounds=[(None, None)] * 3 + [(0, 1)],
                    method="highs",
                )
                if result.success and result.x[3] > 0.00005:
                    intersections.append([pa["name"], pb["name"], float(result.x[3])])
    gaps = []
    ids = [o["id"] for o in hall.objects.values() if o.get("zone") == "A"]
    for a, b in itertools.combinations(ids, 2):
        alo, ahi = bounds[a]
        blo, bhi = bounds[b]
        gap = np.maximum(np.maximum(alo[:2] - bhi[:2], blo[:2] - ahi[:2]), 0)
        gaps.append({"stations": [a, b], "aabb_gap_m": float(np.linalg.norm(gap))})
    removed = [p["name"] for o in hall.objects.values() for p in o["parts"] if p["name"].endswith("_brace")]
    station_types = Counter(s["kind"] for s in hall.config["stations"])
    levels = {
        kind: sorted(
            {
                round(s["table_height_m"] + s["plate_thickness_m"], 3)
                for s in hall.config["stations"]
                if s["kind"] == kind
            }
        )
        for kind in ("t_joint", "s_curve")
    }
    butt_angles = {st["id"]: st["incline_deg"] for st in hall.config["stations"] if st["kind"] == "butt_plate"}
    nav = json.loads((ROOT / "exports/navigation.json").read_text())
    robot = Audit()
    folded_options = []
    for calf in hall.config["robot"]["stance_calves_rad"]:
        height = robot.stance(0, 0, 0, calf)
        lo, hi = [], []
        for i in robot.robot_geoms:
            a = robot.m.geom_aabb[i]
            rot = robot.d.geom_xmat[i].reshape(3, 3)
            center = robot.d.geom_xpos[i] + rot @ a[:3]
            extent = np.abs(rot) @ a[3:]
            lo.append(center - extent)
            hi.append(center + extent)
        low, high = np.min(lo, 0), np.max(hi, 0)
        radius = float(np.hypot(np.maximum(abs(low[0]), abs(high[0])), np.maximum(abs(low[1]), abs(high[1]))))
        folded_options.append(
            {"calf_rad": calf, "base_height_m": height, "folded_top_z_m": float(high[2]), "folded_xy_radius_m": radius}
        )
    folded_clearance_ok = (
        max(v["folded_top_z_m"] for v in folded_options) <= hall.config["map"]["folded_height_m"]
        and max(v["folded_xy_radius_m"] for v in folded_options) + 0.04 <= hall.config["map"]["envelope_radius_m"]
    )
    result = {
        "hall_size_m": hall.config["hall"]["size_m"],
        "folded_options": folded_options,
        "folded_map_clearance_sufficient": folded_clearance_ok,
        "hall_area_m2": float(4 * np.prod(half)),
        "old_counts": totals(old) if old else None,
        "current_counts": totals(hall),
        "A_station_types": dict(station_types),
        "actual_root_height_levels_m": levels,
        "remaining_floating_table_braces": removed,
        "plate_stack_present": "plate_stack" in hall.objects,
        "butt_inclines_deg": butt_angles,
        "A05_kind": hall.objects["A05"]["kind"],
        "outside_floor": outside,
        "inter_assembly_intersections": intersections,
        "minimum_A_table_aabb_gap_m": min(g["aabb_gap_m"] for g in gaps),
        "closest_A_tables": sorted(gaps, key=lambda g: g["aabb_gap_m"])[:8],
        "panel_sizes_m": {p["id"]: p["size_m"] for p in panel_configs(hall.config)},
        "main_aisle_width_m": hall.config["hall"]["main_aisle_m"],
        "navigation_all_major_areas_connected": nav["all_major_areas_connected"],
        "navigation_goal_count": len(nav["goals"]),
        "stance_scope": "table spacing is geometry; each representative stance is checked separately with full robot geometry, folded access and four-foot support",
    }
    result["pass"] = (
        not (removed or result["plate_stack_present"] or outside or intersections)
        and nav["all_major_areas_connected"]
        and all(0 <= angle < 90 for angle in butt_angles.values())
        and folded_clearance_ok
    )
    write_json(ROOT / "exports/layout_validation.json", result)
    print(json.dumps(result, indent=2))
    assert result["pass"]
    return result


if __name__ == "__main__":
    validate_layout()
