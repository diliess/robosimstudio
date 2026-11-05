"""Metrology of the delivered solids: no simulator state or scene.py execution."""

import itertools

import numpy as np
from hall_geometry import ROOT, Hall
from scipy.optimize import linprog
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation as R
from validate_scene import write_json


def vertices(part):
    if part["type"] == "convex":
        return np.array(part["vertices"])
    half = np.array(part["size"]) / 2
    return R.from_quat(part["quat"]).apply(np.array(list(itertools.product(*[[-v, v] for v in half])))) + part["center"]


def audit():
    hall = Hall()
    overlaps = []
    root_errors = []
    for o in hall.objects.values():
        parts = [p for p in o["parts"] if p["role"] == "steel" and p["collision"]]
        solids = []
        for p in parts:
            v = vertices(p)
            eq = ConvexHull(v).equations
            solids.append((p["name"], v, eq))
        for (an, av, ae), (bn, bv, be) in itertools.combinations(solids, 2):
            if np.any(np.minimum(av.max(0), bv.max(0)) - np.maximum(av.min(0), bv.min(0)) < -0.00001):
                continue
            eq = np.vstack([ae, be])
            A = np.column_stack([eq[:, :3], np.ones(len(eq))])
            result = linprog(
                [0, 0, 0, -1], A_ub=A, b_ub=-eq[:, 3], bounds=[(None, None)] * 3 + [(0, 1)], method="highs"
            )
            if result.success and result.x[3] > 0.00005:
                overlaps.append(
                    {"workpiece": o["id"], "parts": [an, bn], "intersection_inscribed_radius_m": float(result.x[3])}
                )
        for s in [s for s in hall.seams if s["workpiece"] == o["id"]]:
            pts = np.array(s["local_points_m"])
            errors = []
            for p in pts:
                errors.append(min(abs(np.max(eq[:, :3] @ p + eq[:, 3])) for _, _, eq in solids))
            root_errors.append({"seam_id": s["id"], "max_distance_to_steel_boundary_m": float(max(errors))})
    maxerror = max(r["max_distance_to_steel_boundary_m"] for r in root_errors)
    result = {
        "units": "m",
        "steel_plate_solid_intersections": overlaps,
        "root_boundary_max_error_m": maxerror,
        "root_boundary_tolerance_m": 0.0011,
        "root_checks": root_errors,
        "heights": [
            {
                "workpiece": s["workpiece"],
                "seam": s["id"],
                "table_height_m": s["table_height_m"],
                "actual_seam_height_m": s["seam_height_range_m"],
            }
            for s in hall.world_seams()
        ],
        "pass": not overlaps and maxerror <= 0.0011,
    }
    write_json(ROOT / "exports/geometry_audit.json", result)
    print({k: v for k, v in result.items() if k not in ("root_checks", "heights")})
    return result


if __name__ == "__main__":
    audit()
