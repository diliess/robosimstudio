"""Parallel per-segment full pose audit; roll alternatives are explicit task data.

Workers run CPU numeric solvers only. No live robot motion is generated here.
"""

import copy
import hashlib
import json
import multiprocessing as mp
import time

import numpy as np
from hall_geometry import ROOT, Hall, panel_configs
from scipy.spatial.transform import Rotation as R
from validate_scene import Audit, write_json

AUDIT = None


def segment_job(job):
    x = AUDIT
    seam_id, seg_id, preferred, *options = job
    roll_deltas = options[0] if options else (0, 90, 180, 270, 45, 135, 225, 315)
    source = next(s for s in x.seams if s["id"] == seam_id)
    seg = next(g for g in source["segments"] if g["id"] == seg_id)
    lo, hi = seg["sample_start"], seg["sample_end"] + 1
    local = {**source}
    for k in ("world_points_m", "world_surface_normals", "world_tangent"):
        local[k] = source[k][lo:hi]
    candidates = []
    for old in preferred:
        candidates.append(old)
        oldmid = np.mean(np.asarray(old.get("desired_tcp_points_m", source["world_tcp_points_m"]))[1:-1], axis=0)
        newmid = np.mean(np.array(source["world_tcp_points_m"])[lo:hi], axis=0)
        shift = newmid - oldmid
        candidates.append(
            {**old, "xy_yaw": [old["xy_yaw"][0] + shift[0], old["xy_yaw"][1] + shift[1], old["xy_yaw"][2]]}
        )
    candidates += x.candidate_stances(local)
    unique = []
    seen = set()
    for c in candidates:
        key = tuple(round(v, 4) for v in c["xy_yaw"])
        if key not in seen:
            unique.append(c)
            seen.add(key)
    x.extra_seeds = [
        np.array(q)
        for v in preferred
        if v.get("joint_trajectory_rad")
        for q in (v["joint_trajectory_rad"][-2], v["joint_trajectory_rad"][0])
    ]
    attempts = 0
    # Try a broad set of physical stance/roll combinations near the nominal
    # geometry first. A different roll is stored per segment, never silently used.
    for group in (unique[:45], unique[45:]):
        for delta in roll_deltas:
            s = copy.deepcopy(source)
            g = copy.deepcopy(seg)
            g["world_tcp_quat_xyzw"] = (
                (R.from_quat(seg["world_tcp_quat_xyzw"]) * R.from_euler("z", delta, degrees=True)).as_quat().tolist()
            )
            for c in group:
                calves = list(dict.fromkeys([c.get("calf_rad", -2.0), *x.c["robot"]["stance_calves_rad"]]))
                for calf in calves:
                    attempts += 1
                    v = x.check_segment(s, g, c, calf)
                    if v:
                        v["torch_roll_deg"] = float(seg["torch_roll_deg"] + delta)
                        v["candidate_search_attempts"] = attempts
                        return {"seam_id": seam_id, "segment_id": seg_id, "verified": v, "attempts": attempts}
    return {
        "seam_id": seam_id,
        "segment_id": seg_id,
        "verified": None,
        "attempts": attempts,
        "last_failure": x.last_fail,
    }


def complete(previous=None, workers=4):
    global AUDIT  # noqa: PLW0603 -- fork workers inherit one prepared MuJoCo world
    started = time.time()
    AUDIT = Audit()
    x = AUDIT
    if previous is None:
        previous = json.loads((x.out / "validation.json").read_text())
    jobs = []
    records = []
    for r in previous["representatives"]:
        first = r.get("verified")
        if not first:
            continue
        seam = next((s for s in x.seams if s["id"] == first["seam_id"]), None)
        if seam is None:
            seam = next(
                s for s in x.seams if s["workpiece"] == r["workpiece"] and not s["access"].startswith("requires")
            )
        good = [v["verified"] for v in r.get("coverage", []) if v.get("verified")]
        preferred = [*good, first]
        for seg in seam["segments"]:
            exact = [v for v in preferred if v["segment_id"] == seg["id"]]
            jobs.append((seam["id"], seg["id"], exact or preferred[:2]))
        records.append(
            {
                "workpiece": r["workpiece"],
                "family": x.hall.objects[r["workpiece"]]["kind"],
                "seam_id": seam["id"],
                "candidates": x.candidate_stances(seam)[:18],
                "coverage": [],
            }
        )
    with mp.get_context("fork").Pool(workers) as pool:
        for result in pool.imap_unordered(segment_job, jobs):
            rec = next(r for r in records if r["seam_id"] == result["seam_id"])
            rec["coverage"].append(result)
            print(result["segment_id"], "PASS" if result["verified"] else "UNRESOLVED", result["attempts"], flush=True)
            write_json(ROOT / "run/coverage_progress.json", {"records": records})
    # Persist every chosen roll into the configuration. Each seam retains its
    # continuous geometry; retract/refold boundaries allow a different tool roll.
    c = x.c
    for rec in records:
        station = next((s for s in [*c["stations"], *panel_configs(c)] if s["id"] == rec["workpiece"]), None)
        if station:
            profile = station.setdefault("segment_torch_roll_deg", {})
            for item in rec["coverage"]:
                if item["verified"]:
                    profile[item["segment_id"].split(".", 1)[1]] = item["verified"]["torch_roll_deg"] % 360
        rec["coverage"].sort(key=lambda v: v["segment_id"])
        rec["verified"] = next((v["verified"] for v in rec["coverage"] if v["verified"]), None)
        rec["whole_representative_seam_verified"] = all(v["verified"] for v in rec["coverage"])
    (ROOT / "config.json").write_text(json.dumps(c, indent=2))
    x.hall = Hall(c)
    x.seams = x.hall.world_seams()
    x.hall.export()
    x.report["config_sha256"] = hashlib.sha256((ROOT / "config.json").read_bytes()).hexdigest()
    x.report["geometry_sha256"] = hashlib.sha256((x.out / "scene_geometry.json").read_bytes()).hexdigest()
    x.report["seams_sha256"] = hashlib.sha256((x.out / "seams.json").read_bytes()).hexdigest()
    x.report["representatives"] = records
    x.report["folded_envelope"] = x.envelope()
    x.report["navigation"] = x.map()
    from robot_initial import apply_initial_pose

    x.report["initial_pose"] = apply_initial_pose(x.m, x.d, c["robot"], world=True)
    bad, w = x.collision()
    x.report["initial_contacts"] = {
        "unexpected_penetrations": bad,
        "worst_m": -w,
        "mass_kg": float(x.m.body_mass.sum()),
    }
    x.report["all_families_verified"] = len(records) == sum(
        o.get("zone") in ("A", "B") for o in x.hall.objects.values()
    ) and all(r["verified"] for r in records)
    x.report["all_representative_seams_fully_covered"] = len(records) == sum(
        o.get("zone") in ("A", "B") for o in x.hall.objects.values()
    ) and all(r["whole_representative_seam_verified"] for r in records)
    x.report["elapsed_s"] = time.time() - started
    write_json(x.out / "validation.json", x.report)
    from export_data import export

    export(x)
    return x.report


if __name__ == "__main__":
    complete()
