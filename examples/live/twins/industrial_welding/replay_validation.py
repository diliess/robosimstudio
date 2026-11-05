"""Replay saved witnesses against final exported poses, contact geometry and support."""

import json

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation as R
from validate_scene import Audit, write_json


def replay():
    x = Audit()
    report = json.loads((x.out / "validation.json").read_text())
    fail = []
    pe_max = 0.0
    ae_max = 0.0
    margin = 1.0
    count = 0
    lookup = {s["id"]: s for s in x.seams}
    for r in report["representatives"]:
        for item in r["coverage"]:
            v = item["verified"]
            if not v:
                fail.append(item["segment_id"])
                continue
            seg = next(g for g in lookup[v["seam_id"]]["segments"] if g["id"] == v["segment_id"])
            pos = np.array(seg["world_tcp_points_m"])
            quat = np.array(seg["world_tcp_quat_xyzw"])
            d = x.c["welding"]["approach_distance_m"]
            pos = np.vstack(
                [pos[0] - R.from_quat(quat[0]).apply([0, 0, d]), pos, pos[-1] - R.from_quat(quat[-1]).apply([0, 0, d])]
            )
            quat = np.vstack([quat[0], quat, quat[-1]])
            x.stance(*v["xy_yaw"], v["calf_rad"], v["support_surface_m"])
            for q, p, rot in zip(v["joint_trajectory_rad"], pos, quat, strict=True):
                x.d.qpos[x.aq] = q
                bad, _ = x.collision()
                limited = np.flatnonzero(x.m.jnt_limited)
                limited_q = x.d.qpos[x.m.jnt_qposadr[limited]]
                limits_ok = bool(
                    np.all(limited_q >= x.m.jnt_range[limited, 0] - 1e-8)
                    and np.all(limited_q <= x.m.jnt_range[limited, 1] + 1e-8)
                )
                pe = float(np.linalg.norm(x.d.site_xpos[x.site] - p))
                ae = float(
                    np.linalg.norm(
                        (R.from_quat(rot) * R.from_matrix(x.d.site_xmat[x.site].reshape(3, 3)).inv()).as_rotvec()
                    )
                )
                hull = ConvexHull(x.d.geom_xpos[x.foot_geoms, :2])
                com = x.d.subtree_com[x.base, :2]
                support_margin = float(-np.max(hull.equations[:, :2] @ com + hull.equations[:, 2]))
                margin = min(margin, support_margin)
                pe_max = max(pe_max, pe)
                ae_max = max(ae_max, ae)
                count += 1
                if (
                    bad
                    or not limits_ok
                    or pe > 0.002
                    or ae > np.deg2rad(1)
                    or support_margin < 0.015
                    or not x.feet_supported()
                ):
                    fail.append(
                        {
                            "segment": item["segment_id"],
                            "position_error_m": pe,
                            "orientation_error_rad": ae,
                            "support_margin_m": support_margin,
                            "contacts": bad,
                            "all_limited_joints_within_source_limits": limits_ok,
                        }
                    )
    result = {
        "pass": not fail,
        "poses_replayed": count,
        "segments": sum(len(r["coverage"]) for r in report["representatives"]),
        "max_position_error_m": pe_max,
        "max_orientation_error_deg": float(np.rad2deg(ae_max)),
        "minimum_projected_com_margin_m": margin,
        "minimum_required_com_margin_m": 0.015,
        "failures": fail,
        "scope": "necessary static support + kinematic/collision checks, not a dynamics or torque certificate",
    }
    write_json(x.out / "witness_replay.json", result)
    print(result)
    return result


if __name__ == "__main__":
    replay()
