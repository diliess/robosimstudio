"""Offline geometry/6D IK/collision audit. Does not run scene.py or move live bodies.

CPU MuJoCo uses the SAME supplied robot and exported physical fixture parts.
A verified segment includes approach, every seam sample, exit and interpolated
joint paths. This is kinematic feasibility, not a gait or process certificate.
"""

from __future__ import annotations

import hashlib
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco as mj
import numpy as np
from hall_geometry import ROOT, Hall, panel_configs
from prepare_assets import ARM_HOME, ARM_JOINTS, LEG_HOME
from robot_assets import prepare_robot
from robot_initial import apply_initial_pose
from scipy.ndimage import distance_transform_edt, label
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R


def numbers(v):
    return " ".join(f"{float(x):.12g}" for x in v)


def wxyz(q):
    return [q[3], *q[:3]]


def write_json(path, value):
    temp = path.with_suffix(".tmp")
    temp.write_text(
        json.dumps(value, indent=2, default=lambda x: x.tolist() if isinstance(x, np.ndarray) else x.item())
    )
    temp.replace(path)


class Audit:
    def __init__(self):
        self.hall = Hall()
        self.c = self.hall.config
        self.out = self.hall.export()
        model, _, _ = prepare_robot(self.c["robot"])
        tree = ET.parse(model)
        root = tree.getroot()
        # Assets stay beside the source snapshot, so relative paths are portable.
        world = root.find("worldbody")
        asset = root.find("asset")
        ET.SubElement(world, "geom", name="ground", type="plane", size="20 20 .1", friction=".75 .005 .0001")
        self.env_names = set()
        for p in self.hall.world_parts():
            if not p["collision"] or p["role"] == "floor":
                continue
            name = p["name"]
            self.env_names.add(name)
            attrs = {"name": name, "friction": ".75 .005 .0001"}
            if p["type"] == "box":
                attrs.update(
                    type="box",
                    pos=numbers(p["center"]),
                    size=numbers(np.array(p["size"]) / 2),
                    quat=numbers(wxyz(p["quat"])),
                )
            else:
                mesh = "fixture_mesh_" + name
                ET.SubElement(
                    asset,
                    "mesh",
                    name=mesh,
                    vertex=numbers(np.array(p["vertices"]).flatten()),
                    face=numbers(np.array(p["faces"]).flatten()),
                )
                attrs.update(type="mesh", mesh=mesh)
            ET.SubElement(world, "geom", **attrs)
        # Static/static pairs do not generate contacts. Preserve source adjacent-body filters.
        self.file = Path(model).with_name("validation_world.xml")
        tree.write(self.file, encoding="unicode")
        self.m = mj.MjModel.from_xml_path(str(self.file))
        self.d = mj.MjData(self.m)
        self.site = mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_SITE, "welding_tcp")
        self.arm_ids = [mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_JOINT, n) for n in ARM_JOINTS]
        self.aq = self.m.jnt_qposadr[self.arm_ids]
        self.av = self.m.jnt_dofadr[self.arm_ids]
        self.bounds = self.m.jnt_range[self.arm_ids].T
        self.base = mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_BODY, "base_link")
        self.robot_geoms = np.flatnonzero(self.m.geom_bodyid != 0)
        self.foot_geoms = [
            i
            for i in self.robot_geoms
            if self.m.geom_type[i] == mj.mjtGeom.mjGEOM_SPHERE and self.m.geom_size[i, 0] > 0.03
        ]
        self.rng = np.random.default_rng(42)
        self.seams = self.hall.world_seams()
        self.report = {
            "config_sha256": hashlib.sha256((ROOT / "config.json").read_bytes()).hexdigest(),
            "schema_version": 1,
            "source": "exact robot MJCF + hall physical geometry",
            "scope": "kinematic full TCP pose, sampled collision, four feet supported; no gait/dynamic welding validation",
            "tolerances": {
                "position_m": 0.002,
                "orientation_deg": 1.0,
                "allowed_penetration_m": 0.0005,
                "joint_interpolation_max_rad": 0.06,
            },
            "representatives": [],
        }

    def fk(self):
        mj.mj_kinematics(self.m, self.d)
        mj.mj_comPos(self.m, self.d)

    def stance(self, x, y, yaw, calf, surface=0):
        self.d.qpos[:] = self.m.qpos0
        self.d.qpos[:3] = [x, y, 0]
        self.d.qpos[3:7] = wxyz(R.from_euler("z", yaw).as_quat())
        for name, home_value in LEG_HOME.items():
            v = home_value
            if "calf" in name:
                v = calf
            elif "thigh" in name:
                v = -calf / 2
            j = mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_JOINT, name)
            self.d.qpos[self.m.jnt_qposadr[j]] = v
        self.d.qpos[self.aq] = self.c["robot"].get("transport_arm_q_rad", ARM_HOME)
        self.fk()
        bottoms = np.array([self.d.geom_xpos[i, 2] - self.m.geom_size[i, 0] for i in self.foot_geoms])
        self.d.qpos[2] += surface - float(bottoms.min()) + 0.0001
        self.fk()
        return float(self.d.qpos[2])

    def collision(self):
        mj.mj_forward(self.m, self.d)
        worst = 0.0
        bad = []
        for c in self.d.contact:
            if c.dist >= -0.0005:
                continue
            a, b = int(c.geom1), int(c.geom2)
            ba, bb = self.m.geom_bodyid[[a, b]]
            if ba == 0 and bb == 0:
                continue
            # Expected contacts of feet on floor/deck are permitted only within .5 mm.
            names = [mj.mj_id2name(self.m, mj.mjtObj.mjOBJ_GEOM, g) or f"robot_geom_{g}" for g in (a, b)]
            worst = min(worst, float(c.dist))
            bad.append({"pair": names, "penetration_m": float(-c.dist)})
        return bad, worst

    def solve(self, p, q, seed):
        target = R.from_quat(q)

        def residual(x):
            self.d.qpos[self.aq] = x
            self.fk()
            return np.r_[
                self.d.site_xpos[self.site] - p,
                0.32 * (target * R.from_matrix(self.d.site_xmat[self.site].reshape(3, 3)).inv()).as_rotvec(),
            ]

        result = least_squares(
            residual,
            np.clip(seed, self.bounds[0] + 1e-7, self.bounds[1] - 1e-7),
            bounds=self.bounds,
            ftol=1e-9,
            xtol=1e-9,
            gtol=1e-9,
            max_nfev=90,
        )
        error = residual(result.x)
        return result.x, float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]) / 0.32)

    def feet_supported(self):
        # All feet must lie on the same selected support surface, inside hall and
        # outside upright ribs/table footprints; collision catches side interference.
        feet = self.d.geom_xpos[self.foot_geoms].copy()
        for p in feet:
            if np.any(np.abs(p[:2]) > np.array(self.c["hall"]["size_m"]) / 2 - 0.2):
                return False
            surface = 0.0
            for mod in panel_configs(self.c):
                if np.all(np.abs(p[:2] - mod["xy"]) < np.array(mod["size_m"]) / 2 - 0.032):
                    surface = mod["plate_thickness_m"]
            if abs(p[2] - 0.032 - surface) > 0.001:
                return False
        return True

    def envelope(self):
        self.stance(0, 0, 0, -2.35)
        mins = []
        maxs = []
        for i in self.robot_geoms:
            aabb = self.m.geom_aabb[i]
            rot = self.d.geom_xmat[i].reshape(3, 3)
            mid = self.d.geom_xpos[i] + rot @ aabb[:3]
            extent = np.abs(rot) @ aabb[3:]
            mins.append(mid - extent)
            maxs.append(mid + extent)
        lo = np.min(mins, 0)
        hi = np.max(maxs, 0)
        return {
            "reference": "configured compact transport pose, calf=-2.35, all visual + collision geometry conservative AABB",
            "min_m": lo,
            "max_m": hi,
            "size_m": hi - lo,
            "circumscribed_xy_radius_m": float(
                max(np.linalg.norm(np.array([x, y])) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]))
            ),
        }

    def map(self):
        import imageio.v3 as iio

        c = self.c["map"]
        res = c["resolution_m"]
        sx, sy = self.c["hall"]["size_m"]
        x = np.arange(-sx / 2 + res / 2, sx / 2, res)
        y = np.arange(-sy / 2 + res / 2, sy / 2, res)
        xx, yy = np.meshgrid(x, y)
        xy = np.stack([xx, yy], -1)
        occ = np.zeros(xx.shape, bool)
        obstacles = []
        for p in self.hall.world_parts():
            if not p["collision"] or p["role"] in ("floor", "ramp"):
                continue
            if p["type"] == "box":
                half = np.array(p["size"]) / 2
                r = R.from_quat(p["quat"])
                local = np.array(
                    [[a, b, d] for a in (-half[0], half[0]) for b in (-half[1], half[1]) for d in (-half[2], half[2])]
                )
                verts = r.apply(local) + p["center"]
            else:
                verts = np.array(p["vertices"])
            # Elevation filter includes the folded arm, rather than just B2's body.
            if verts[:, 2].max() <= 0.025 or verts[:, 2].min() > c["folded_height_m"]:
                continue
            from scipy.spatial import ConvexHull

            hull = ConvexHull(verts[:, :2])
            poly = verts[hull.vertices, :2]
            eq = hull.equations
            mask = np.all(
                xy.reshape(-1, 2) @ eq[:, :2].T + eq[:, 2] <= (res / 2 * np.sum(np.abs(eq[:, :2]), axis=1) + 0.001),
                axis=1,
            )
            occ |= mask.reshape(xx.shape)
            obstacles.append(
                {
                    "name": p["name"],
                    "polygon_xy_m": poly.tolist(),
                    "z_range_m": [float(verts[:, 2].min()), float(verts[:, 2].max())],
                }
            )
        occ[[0, -1], :] = True
        occ[:, [0, -1]] = True
        clear = distance_transform_edt(~occ) * res
        radius = max(c["envelope_radius_m"], self.envelope()["circumscribed_xy_radius_m"] + 0.04)
        free = (~occ) & (clear >= radius)
        labels, n = label(free)

        def cell(p):
            return (
                int(np.clip((p[1] + sy / 2) / res, 0, len(y) - 1)),
                int(np.clip((p[0] + sx / 2) / res, 0, len(x) - 1)),
            )

        start = self.c["robot"]["start_xy_yaw"][:2]
        component = int(labels[cell(start)])
        targets = []
        for o in self.hall.objects.values():
            if o.get("zone") not in ("A", "B"):
                continue
            center = np.array(o["pose"][:2])
            candidates = np.column_stack([xx[free], yy[free]])
            index = np.argmin(np.linalg.norm(candidates - center, axis=1))
            goal = candidates[index]
            targets.append(
                {
                    "workpiece": o["id"],
                    "goal_xy_m": goal.tolist(),
                    "connected": component > 0
                    and int(labels[cell(goal)]) == component
                    and np.linalg.norm(goal - center) < (2.2 if o["zone"] == "A" else 3.5),
                    "distance_to_workpiece_origin_m": float(np.linalg.norm(goal - center)),
                    "status": "navigation approach only; welding stance checked separately",
                }
            )
        pixels = np.where(occ, 0, 254).astype(np.uint8)
        iio.imwrite(self.out / "map.pgm", np.flipud(pixels))
        iio.imwrite(self.out / "map_inflated.pgm", np.flipud(np.where(free, 254, 0).astype(np.uint8)))
        (self.out / "map.yaml").write_text(
            f"image: map.pgm\nresolution: {res}\norigin: [{-sx / 2}, {-sy / 2}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
        )
        result = {
            "source": "ground truth geometry, NOT SLAM",
            "resolution_m": res,
            "origin_m": [-sx / 2, -sy / 2, 0],
            "inflation_radius_m": radius,
            "height_filter_m": c["folded_height_m"],
            "connected_start_component": component,
            "component_count": n,
            "goals": targets,
            "obstacles": obstacles,
            "all_major_areas_connected": component > 0 and all(t["connected"] for t in targets),
        }
        write_json(self.out / "navigation.json", result)
        self.map_free = free
        self.map_labels = labels
        self.map_component = component
        self.map_cell = cell
        return {k: v for k, v in result.items() if k != "obstacles"}

    def candidate_stances(self, s):
        points = np.array(s["world_points_m"])
        mid = points[len(points) // 2]
        normal = np.array(s["world_surface_normals"])[len(points) // 2].sum(0)
        normal[2] = 0
        if np.linalg.norm(normal) < 1e-8:
            normal = np.array([0.0, -1.0, 0])
        normal /= np.linalg.norm(normal)
        candidates = []
        for offset in (0, -0.25, 0.25, -0.5, 0.5, -0.75, 0.75, -1.0, 1.0):
            tangent = np.array(s["world_tangent"])[len(points) // 2]
            tangent[2] = 0
            for distance in (0.75, 0.60, 0.90, 1.05, 0.45, 1.20):
                p = mid + normal * distance + tangent * offset
                surface = 0.0
                for mod in panel_configs(self.c):
                    local = p[:2] - mod["xy"]
                    half = np.array(mod["size_m"]) / 2
                    if np.all(np.abs(local) < half):
                        surface = mod["plate_thickness_m"]
                yaw = np.arctan2(mid[1] - p[1], mid[0] - p[0])
                # Actual full geometry determines working stance feasibility;
                # conservative navigation disk is for folded travel only.
                for dyaw in (0, 0.8, -0.8, np.pi / 2, -np.pi / 2):
                    candidates.append(
                        {
                            "xy_yaw": [float(p[0]), float(p[1]), float(yaw + dyaw)],
                            "support_surface_m": surface,
                            "status": "candidate_unverified",
                        }
                    )
        return candidates

    def access_region(self, candidate, calf):
        self.stance(*candidate["xy_yaw"], calf, candidate["support_surface_m"])
        if self.collision()[0] or not self.feet_supported():
            return None
        lo = []
        hi = []
        for i in self.robot_geoms:
            a = self.m.geom_aabb[i]
            rot = self.d.geom_xmat[i].reshape(3, 3)
            p = self.d.geom_xpos[i] + rot @ a[:3]
            e = np.abs(rot) @ a[3:]
            lo.append(p - e)
            hi.append(p + e)
        low = np.min(lo, 0)
        high = np.max(hi, 0)
        mode = "open_floor"
        for mod in panel_configs(self.c):
            center = np.array(mod["xy"])
            half = np.array(mod["size_m"]) / 2
            xy = np.array(candidate["xy_yaw"][:2]) - center
            if np.all(np.abs(xy) < half):
                w = mod.get("entry_width_m", mod.get("cross_lane_width_m")) / 2 - 0.012
                bx = max(abs(low[0] - center[0]), abs(high[0] - center[0]))
                by = max(abs(low[1] - center[1]), abs(high[1] - center[1]))
                if bx > w and ("cross_lane_width_m" not in mod or by > w):
                    return None
                mode = "entry_cross_lane"
        return mode

    def check_segment(self, s, seg, candidate, calf):
        access_mode = self.access_region(candidate, calf)
        if access_mode is None:
            self.last_fail = "folded access blocked or outside permitted lane"
            return None
        a, b = seg["sample_start"], seg["sample_end"]
        pos = np.array(seg.get("world_tcp_points_m", s["world_tcp_points_m"][a : b + 1]))
        quat = np.array(seg.get("world_tcp_quat_xyzw", s["world_tcp_quat_xyzw"][a : b + 1]))
        d = self.c["welding"]["approach_distance_m"]
        z0 = R.from_quat(quat[0]).apply([0, 0, 1])
        z1 = R.from_quat(quat[-1]).apply([0, 0, 1])
        positions = np.vstack([pos[0] - z0 * d, pos, pos[-1] - z1 * d])
        quats = np.vstack([quat[0], quat, quat[-1]])
        height = self.stance(*candidate["xy_yaw"], calf, candidate["support_surface_m"])
        seeds = [
            *list(getattr(self, "extra_seeds", [])),
            np.array([0, 1, 1, 0, -1, 0]),
            np.array([0, -0.41, -2.15, 0, -1.61, 3.12]),
            np.array(ARM_HOME),
        ]
        seeds += [self.rng.uniform(self.bounds[0] * 0.7, self.bounds[1] * 0.7) for _ in range(2)]
        self.last_fail = "approach IK/collision"
        for seed in seeds:
            best, pe, ae = self.solve(positions[0], quats[0], seed)
            if pe > 0.002 or ae > np.deg2rad(1) or self.collision()[0]:
                continue
            prev = best.copy()
            solutions = []
            maxpos = 0
            maxang = 0
            worst = 0
            failed = False
            for p, q in zip(positions, quats, strict=False):
                sol, pe, ae = self.solve(p, q, prev)
                if pe > 0.002 or ae > np.deg2rad(1):
                    self.last_fail = f"path IK: {pe:.4f}m {ae:.4f}rad"
                    failed = True
                    break
                n = max(2, int(np.ceil(np.max(np.abs(sol - prev)) / 0.06)) + 1)
                for blend in np.linspace(0, 1, n):
                    self.d.qpos[self.aq] = prev + (sol - prev) * blend
                    bad, w = self.collision()
                    worst = min(worst, w)
                    if bad:
                        self.last_fail = str(bad[:2])
                        failed = True
                        break
                if failed:
                    break
                if not self.feet_supported():
                    failed = True
                    self.last_fail = "unsupported feet"
                    break
                solutions.append(sol.tolist())
                prev = sol
                maxpos = max(maxpos, pe)
                maxang = max(maxang, ae)
            if failed:
                continue
            return {
                **candidate,
                "geometric_access_mode": access_mode,
                "folded_pose_collision_checked": True,
                "status": "verified_full_pose_sampled_collision",
                "base_height_m": height,
                "calf_rad": calf,
                "thigh_rad": -calf / 2,
                "seam_id": s["id"],
                "segment_id": seg["id"],
                "sample_start": a,
                "sample_end": b,
                "max_position_error_m": maxpos,
                "max_orientation_error_deg": float(np.rad2deg(maxang)),
                "worst_penetration_m": -worst,
                "joint_trajectory_rad": solutions,
                "desired_tcp_points_m": positions.tolist(),
                "desired_tcp_quat_xyzw": quats.tolist(),
                "includes_approach_exit": True,
                "poses_checked": len(positions),
                "feet_support": "four source foot spheres on same deck/floor; no stepping over ribs",
                "dynamic_status": "not validated",
            }
        return None

    def run(self):
        started = time.time()
        self.report["folded_envelope"] = self.envelope()
        self.report["navigation"] = self.map()
        print("Envelope:", self.report["folded_envelope"], flush=True)
        self.report["initial_pose"] = apply_initial_pose(self.m, self.d, self.c["robot"], world=True)
        bad, worst = self.collision()
        self.report["initial_contacts"] = {
            "unexpected_penetrations": bad,
            "worst_m": -worst,
            "mass_kg": float(self.m.body_mass.sum()),
        }
        for o in self.hall.objects.values():
            if o.get("zone") not in ("A", "B"):
                continue
            ss = [
                s
                for s in self.seams
                if s["workpiece"] == o["id"]
                and s["category"] not in ("rib_end", "intersection_fillet")
                and not s["access"].startswith("restricted")
            ]
            preferred = next((st.get("representative_seam") for st in self.c["stations"] if st["id"] == o["id"]), None)
            if preferred:
                ss.sort(key=lambda seam: seam["id"] != o["id"] + "." + preferred)
            record = {"workpiece": o["id"], "family": o["kind"], "candidates": [], "verified": None, "attempts": 0}
            # Prefer root sections adjacent to an entry/perimeter; preserve all others
            # as unverified rather than claiming all points of a grid reachable.
            for s in ss:
                if record["verified"]:
                    break
                segments = sorted(
                    s["segments"],
                    key=lambda seg: abs(
                        (seg["sample_start"] + seg["sample_end"]) / 2 - (len(s["world_points_m"]) - 1) / 2
                    ),
                )
                for seg in segments[:3]:
                    # Generate stances at this segment center, not the whole long seam.
                    local = {**s}
                    lo, hi = seg["sample_start"], seg["sample_end"] + 1
                    for k in ("world_points_m", "world_surface_normals", "world_tangent"):
                        local[k] = s[k][lo:hi]
                    candidates = self.candidate_stances(local)
                    record["candidates"] = candidates[:15]
                    for cand in candidates:
                        if record["verified"]:
                            break
                        for calf in sorted(self.c["robot"]["stance_calves_rad"], key=lambda c: abs(c + 2.0)):
                            record["attempts"] += 1
                            verified = self.check_segment(s, seg, cand, calf)
                            if verified:
                                record["verified"] = verified
                                break
                    if record["verified"]:
                        break
            self.report["representatives"].append(record)
            print(
                o["id"],
                o["kind"],
                "PASS" if record["verified"] else "UNRESOLVED",
                "attempts",
                record["attempts"],
                flush=True,
            )
            write_json(self.out / "validation.json", self.report)
        self.report["all_families_verified"] = all(r["verified"] for r in self.report["representatives"])
        self.report["elapsed_s"] = time.time() - started
        write_json(self.out / "validation.json", self.report)
        print("Done", self.report["all_families_verified"], round(self.report["elapsed_s"], 1), "s", flush=True)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--fresh", action="store_true", help="rediscover representative seams before coverage")
    args = parser.parse_args()
    if args.fresh or not (ROOT / "exports/validation.json").exists():
        Audit().run()
    from coverage_validation import complete

    complete()
    from geometry_audit import audit

    audit()
