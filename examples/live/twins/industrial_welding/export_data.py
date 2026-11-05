"""Build candidate/verified stance index and a human-readable ground-truth map."""

import hashlib
import json

import numpy as np
from hall_geometry import ROOT, panel_configs
from PIL import Image, ImageDraw, ImageFont
from validate_scene import Audit, write_json


def export(audit=None):
    x = audit or Audit()
    report = json.loads((x.out / "validation.json").read_text())
    records = report.get("representatives", [])
    digest = hashlib.sha256((ROOT / "config.json").read_bytes()).hexdigest()
    result = {
        "config_sha256": digest,
        "geometry_sha256": report.get("geometry_sha256"),
        "seams_sha256": report.get("seams_sha256"),
        "status": "candidates are NOT IK or collision validated; verified poses are sampled kinematic results, no gait",
        "seams": {},
    }
    height_options = []
    for calf in x.c["robot"]["stance_calves_rad"]:
        h = x.stance(0, 0, 0, calf)
        height_options.append({"calf_rad": calf, "thigh_rad": -calf / 2, "base_height_above_support_m": h})
    for seam in x.seams:
        candidates = []
        for seg in seam["segments"]:
            lo, hi = seg["sample_start"], seg["sample_end"] + 1
            local = {**seam}
            for k in ("world_points_m", "world_surface_normals", "world_tangent"):
                local[k] = seam[k][lo:hi]
            for c in x.candidate_stances(local)[:9]:
                candidates.append(
                    {
                        **c,
                        "segment_id": seg["id"],
                        "body_height_options": height_options,
                        "access_policy": seam["access"],
                    }
                )
        verified = []
        if report.get("config_sha256") == digest:
            for r in records:
                values = [r.get("verified")] + [v.get("verified") for v in r.get("coverage", [])]
                for v in values:
                    if (
                        v
                        and v["seam_id"] == seam["id"]
                        and all(old["segment_id"] != v["segment_id"] for old in verified)
                    ):
                        verified.append(v)
        result["seams"][seam["id"]] = {"candidates": candidates, "verified": verified}
    write_json(x.out / "stances.json", result)
    nav = x.map()
    free = x.map_free
    raw = np.asarray(Image.open(x.out / "map.pgm"))
    image = np.zeros((*raw.shape, 3), np.uint8)
    image[:] = [67, 75, 80]
    image[raw > 0] = [219, 224, 224]
    image[np.flipud(free)] = [148, 190, 170]
    sx, sy = x.c["hall"]["size_m"]
    height = int(1200 * sy / sx)
    im = Image.fromarray(image).resize((1200, height), Image.Resampling.NEAREST)
    canvas = Image.new("RGB", (1200, height + 100), (25, 34, 42))
    canvas.paste(im, (0, 60))
    draw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 18)
    except OSError:
        font = ImageFont.load_default()

    def pos(xy):
        return ((xy[0] + sx / 2) / sx * 1200, (sy / 2 - xy[1]) / sy * height + 60)

    draw.text((18, 15), f"WELDING HALL | {sx:g} x {sy:g} m | Z up | scene geometry, NOT SLAM", fill="white", font=font)
    for o in x.hall.objects.values():
        if o.get("zone") not in ("A", "B"):
            continue
        p = pos(o["pose"][:2])
        draw.text((p[0] - 18, p[1] - 12), o["id"], fill=(230, 150, 35), font=font)
    for g in nav["goals"]:
        px, py = pos(g["goal_xy_m"])
        draw.ellipse((px - 5, py - 5, px + 5, py + 5), fill=(30, 90, 240))
    px, py = pos(x.c["robot"]["start_xy_yaw"][:2])
    draw.ellipse((px - 7, py - 7, px + 7, py + 7), fill=(230, 60, 70))
    draw.text((px + 10, py - 10), "START", fill=(190, 30, 40), font=font)
    draw.text(
        (18, height + 76),
        f"Green: free after {nav['inflation_radius_m']:.2f} m inflation | Grey: blocked/clearance | Blue: approach goals",
        fill="white",
        font=font,
    )
    canvas.save(x.out / "map_preview.png")
    summary = {
        "workpieces": sum(o.get("zone") in ("A", "B") for o in x.hall.objects.values()),
        "seams": len(x.seams),
        "segments": sum(len(s["segments"]) for s in x.seams),
        "verified_segments": sum(len(v["verified"]) for v in result["seams"].values()),
        "candidate_stances": sum(len(v["candidates"]) for v in result["seams"].values()),
        "main_aisle_clear_width_m": x.c["hall"]["main_aisle_m"],
        "hall_size_m": x.c["hall"]["size_m"],
        "A_stations": len(x.c["stations"]),
        "B_panels": len(panel_configs(x.c)),
        "panel_entrances": {
            p["id"]: p["entry_width_m"] if "entry_width_m" in p else p["cross_lane_width_m"] - p["plate_thickness_m"]
            for p in panel_configs(x.c)
        },
        "B01_entry_clear_width_m": x.c["parallel"]["entry_width_m"],
        "B02_cross_lane_min_clear_width_m": x.c["grid"]["cross_lane_width_m"] - x.c["grid"]["plate_thickness_m"],
        "all_major_areas_connected": nav["all_major_areas_connected"],
        "body_height_options": height_options,
    }
    write_json(x.out / "summary.json", summary)
    print(summary)


if __name__ == "__main__":
    export()
