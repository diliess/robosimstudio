"""Read-only task/data contract for future tablet, navigation and posture clients.

No gait, base teleportation or automatic weld execution is hidden behind this API.
Call get_seam with current fixture poses to obtain synchronized world coordinates.
"""

import argparse
import copy
import hashlib
import json

import numpy as np
from hall_geometry import ROOT, Hall


class WeldingSceneAPI:
    def __init__(self, pose_provider=None):
        self.hall = Hall()
        self.pose_provider = pose_provider
        self.selection = {"workpiece": None, "seam": None}

    def workpieces(self):
        return [
            {k: v for k, v in o.items() if k != "parts"}
            for o in self.hall.objects.values()
            if o.get("zone") in ("A", "B")
        ]

    def seams(self, workpiece=None):
        poses = self.pose_provider() if self.pose_provider else None
        return [s for s in self.hall.world_seams(poses) if workpiece is None or s["workpiece"] == workpiece]

    def get_seam(self, seam_id):
        return next(s for s in self.seams() if s["id"] == seam_id)

    def select(self, seam_id):
        s = self.get_seam(seam_id)
        self.selection = {"workpiece": s["workpiece"], "seam": seam_id}
        return dict(self.selection)

    def stances(self, seam_id):
        path = ROOT / "exports/stances.json"
        data = json.loads(path.read_text()) if path.exists() else {"seams": {}}
        digest = hashlib.sha256((ROOT / "config.json").read_bytes()).hexdigest()
        result = copy.deepcopy(data["seams"].get(seam_id, {"candidates": [], "verified": []}))
        geometry = ROOT / "exports/scene_geometry.json"
        geometry_changed = (
            geometry.exists() and data.get("geometry_sha256") != hashlib.sha256(geometry.read_bytes()).hexdigest()
        )
        poses_changed = False
        if self.pose_provider:
            poses = self.pose_provider()
            poses_changed = any(
                not np.allclose(pose, self.hall.objects[key]["pose"], atol=1e-6, rtol=0)
                for key, pose in poses.items()
                if key in self.hall.objects
            )
        if data.get("config_sha256") != digest or geometry_changed or poses_changed:
            result["verified"] = []
            result["validation_status"] = "stale configuration: rerun validate_scene.py"
        return result

    def task(self, seam_id):
        s = self.get_seam(seam_id)
        stances = self.stances(seam_id)
        steps = []
        for seg in s["segments"]:
            verified = [v for v in stances["verified"] if v["segment_id"] == seg["id"]]
            steps.append(
                {
                    "segment": seg,
                    "states": [
                        "WAIT_NAVIGATION",
                        "ADJUST_BASE_POSTURE",
                        "APPROACH",
                        "WELD_TCP_PATH",
                        "RETRACT",
                        "REFOLD_ARM",
                    ],
                    "verified_stances": verified,
                    "candidate_stances": [c for c in stances["candidates"] if c["segment_id"] == seg["id"]],
                    "requires_external_navigation": True,
                    "requires_external_posture_and_trajectory_controller": True,
                }
            )
        return {
            "seam_id": seam_id,
            "execution_enabled": False,
            "segments": steps,
            "transition_contract": "Move base only after RETRACT and REFOLD_ARM acknowledged; revalidate after any workpiece or mount change.",
        }

    def export(self):
        return self.hall.export(self.pose_provider() if self.pose_provider else None)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("command", choices=["workpieces", "seams", "task"])
    p.add_argument("--workpiece")
    p.add_argument("--seam")
    a = p.parse_args()
    api = WeldingSceneAPI()
    result = (
        api.workpieces()
        if a.command == "workpieces"
        else api.seams(a.workpiece)
        if a.command == "seams"
        else api.task(a.seam)
    )
    print(json.dumps(result, indent=2))
