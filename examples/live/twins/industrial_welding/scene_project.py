"""Portable scene project: check, rebuild and package without importing scene.py.

The optional RoboSimStudio live UI still requires that repository. This module
only depends on the small pinned scene toolchain, and runs on CPU for USD export.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parent
EXPORTS = (
    "scene_geometry.json", "seams.json", "stances.json", "navigation.json",
    "map.yaml", "map.pgm", "map_inflated.pgm", "map_preview.png",
    "asset_manifest.json", "validation.json", "witness_replay.json",
    "geometry_audit.json", "layout_validation.json", "summary.json",
    "initial_pose_validation.json", "butt_joint_validation.json", "project_build.json", "project_inputs.json", "project_portability_validation.json",
)
VIEWS = ("overview", "robot", "area_A", "grid_detail", "butt_flat_detail", "butt_inclined_detail", "S_detail")
EDIT_LAYER = "industrial_welding_edit.usda"


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def delivery_files():
    """Explicit delivery surface: no environments, run logs, backups or ZIPs."""
    files = set()
    for pattern in ("*.py", "*.sh", "*.md", "requirements-scene.*", "isaacsim_requirements.lock.txt"):
        files.update(ROOT.glob(pattern))
    files.update(ROOT / n for n in (".gitignore", "config.json", "ctl", "LICENSE", "NOTICE"))
    files.update(p for p in (ROOT / "assets/assembled/source").rglob("*") if p.is_file())
    files.update(ROOT / "exports" / n for n in EXPORTS if (ROOT / "exports" / n).is_file())
    usd = ROOT / "exports/usd"
    files.update(usd / n for n in ("industrial_welding.usdc", "industrial_welding.usdz", EDIT_LAYER, "manifest.json", "usd_validation.json"))
    files.update((usd / "textures").glob("*.png"))
    files.update(p for n in VIEWS if (p := ROOT / "screenshots/isaacsim" / (n + ".png")).is_file())
    missing = [str(p.relative_to(ROOT)) for p in files if not p.is_file()]
    if missing:
        raise FileNotFoundError("Delivery inputs missing: " + ", ".join(sorted(missing)))
    for path in files:
        if path.is_symlink() or not path.resolve().is_relative_to(ROOT):
            raise ValueError("Delivery file must be local, not a symlink: " + str(path))
    return sorted(files)


def input_records():
    names = ("config.json", "hall_geometry.py", "prepare_assets.py", "robot_assets.py",
             "robot_initial.py", "export_usd.py", "steel_surface.py", "camera_views.py")
    paths = [ROOT / n for n in names]
    paths += sorted(p for p in (ROOT / "assets/assembled/source").rglob("*") if p.is_file())
    return {str(p.relative_to(ROOT)): digest(p) for p in paths}


def record_inputs():
    write_json(ROOT / "exports/project_inputs.json", {
        "stage_sha256": digest(ROOT / "exports/usd/industrial_welding.usdc"),
        "inputs": input_records(),
        "scope": "generated base scene; editable layer overrides require separate task-data revalidation",
    })


def check():
    from hall_geometry import Hall
    from pxr import Usd, UsdGeom, UsdUtils
    import mujoco
    from robot_assets import prepare_robot

    hall = Hall()
    files = delivery_files()
    for path in files:
        if path.stat().st_size < 256 and path.read_bytes().startswith(b"version https://git-lfs.github.com/spec/v1"):
            raise ValueError("Asset is an unresolved Git LFS pointer: " + str(path))
    usd = ROOT / "exports/usd"
    manifest = json.loads((usd / "manifest.json").read_text())
    current_config = digest(ROOT / "config.json")
    if current_config != manifest["source_config_sha256"]:
        raise ValueError("config.json differs from the exported stage; run ./project.sh build")
    if digest(usd / "industrial_welding.usdc") != manifest["stage_sha256"]:
        raise ValueError("Generated base USD changed; save GUI edits to industrial_welding_edit.usda or rebuild")
    inputs_file = ROOT / "exports/project_inputs.json"
    if not inputs_file.exists():
        raise ValueError("Project input record is missing; run ./project.sh build")
    recorded = json.loads(inputs_file.read_text())
    if recorded["inputs"] != input_records() or recorded["stage_sha256"] != manifest["stage_sha256"]:
        raise ValueError("Scene source or robot snapshot changed; run ./project.sh build")
    dependency_counts = {}
    for name in ("industrial_welding.usdc", "industrial_welding.usdz", EDIT_LAYER):
        stage = Usd.Stage.Open(str(usd / name))
        if not stage or UsdGeom.GetStageUpAxis(stage) != "Z" or UsdGeom.GetStageMetersPerUnit(stage) != 1:
            raise ValueError("Invalid metre/Z-up USD: " + name)
        layers, assets, missing = UsdUtils.ComputeAllDependencies(str(usd / name))
        if missing:
            raise FileNotFoundError(str(missing))
        for asset in assets:
            # Package-relative paths are represented as archive[member].
            outer = str(asset).split("[", 1)[0]
            if not Path(outer).resolve().is_relative_to(ROOT):
                raise ValueError("External asset dependency: " + str(asset))
        dependency_counts[name] = {"layers": len(layers), "assets": len(assets)}
    model_file, _, _ = prepare_robot(hall.config["robot"])
    model = mujoco.MjModel.from_xml_path(str(model_file))
    seams = json.loads((ROOT / "exports/seams.json").read_text())["seams"]
    if seams != hall.world_seams():
        raise ValueError("Exported seam geometry is stale; run ./project.sh build")
    geometry = json.loads((ROOT / "exports/scene_geometry.json").read_text())["objects"]
    if geometry != json.loads(json.dumps(hall.objects, default=lambda x: x.item())):
        raise ValueError("Exported fixture geometry is stale; run ./project.sh build")
    verification = ROOT / "exports/validation.json"
    validation = json.loads(verification.read_text()) if verification.exists() else {}
    result = {
        "passed": True, "files": len(files), "uncompressed_bytes": sum(p.stat().st_size for p in files),
        "max_file_bytes": max(p.stat().st_size for p in files),
        "config_sha256": current_config, "source_stage_sha256": manifest["stage_sha256"],
        "dependencies": dependency_counts, "robot_bodies": model.nbody - 1,
        "arm_and_leg_joints": model.njnt - 1, "seams": len(seams),
        "verified_results_match_config": bool(validation.get("representatives")) and validation.get("config_sha256") == current_config,
        "verification_scope": "generated base scene only; GUI overrides are not independently feasibility-checked",
        "execution_scope": "geometry / source robot / editable USD; no navigation, gait or welding execution",
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return result


def build(full_validation=False):
    """Regenerate from config; never retain prior feasibility claims as current."""
    from hall_geometry import Hall

    history = ROOT / "run/build_history" / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + f"_{os.getpid()}")
    history.mkdir(parents=True)
    usd = ROOT / "exports/usd"
    edit = (usd / EDIT_LAYER).read_bytes() if (usd / EDIT_LAYER).exists() else None
    for name in EXPORTS:
        p = ROOT / "exports" / name
        if p.exists():
            shutil.move(str(p), history / name)
    if usd.exists():
        shutil.move(str(usd), history / "usd")
    pictures = ROOT / "screenshots/isaacsim"
    if pictures.exists():
        shutil.move(str(pictures), history / "screenshots")
    hall = Hall()
    hall.export()
    # Export candidate stances and a fresh geometry map, with no verified poses.
    write_json(ROOT / "exports/validation.json", {
        "schema_version": 1, "config_sha256": digest(ROOT / "config.json"),
        "geometry_sha256": digest(ROOT / "exports/scene_geometry.json"),
        "seams_sha256": digest(ROOT / "exports/seams.json"),
        "passed": False, "representatives": [], "status": "not validated after rebuild",
        "scope": "scene rebuilt; run ./project.sh validate for sampled feasibility checks",
    })
    from export_data import export
    export()
    if full_validation:
        validate()
    from export_usd import Exporter
    Exporter(usd).save()
    if edit is None:
        edit = b'#usda 1.0\n(\n defaultPrim = "World"\n metersPerUnit = 1\n upAxis = "Z"\n subLayers = [@industrial_welding.usdc@]\n)\n'
    (usd / EDIT_LAYER).write_bytes(edit)
    write_json(ROOT / "exports/project_build.json", {
        "config_sha256": digest(ROOT / "config.json"), "full_validation_requested": full_validation,
        "verification_status": "see validation.json" if full_validation else "unverified candidates only",
        "previous_outputs": str(history.relative_to(ROOT)),
        "edit_layer_preserved": True,
        "edit_layer_note": "GUI overrides may mask config changes; clear conflicting overrides in Isaac if needed",
    })
    record_inputs()
    check()


def validate():
    commands = [
        ["validate_scene.py", "--fresh"], ["coverage_validation.py"], ["export_data.py"],
        ["replay_validation.py"], ["geometry_audit.py"], ["layout_validation.py"],
    ]
    for command in commands:
        subprocess.run([sys.executable, *command], cwd=ROOT, check=True)


def package(output):
    result = check()
    files = delivery_files()
    manifest = {
        "format": "industrial-welding-editable-project-v1",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "source": "RoboSimStudio dev industrial_welding twin",
        "check": result,
        "files": [{"path": str(p.relative_to(ROOT)), "bytes": p.stat().st_size, "sha256": digest(p)} for p in files],
    }
    output = output.resolve()
    if output in files:
        raise ValueError("Archive cannot overwrite an input")
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for path in files:
            archive.write(path, Path("industrial_welding") / path.relative_to(ROOT))
        archive.writestr("industrial_welding/project_manifest.json", json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    with zipfile.ZipFile(output) as archive:
        if archive.testzip() is not None:
            raise ValueError("Archive CRC validation failed")
    output.with_suffix(".zip.sha256").write_text(digest(output) + "  " + output.name + "\n")
    print("PROJECT_ARCHIVE", output, output.stat().st_size, "bytes")


def verify():
    manifest = json.loads((ROOT / "project_manifest.json").read_text())
    for record in manifest["files"]:
        path = (ROOT / record["path"]).resolve()
        if not path.is_relative_to(ROOT) or not path.is_file() or digest(path) != record["sha256"]:
            raise ValueError("Missing or modified delivery file: " + record["path"])
    print("ARCHIVE_INTEGRITY_PASSED", len(manifest["files"]), "files")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("check", help="check assets, source robot, data and composed USD")
    b = sub.add_parser("build", help="rebuild geometry, seams, candidates, map and USD")
    b.add_argument("--validate", action="store_true", help="also recompute sampled feasibility (slow)")
    sub.add_parser("validate", help="recompute sampled IK/collision/support results (slow)")
    p = sub.add_parser("pack", help="write an editable project ZIP; excludes environments and run/")
    p.add_argument("--output", type=Path, default=ROOT / "exports/industrial_welding_project.zip")
    sub.add_parser("verify", help="verify original ZIP file hashes; use before making edits")
    args = parser.parse_args()
    if args.command == "check": check()
    elif args.command == "build": build(args.validate)
    elif args.command == "validate": validate()
    elif args.command == "pack": package(args.output)
    elif args.command == "verify": verify()


if __name__ == "__main__":
    main()
