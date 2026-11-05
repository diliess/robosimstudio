"""Open the portable welding USD in Isaac Sim 6.2, with timeline paused.

Use start_isaac.sh. Optional headless capture validates the real Isaac renderer.
This launcher does not install or replay RoboSimStudio demonstration routines.
"""

from __future__ import annotations

import argparse
import faulthandler
import hashlib
import json
import os
import signal
import time
from pathlib import Path

from camera_views import VIEWS

ROOT = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--usd", type=Path, default=ROOT / "exports/usd/industrial_welding.usdc")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--editable-stage", action="store_true", help="allow authored layout/robot state overrides; report manifest differences without blocking display")
parser.add_argument("--frames", type=int, default=0, help="0 leaves the viewer open")
parser.add_argument("--camera", default="overview", choices=list(VIEWS))
parser.add_argument("--capture-all", action="store_true", help="capture every authored camera")
parser.add_argument("--capture", action="store_true", help="save the selected camera")
parser.add_argument("--capture-cameras", nargs="+", choices=list(VIEWS))
parser.add_argument("--capture-dir", type=Path, default=ROOT / "screenshots/isaacsim")
parser.add_argument("--physics-inspector", action="store_true", help="open the native Physics Inspector")
parser.add_argument("--probe-inspector", action="store_true", help="validate joint control and restore the saved pose")
parser.add_argument("--capture-ui", action="store_true", help="save the Isaac application window")
parser.add_argument("--width", type=int, default=1600)
parser.add_argument("--height", type=int, default=1000)
parser.add_argument(
    "--idle-check-seconds",
    type=float,
    default=0,
    help="bounded GUI idle regression, then check redraw and Inspector",
)
args = parser.parse_args()
if args.idle_check_seconds < 0 or (args.idle_check_seconds and args.headless):
    parser.error("--idle-check-seconds requires a GUI and a nonnegative duration")
if not args.usd.is_file():
    parser.error(f"USD not found: {args.usd}. Run export_usd.py first.")

RUN = ROOT / "run"
RUN.mkdir(exist_ok=True)
# SIGUSR1 provides a Python stack without ptrace/root if a future stall occurs.
stack_file = (RUN / f"isaac_{os.getpid()}_stacks.log").open("a", buffering=1)
faulthandler.enable(file=stack_file, all_threads=True)
if hasattr(signal, "SIGUSR1"):
    faulthandler.register(signal.SIGUSR1, file=stack_file, all_threads=True)

# This project is explicitly installed/launched at the user's request.
# NVIDIA's runtime license prompt is accepted for this process only.
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
from isaacsim import SimulationApp

app = SimulationApp(
    {
        "headless": args.headless,
        "width": args.width,
        "height": args.height,
        "window_width": 1600,
        "window_height": 1050,
        "active_cuda_gpus": [0],
        "multi_gpu": False,
        "renderer": "RaytracedLighting",
        "sync_loads": True,
        "open_usd": str(args.usd.resolve()),
        "display_options": 3286,
        "extra_args": [
            "--/app/window/title=Industrial Welding | B2 + E05L-Pro | Isaac Sim 6.2",
            # Kit 110.4.0 may poll forever when a due timer was cancelled
            # (OMPE-113948). Its supported alternate pump sets stop() before
            # polling and remains nonblocking; no installed code is patched.
            "--/exts/omni.kit.async_engine/keep_loop_running=false",
        ],
    }
)

# All Omniverse/pxr imports must follow SimulationApp initialization.
import asyncio

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from omni.kit.viewport.utility import get_active_viewport
from pxr import UsdGeom, UsdPhysics

# Fail clearly if a future package ceases to honor the compatibility setting.
assert carb.settings.get_settings().get("/exts/omni.kit.async_engine/keep_loop_running") is False
omni.timeline.get_timeline_interface().stop()
for _ in range(60):
    app.update()
context = omni.usd.get_context()
stage = context.get_stage()
if stage is None:
    raise RuntimeError("Isaac Sim did not open the USD stage")
viewport = get_active_viewport()
if viewport is None:
    raise RuntimeError("Isaac Sim did not create a viewport")
viewport.set_texture_resolution((args.width, args.height))
viewport.camera_path = "/World/Cameras/" + args.camera

report = {
    "usd": str(args.usd.resolve()),
    "usd_sha256": hashlib.sha256(args.usd.read_bytes()).hexdigest(),
    "isaacsim_version": __import__("importlib.metadata", fromlist=["version"]).version("isaacsim"),
    "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
    "up_axis": UsdGeom.GetStageUpAxis(stage),
    "rigid_bodies": sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in stage.Traverse()),
    "revolute_joints": sum(p.IsA(UsdPhysics.RevoluteJoint) for p in stage.Traverse()),
    "collision_shapes": sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in stage.Traverse()),
    "timeline_playing": omni.timeline.get_timeline_interface().is_playing(),
    "screenshots": [],
    "physics_executed": bool(args.probe_inspector),
    "physics_scope": "Inspector authoring only" if args.probe_inspector else "timeline paused; no physics executed",
    "headless": args.headless,
    "async_engine_keep_loop_running": False,
    "stack_log": str(stack_file.name),
    "versions": {
        n: __import__("importlib.metadata", fromlist=["version"]).version(n)
        for n in ["newton", "mujoco", "mujoco-warp", "torch", "numpy"]
    },
}
assert report["up_axis"] == "Z" and report["meters_per_unit"] == 1
manifest = json.loads((args.usd.parent / "manifest.json").read_text())
report["manifest_counts_match"] = (
    report["rigid_bodies"] == manifest["counts"]["robot_bodies"]
    and report["revolute_joints"] == manifest["counts"]["revolute_joints"]
    and report["collision_shapes"] == manifest["counts"]["fixture_colliders"] + manifest["counts"]["robot_colliders"]
)
if not args.editable_stage:
    assert report["manifest_counts_match"]
report["editable_stage"] = args.editable_stage
report["source_stage_sha256"] = manifest["stage_sha256"]
# Read the actual USD joint-state and drive values rather than only the manifest.
if "initial_pose" in manifest:
    import math

    init = manifest["initial_pose"]
    arm_state = []
    arm_drive = []
    for name in init["arm_joint_names"]:
        prim = stage.GetPrimAtPath("/World/Robot/Joints/" + name)
        arm_state.append(math.radians(float(prim.GetAttribute("state:angular:physics:position").Get())))
        arm_drive.append(math.radians(float(UsdPhysics.DriveAPI.Get(prim, "angular").GetTargetPositionAttr().Get())))
    base = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath("/World/Robot/base_link"))
    initial_matches = (
        all(abs(a - b) < 1e-6 for a, b in zip(arm_state, init["arm_q_rad"], strict=True))
        and all(abs(a - b) < 1e-6 for a, b in zip(arm_drive, init["arm_q_rad"], strict=True))
        and abs(float(base.ExtractTranslation()[2]) - init["base_position_m"][2]) < 1e-6
    )
    if not args.editable_stage:
        assert initial_matches
    report["initial_pose_usd_readback"] = {
        "arm_q_rad": arm_state,
        "arm_drive_q_rad": arm_drive,
        "base_world_position_m": list(base.ExtractTranslation()),
        "matches_generated_initial_pose": initial_matches,
        "passed": initial_matches,
    }
from isaac_physics_tools import configure

physics_tools = configure(app, stage, manifest, open_inspector=args.physics_inspector or args.probe_inspector)
if args.probe_inspector:
    report["inspector_validation"] = physics_tools.probe()
else:
    report["physics_inspector_extension_enabled"] = physics_tools.enabled

if args.capture or args.capture_all or args.capture_cameras:
    # Use the actual RTX render product, usable with or without a desktop.
    import carb
    import numpy as np
    import omni.replicator.core as rep

    carb.settings.get_settings().set("/exts/omni.replicator.core/maxAssetLoadingTime", 10.0)
    from PIL import Image

    out = args.capture_dir
    out.mkdir(parents=True, exist_ok=True)
    cameras = args.capture_cameras or (list(VIEWS) if args.capture_all else [args.camera])
    for camera in cameras:
        product = rep.create.render_product("/World/Cameras/" + camera, (args.width, args.height))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(product)
        for _ in range(32):
            app.update()
        rep.orchestrator.step(rt_subframes=16, pause_timeline=True, delta_time=0.0)
        data = np.asarray(rgb.get_data())
        if data.size == 0 or data.std() < 1:
            raise RuntimeError("RTX returned an empty/constant image for " + camera)
        file = out / (camera + ".png")
        Image.fromarray(data[..., :3]).save(file)
        report["screenshots"].append(str(file))
        rgb.detach(product)
        product.destroy()
        print("CAPTURE", file, flush=True)
    # Restore the selected view after render-product capture.
    viewport.camera_path = "/World/Cameras/" + args.camera
if args.capture_ui:
    import omni.kit.renderer.capture

    out = args.capture_dir
    out.mkdir(parents=True, exist_ok=True)
    file = out / "isaac_editor.png"
    for _ in range(32):
        app.update()
    renderer = omni.kit.renderer.capture.acquire_renderer_capture_interface()
    renderer.capture_next_frame_swapchain(str(file))
    app.update()
    renderer.wait_async_capture()
    for _ in range(20):
        if file.exists():
            break
        app.update()
    if not file.exists():
        raise RuntimeError("Isaac editor screenshot was not written")
    report["editor_screenshot"] = str(file)
report["passed"] = True
(args.usd.parent / "isaac_runtime_validation.json").write_text(json.dumps(report, indent=2))
print("ISAAC_WELDING_READY", json.dumps(report), flush=True)

started = time.monotonic()
count = 0
last_status = 0.0
max_update = 0.0
idle_complete = False
exit_code = 0
# This watchdog only dumps a stack if the main loop stops for two minutes.
# Each heartbeat resets it; it does not keep a hung process artificially alive.
faulthandler.dump_traceback_later(120, repeat=True, file=stack_file)
try:
    if args.idle_check_seconds:
        # Exercise the exact known failure condition in the actual Kit loop.
        loop = asyncio.get_event_loop()
        for _ in range(8):
            timer = loop.call_at(loop.time() - 1.0, lambda: None)
            timer.cancel()
        print("IDLE_CHECK_CANCELLED_TIMERS_QUEUED", flush=True)
    while app.is_running() and (args.frames == 0 or count < args.frames):
        before = time.monotonic()
        app.update()
        now = time.monotonic()
        max_update = max(max_update, now - before)
        count += 1
        if now - last_status >= 5:
            status = {
                "pid": os.getpid(),
                "updates": count,
                "elapsed_s": now - started,
                "updated_unix": time.time(),
                "max_update_s": max_update,
                "timeline_playing": omni.timeline.get_timeline_interface().is_playing(),
            }
            temporary = RUN / f"isaac_status_{os.getpid()}.tmp"
            temporary.write_text(json.dumps(status, indent=2))
            temporary.replace(RUN / "isaac_status.json")
            faulthandler.dump_traceback_later(120, repeat=True, file=stack_file)
            last_status = now
        if args.idle_check_seconds and now - started >= args.idle_check_seconds:
            idle_complete = True
            break
    if args.idle_check_seconds:
        if not idle_complete:
            raise RuntimeError("Viewer exited before the requested idle duration")
        idle = {
            "pid": os.getpid(),
            "requested_idle_s": args.idle_check_seconds,
            "elapsed_idle_s": time.monotonic() - started,
            "updates": count,
            "max_update_s": max_update,
            "cancelled_timer_case_returned": True,
            "headless": args.headless,
            "usd_sha256": report["usd_sha256"],
        }
        (RUN / "isaac_idle_progress.json").write_text(json.dumps(idle, indent=2))
        # Request a different camera after idling: these frames must redraw.
        viewport.camera_path = "/World/Cameras/area_B"
        for _ in range(30):
            app.update()
        if physics_tools.model is None:
            physics_tools.open()
        idle["inspector_after_idle"] = physics_tools.probe()
        import omni.kit.renderer.capture

        file = RUN / "isaac_after_idle.png"
        if file.exists():
            file.unlink()
        renderer = omni.kit.renderer.capture.acquire_renderer_capture_interface()
        renderer.capture_next_frame_swapchain(str(file))
        app.update()
        renderer.wait_async_capture()
        for _ in range(20):
            if file.exists():
                break
            app.update()
        if not file.exists():
            raise RuntimeError("No GUI redraw screenshot after idle")
        idle["redraw_screenshot"] = str(file)
        idle["passed"] = idle["inspector_after_idle"]["passed"] and max_update < 10
        if not idle["passed"]:
            raise RuntimeError("Idle regression failed: inspector control or a >10s update stall")
        (ROOT / "exports/usd/isaac_idle_validation.json").write_text(json.dumps(idle, indent=2))
        print("ISAAC_IDLE_CHECK_PASSED", json.dumps(idle), flush=True)
except BaseException:
    import traceback

    exit_code = 1
    traceback.print_exc()
    raise
finally:
    faulthandler.cancel_dump_traceback_later()
    # Kit uses os._exit() here; pass failure through instead of hiding it.
    app.close(exit_code=exit_code)
