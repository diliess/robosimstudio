"""Native Isaac 6.2 Physics Inspector integration and bounded joint-control probe.

Uses NVIDIA's shipped inspector UI/model, not a replacement joint animation.
Temporary fix-base/gravity options are confined to inspector authoring mode.
"""

from __future__ import annotations

import gc
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent


class PhysicsTools:
    def __init__(self, app, stage, manifest, open_inspector=False):
        import omni.kit.app
        from pxr import PhysxSchema

        self.app, self.stage, self.manifest = app, stage, manifest
        self.manager = omni.kit.app.get_app().get_extension_manager()
        from omni.kit.async_engine import run_coroutine

        async def enable_native_inspector():
            # Kit 110.4 creates the inspector on an active asyncio loop.
            self.manager.set_extension_enabled_immediate("omni.physx.supportui", True)

        future = run_coroutine(enable_native_inspector())
        while not future.done():
            app.update()
        future.result()
        self.enabled = self.manager.is_extension_enabled("omni.physx.supportui")
        if not self.enabled:
            raise RuntimeError("Cannot enable native omni.physx.supportui extension")
        # Preserve the source's multi-turn joint coordinate, not only its
        # wrapped quaternion. In particular E05L joint4 starts below -180 deg.
        self.schema_tokens = []
        for record in manifest["joints"]:
            prim = stage.GetPrimAtPath(record["path"])
            state = PhysxSchema.JointStateAPI.Apply(prim, "angular")
            if not state.GetPositionAttr().HasAuthoredValueOpinion():
                state.CreatePositionAttr(float(math.degrees(record["q_rad"])))
            if not state.GetVelocityAttr().HasAuthoredValueOpinion():
                state.CreateVelocityAttr(0.0)
            self.schema_tokens = prim.GetAppliedSchemas()
        self.window = self.model = None
        if open_inspector:
            self.open()

    def open(self):
        import omni.kit.actions.core
        import omni.ui
        import omni.usd
        from omni.physxsupportui.bindings import _physxSupportUi as support
        from omni.physxsupportui.scripts.inspector_window import PhysXInspectorWindow

        omni.usd.get_context().get_selection().set_selected_prim_paths(["/World/Robot/base_link"], False)
        omni.kit.actions.core.get_action_registry().execute_action("omni.physx.supportui", "show_physics_inspector")
        for _ in range(20):
            self.app.update()
        # The shipped 110.4 inspector has no public window accessor. Retrieve
        # its real Python window; Workspace can expose only the base wrapper.
        windows = [w for w in omni.ui.Workspace.get_windows() if isinstance(w, PhysXInspectorWindow)]
        if not windows:
            windows = [
                w
                for w in gc.get_objects()
                if isinstance(w, PhysXInspectorWindow) and getattr(w, "_model_inspector", None)
            ]
        if not windows:
            raise RuntimeError("Native Physics Inspector action did not create a window")
        self.window = windows[0]
        # Use the tall Property dock rather than the default small split below
        # Stage, which otherwise exposes only two or three joint rows.
        property_window = omni.ui.Workspace.get_window("Property")
        if property_window:
            self.window.dock_in(property_window, omni.ui.DockPosition.SAME)
            self.window.focus()
        self.model = self.window._model_inspector
        self.model.set_selected_paths(["/World/Robot/base_link"])
        self.window._inspector_toolbar._select_current()
        # These are native inspector-only checkboxes. They do not create a
        # world joint or change the free-base USD physics model.
        self.model.get_fix_articulation_base_model().set_value(True)
        for _ in range(20):
            self.app.update()
        self.model.get_control_type_model().set_value(int(support.PhysXInspectorModelControlType.JOINT_DRIVE))
        print("PHYSICS_INSPECTOR_OPEN", self.window.title, self.schema_tokens, flush=True)

    def probe(self):
        from omni.physxsupportui import get_physx_supportui_private_interface
        from pxr import PhysxSchema, UsdGeom, UsdPhysics

        private = get_physx_supportui_private_interface()
        from omni.physxsupportui.bindings._physxSupportUi import PhysXInspectorModelState

        # Reset/discard ends authoring. Repeated probes use the same native
        # Re-Enable authoring action as the Inspector, before applying targets.
        if private.get_inspector_state() != PhysXInspectorModelState.AUTHORING:
            private.enable_inspector_authoring_mode()
            for _ in range(20):
                self.app.update()
        print("INSPECTOR_STATE", private.get_inspector_state(), flush=True)
        joint = self.stage.GetPrimAtPath("/World/Robot/Joints/e05l_joint1")
        drive = UsdPhysics.DriveAPI.Get(joint, "angular")
        original = float(drive.GetTargetPositionAttr().Get())
        # Same native model setter and engine stepping as the built-in slider.
        before = UsdGeom.XformCache().GetLocalToWorldTransform(
            self.stage.GetPrimAtPath("/World/Robot/e05l_welding_torch")
        )
        self.model.set_joint_value(str(joint.GetPath()), original + 5)
        for _ in range(180):
            private.step_inspector_simulation(1.0 / 60)
            self.app.update()
        self.model.refresh_model_values()
        value = PhysxSchema.JointStateAPI.Get(joint, "angular").GetPositionAttr().Get()
        after = UsdGeom.XformCache().GetLocalToWorldTransform(
            self.stage.GetPrimAtPath("/World/Robot/e05l_welding_torch")
        )
        items = self.model.get_items_matching_paths([str(joint.GetPath())])
        for item in items:
            print(
                "INSPECTOR_ROW",
                [
                    self.model.get_item_value_model(item, col).as_string
                    for col in range(self.model.get_item_value_model_count(item))
                ],
                flush=True,
            )
        report = {
            "enabled": self.enabled,
            "window": self.window.title,
            "native_model": str(type(self.model)),
            "state": str(private.get_inspector_state()),
            "target_deg": original + 5,
            "measured_joint_deg": value,
            "torch_translation_change_m": float((after.ExtractTranslation() - before.ExtractTranslation()).GetLength()),
            "joint_count": len(self.manifest["joints"]),
            "joint_state_schema_tokens": self.schema_tokens,
        }
        print("INSPECTOR_PROBE", json.dumps(report), flush=True)
        # Restore every authoring change, then restore the source target.
        self.window._inspector._inspector_simulation.stop_authoring_simulation()
        private.reset_inspector_to_authoring_start()
        drive.GetTargetPositionAttr().Set(original)
        for _ in range(20):
            self.app.update()
        restored = UsdGeom.XformCache().GetLocalToWorldTransform(
            self.stage.GetPrimAtPath("/World/Robot/e05l_welding_torch")
        )
        report["restored_torch_translation_error_m"] = float(
            (restored.ExtractTranslation() - before.ExtractTranslation()).GetLength()
        )
        report["inspector_physics_steps"] = 180
        report["scope"] = (
            "native Inspector authoring only; temporary tool fix-base; not free-base locomotion validation"
        )
        report["passed"] = (
            report["restored_torch_translation_error_m"] < 1e-5
            and abs(float(value) - (original + 5)) < 0.5
            and report["torch_translation_change_m"] > 0.001
        )
        (ROOT / "exports/usd/inspector_validation.json").write_text(json.dumps(report, indent=2))
        if not report["passed"]:
            raise RuntimeError("Native inspector joint control did not reach the requested angle")
        return report


def configure(app, stage, manifest, open_inspector=False):
    return PhysicsTools(app, stage, manifest, open_inspector)
