"""Welding inspection panel using the existing Studio/Viser extension interface."""

import hashlib
import json

import numpy as np
from hall_geometry import ROOT
from scipy.spatial.transform import Rotation as R


def install(sim, studio, hall, views):
    if getattr(studio, "_welding_sim", None) is sim:
        return
    # Studio preserves user each_step callbacks across live rebuilds. Remove only
    # this twin's old callback; other panels and library code remain untouched.
    studio._each_step[:] = [fn for fn in studio._each_step if getattr(fn, "__module__", "") != __name__]
    old_root = getattr(studio, "_welding_root", None)
    if old_root is not None:
        old_root.remove()
    studio._welding_sim = sim
    gui, scene = studio.gui, studio.server.scene
    studio._welding_root = scene.add_frame("/welding", show_axes=False)
    seams = hall.world_seams()
    by_id = {s["id"]: s for s in seams}
    markers = {}
    pending = {"redraw": True, "view": True, "preview": False}
    stations = [o["id"] for o in hall.objects.values() if o.get("zone") in ("A", "B")]
    with gui.add_folder("Welding hall / 焊接场景", expand_by_default=True):
        gui.add_markdown(
            "**B2 + E05L-Pro · 几何场景**\n\n米制 / Z 向上。地图来自场景几何；未接入 SLAM 或自主导航。绿色站位经过离线姿态与碰撞采样，蓝色仅为候选。"
        )
        station = gui.add_dropdown("工位", options=stations, initial_value="A01")
        seam = gui.add_dropdown("焊缝", options=[s["id"] for s in seams if s["workpiece"] == "A01"])
        camera = gui.add_dropdown("视角", options=list(views) + ["selected station"], initial_value="overview")
        toggles = {
            label: gui.add_checkbox(label, initial_value=initial)
            for label, initial in [
                ("焊缝高亮", True),
                ("TCP 坐标系", True),
                ("候选站位", False),
                ("碰撞体", False),
                ("导航目标", False),
            ]
        }
        info = gui.add_markdown("")
        preview = gui.add_button("预览 TCP 轨迹（几何演示）")
        export = gui.add_button("导出当前焊缝数据")
        note = gui.add_markdown("轨迹预览不移动机器人；完成状态表示预览进度。")

    def dirty(_):
        pending["redraw"] = True

    def choose_station(_):
        opts = [s["id"] for s in seams if s["workpiece"] == station.value]
        seam.options = opts
        seam.value = opts[0]
        pending["redraw"] = True

    station.on_update(choose_station)
    seam.on_update(dirty)
    camera.on_update(lambda _: pending.update(view=True))
    for widget in toggles.values():
        widget.on_update(dirty)
    preview.on_click(lambda _: pending.update(preview=True))
    export.on_click(lambda _: pending.update(export=True))
    trajectory = {"index": -1, "points": None, "id": None}
    tcp = scene.add_frame("/welding/tcp", axes_length=0.12, axes_radius=0.004)
    validation = {}
    path = ROOT / "exports/validation.json"
    if path.exists():
        try:
            validation = json.loads(path.read_text())
        except json.JSONDecodeError:
            validation = {}
    digest = hashlib.sha256((ROOT / "config.json").read_bytes()).hexdigest()
    if validation.get("config_sha256") != digest:
        validation = {}
    for label in stations:
        p = hall.objects[label]["pose"]
        pos = np.array(p[:3]) + [0, 0, 1.5 if label.startswith("A") else 0.45]
        scene.add_label(f"/welding/labels/{label}", text=label, position=pos)

    def clear():
        for handle in markers.values():
            handle.remove()
        markers.clear()

    def line(name, points, color, width=3):
        p = np.asarray(points, float)
        if len(p) < 2:
            return
        markers[name] = scene.add_line_segments(
            "/welding/" + name, points=np.stack([p[:-1], p[1:]], 1).astype(np.float32), colors=color, line_width=width
        )

    stance_index = {}
    index_path = ROOT / "exports/stances.json"
    if index_path.exists():
        loaded = json.loads(index_path.read_text())
        if (
            loaded.get("config_sha256") == digest
            and loaded.get("geometry_sha256")
            == hashlib.sha256((ROOT / "exports/scene_geometry.json").read_bytes()).hexdigest()
        ):
            stance_index = loaded["seams"]
    nav_data = (
        json.loads((ROOT / "exports/navigation.json").read_text())
        if (ROOT / "exports/navigation.json").exists()
        else {"goals": []}
    )
    studio.welding_controls = {
        "station": station,
        "seam": seam,
        "camera": camera,
        "toggles": toggles,
        "pending": pending,
        "info": info,
        "preview": trajectory,
    }
    studio.server.on_client_connect(lambda _: pending.update(view=True))

    def redraw():
        clear()
        selected = by_id[seam.value]
        if toggles["焊缝高亮"].value:
            for s in seams:
                line(
                    "seams/" + s["id"],
                    s["world_points_m"],
                    (255, 235, 90) if s["id"] == seam.value else (240, 100, 35),
                    5 if s["id"] == seam.value else 2,
                )
        indexed = stance_index.get(seam.value, {"candidates": [], "verified": []})
        if toggles["候选站位"].value:
            for i, c in enumerate(indexed["candidates"][:18]):
                x, y, yaw = c["xy_yaw"]
                z = c.get("support_surface_m", 0) + 0.03
                q = R.from_euler("z", yaw).as_quat()
                markers[f"candidate{i}"] = scene.add_box(
                    f"/welding/candidate/{i}",
                    dimensions=(1.40, 0.76, 0.025),
                    position=(x, y, z),
                    wxyz=(q[3], *q[:3]),
                    color=(40, 180, 235),
                    opacity=0.25,
                    wireframe=True,
                )
            for i, v in enumerate(indexed["verified"]):
                x, y, yaw = v["xy_yaw"]
                q = R.from_euler("z", yaw).as_quat()
                markers[f"verified{i}"] = scene.add_box(
                    f"/welding/verified/{i}",
                    dimensions=(1.40, 0.76, 0.035),
                    position=(x, y, v["support_surface_m"] + 0.045),
                    wxyz=(q[3], *q[:3]),
                    color=(70, 240, 110),
                    opacity=0.8,
                    wireframe=True,
                )
        if toggles["碰撞体"].value:
            for p in hall.world_parts():
                if not p["collision"] or p["role"] == "floor":
                    continue
                name = "collision/" + p["name"]
                if p["type"] == "box":
                    q = p["quat"]
                    markers[name] = scene.add_box(
                        "/welding/" + name,
                        dimensions=p["size"],
                        position=p["center"],
                        wxyz=(q[3], *q[:3]),
                        color=(240, 80, 90),
                        opacity=0.35,
                        wireframe=True,
                    )
                else:
                    markers[name] = scene.add_mesh_simple(
                        "/welding/" + name,
                        vertices=np.array(p["vertices"]),
                        faces=np.array(p["faces"]),
                        color=(240, 80, 90),
                        opacity=0.35,
                        wireframe=True,
                    )
        if toggles["导航目标"].value:
            for i, g in enumerate(nav_data["goals"]):
                p = [*g["goal_xy_m"], 0.05]
                markers[f"nav{i}"] = scene.add_frame(
                    f"/welding/nav/{i}", position=p, axes_length=0.25, axes_radius=0.01
                )
        result = (
            f"已验证 {len(indexed['verified'])}/{len(selected['segments'])} 段（完整姿态与采样碰撞）"
            if indexed["verified"]
            else "候选；本条焊缝尚未验证"
        )
        info.content = f"**{selected['id']}** · {selected['category']}\n\n长度 {selected['length_m']:.3f} m；焊缝高度 {selected['seam_height_range_m'][0]:.3f}–{selected['seam_height_range_m'][1]:.3f} m；台面 {selected['table_height_m']:.3f} m。\n\n{len(selected['segments'])} 个段；{selected['access']}。\n\n{result}；验证细节见 exports/validation.json。"

    def tick():
        if studio.env is not sim:
            return
        # Read live fixture frames, so consumers never keep stale world seam poses.
        poses = {key: list(sim[key].handle.xform) for key in stations}
        if poses != pending.get("last_poses"):
            updated = hall.world_seams(poses)
            for s in updated:
                by_id[s["id"]] = s
            pending["last_poses"] = poses
        if pending.pop("redraw", False):
            redraw()
        tcp.visible = toggles["TCP 坐标系"].value
        studio.viewer.show_collision = toggles["碰撞体"].value
        p, q = sim.arm("e05l").ee_pose(sim.state)
        tcp.position = p
        tcp.wxyz = (q[3], *q[:3])
        if pending.pop("view", False):
            if camera.value == "selected station":
                p = np.asarray(hall.objects[station.value]["pose"][:3])
                eye = p + [1.9, -2.3, 1.9]
                look = p + [0, 0, 0.5]
            else:
                eye, look, _, _ = views[camera.value]
            for client in studio.server.get_clients().values():
                client.camera.position = eye
                client.camera.look_at = look
                client.camera.up_direction = (0, 0, 1)
        if pending.pop("export", False):
            hall.export(poses)
            note.content = "已导出 exports/seams.json（场景几何真值）。"
        if pending.pop("preview", False):
            trajectory.update(index=0, points=by_id[seam.value]["world_tcp_points_m"], id=seam.value)
        if trajectory["index"] >= 0:
            trajectory["index"] += 1
            p = trajectory["points"]
            n = min(trajectory["index"], len(p))
            if n > 1:
                old = markers.pop("preview", None)
                if old:
                    old.remove()
                line("preview", p[:n], (80, 255, 130), 6)
            note.content = f"几何演示 {trajectory['id']}: {100 * n / len(p):.0f}%（不驱动机器人）"
            if n == len(p):
                (ROOT / "exports/task_state.json").write_text(
                    json.dumps(
                        {
                            "mode": "geometric_preview",
                            "seam_id": trajectory["id"],
                            "completion": 1.0,
                            "robot_executed": False,
                        },
                        indent=2,
                    )
                )
                trajectory["index"] = -1

    studio.each_step(tick)
    tick()
