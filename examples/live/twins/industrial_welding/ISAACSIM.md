# Isaac Sim 6.2 场景迁移

## 可编辑工程交付（2026-11-05）

新电脑使用 `bash install_scene.sh` 安装独立场景工具，`bash install_isaac.sh` 安装独立 Isaac 6.2 环境，再用 `bash edit_isaac.sh` 打开 `exports/usd/industrial_welding_edit.usda` 保存 GUI 修改。修改 config.json 后执行 `bash project.sh build`；重打包执行 `bash project.sh pack`。也可设置 `ISAAC_PYTHON` 使用已有 Isaac Python。详见 [PORTABLE_PROJECT.md](PORTABLE_PROJECT.md)。GUI 覆盖层不会自动改写外部焊缝/地图/验证数据。


当前版本 2026-11-03：32×32 m 厂房、24 个 A 区工位、4 块 B 区板架。A 区保留 8 个普通 T、7 个开放 S 和 3 个其他工位，另将六个 T 改为双板 V 坡口对接：两件水平、15°/30°/45°/60° 各一件。中央为真实钢板边缘和坡口，两侧打磨区不表示已完成焊接。布局、来源、根部/台面高度见 README.md。

## 环境

按用户已确认版本安装 `isaacsim==6.2.0.0`，位于本 twin 的独立 uv 环境 `.venv-isaacsim/`。该环境使用 Python 3.12.15、Newton 1.6.1、MuJoCo 3.12；RoboSimStudio 原 `.venv` 使用 Newton 1.7/MuJoCo 3.15。两者独立，未改仓库 pyproject.toml/uv.lock。安装脚本 `uv --no-config` 避免继承主仓库的依赖覆盖。锁定清单见 isaacsim_requirements.lock.txt。

```bash
cd ~/code/robosimstudio/examples/live/twins/industrial_welding
./install_isaac.sh                    # 仅首次安装/重建环境
./start_isaac.sh
```

实测主机 Ubuntu 24.04、RTX 5090 32 GB、NVIDIA 580.178.04。保留现有系统驱动和版本检查，当前版本可实际 RTX 渲染。官方安装/硬件说明：[pip 安装](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/install_python.html)、[要求](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html)。

## 当前初始姿态

2026-11-02 按用户要求，B2 改为源 source_zero_standing 的正常站姿，按四脚接地计算 base_link 高度约 0.522968 m；机械臂 e05l_joint1–6 初始角为 [0,1.2,2.3,0,1.3,0] rad。配置、MJCF 初始浮动基座、RoboSimStudio 关节/目标、USD body 变换、JointState 和 DriveTarget 共用 robot_initial.py。初始姿态与 transport_arm_q_rad 的导航收臂配置独立。

Isaac 启动时直接读回 USD 中六轴状态/驱动值和基座高度，并写入 isaac_runtime_validation.json 的 initial_pose_usd_readback。采样碰撞为 0，四脚距地约 0.1 mm，初姿静态质心支撑余量约 0.2400 m；这些不是起立过程或动态平衡验收。证据见 metadata/initial_pose_validation.json 和 initial_pose_usd_validation.json。已有焊接见证重新重放通过。

## 使用

```bash
./start_isaac.sh --camera overview
./start_isaac.sh --camera area_A
./start_isaac.sh --camera area_B
./start_isaac.sh --camera T_heights
./start_isaac.sh --camera S_heights
./start_isaac.sh --camera table_detail
./start_isaac.sh --camera robot --physics-inspector
```

默认时间轴停止。相机位于 `/World/Cameras`：overview、top、area_A、area_B、robot、S_detail、A05_detail、A08_detail、grid_detail、T_heights、S_heights、table_detail、butt_flat_detail、butt_inclined_detail、butt_steep_detail。

工件在 `/World/Fixtures/A01`–`A24`、`B01`–`B04`；每个工件的 WeldSeams 默认隐藏，可切换 visibility 看参考焊缝。焊缝和工件共享父变换；USD 编辑不会同步改外部任务 JSON。机器人在 `/World/Robot`，TCP 为 `/World/Robot/e05l_welding_torch/welding_tcp`。

```bash
./start_isaac.sh --headless --capture-all --frames 10
./start_isaac.sh --camera area_A --physics-inspector --probe-inspector --capture-ui --frames 10
```

第一条生成 15 个实际 RTX 视角，第二条是有界桌面关节控制检查并恢复源姿态；截图写入 screenshots/isaacsim。日常展示省略 probe 和 frames 参数。

## 原生 Physics Inspector

启动器通过 Kit 活跃异步循环加载 `omni.physx.supportui`；GUI 入口为 Tools > Physics > Physics Inspector。选择 `/World/Robot/base_link`，点击同步/刷新选择，滚动到 e05l_joint1–e05l_joint6。使用 Drive Target Position 的数值或滑块，单位为度，共有 18 个关节。

Fix Articulation Base 是 Inspector 内调试选项，防止局部调臂时底盘下落；交付 USD 没有世界固定关节，仍保留自由基座。Commit Changes 再 Ctrl+S 保存姿态；Discard 放弃修改。保持主时间轴停止，完整仿真前关闭 Inspector。

当前导出已补齐初始 JointState（含大于 180° 的多圈角）和驱动，原生 authoring 模式已实际请求 +5°、读回接近目标并恢复当前初始姿态。精确结果和 USD 哈希在 inspector_validation.json / isaac_gui_probe_validation.json；此检查不覆盖全工作空间或步态。[官方 Inspector 说明](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/physics/joint_inspector.html)。

## 文件与重新生成

- exports/usd/industrial_welding.usdc：正式二进制 OpenUSD；网格嵌入，需同级 textures/ 的三张 PBR 贴图。
- exports/usd/industrial_welding.usdz：含全部贴图、可单独转移打开。
- exports/industrial_welding_usd.zip：USD、贴图、元数据、文档、截图和校核记录。
- exports/usd/metadata/：配置快照、几何、焊缝、候选/见证、地图、来源、几何/布局/重放结果；侧录 JSON 不嵌入 USDZ。
- manifest.json / usd_validation.json：部件、质量、关节、哈希、独立重开与数值误差。
- isaac_rtx_capture_validation.json：当前 USD 的无窗口 RTX 15 视角结果。
- isaac_gui_probe_validation.json / inspector_validation.json：当前 USD 的桌面与原生关节检查。
- isaac_runtime_validation.json：最后一次启动记录，可能被普通展示启动覆盖。
- expansion_validation.json：当前场景完整性和独立 USDZ 转移检查。
- metadata/butt_joint_validation.json：真实双板、坡口、间隙和斜楔接触量测。
- initial_pose_usd_validation.json：当前初姿的配置、Isaac 状态/驱动读回、基座高度和原生关节控制复位结果。

在其他机器直接 File > Open USDC/USDZ，无需 RoboSimStudio。完整重建配置和验证命令见 README；仅重新打包当前源几何：

```bash
../../../../.venv/bin/python export_usd.py
```

## 对接焊定位接口

六件新焊缝 ID 为 Axx.butt_center，替代同工位旧 root_front。metadata/seams.json 中 joint_geometry 记录两块板的实体名称、1 mm 根部间隙、5.04 mm 顶部坡口开口、倾角及局部倾转轴；世界曲线和逐段完整 TCP 姿态随工件几何生成。打磨带是无碰撞外观层，实际钢板和楔座都有碰撞。可用 --camera butt_flat_detail / butt_inclined_detail / butt_steep_detail 查看。

## 保留内容与验证边界

环境含 32 个装配、1124 个几何部件（含 12 片双板工位的无碰撞打磨薄面）、1087 个静态碰撞体，另有 12 片 A08 打磨材质贴面（无碰撞/质量）。机器人保留 24 刚体、106 几何、67 碰撞体、18 转动关节、5 固定连接和总质量 96.6850 kg；共 1154 个碰撞形状。尺寸/惯性/限位来自 MuJoCo 编译的原 B2 + E05L-Pro 数据，避免 mesh 中心化/缩放丢失。临时安装板继续保持完全移除。

碰撞体是 guide purpose、默认隐藏；需查看时启用 guide 显示并解除其隐藏。环境碰撞固定，机器人保持自由基座；源相邻关节和显式碰撞排除按 USD joint/filter schema 记录。驱动参数是迁移初值，尚未完成 PhysX 接触等效、全身控制、步态和动态焊接验收。

OpenUSD 合规零错误、零警告，3 个本地贴图引用均解析。环境顶点最大误差约 1.91e-7 m、机器人网格误差 3.45e-13 m、TCP 误差 0、关节锚点误差 1.32e-8 m。当前 189 个代表段、2094 保存姿态在 MuJoCo 中通过完整姿态、限位、碰撞、支撑复查，其余 1757 段是候选。这不能直接视为 PhysX 验证。

Viser 面板、轨迹演示、焊接完成状态、导航/步态控制器没有迁移。交付优先保证场景和任务数据；没有执行脚本瞬移机器人演示。

钢材使用 USD Preview Surface 与本地程序 albedo（sRGB）、roughness（raw）、normal（raw，-1..1 切线法线），UV 按米铺设；steel_surface.py 可重建。八盏大面光和中性环境补光减轻硬阴影。材质是可调视觉近似，不是扫描或实测 BRDF。A08 亮窄带是打磨区而非焊道。[USD Preview Surface 定义](https://openusd.org/release/spec_usdpreviewsurface.html)。

## 闲置无响应规避与历史证据

Isaac Sim 6.2.0.0 自带 Kit 110.4.0；已独立复现其被取消、已到期 timer 导致异步循环停在 select 的缺陷。Kit 110.4.1 修复 OMPE-113948（[发布说明](https://docs.omniverse.nvidia.com/dev-guide/latest/release-notes/110_4_1.html)）。原卡死进程没有取得栈，不能据此认定唯一根因。

当前 start_isaac.sh 保留 `--/exts/omni.kit.async_engine/keep_loop_running=false`。不修改 NVIDIA 包；run/isaac_status.json 每 5 秒记录主循环心跳，120 秒无心跳输出栈至 run/isaac_<PID>_stacks.log。需要诊断时对 PID 发送 SIGUSR1 可输出 Python 栈。

2026-10-30 的真实 GUI 闲置 420 秒与随后短时两次 Inspector 控制复位记录属于历史 USD；档案在 run/before_visual_update_20261031/usd/。当时最后心跳 415.173 s、98,933 次更新、最长更新 68.33 ms；第一次终检因 Inspector 已被先前 probe 复位为 DISABLED，未完成组合终检，修正后短时验证通过。本版只重新检查当前 USD 的渲染和原生关节控制，未声称重跑七分钟闲置回归。上版完整场景保存在 run/before_expansion_20261102/。
