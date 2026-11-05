# B2 + E05L-Pro 工业焊接厂房


## 可上传、可编辑工程（2026-11-05）

完整工程包为 `exports/industrial_welding_project.zip`，包含源代码、配置、机器人源资产快照、材质和可编辑 USD。既可随 Git 同步，也可独立解压使用。完整说明见 [PORTABLE_PROJECT.md](PORTABLE_PROJECT.md)。

```bash
bash install_scene.sh        # 小型场景生成环境，第一次执行
bash install_isaac.sh        # 独立 Isaac Sim 环境，第一次执行
bash edit_isaac.sh           # 打开可保存修改的 USDA 覆盖层
# 修改 config.json 后：
bash project.sh build        # 重建几何/焊缝/候选/地图/USD，旧验证自动归档
bash project.sh pack         # 生成新的完整工程包
```

GUI 修改保存到 `exports/usd/industrial_welding_edit.usda`，参数化修改保存在 `config.json`。两者不会自动双向同步，详见使用说明。工程已设置 Git 白名单，不包含虚拟环境、run/ 和历史备份；本次未自动提交或推送。


基于 `dev` 工作区（基线提交 `41008da`）的独立 live twin。场景几何和焊缝由 `config.json`、`hall_geometry.py` 共同生成；Isaac Sim 直接加载导出的 USD。本次只修改本 twin，未修改库和旧的 unitree_b2_z1 场景。

## 当前场景：2026-11-03

按用户最新截图去掉桌下悬空横条（原静态碰撞横撑，与四腿没有连接、无控制用途）和白色钢板堆。保留台面、四腿、脚板及相应碰撞。按新参考图将六个 T 型工位改为水平/倾斜双板对接焊；A05 当前为水平对接焊，坡口根部距地 0.85 m。

厂房由 24×18 m 扩为 **32×32 m（1024 m²）**。北侧 A 区从 8 增至 **24 个工位：8 个 T 型、6 个双板对接焊、7 个开放 S 型、1 个竖直 L 角接、1 个倾斜 T 接头、1 个开口加强支架**。普通 T 型保留七档根部高度（低位两件、其余各一件），S 型各放一件：0.25、0.40、0.55、0.70、0.85、1.00、1.15 m。四排六列、部分工位斜放，台架平面 AABB 间距最小约 2.22 m；实际作业站位另以完整机器人碰撞检查。

南侧 B 区从 2 增至 **4 块大型板架**，总焊缝长度从约 160.7 m 增至 444.3 m。主通道 2.8 m，待命区、外围环线和全部 28 个工件接近目标连通。保留开放屋顶的 6 m 梁柱；未加入参考图的龙门设备。坐标统一米制、Z 向上，四元数 XYZW。

| 板架 | 类型 | 尺寸 m | 内部作业通道最小净宽 m |
|---|---|---:|---:|
| B01 | 5 条平行筋，间距 0.70 m | 12×3.6 | 2.100 |
| B02 | 网格筋板、十字通道 | 10×6.2 | 2.088 |
| B03 | 新增网格筋板、十字通道 | 10×5.6 | 2.088 |
| B04 | 新增 4 条平行筋，间距 0.65 m | 10×2.8 | 2.100 |

筋板高 0.20/0.22 m，厚 12 mm，交汇处分段并留 1 mm 拼接间隙；各通道入口有 0.4 m 长实体坡道。封闭格内部保留 `restricted_closed_cell` 访问标记，不默认 B2 跨越筋板。

## 启动 Isaac Sim

```bash
cd ~/code/robosimstudio/examples/live/twins/industrial_welding
./start_isaac.sh
./start_isaac.sh --camera area_A
./start_isaac.sh --camera S_heights
./start_isaac.sh --camera robot --physics-inspector
```

默认时间轴停止，用于展示和编辑。15 个相机包括 overview、top、area_A、area_B、robot、S_detail、A05_detail、A08_detail、grid_detail、T_heights、S_heights、table_detail、butt_flat_detail、butt_inclined_detail、butt_steep_detail。原生 Physics Inspector 选择 `/World/Robot/base_link`，在主时间轴停止时使用；详见 [ISAACSIM.md](ISAACSIM.md)。

单独转移 `exports/usd/industrial_welding.usdz` 可在其他 Isaac Sim 安装中 File > Open。普通 `industrial_welding.usdc` 需保留同级 `textures/`。完整交付包为 `exports/industrial_welding_usd.zip`，另含地图、焊缝、验证和截图。

## 双板对接焊工位（2026-11-03）

A02、A05、A09、A11、A17、A24 已由 T 接头改成两块并排、分别建模的 12 mm 厚钢板，组合尺寸约 1.00×0.56 m。中央为 1 mm 根部装配间隙、深 3.5 mm、夹角 60° 的浅 V 坡口，顶部开口约 5.04 mm，焊缝长 0.86 m。两侧各 20 mm 的浅色窄带表示打磨准备区，中央接缝来自真实钢板边界，不是画在整块平板上的假线；没有生成完成焊道。

| 工位 | 钢板倾角 | 台面高度 m | 坡口根部高度 m |
|---|---:|---:|---:|
| A02 | 0° | 0.5295 | 0.550 |
| A05 | 0° | 0.8295 | 0.850 |
| A09 | 15° | 0.4453 | 0.550 |
| A11 | 30° | 0.5286 | 0.700 |
| A17 | 45° | 0.6220 | 0.850 |
| A24 | 60° | 0.7293 | 1.000 |

倾角指钢板平面相对水平面，绕工件局部 +X（焊缝方向）倾转。六件全部低于 90°，没有完全竖直。水平件直接承载于底垫板；倾斜件由两只实体斜楔支承，并有四个接触钢板表面的夹块。两块工件保持独立的有限厚度凸实体和碰撞体。

其余 8 个普通 T 接头、7 个开放 S 工件和 3 个其他工位保留；T/S 仍覆盖 0.25–1.15 m 七档根部高度。新接口 ID 为 A02.butt_center、A05.butt_center、A09.butt_center、A11.butt_center、A17.butt_center、A24.butt_center；对应旧 root_front ID 已移除，客户端应重新枚举。

参数位于 config.json 对应工位：pair_size_m、incline_deg、root_gap_m、groove_depth_m、groove_included_angle_deg、backing_thickness_m、fixture_lift_m、preparation_band_width_m、plate_colors_rgb、prepared_color_rgb。table_height_m 是实际输入支撑台高度；root_height_target_m 是当前布局的校核参考，修改高度/角度后以导出的实际根部高度为准并重新验证。

近景命令：

```bash
./start_isaac.sh --camera butt_flat_detail
./start_isaac.sh --camera butt_inclined_detail
./start_isaac.sh --camera butt_steep_detail
```

## 工位高度

台面高度是支撑面，实际根部焊缝高度包含底板厚度。A 区板厚 10–12 mm，开放 S 筋板沿同侧根部连续、含两个反向弯曲段，长约 1 m，由 80 片相接凸实体近似。

| 工位 | 工件 | 台面高度 m | 实际焊缝高度 m |
|---|---|---:|---:|
| A01 | T 型根部角焊 | 0.238 | 0.250 |
| A02 | 水平双板 V 坡口对接 | 0.5295 | 0.550 |
| A03 | T 型根部角焊 | 1.138 | 1.150 |
| A04 | 竖直 L 角接 | 0.338 | 0.378–0.718 |
| A05 | 水平双板 V 坡口对接 | 0.8295 | 0.850 |
| A06 | 22° 倾斜 T 接头 | 0.538 | 0.613–0.898 |
| A07 | 开放 S 型根部角焊 | 0.538 | 0.550 |
| A08 | 开口 L 支架与三角肋 | 0.538 | 代表根部 0.548；其他见数据 |
| A09 | 15° 双板 V 坡口对接 | 0.4453 | 0.550 |
| A10 | 开放 S 型根部角焊 | 0.238 | 0.250 |
| A11 | 30° 双板 V 坡口对接 | 0.5286 | 0.700 |
| A12 | 开放 S 型根部角焊 | 0.388 | 0.400 |
| A13 | T 型根部角焊 | 0.388 | 0.400 |
| A14 | 开放 S 型根部角焊 | 0.688 | 0.700 |
| A15 | T 型根部角焊 | 0.688 | 0.700 |
| A16 | 开放 S 型根部角焊 | 0.838 | 0.850 |
| A17 | 45° 双板 V 坡口对接 | 0.6220 | 0.850 |
| A18 | 开放 S 型根部角焊 | 0.988 | 1.000 |
| A19 | T 型根部角焊 | 0.988 | 1.000 |
| A20 | 开放 S 型根部角焊 | 1.138 | 1.150 |
| A21 | T 型根部角焊 | 0.238 | 0.250 |
| A22 | T 型根部角焊 | 0.538 | 0.550 |
| A23 | T 型根部角焊 | 0.838 | 0.850 |
| A24 | 60° 双板 V 坡口对接 | 0.7293 | 1.000 |

## 机器人与来源

B2 使用用户提供的 `~/code/go2_rl_lab` 无雷达/防护架简化模型及 E05L-Pro，保留尺寸、关节限制、碰撞和惯性，包括源 Link6/end_Link 重复质量。机器人未换型号。所需来源文件和 LICENSE 已快照至 `assets/assembled/source/`，来源路径及 SHA256 在 `exports/asset_manifest.json`。

源安装位姿 `(-0.025,0.020,0.075) m`、RPY `(0,0,0)` 保留。按此前要求，临时安装板完全移除（实体、6.7824 kg 质量和碰撞），`robot.mount_plate_enabled=false`。实机安装连接仍待确认。当前机器人总质量约 96.6850 kg。

焊枪复用源包共享 CAD（源 E10L 工具资产），机械臂仍是 E05L-Pro。焊枪碰撞由 26 个连通组件凸体组成，沿用 5 kg 载荷设定；惯性和 TCP 为 CAD 估计而非硬件标定。`torch_mount_delta_*` 改实体安装，`torch_tcp_delta_*` 校准 TCP；TCP +Z 为喷嘴出口方向。

五档源模型限位内腿姿对应机身距承载面约 0.289/0.396/0.493/0.575/0.595 m。最高档用于高位 S 焊缝的姿态校核；这是几何可行值，不是厂家认证的连续工作高度范围。收臂 XY 包络圆半径最大约 0.8085 m、最高点约 1.6544 m。地图采用 0.85 m 膨胀半径、1.70 m 高度过滤，并复查五档包络。

## 场景初始姿态

启动时 B2 使用来源 `flat_b2_e05l_pro_simplified.xml` 的 `source_zero_standing` 正常站姿：前腿 thigh=0.8 rad、后腿 thigh=1.0 rad、四条 calf=-1.5 rad，hip 左/右为 ±0.1 rad，沿用该关键帧的轻微机身倾角。按四个源脚底球对齐地面，机身基准高度约 **0.522968 m**，保留 0.1 mm 数值间隙。

机械臂初始角度按 e05l_joint1 到 e05l_joint6 顺序设为 **[0, 1.2, 2.3, 0, 1.3, 0] rad**。配置入口是 robot.initial_arm_q_rad、initial_leg_q_rad、initial_base_quat_xyzw、initial_ground_clearance_m。robot_initial.py 统一 MJCF、RoboSimStudio 导入器和 USD 的初始化，驱动目标与初始关节状态同步写入。

初始展示姿态与 robot.transport_arm_q_rad 分开，后者仍用于导航包络及工作站位校核。初姿 18 个关节限位、四脚支撑和碰撞检查通过，最小静态质心投影余量约 0.2400 m；Isaac 读回机械臂角度与配置误差小于 1e-6 rad。已有 189 段、2094 个焊接姿态在最终机器人模型上重新重放通过。没有增加自动起立或行走控制，主时间轴仍停止。

## 配置和数据接口

编辑 `config.json` 可修改厂房、A 区列表、B 区 parallel/grid/additional_panels、位置朝向、台面高度、板厚、筋板尺寸与间距、通道、障碍、源安装增量、焊枪/TCP、灯光。`local_aisle_m` 是设计参考值，不会自动挪动工位。几何改变后必须重新验证；不能继续使用旧站位见证。

```bash
# 重新求解代表焊缝并重建几何/地图/站位（较慢）
../../../../.venv/bin/python validate_scene.py --fresh
../../../../.venv/bin/python coverage_validation.py
../../../../.venv/bin/python export_data.py
../../../../.venv/bin/python replay_validation.py
../../../../.venv/bin/python geometry_audit.py
../../../../.venv/bin/python layout_validation.py
# 输出 USD，再由 Isaac 加载
../../../../.venv/bin/python export_usd.py
./start_isaac.sh
```

`coverage_validation.py` 在来源关节限制内查找逐段完整姿态，保存成功见证；个别困难段可能需要调整配置中显式记录的焊枪滚转角，再重做验证。不得仅凭 TCP 位置接近标记成功。

| 数据 | 内容 |
|---|---|
| exports/scene_geometry.json | 工件局部实体、碰撞、装配位姿 |
| exports/seams.json | 317 个唯一焊缝、采样、切向/面法向、世界/局部位姿、逐段完整 TCP 姿态、接近/退出、1946 段与访问限制 |
| exports/stances.json | 17514 个未验证候选站位及独立 verified 见证 |
| exports/validation.json | 28 个工件各一条代表焊缝的全段验证及配置/几何/焊缝哈希 |
| exports/initial_pose_validation.json | 正常站姿、用户机械臂初角、关节限位、脚底高度、初始碰撞和静态支撑 |
| exports/witness_replay.json | 保存姿态独立重放、限位、碰撞、四足支撑和质心投影 |
| exports/butt_joint_validation.json | 六件双板厚度、间隙、坡口开口、实体体积、倾角与支承接触 |
| exports/geometry_audit.json | 厚板交叠、焊缝根部边界、支撑高度 |
| exports/layout_validation.json | 新旧统计、台间距、装配交叠、收臂包络与连通性 |
| exports/map.pgm、map.yaml、map_inflated.pgm、map_preview.png | 0.10 m 几何真值占据图及膨胀预览 |
| exports/navigation.json | 多高度障碍、28 个目标、连通性与地图来源 |

地图由场景几何生成，不是 SLAM。尚未接入雷达、定位、路径执行和步态。薄板按栅格覆盖保守投影。工件整体移动时局部焊缝同步；`WeldingSceneAPI(pose_provider=...)` 可接实时位姿，USD 编辑不会自动回写外部 JSON 快照。

```bash
../../../../.venv/bin/python welding_api.py workpieces
../../../../.venv/bin/python welding_api.py seams --workpiece A20
../../../../.venv/bin/python welding_api.py task --seam A20.open_S_root
```

任务接口预留 `WAIT_NAVIGATION → ADJUST_BASE_POSTURE → APPROACH → WELD_TCP_PATH → RETRACT → REFOLD_ARM`。`execution_enabled=false`，逐段退枪/收臂后再换位或切换工艺姿态。

## 校核范围

317 条焊缝（A 区 29、B 区 288），1946 段；28 条代表焊缝共 **189 段、2094 个保存姿态**通过完整 TCP 位置/方向、六轴关节限位、采样自碰撞/场景碰撞、四足承载面、接近/退出和关节插值检查。最大位置重放误差约 0.0005625 m、姿态误差 0.1358°，最小静态质心投影余量约 0.14362 m。其余 **1757 段未验证**，仍是候选接口，封闭格访问限制继续保留。

几何审计未发现钢板或不同装配的正体积穿插，根部与边界最大误差 1 mm（预留间隙）。初始接触和全部代表站位记录在 JSON。以上是 MuJoCo 数值采样及必要静态条件，不是连续碰撞、力矩、硬件精度或 PhysX 动态稳定性证明。没有新增自动焊接、全身协调、步态、熔池或热过程。

## 原 RoboSimStudio live 入口

```bash
./start.sh                           # 默认 http://localhost:8081
./reload.sh                          # 配置更新后重建
./ctl play 'Welding controls' --wait
./ctl quit
```

不要直接运行 `scene.py`。原侧栏保留工位/焊缝选择、相机、焊缝/TCP/候选/验证站位/碰撞/导航目标显示。TCP 预览只画轨迹，不移动机器人，task_state 中记录 `robot_executed=false`。这些 Viser 控件和焊接演示功能未迁移至 Isaac。

本次修改验证集中于几何、任务数据和 Isaac 运行，未启动原 live 服务。此前 live 漂移测试与闲置回归属于历史配置，不作为本版动态验收。

## 图片与材质

当前截图均在 `screenshots/isaacsim/`：全景、俯视、两区、机器人、S/T/网格/支架近景、七档高度、桌下结构，以及水平/45°/60° 双板对接焊近景。灰蓝轧制钢采用 3 张本地程序 PBR 贴图，八盏面光配合中性补光减少硬阴影。材质为视觉近似，未宣称实测光学参数；A08 的窄亮带是打磨准备区，不是焊接完成状态。
