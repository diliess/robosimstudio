# B2（无雷达/护罩）+ E05L-Pro 船板格架场景

当前机器人为 `unitree_b2_e05l_pro`。沿用原场景目录 `unitree_b2_z1/`，既有启动路径不变。
场景包含 3×3 船板格架，机器狗位于中央格；E05L-Pro 六轴通过页面的 `Arm: e05l` 操作。

## 启动

项目根目录已有 uv Python 3.12 环境。首次接入模型默认读取同级的 `go2_rl_lab`：

```bash
cd ~/code/robosimstudio
uv sync --frozen --extra cpu --extra dev
cd examples/live/twins/unitree_b2_z1
CUDA_VISIBLE_DEVICES=0 ./ctl up --port 8081
./ctl status --wait
```

打开 `up` 输出的 URL；8081 空闲时为 http://localhost:8081。
保存 `scene.py` 或辅助 Python 模块后自动重建。`./ctl quit` 停止本页面。
首次 RTX 截图编译着色器可能需要数分钟。

如果来源仓库不在同级目录，在首次启动前指定它：

```bash
GO2_RL_LAB_ROOT=/your/path/go2_rl_lab CUDA_VISIBLE_DEVICES=0 ./ctl up --port 8081
```

首次加载会把需要的 MJCF、网格、安装元数据和 TCP 元数据复制到
`assets/assembled/source/`，并按内容哈希生成 `b2_e05l_pro.xml`。
已有完整快照时可离开来源仓库运行。这里不再要求下载 Z1 或官方 B2 仓库。
`assets/assembled/` 和运行输出仍由现有 Git 忽略规则排除。

## 模型和安装位置

来源仓库：`/home/idea/code/go2_rl_lab`。

- 组合 MJCF：`deploy/deploy_mujoco/assets/b2_e05l_pro_simplified.xml`。
- 无雷达/护罩外壳：`b2/base_link_no_lidar_guard.obj`；保留来源模型的 D435i。
- E05L-Pro 的原始网格、质量、惯性和六轴关节限位保持来源值。
- 保留来源组合模型的 5 kg 焊枪及 26 个部件凸包。组合总质量约 96.685 kg，
  包括来源有意保留的 Link6/end_Link 重复端部质量。
- 安装位置由 `source/go2_rl_lab/go2_rl_lab/assets/urdf/b2_e05l_pro/integration.json`
  和组合 MJCF 共同给出：B2 `base_link` 下 `(-0.025, +0.020, +0.075) m`，RPY 为零。
  +X 向狗头，+Y 向左。来源研究报告另推荐 `(0, 0, 0.075)` 作为已复核候选；
  正式模型未采用该候选，本场景沿用正式默认值。
- `scene.py` 的 `ARM_MOUNT_XYZ`、`ARM_MOUNT_RPY` 可调整安装位置与姿态。
- TCP 来自 `welding_tip.json` 的短弯头出口中心，在 `e05l_welding_torch` 坐标系中为
  `(-0.01118192, -0.00097361, +0.31412248) m`。工具 +Z 沿来源出口法线。
  这是 CAD 几何参考，尚未硬件标定。
- 初始腿姿、机械臂姿态及低站姿高度来自来源 `visualization_pose.json` 和平地场景。

B2 保留自由底座，由 12 个腿部位置伺服和真实足端接触支承。
腿部驱动力限制沿用来源的 200/320 Nm；E05L 使用仿真位置伺服（Kp=50000、Kd=400），
未假设厂家未提供的电机额定能力。Newton IK 使用当前自由底座位置，避免固定根坐标的偏移。
另生成保留来源关节变换和限位的独立 `e05l_kinematics.urdf` 供核对。

## 格架和视角

| 参数 | 值 |
| --- | --- |
| 格数 | 3×3，中央格索引 `(1, 1)` |
| 单格净空 | 2.00×2.00 m |
| 立板高度 | 220 mm |
| 板厚 | 10 mm |
| T 形足板宽 | 180 mm |
| 底板 | 约 6.45×6.45 m |

布局参数位于 `scene.py`。相机为 `overview`、`top`、`center_cell`、`part_detail`。
页面的 Camera 选中相机后点击 Look through 查看；机械臂末端 gizmo 对应焊枪 TCP。

```bash
./ctl snapshot --camera center_cell --size 960x600
./ctl check --camera center_cell
```

当前是站立和机械臂交互场景，未接入四足行走、跨格策略或焊接工艺仿真。
`export_environment.py` 继续只导出格架；本次没有重新生成或验证历史 QuadWeld 导出。
本次验证见 `REPORT.md` 与 `run/b2_e05l_validation.json`。
