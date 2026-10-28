# 宇树 B2 + Z1 船板焊接板格场景

在本目录运行 `./ctl up` 启动实时页面；当前地址为 http://localhost:8081。
`./ctl status --wait` 查看构建状态，`./ctl quit` 停止本场景。
保存 `scene.py` 或旁边的辅助 Python 模块后，页面自动重建，无需重启。

## 当前布局

用户确认使用 3×3 个矩形格，机器狗位于中央格。每个格子由四条 T 形边界围成，
相邻格子共用边界，所有板件落在同一连续钢底板上，模拟船板与纵横加强肋的布置。

| 参数 | 当前值 |
| --- | --- |
| 格子数量 | 3 行 × 3 列，共 9 格 |
| 每格净空 | 2.00 × 2.00 m，按立板内表面计算 |
| 立板高度 | 220 mm |
| 板厚 | 10 mm，底板、足板和立板相同 |
| T 形足板宽度 | 180 mm |
| 肋条总高度 | 230 mm，高于连续底板上表面 |
| 整体底板尺寸 | 约 6.45 × 6.45 m |
| 机器人所在格 | 中央格，索引 `(1, 1)`，从 0 开始 |

格架是固定且不可分离的船板结构。B2 保留自由底座和 12 个腿部关节，
足端真实接触钢底板承重；背部 Z1 保留六轴机械臂和原生夹爪。
腿部位置伺服维持站立，Z1 可通过页面的机械臂面板操作。

本次完成场景布局；尚未接入四足行走、跨格、焊枪、焊接轨迹、热影响或熔池仿真。

## 只导出环境到 QuadWeld

`export_environment.py` 从实时编译模型提取格架，不包含 B2、Z1 或其安装板。
通过 `./ctl eval` 调用，输出在 `assets/export/ship-grid/`，目录内包含
`ship_grid.usda`、两张相对路径贴图、实际几何清单 `manifest.json` 和导入说明。

```bash
./ctl eval - --wait <<'PY'
from export_environment import export_environment
export_environment(sim, grid)
PY
```

资产已复制到 `/home/hlq/QuadWeld/assets/environments/ship-grid/`，
该项目新增 `scene/environment.py` 和 `main.py --environment-usd` 预览入口。
目标项目继续使用自己的机器人配置。米制、Z 向上，中央格底板上表面为 z=0。
33 个静态碰撞箱体保留，周围场地仅显示；导出不包含原场景的无限地平面。
USD 标准检查及目标项目 13 项相关 CPU 测试通过；本机没有 Isaac Sim 6，未验证其中的渲染或任务运行。
详细接口见导出目录的 `README.md`，验证记录见 QuadWeld 的 `outputs/runs/ship_grid_import/validation.json`。

## 修改布局

编辑 `scene.py` 开头的参数：

```python
GRID_ROWS = 3
GRID_COLUMNS = 3
CELL_CLEAR_SIZE = 2.0
WEB_HEIGHT = 0.220
PLATE_THICKNESS = 0.010
FLANGE_WIDTH = 0.180
ACTIVE_CELL = (GRID_ROWS // 2, GRID_COLUMNS // 2)
```

几何、机器狗初始位置及总览和俯视相机随布局参数生成；
`part_detail` 是当前中央格一处交叉节点的固定细节视角，需要时可单独调整。

## 文件与视角

- `scene.py`：布局参数、机器人、场地、相机。
- `ship_grid.py`：共享边界的格架、T 形足板、立板、连续底板与金属外观。
- `scene_parts.py`：保留 B2 浮动底座的导入器；旧单件工件实现保留但不用于当前布局。
- `prepare_assets.py`：官方 B2 与 Z1 装配、独立 Z1 运动学模型和程序金属纹理。
- `ref/unwelded_01.png`：用户原始 T 形工件图片，原样保存，用于截面及材质参考。
- `assets/`：官方模型、许可证、生成的装配 XML 和纹理。
- `REPORT.md`：当前简化、假设和验证结果。
- `run/`：截图、对照、验证 JSON 和日志；原工作台布局备份为 `scene_before_ship_grid.py`。

相机：`overview` 看整个 3×3 板格，`top` 俯视格子分布，
`center_cell` 近看中央格中的机器狗，`part_detail` 看板件交叉节点。

## 官方模型来源

B2：[unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)，
提交 `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`。
Z1：[unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros)，
提交 `073aec51f548add65baf482ea8bb3be4cb81b7ad`。
原生夹爪来自官方 xacro 可选分支；BSD-3-Clause 许可证保留在对应目录。
