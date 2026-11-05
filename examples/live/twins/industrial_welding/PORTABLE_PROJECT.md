# 工业焊接场景：上传、加载和修改

本文件对应 2026-11-05 的可编辑工程交付。工程可以作为当前 RoboSimStudio 仓库的一部分同步，也可以将 industrial_welding_project.zip 解压为独立文件夹。独立使用时，编辑场景和导出 USD 不依赖 RoboSimStudio、go2_rl_lab 或原电脑的绝对路径。

包含场景 Python 源码、config.json、B2/E05L-Pro/焊枪源模型快照及许可、材质生成器、本地 PBR 贴图、USDC/USDZ、可编辑 USD 图层、焊缝与地图数据、当前验证记录、示例截图、固定依赖及安装脚本。不包含虚拟环境、缓存、历史备份或运行日志。

## 1. 在新电脑加载

以下脚本针对 Linux Bash / Python 3.12，Isaac 图形显示还需要兼容的 NVIDIA GPU、驱动和桌面环境。先安装 uv，并确保 `uv` 可从 PATH 找到；也可以设置 `UV=/path/to/uv`。

在解压后的 industrial_welding 目录，或仓库的 examples/live/twins/industrial_welding 目录执行：

```bash
# 第一次使用：安装小型场景生成环境（不安装 Isaac 或 RoboSimStudio）
bash install_scene.sh

# 验证包内资产、焊缝数据和 USD 引用
bash project.sh check

# 需要 Isaac GUI 时，第一次执行一次，保持独立环境
bash install_isaac.sh

# 打开可编辑工程层
bash edit_isaac.sh
```

install_scene.sh 根据 requirements-scene.lock.txt 的固定版本与哈希安装到 .venv-scene。install_isaac.sh 安装 Isaac Sim 6.2.0.0 / Torch 2.11.0 到独立 .venv-isaacsim；不要将两套环境合并。isaacsim_requirements.lock.txt 是原验证机的完整版本清单，安装入口使用 install_isaac.sh 中的主要版本约束。

已经有可运行的 Isaac Python 环境时，也可设置 `ISAAC_PYTHON=/path/to/isaac/python` 后运行 edit_isaac.sh。已经有 Isaac Sim 的机器无需重装，可以直接 File > Open 打开 exports/usd/industrial_welding_edit.usda；显示参数和闲置卡死修复应参考 ISAACSIM.md，启动脚本已经包含该修复。

## 2. 在 Isaac GUI 中修改

编辑入口是 exports/usd/industrial_welding_edit.usda。它用相对路径引用 industrial_welding.usdc，打开后以该 USDA 为编辑目标，移动工位、调整灯光、材质或相机，再 Ctrl+S 保存。需要在 Layer 面板确认编辑目标为 industrial_welding_edit.usda。

生成的 industrial_welding.usdc 是底层场景；USDA 保存 GUI 覆盖层。重建会保留覆盖层。若覆盖层中已修改某个属性，它会继续覆盖 config.json 的新生成结果；需要时在 Isaac 中清除对应 override，或另存一个新的编辑层。

USD 中每个工件下的 WeldSeams 曲线会随工件移动。GUI 修改不会自动回写 config.json、外部 seams.json、地图、候选站位或验证结果。需要同步研究数据时，请使用下节的参数化方式修改并重新生成。不要将旧验证记录用于 GUI 移动后的布局。

```bash
# 原始生成场景的展示入口仍保留
bash start_isaac.sh
# 编辑层 + 机械臂关节面板
bash edit_isaac.sh --camera robot --physics-inspector
```

## 3. 修改参数后重新生成

编辑 config.json：hall.size_m、stations、parallel、grid、additional_panels、robot、lighting、appearance 分别控制厂房、A 区工位、B 区板架、机器人初姿/安装与光照材质。A 区工位通过 xy（米）和 yaw_deg（度）配置平面位置与朝向；导出的 pose 接口前三项为世界位置、后四项为 XYZW 四元数。统一米制、Z 向上。更复杂的工件几何可修改 hall_geometry.py；材质和导出逻辑在 steel_surface.py、export_usd.py。

```bash
# 快速重建：实体、焊缝、候选站位、几何地图和 USD
bash project.sh build
bash edit_isaac.sh

# 需要重新计算完整代表性焊缝的采样可行性时（耗时较长）
bash project.sh build --validate
```

build 会把上一版导出物和旧截图归档到 run/build_history/，重新导出当前几何与焊缝，并清除旧的“已验证”标记。快速构建输出的站位均为未验证候选，不会伪装为已通过 IK/碰撞检查。build --validate 额外调用现有离线验证、覆盖与重放流程，不运行机器人或导航控制器。已有的焊接工艺/硬件标定假设见 README.md、REPORT.md。

新布局需要新截图时：

```bash
bash edit_isaac.sh --headless --capture-cameras overview robot area_A grid_detail --frames 10
```

## 4. Git 上传

在 RoboSimStudio 仓库中，根 .gitignore 已为 industrial_welding 添加例外；场景自身 .gitignore 使用交付文件白名单。源码、配置、源模型快照、材质和必要导出文件可随 Git 同步，.venv*、run/、历史备份、临时文件和生成的 ZIP 均被排除。

只添加本次工程和根忽略规则，避免带入其他场景的未提交修改：

```bash
# 在 RoboSimStudio 仓库根目录执行
git add .gitignore examples/live/twins/industrial_welding
git diff --cached --stat
git commit -m "scene: package the editable industrial welding project"
git push
```

本次整理没有自动提交或推送。包内模型文件为实体数据，不依赖额外 Git LFS 下载。独立工程也可以自行 git init 后上传；保留文件层级和许可文件。

## 5. 重新打包与校验

```bash
# 输出 exports/industrial_welding_project.zip 及同名 .zip.sha256
bash project.sh pack

# 刚解压、尚未修改时验证逐文件 SHA256
bash project.sh verify
```

project_manifest.json 在 ZIP 内记录逐文件路径、大小和 SHA256。verify 检查交付原件，主动编辑后哈希不再匹配是正常现象；此时用 check 检查工程状态，再 pack 生成新交付。

仅用于打开现有场景时可以继续单独发送 USDZ；需要修改、重新生成或继续研究时，应发送完整 project ZIP。早期 industrial_welding_usd.zip 是只含 USD 与元数据的展示包，不是本次可编辑源码工程。

## 可选：原 RoboSimStudio 交互页面

scene.py、fixtures.py、welding_ui.py 等原入口已保留。在完整 RoboSimStudio 仓库按其 uv.lock 安装后，使用 bash start.sh 启动。独立 ZIP 不包含 RoboSimStudio 主库，不能把 start.sh 当作独立环境入口；独立场景修改/重建使用 project.sh，图形显示使用 edit_isaac.sh。
