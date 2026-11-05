# B2（无雷达/护罩）+ E05L-Pro 船板格架场景

## What still differs from the reference

按用户指定的 go2_rl_lab 模型替换 B2+Z1。没有新的机器人照片或动作视频；
ref/unwelded_01.png 仍只作为既有 T 形工件参考，不用于声称整场景或机器人照片匹配。
当前支持低站姿与机械臂交互，未接入四足行走、跨格、焊接路径、熔池或热影响仿真。

## Shortcuts and departures from the reference

B2 保留 FREE 底座、12 个腿关节和真实足端接触，没有锁定底座或施加外部支撑。
机器人自身碰撞沿用原场景关闭设置，与船板碰撞保留。
腿部位置伺服 Kp=1800、Kd=100，力矩上限沿用来源 200/320 Nm；
E05L 位置伺服 Kp=50000、Kd=400，不设电机力矩上限。这是仿真设置，
来源 URDF 的 effort/velocity=0 未提供真实电机额定能力；本次不作硬件能力结论。
保留来源质量、惯性、角度限位、Link6/end_Link 端部质量及 5 kg 焊枪与 26 部件凸包。
CAD 估计的焊枪惯性和 TCP 均未经硬件标定。

使用现有 Newton IK 跟随实时自由底座。固定根坐标 PyRoKi 在底座沉降后产生约 20 mm
目标偏差，未用于当前交互。仍生成来源关节变换和限位对应的独立运动学 URDF供核对。
船板格架仍为固定结构；本次未重建或验证历史 QuadWeld 导出。

## Readings judged

- run/snap_001.png：已打开查看 RTX 近景，确认无雷达/护罩 B2、E05L-Pro、焊枪及中央格显示正确。
- run/check.json：builds、finite、camera、objects、penetration、rest、finite_after 七项 PASS。
  明确检查 robot_asset、ship_grid、unitree_b2_e05l_pro。最大穿透约 0.31 mm，低于 1 mm；
  60 步可读对象最大漂移约 0.01 mm。格架和无独立位置的机器人句柄未由 rest 测量。
- run/b2_e05l_validation.json：实际质量 96.68501282 kg，焊枪 5 kg，底座为 FREE；
  安装位置还原误差小于 1e-6 m，站稳后 120 步底座漂移 0.0166 mm。
  三个方向各 20 mm 的 IK 目标最大位置误差
  0.000993 mm。
  通过实际页面面板完成 20 mm TCP 往返，去程误差 1.448 mm，
  回程 1.509 mm，均低于 6 mm 容差，状态始终有限。
- run/validate_e05l.py：可复现上述诊断，通过 ctl eval 注册并由 ctl play 执行，临时 routines 已清除。
- run/b2_e05l_portability.json：隐藏来源仓库后，场景内资产快照可独立使用；
  另用 MuJoCo 加载快照模型，得到源总质量 96.68502053281073 kg。
- 三个修改的 Python 文件通过 Ruff 检查与格式检查。此为 live 场景修改，未运行库测试 tiers。

## Assumptions

来源：go2_rl_lab/deploy/deploy_mujoco/assets/b2_e05l_pro_simplified.xml 及配套组合 URDF、
integration.json、visualization_pose.json、welding_tip.json。所需网格和元数据复制到
assets/assembled/source/；prepared_e05l.json 保存输入 SHA-256。

正式默认安装位置：B2 base_link 下 (-0.025,+0.020,+0.075) m，RPY=(0,0,0)，
+X 向狗头、+Y 向左。来源报告另有候选推荐点 (0,0,0.075)，本场景未自动套用。
可在 scene.py 的 ARM_MOUNT_XYZ / ARM_MOUNT_RPY 调整。
短弯头 TCP 在 e05l_welding_torch 中为 (-0.01118192,-0.00097361,+0.31412248) m，
工具 +Z 沿来源出口法线。D435i 保留。
初始腿姿和机械臂姿态来自来源可视化配置：髋 ±0.1、大腿 1.175、小腿 -2.35 rad，
初始基座约 0.288588 m，平衡后约 0.272054 m。
格架为 3×3，净空 2×2 m，立板 220 mm、板厚 10 mm、足板宽 180 mm，机器人位于中央格。
