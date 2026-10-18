# H7 与 PC Python 控制链路

固件基线：主分支 `2bbb192`。Python 控制器使用显式时间调度，每臂独立保存状态。关节输入使用 deg、deg/s。电机接口使用 rad、rad/s、N·m。

## 源码对应关系

H7 源码路径相对于 [H7 控制工程](https://github.com/Crystal-1438/ARX_X5_DMH7) 根目录。
Python 实现位于 SDK 仓库的 `arx_x5_control/`。

| H7 源码 | 控制内容 | Python 实现 |
| --- | --- | --- |
| `App/arm.c`、`cmd.c` | 双臂实例、命令处理顺序 | `Arm`、`ControlSystem.step` |
| `App/robot_state.c` | 握手、状态机、看门狗、反馈 | `Arm.handle/state_step/snapshot` |
| `App/gimbal_cmd.c` | 控制源、软限位、TD、五次样条、参考值检查 | `CommandLayer` |
| `App/arm_auto.c` | 复位、夹爪找零 | `CommandLayer._reset/_gripper_zero` |
| `App/arm_zero.c` | 电机校准、ACK、回读、超时 | `ZeroCalibration` |
| `App/gimbal_arm.c` | 坐标转换、动力学、模式增益、电机参考量 | `Arm.drive`、`Motor.control` |
| `App/inc/gimbal_arm_def.h` | 方向、减速比、零点、前馈配置 | `ArmConfig` |
| `App/inc/gimbal_cmd_def.h` | 软限位、规划速度系数、样条时长 | `config.py` |
| `libs/controller/pid.c` | 位置 PID | `PID` |
| `libs/controller/adrc.c` | TD 位置、速度、加速度 | `TrackingDifferentiator` |
| `libs/controller/controller.c` | 位置与速度 PID 级联 | `Motor.control` |
| `libs/move_plan/` | 样条、多段轨迹、同步规划组 | `Spline`、`Trajectory`、`PlanGroup` |
| `libs/gravity_compensation/c/x5_gravity.c` | 静态重力补偿 | `GravityCompensator` |
| `libs/gravity_compensation/c/x5_dynamics.c` | 六轴递归牛顿欧拉逆动力学 | `DynamicsCompensator` |
| `libs/dm_motor/`、`libs/encos_motor/` | 反馈有效性、启停、设零、输出限制 | `Motor`、`MotorBank` |
| `libs/monitor/monitor.c` | 在线监控、维护期间保活 | `Motor.keep_alive/online` |
| `libs/can/can_config.h` | 电机发送周期和相位 | `TX_GROUPS`、`MotorBank.tick` |

上层通过 `Command` 提交控制输入。电机适配器通过 `MotorFeedback` 提交反馈，并交付 `MotorRequest`。`LocalLink` 将 `sdk.Arm` 调用连接到控制核心。

`CANMotorIO` 通过 SLCAN 或 SocketCAN 实现电机通信。帧编解码覆盖当前固件的 ENCOS 混控和达妙 MIT。收发、故障和生命周期约定见 [CAN 接口](CAN_IO.md)。

## 轨迹与动力学

命令周期为 2 ms。TD 使用 `r=[1000,4000,4000,2000,2000,2000]`、`h=.002`、`h0=.004`。`acceleration` 对应固件 `TD_t.v3`。`follow()` 同步位置，并清零速度和加速度。

| 参考值来源 | 位置 | 速度 | 加速度 |
| --- | --- | --- | --- |
| 自动规划 | 规划位置 | 规划速度 × `PLAN_SPEED_COMP_COEF` | 规划解析加速度 |
| 外部位置速度命令 | TD 位置 | 外部速度 | 0 |
| RUN 普通目标 | 0.16 s 五次样条 | 样条解析速度 | 样条解析加速度 |
| 被动模式 | TD 位置 | TD 速度 | 0 |

普通目标改变时，样条从当前参考位置、速度和加速度重规划。每次重规划的时长为 0.16 s。自动规划和外部速度通道同步样条状态。被动模式同步反馈位置，并清零样条速度和加速度。到达终点时，样条输出目标位置及零速度、零加速度。

RUN 补偿为 `M(q)qdd_ref + C(q,qd)qd + G(q)`。位置和速度使用本周期反馈，加速度使用命令层参考值。GRAVITY 补偿为 `G(q)`。STOP 和 DAMPING 的动力学补偿为零。

模型使用 `base_link → link6` 的 URDF 质量、质心和惯量。惯量按 inertial RPY 转到连杆系。默认模型为 `2023`，基座重力为 `(0,0,-9.81)` m/s²。SDK 系数 `[.8,.8,.8,1.32,1.32,1.32]` 作用于重力项。关节角范围为 ±128 rad。

粘性摩擦前馈为 `radians(参考关节速度) × viscous_damping`。电机输出为 `direction × (动力学补偿 + 粘性摩擦前馈) × torque_scale`。方向、减速比、力矩系数和速度系数默认均为 1。粘性摩擦系数默认均为 0。

参考值无效时，本周期输出 DAMPING，并清零速度和加速度。动力学失败时，本周期输出 STOP，并清零补偿。反馈包含 `dynamics_torque` 和 `dynamics_status`。状态值与固件一致：0=成功，-1=无效参数，-2=角度超限，-3=非有限结果。

## 电机参数和调度

| 电机 | ID | 类型 | 更新频率及相位 | 电机 kp/kd | 速度与力矩量程 |
| --- | --- | --- | --- | --- | --- |
| j1 | 1 | ENCOS | 500 Hz，0/2/4/6 ms | 0/35 | ±18 rad/s，±30 N·m |
| j2 | 2 | ENCOS | 250 Hz，1/5 ms | 10/50 | ±18 rad/s，±30 N·m |
| j3 | 4 | ENCOS | 250 Hz，1/5 ms | 10/50 | ±18 rad/s，±30 N·m |
| j4 | 5 | 达妙 | 250 Hz，2/6 ms | 50/.8 | ±45 rad/s，±10 N·m |
| j5 | 6 | 达妙 | 250 Hz，2/6 ms | 25/.8 | ±45 rad/s，±10 N·m |
| j6 | 7 | 达妙 | 250 Hz，3/7 ms | 10/1 | ±45 rad/s，±10 N·m |
| 夹爪 | 8 | 达妙 | 250 Hz，3/7 ms | 8/.1 | ±45 rad/s，±10 N·m |

相位按 8 ms 重复。电机位置量程为 ±12.5 rad。达妙每 75 ms 轮流发送一个 ENABLE。

位置 PID 的 `(kp,ki,kd)`：j1 为 `(6,0,50)`，j2/j3 为 `(4,0,50)`，j4 至夹爪为 `(6,.08,50)`。积分上限为 200。位置 PID 输出上限为 1000 deg/s。j1 至 j3 的积分范围为 1°，其余为 0°。速度 PID 增益均为零，其输出上限分别为 12、12、12、10、10、10、10。

PID 按电机更新周期执行。误差保持不变时，PID 保留符合固件条件的微分量。ENCOS 进入使能状态时清零 PID 历史。

STOP 输出软停机请求。DAMPING 的位置环增益和电机 kp 为零，kd 使用表中值。GRAVITY 六关节的位置环增益及电机 kp/kd 为零。GRAVITY 夹爪保持普通增益，其位置参考跟随反馈。

## 限位、自动动作与监控

关节角下限为 `[-115,.5,.5,-90,-90,-112.87]` deg，上限为 `[90,206.26,171.88,90,90,119.17]` deg。夹爪标定后的范围为 `[-142,142]` deg。复位目标为 `[0,.5,.5,0,0,0]` deg，五次样条时长为 2 s。

夹爪找零每命令周期推进 .2°，目标超前限制为 20°。闭合阶段前馈为 .5 N·m。每 200 ms 检查移动量。连续 3 个窗口移动量小于 .5°时，控制器确认机械限位。随后反向退回 142°。误差不超过 4°时设零，并等待 400 ms。闭合、退回、设零阶段的上限分别为 10、6、12 s。

电机校准默认处理六个关节。夹爪由专用流程设零。达妙失能确认上限为 1500 ms，每 100 ms 重试。设零后等待 20 ms，必要时使能唤醒并等待 100 ms。每台达妙设零上限为 1000 ms。

ENCOS 设零后静默 501 ms。ACK 等待上限为 1500 ms，每台上限为 2500 ms。无位置反馈时，每 120 ms 尝试软停机获取反馈。零位容差为 .01 rad。校准总时限为 15 s。

ENCOS 在线监控按 10 ms 网格重载 10 次，达妙重载 2 次。设零期间保活本臂监控。每臂独立执行 3 s 命令看门狗。反馈周期为 4 ms。

## PC 执行约定与验证

控制器按显式单调时间执行。漏拍后执行一次当前更新，并记录漏拍计数。普通目标样条每次更新推进 2 ms。自动规划组按绝对经过时间采样，并输出规划终点。

输入验证在状态更新前完成。初始化期间掉线或通信超时会取消自动动作。夹爪找零成功后，控制器才确认初始化完成。校准结束或取消时，控制器对临时唤醒的达妙输出 DISABLE。

数值计算使用 Python 双精度。C/Python 对照直接编译固件源码：

| 对照对象 | 覆盖范围 | 允许绝对误差 |
| --- | --- | --- |
| PID | 7 组配置及阶跃、保持、随机输入 | .002 deg/s |
| TD | r 为 1000、2000、4000，各 3000 次更新 | .002°、.02°/s、3°/s² |
| 样条 | 一次、三次、五次，各 200 组随机端点 | .02，按各输出单位 |
| 命令层 | 400 次重规划、外部速度、自动规划和模式切换 | .002°、.02°/s、2°/s² |
| 重力 | 3 模型、2 重力向量、300 位姿、2 模式 | 1e-5 N·m |
| 逆动力学 | 3 模型、4 重力向量、80 组输入、2 模式 | `5e-5 + 4e-6 × abs(C结果)` N·m |

运行 `python3 -m unittest discover -s tests -v` 执行验证。测试还覆盖双臂初始化、复位、校准、异常停机、软限位、调度、SDK 和日志语言。测试电机模型验证接口和状态流程。实机闭环及控制稳定性需要硬件验证。
