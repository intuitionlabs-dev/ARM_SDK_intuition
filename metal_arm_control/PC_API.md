# Python 控制接口

与 `arx_x5_control` 的同名模块、公开类和方法保持对应。

本页为 Metal PC 控制核心接口；`api_version=1`、发行包 `0.6.0` 和 SDK 协议哈希分别维护。限位与复位参考来自 `sdk.board_defs.metal_arm`，由 `config.py` 引用；`ArmConfig` 保存方向、零点偏移、模型和力矩缩放等运行配置。

- `ControlSystem(configurations).step(now_ms, commands=(), motor_feedback=())`：输入验证后执行一次调度，返回 `TickResult`；时间严格递增。
- `Command(arm_id, kind, joint_angle=(), gripper_angle=0, joint_velocity=(), api_version=1)`：关节 deg/deg/s，夹爪原始 deg。
- `MotorFeedback`：电机 rad/rad/s/N·m，温度 °C；原始状态 0=disabled，1=enabled，其他为故障。
- `MotorRequest`：语义动作 `CONTROL/SOFT_STOP/ENABLE/DISABLE/SAVE_ZERO`，编码时检查 Metal ID 和 `family="dm"`。
- `ArmFeedback`：状态、请求/实际模式、角度、速度、掉线位、初始化位、温度与动力学状态。
- `shutdown()`：返回必须交付给 MotorIO 的终止请求，可重复调用。

状态：0 等待握手、1 离线等待、2 待初始化、3 初始化、4 正常、5 通信失联、6 显式校准、7 复位。
模式：0 停止、1 位控、2 阻尼、3 重力补偿。

初始化要求全部七电机在线、反馈有效、六轴距零参考不超过 20°。初始化执行六轴复位，夹爪保持入口锁存的反馈角；初始化参考和复位目标均为 J1～J6 全 0°。正常状态下才能切模式，NORMAL/RUN 才接受运动目标。

STOP 对无已知故障的电机继续周期发送 ENABLE（每台至少 75 ms，4 ms 调度下通常为 76 ms）和零增益 SOFT_STOP，不保持位置或重力补偿。标零参与电机的周期使能暂停，已知故障电机不使能；`shutdown()` / 关闭链路交付全部七台电机的 DISABLE。

模型默认 `metal_with_gripper`，也可选择 `metal_no_gripper`（只改变负载模型；本 SDK 拓扑仍要求七台电机）。`gravity_sdk_scale=True` 使用厂家的重力/科氏缩放和摩擦项；False 使用原始 RNE 的 M*qdd+C+G。机械减速比不应用于已是输出轴坐标的 CAN 反馈。

上层规划器通过关节位置或位置速度接口提交目标：公共 `sdk.Arm` 使用 rad/rad/s，控制核心 `Command` 使用 deg/deg/s。
