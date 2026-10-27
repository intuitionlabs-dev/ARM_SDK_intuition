# Python 控制接口

与 `arx_x5_control` 的同名模块、公开类和方法保持对应。

- `ControlSystem(configurations).step(now_ms, commands=(), motor_feedback=())`：输入验证后执行一次调度，返回 `TickResult`；时间严格递增。
- `Command(arm_id, kind, joint_angle=(), gripper_angle=0, joint_velocity=(), api_version=1)`：关节 deg/deg/s，夹爪原始 deg。
- `MotorFeedback`：电机 rad/rad/s/N·m，温度 °C；原始状态 0=disabled，1=enabled，其他为故障。
- `MotorRequest`：语义动作 `CONTROL/SOFT_STOP/ENABLE/DISABLE/SAVE_ZERO`，编码时检查 Metal ID 和 `family="dm"`。
- `ArmFeedback`：状态、请求/实际模式、角度、速度、掉线位、初始化位、温度与动力学状态。
- `shutdown()`：返回必须交付给 MotorIO 的终止请求，可重复调用。

状态：0 等待握手、1 离线等待、2 待初始化、3 初始化、4 正常、5 通信失联、6 显式校准、7 复位。
模式：0 停止、1 位控、2 阻尼、3 重力补偿。

初始化要求全部七电机在线、反馈有效、六轴距零参考不超过 20°。初始化仅执行六轴复位，入口锁存夹爪当前反馈角；没有夹爪校准或核验阶段。复位角待用户提供，目前仍为旧六轴零角。正常状态下才能切模式，NORMAL/RUN 才接受运动目标。

模型默认 `metal_with_gripper`，也可选择 `metal_no_gripper`（只改变负载模型；本 SDK 拓扑仍要求七台电机）。`gravity_sdk_scale=True` 使用厂家的重力/科氏缩放和摩擦项；False 使用原始 RNE 的 M*qdd+C+G。机械减速比不应用于已是输出轴坐标的 CAN 反馈。

无新增笛卡尔 IK 接口；上层规划器输出关节目标。原厂例程中的速度档位也不直接等于本 SDK 的 rad/s 参数。
