# Metal Arm 架构与运行说明

`metal_arm_control` 是 Metal Arm 的 Python 控制后端，提供单臂和双臂控制、CAN 收发、动力学补偿、模拟器及 TUI。业务程序通过公共 `sdk.Arm` 接口操作机械臂。

发行包为 `arx-x5-control 0.6.0`，支持 Python 3.10 及以上。安装和运行入口见 [Metal 概览](metal_arm_control/README.md)，完整代码示例见 [SDK 接入](metal_arm_control/SDK_LOCAL.md)。

## 代码结构

| 入口 | 职责 |
| --- | --- |
| [board_defs](board_defs) | `arx_x5` / `metal_arm` 参数模块，类型列表及参数选择 |
| [link.py](link.py)、[arm.py](arm.py) | H7 串口链路、公共控制 API、单位转换和预检 |
| [sdk_adapter/link.py](metal_arm_control/sdk_adapter/link.py) | Metal 本地控制链路、命令队列和 SDK 反馈 |
| [system.py](metal_arm_control/system.py)、[arm.py](metal_arm_control/arm.py) | 调度、单臂状态机、模式和故障处理 |
| [command.py](metal_arm_control/command.py)、[trajectory.py](metal_arm_control/trajectory.py) | TD、五次轨迹、初始化和复位 |
| [config.py](metal_arm_control/config.py)、[motor.py](metal_arm_control/motor.py) | 电机配置、MIT PD 和输出策略 |
| [can_protocol.py](metal_arm_control/can_protocol.py) | Damiao MIT 帧编解码 |
| [can_io.py](metal_arm_control/can_io.py)、[can_process.py](metal_arm_control/can_process.py) | CAN 收发线程、独立进程及退出交付 |
| [calibration.py](metal_arm_control/calibration.py) | 六轴电机零点写入流程 |
| [gravity_compensation](metal_arm_control/gravity_compensation) | URDF 模型、重力和逆动力学 |
| [sdk_adapter/tui.py](metal_arm_control/sdk_adapter/tui.py) | 终端操作和运行记录 |

控制链路为 `sdk.Arm → LocalLink → ControlSystem → 单臂控制器 → MotorRequest → CAN → 电机`。反馈沿反方向进入 SDK 快照和界面。

## 后端和参数选择

| 入口 | 执行位置 | 参数选择 |
| --- | --- | --- |
| `metal_arm_control.sdk_adapter.LocalLink` | PC | `sdk.board_defs.metal_arm` |
| `arx_x5_control.sdk_adapter.LocalLink` | PC | `sdk.board_defs.arx_x5` |
| `sdk.Link(robot_type="metal_arm")` | H7 固件 | 上位机使用 Metal 参数，默认选项为 `arx_x5` |

`arm.board_defs` 和 `link.board_defs` 提供所选机械臂的限位、复位参考和辅助函数。同一条 H7 链路的两台臂共用一份配置；不同链路独立选择。H7 的控制动作由固件执行，夹爪米制换算使用文件标定。

`arm_id` 表示连接中的臂编号。TUI 的 `vcode 0xF81DD236` 为公共协议字符串的 FNV-1a 哈希；H7 握手版本来自版本帧，PC 适配层使用本地协议码。发行包版本由 `pyproject.toml` 定义。

## 控制流程

1. 创建 CAN 适配器和 `LocalLink`，每条 CAN 通道对应一台臂；离线运行使用 `LocalLink.simulated()`。
2. 调用 `connect()`，等待七台电机的有效反馈和 `WAIT_INIT` 状态。
3. 调用 `init_arm()`，六轴复位完成后进入 `NORMAL/DAMPING`，夹爪保持初始化入口的反馈角。
4. 调用 `start()`，等待 `RUN` 反馈后提交关节位置或位置速度目标。
5. 每台臂持续接收目标或 `keepalive()`，有效命令间隔小于 3 s。
6. 通过反馈和 `link.drain_events()` 判断执行结果。退出时先关闭 `LocalLink`，再关闭 CAN 适配器。

初始化和 `reset()` 的六轴目标均为 `(0, 0, 0, 0, 0, 0)`°。初始化姿态容差为 20°，复位轨迹时长为 2 s。`set_zero()` 将 J1～J6 当前角度写为电机零点，应在关节处于标定基准位姿时执行。

| 层级 | 单位 |
| --- | --- |
| `sdk.Arm` 关节角、速度 | rad、rad/s |
| `gripper_m` / `gripper_raw_deg` | m / deg |
| 控制核心 `Command` | deg、deg/s |
| `MotorRequest` | rad、rad/s、N·m |

## 电机与控制参数

每台臂包含六个关节和一个夹爪，全部使用 Damiao MIT 协议，经典 CAN 标准帧，速率为 1 Mbps。

| 参数 | J1～J6 及夹爪 |
| --- | --- |
| 发送 ID | `1, 2, 3, 4, 5, 6, 7` |
| 接收 ID | `0x11..0x17`，校验载荷中的电机 ID |
| PMAX | 全部 `6.28 rad` |
| VMAX | `10, 10, 10, 10, 30, 30, 30 rad/s` |
| TMAX | `30, 120, 30, 30, 20, 20, 20 N·m` |
| 原生 Kp | `300, 500, 500, 400, 25, 20, 20` |
| 原生 Kd | `5, 5, 5, 4, 0.2, 0.2, 0.1` |
| 六轴下限 | `(-160, -180, 0, -123, -85, -145)°` |
| 六轴上限 | `(160, 0, 180, 81, 85, 145)°` |
| 夹爪角度范围 | `0..2.4 rad` |

PMAX/VMAX/TMAX 用于协议编码。CAN 反馈使用关节输出轴坐标，默认 `direction=1`、`reduction=1`。电机使用原生 MIT PD，主机级联 PID 增益为零。

主控制循环目标为 1 ms，命令与动力学更新为 2 ms，反馈发布为 4 ms，七台电机各以 4 ms 周期分相发送。普通 PC 的实际时序由运行负载和操作系统调度决定，统计方法见 [控制周期](metal_arm_control/CONTROL_TIMING.md)。

## STOP、故障和退出

STOP 输出零增益 SOFT_STOP，并对无已知故障的电机每台至少间隔 75 ms 发送 ENABLE；在 4 ms 电机调度下通常为 76 ms。STOP 不提供位置保持或重力补偿。标零流程独占参与电机，已知故障电机停止使能。

正常运行时应用命令超时转阻尼；电机反馈超过 100 ms 或出现故障使对应整臂进入停止，另一臂独立处理。关闭链路会向全部七台电机交付 DISABLE，退出前应放稳或支撑机械臂。

严格只读检查使用 `metal-arm-can-check`。CAN 线程和进程的队列、过期检查及看门狗见 [CAN 接入](metal_arm_control/CAN_IO.md)。

## 夹爪与动力学

夹爪默认使用厂家最近邻查表，SDK 开口范围为 `0..0.1 m`，依赖有效的出厂零点。`sdk.Arm(..., calib_store=...)` 可指定实测标定存储。

动力学模型为 `metal_with_gripper` 或 `metal_no_gripper`，链为 `base_link → Link6`，默认重力为 `(0, 0, -9.81)`。模型选项控制负载模型，控制拓扑包含七台电机。

`gravity_sdk_scale=True` 时，RUN 前馈为缩放的重力、科氏项和粘性摩擦之和；GRAVITY 模式使用缩放重力。关闭 SDK 缩放时使用完整 `M*qdd + C + G`。模型和算法许可证见 [来源说明](metal_arm_control/gravity_compensation/PROVENANCE.md)。

## 验证

| 检查 | 覆盖范围 |
| --- | --- |
| 仓库单元测试 | 33 项，覆盖 Metal 状态机、STOP、夹爪保持、CAN 生命周期、双臂隔离、SDK 参数和 H7/TUI 选择 |
| 原厂编码对照 | 七个电机各 100 组输入，共 700 组命令字节对照，包含在单元测试中 |
| SDK 自测 | 协议、单位、预检、模拟状态机和串口重连；外部 H7 头文件缺失时跳过对应检查 |
| TUI 自测 | 48 项，覆盖会话、按键、限位和命令发送 |
| CI | Python 3.10/3.12 安装包测试及源码目录之外的模拟运行 |

```bash
python3 -m pip install -e '.[can]'
python3 -m unittest discover -s tests -v
python3 -m sdk.selftest
python3 -m sdk.tui --selftest
python3 -m metal_arm_control --demo --seconds 8 --log-level ERROR
python3 -m metal_arm_control.sdk_adapter.demo --log-level ERROR
python3 -m arx_x5_control --demo --log-level ERROR
```

CAN 测试使用模拟传输，模拟器验证接口和控制状态流程。Metal 的实机稳定性、负载能力和总线时序需要在目标设备上验证。
