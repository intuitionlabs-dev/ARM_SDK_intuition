# 控制结构

Python 模块与 `arx_x5_control` 一一对应：

| 模块 | 职责 |
|---|---|
| system / arm | 调度、状态机、模式、坐标和动力学 |
| command / trajectory / numerics | 命令、轨迹、数值算法 |
| motor / calibration | 原生 MIT PD、电机状态、显式设零 |
| can_protocol / can_io / can_process | Metal 报文、队列、线程和进程 |
| usb2can | SLCAN / SocketCAN，可配置传输注册表 |
| gravity_compensation | Metal URDF、重力和逆动力学 |
| sdk_adapter | 相同 sdk.Arm 接口、模拟示例、TUI、记录器 |
| simulation / diagnostics / performance / log | 离线对象、诊断、计时和日志 |

限位、夹爪角度范围及复位参考集中于 `sdk.board_defs.metal_arm`（源码 [board_defs/metal_arm.py](../board_defs/metal_arm.py)），`config.py` 引用同一组定义，并保存电机 ID、编码范围、MIT 增益和动力学系数。ARX 参数位于独立模块。

`can_protocol.py` 负责 Metal 帧编解码，`motor.py` 负责电机输出与周期使能，`calibration.py` 负责显式设零，`command.py` 负责目标和自动动作。初始化复位六轴并保持夹爪入口角，六轴复位目标为全 0°；STOP 周期使能并输出零增益帧。控制流程详见 [README](README.md)。

H7 的 `Link(robot_type="metal_arm")` 选择上位机参数模块，实际控制由 H7 固件执行。

来源和模型许可证见 [PROVENANCE](gravity_compensation/PROVENANCE.md)。
