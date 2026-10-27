# 结构与参数来源

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

电机协议与初始化中存在无法仅改数字实现的差异，分别收敛在 `can_protocol.py`、`motor.py`、`calibration.py`、`command.py`。Metal 初始化只复位六轴并保持夹爪入口角，STOP 继续周期使能和零增益输出；复位六轴全 0° 已获用户确认。API 及状态编号保持一致，详见 [README](README.md)。

H7 的 `Link(robot_type="metal_arm")` 仅复用上位机参数模块；它不运行这里的控制核心，也不会更改 H7 固件。

来源和模型许可证见 [PROVENANCE](gravity_compensation/PROVENANCE.md)。
