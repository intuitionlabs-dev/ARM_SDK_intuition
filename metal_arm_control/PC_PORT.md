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

数值参数集中于 `config.py`。电机协议与初始化中存在无法仅改数字实现的差异，分别收敛在 `can_protocol.py`、`motor.py`、`calibration.py`、`command.py`。API 及状态编号保持一致，详见 [README](README.md)。

来源和模型许可证见 [PROVENANCE](gravity_compensation/PROVENANCE.md)。
