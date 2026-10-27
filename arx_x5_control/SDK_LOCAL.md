# ARX X5 链路配置

创建链路时选择本型号的模块；连接、初始化、目标发送、保活和反馈读取使用 [公共 Python 流程](../docs/PYTHON.md#公共控制流程)。

## 创建入口

```python
from arx_x5_control import ArmConfig, CANChannel, ProcessCANMotorIO
from arx_x5_control.sdk_adapter import LocalLink
```

| 连接 | 创建方式 |
| --- | --- |
| PC 模拟 | `LocalLink.simulated()`，默认双臂 |
| PC 单臂 | `LocalLink(motor_io, configurations=[ArmConfig()])` |
| PC 双臂 | `LocalLink(motor_io, configurations=[ArmConfig(), ArmConfig()])` |
| H7 上位机参数 | `sdk.Link(port, robot_type="arx_x5")` |

PC 的 `ArmConfig` 与 CAN 对象来自同一个型号包。`arm.board_defs` 为 `sdk.board_defs.arx_x5`。H7 控制能力取决于设备固件，后端差异见 [链路说明](../docs/BACKENDS.md)。

## 双臂实机接入例程

完整设备创建见 [Python CAN 连接](../docs/PYTHON.md#pc-can-连接)。将本页导入用于设备配置后，把创建的 `link` 交给公共 `operate()` 流程。进程适配器入口使用 `if __name__ == "__main__"`，退出顺序为控制链路、CAN 适配器。

方法签名、MotorIO、命令映射、手动时钟和故障处理见 [PC 本地链路参考](../docs/PC_LOCAL.md)。

## SDK TUI

| 用途 | 命令 |
| --- | --- |
| 离线模拟 | `arx-x5-tui --simulate` |
| SocketCAN 单臂 | `arx-x5-tui --interface socketcan --channel can0` |
| SocketCAN 双臂 | `arx-x5-tui --interface socketcan --channel can0 --channel can1` |

界面操作及参数说明见 [TUI 调用](../docs/TUI.md)，本型号的物理参数和初始化动作见 [型号对照](../docs/ROBOTS.md)。
