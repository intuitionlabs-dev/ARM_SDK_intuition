# ARX X5 控制核心接口

本型号使用 [PC 控制核心接口](../docs/PC_CONTROL.md) 中的类型、命令和状态约定。上层应用使用 [公共 `sdk.Arm` API](../API.md)。

| 入口 | ARX X5 类型 |
| --- | --- |
| 控制器 | `arx_x5_control.ControlSystem` |
| 运行配置 | `arx_x5_control.ArmConfig` |
| 命令与反馈 | `arx_x5_control.types` |
| MotorIO 协议 | `arx_x5_control.interfaces` |
| 机械参数 | `sdk.board_defs.arx_x5` |

接口版本为 `api_version=1`。初始化、STOP 和显式设零的硬件动作见 [控制行为对照](../docs/ROBOTS.md#控制行为)；增益及模型见 [控制参数](PC_PORT.md)。
