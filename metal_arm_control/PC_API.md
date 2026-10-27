# Metal Arm 控制核心接口

本型号使用 [PC 控制核心接口](../docs/PC_CONTROL.md) 中的类型、命令和状态约定。上层应用使用 [公共 `sdk.Arm` API](../API.md)。

| 入口 | Metal Arm 类型 |
| --- | --- |
| 控制器 | `metal_arm_control.ControlSystem` |
| 运行配置 | `metal_arm_control.ArmConfig` |
| 命令与反馈 | `metal_arm_control.types` |
| MotorIO 协议 | `metal_arm_control.interfaces` |
| 机械参数 | `sdk.board_defs.metal_arm` |

接口版本为 `api_version=1`。初始化、STOP 和显式设零的硬件动作见 [控制行为对照](../docs/ROBOTS.md#控制行为)；增益及模型见 [控制参数](PC_PORT.md)。
