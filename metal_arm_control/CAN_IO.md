# Metal Arm CAN 配置

通道参数、队列、进程、看门狗和退出流程见 [公共 CAN 接口](../docs/CAN.md)。本页列出 Metal Arm 的协议与创建配置。

| 项目 | 配置 |
| --- | --- |
| 类型导入 | `metal_arm_control` 中的 `CANChannel`、`CANMotorIO`、`ProcessCANMotorIO` |
| 总线 | 每台臂独立的 1 Mbps 经典 CAN 标准帧总线 |
| 电机协议 | 全部达妙 MIT |
| 发送 ID | 1、2、3、4、5、6、7 |
| 接收路由 | 仲裁 ID 0x11..0x17，并校验载荷电机 ID |
| 只读检查 | `metal-arm-can-check --interface socketcan --channel can0` |
| 控制入口 | `metal-arm-tui --interface socketcan --channel can0` |
| 退出请求 | 全部七台电机 DISABLE |

只读检查使用专用入口，正常控制链路在 STOP 下也可能发送周期使能。驱动、接口权限和 SLCAN 依赖见 [传输配置](usb2can/README.md)。同一总线由一个控制程序占用。
