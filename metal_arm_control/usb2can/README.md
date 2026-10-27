# Metal Arm USB-CAN 配置

传输方法、JSON 格式、权限要求和外部设备扩展遵循 [公共 USB-CAN 接口](../../docs/USB_CAN.md)。

| 项目 | Metal Arm 配置 |
| --- | --- |
| 包路径 | `metal_arm_control.usb2can` |
| 默认映射 | [transports.json](transports.json) |
| SLCAN 类 | `metal_arm_control.usb2can.slcan:CANableTransport` |
| SocketCAN 类 | `metal_arm_control.usb2can.socketcan:SocketCANTransport` |
| 显式配置 | `TransportRegistry(path)` 或上层 `transport_config` |
| 环境变量 | `USB2CAN_CONFIG` |

两种内置接口均使用 1 Mbps 经典 CAN 标准帧。安装和设备连接见 [SDK 手册](../../README.md#安装) 与 [CAN 配置](../CAN_IO.md)。
