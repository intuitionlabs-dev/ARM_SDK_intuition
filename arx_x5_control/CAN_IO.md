# ARX X5 CAN 配置

通道参数、队列、进程、看门狗和退出流程见 [公共 CAN 接口](../docs/CAN.md)。本页列出 ARX X5 的协议与创建配置。

| 项目 | 配置 |
| --- | --- |
| 类型导入 | `arx_x5_control` 中的 `CANChannel`、`CANMotorIO`、`ProcessCANMotorIO` |
| 总线 | 每台臂独立的 1 Mbps 经典 CAN 标准帧总线 |
| 电机协议 | J1～J3 ENCOS，其余达妙 MIT |
| 发送 ID | 1、2、4、5、6、7、8 |
| 接收路由 | ENCOS 按电机 ID；达妙仲裁 ID 0，加载荷电机 ID |
| 只读检查 | `arx-x5-can-check --interface socketcan --channel can0` |
| 控制入口 | `arx-x5-tui --interface socketcan --channel can0` |
| 退出请求 | 全部软停机，达妙额外 DISABLE |

只读检查使用专用入口，正常控制链路在 STOP 下也可能发送周期使能。驱动、接口权限和 SLCAN 依赖见 [传输配置](usb2can/README.md)。同一总线由一个控制程序占用。
