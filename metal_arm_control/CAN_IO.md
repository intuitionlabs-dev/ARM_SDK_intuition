# Metal CAN 接入

本页适用于 `arx-x5-control 0.6.0` 发行包中的 `metal_arm_control`；安装方式见 [Metal 概览](README.md)。它与 H7 串口 TUI 使用不同入口，`--robot-type` 是 H7 TUI 参数，Metal PC TUI 已固定使用本后端。

每台臂一条独立总线，经典 CAN 标准帧，1 Mbps。发送 0x01..0x07，接收 0x11..0x17。不要使用 ARX 的接收 ID=0 或 ENCOS 查询。

```bash
metal-arm-can-check --interface socketcan --channel can0
metal-arm-tui --interface socketcan --channel can0
# 双臂增加 --channel can1
# SLCAN 则使用 --interface slcan --channel /dev/ttyACM0
```

只读检查使用达妙 `0x7ff` 状态读取帧，不使能、不设零。SocketCAN 要求主机驱动可用且有配置接口的权限；SLCAN 需要 pySerial 和设备访问权限。传输选项见 [usb2can](usb2can/README.md)。

CANMotorIO 保留有界队列、过期检查、命令流看门狗和关闭交付；ProcessCANMotorIO 将其放在独立进程。故障退出对七台电机全部发送 disable。此动作会释放承重力矩。

正常 `LocalLink` 一经打开便运行控制循环，即使未初始化且处于 STOP，也会对无已知故障的电机发送周期使能和零增益帧。需要严格只读时使用 `metal-arm-can-check`，该入口过滤控制核心的所有电机输出，只交付查询请求。

只有实际 CAN 帧交付并收到反馈才可确认硬件工作；模拟器不能验证 USB-CAN 固件或电机固件配置。不要同时打开原厂控制器和本 SDK。
