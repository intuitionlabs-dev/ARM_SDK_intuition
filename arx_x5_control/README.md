# ARX X5 型号说明

`arx_x5_control` 提供 ARX X5 的 PC 控制实现，通过公共 `sdk.Arm` 接口使用。SDK 的 Python 与 TUI 操作分别见 [Python 调用](../docs/PYTHON.md) 和 [TUI 调用](../docs/TUI.md)。安装见 [SDK 手册](../README.md#安装)。

## 接入配置

| 项目 | ARX X5 配置 |
| --- | --- |
| PC 链路 | `arx_x5_control.sdk_adapter.LocalLink` |
| PC 配置与 CAN 类型 | 从 `arx_x5_control` 导入 |
| H7 上位机参数 | `sdk.Link(robot_type="arx_x5")`；设备动作由匹配的 H7 固件执行 |
| 参数模块 | `sdk.board_defs.arx_x5`，运行时使用 `arm.board_defs` |
| TUI 入口 | `arx-x5-tui` |
| 只读 CAN 检查 | `arx-x5-can-check` |
| PC 默认模型 | `2023` |

## 型号行为

| 项目 | ARX X5 PC |
| --- | --- |
| 六轴复位角 | `(0, 0.5, 0.5, 0, 0, 0)°` |
| 初始化动作 | 六轴复位后执行夹爪找零 |
| 默认夹爪换算 | 文件两点标定 |
| 电机协议 | J1～J3 ENCOS，其余达妙 MIT |
| 电机输出周期 | J1 为 2 ms，其余 4 ms |
| STOP 周期使能 | 达妙每 75 ms 轮流使能一台 |
| 关闭输出 | 全部软停机，达妙额外 DISABLE |

跨型号的完整限位、夹爪、控制和协议对照见 [机械臂差异](../docs/ROBOTS.md)。初始化后进入正常阻尼模式，后续模式和控制方法使用公共接口。

## 参考文档

| 文档 | 内容 |
| --- | --- |
| [链路配置](SDK_LOCAL.md) | 本型号创建入口、单臂和双臂接入 |
| [控制参数](PC_PORT.md) | 增益、动力学及维护动作 |
| [控制接口](PC_API.md) | 公共核心接口及本型号类型入口 |
| [CAN 配置](CAN_IO.md) / [传输配置](usb2can/README.md) | 电机路由及传输类 |
| [控制周期](CONTROL_TIMING.md) / [日志](LOGGING.md) | 输出周期及测量入口 |
| [来源与许可证](gravity_compensation/PROVENANCE.md) | 模型、算法及参数来源 |
| [验证入口](../HANDOFF_METAL_ARM.md#验证) | SDK 自测、单元测试与模拟运行 |
