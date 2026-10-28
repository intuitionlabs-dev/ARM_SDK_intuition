# SDK 架构与验证

SDK 的公共入口为 `sdk.Arm`。Python 程序直接调用它，TUI 将按键和目标转换为同一组方法。链路负责连接所选的 H7 单片机或 PC 本地控制实现。

## 代码结构

| 层 | 位置 | 职责 |
| --- | --- | --- |
| 公共控制接口 | [arm.py](arm.py) | 命令、单位换算、反馈和预检 |
| 型号参数 | [board_defs](board_defs) | `arx_x5` / `metal_arm` 的限位、复位参考和辅助函数 |
| H7 链路 | [link.py](link.py)、[protocol.py](protocol.py) | 串口、组帧解析、按臂分流和重连 |
| 公共终端界面 | [tui](tui) | 会话、按键、显示及目标发送 |
| PC 型号实现 | [arx_x5_control](arx_x5_control)、[metal_arm_control](metal_arm_control) | 各型号的控制核心与设备配置 |

两个 PC 包使用相同模块布局：

| 相对模块 | 职责 |
| --- | --- |
| `sdk_adapter/link.py` | `sdk.Link` 子类、命令队列、公共反馈与设零 ACK |
| `sdk_adapter/tui.py`、`telemetry.py` | 本地终端链路及运行记录 |
| `system.py`、`arm.py` | 调度、每臂状态机、模式和故障处理 |
| `command.py`、`trajectory.py`、`numerics.py` | 目标、TD、样条和自动动作 |
| `config.py`、`motor.py` | 电机配置、增益、状态与输出策略 |
| `calibration.py` | 电机零点写入与确认 |
| `can_protocol.py` | 对应型号的 CAN 帧编解码 |
| `can_io.py`、`can_process.py`、`usb2can/` | 队列、收发线程、独立进程与传输配置 |
| `gravity_compensation/` | URDF、重力和逆动力学 |
| `simulation.py`、`diagnostics.py`、`performance.py` | 模拟电机、控制记录和周期统计 |

## 接口边界

| 边界 | 输入与输出 |
| --- | --- |
| 应用 / TUI → `sdk.Arm` | rad、rad/s、m，公共控制方法 |
| H7 `sdk.Link` → 固件 | 带臂编号的串口命令和反馈帧 |
| PC `LocalLink` → 控制核心 | `Command`，关节 deg / deg/s |
| PC 控制核心 → MotorIO | `MotorRequest` / `MotorFeedback`，rad / rad/s / N·m |

公共用法见 [Python](docs/PYTHON.md) 和 [TUI](docs/TUI.md)。后端执行差异见 [后端表](docs/BACKENDS.md)，物理参数与硬件动作见 [型号表](docs/ROBOTS.md)。低层接口见 [PC 控制核心](docs/PC_CONTROL.md) 和 [CAN](docs/CAN.md)。

## 验证

| 检查 | 覆盖范围 |
| --- | --- |
| 仓库单元测试 | 33 项，覆盖 Metal 状态机、STOP、夹爪保持、CAN 生命周期、双臂隔离、SDK 参数和 H7/TUI 选择 |
| 原厂编码对照 | 七个电机各 100 组输入，共 700 组命令字节对照，包含在单元测试中 |
| SDK 自测 | 协议、单位、预检、模拟状态机和串口重连；外部 H7 头文件缺失时跳过对应检查 |
| TUI 自测 | 48 项，覆盖会话、按键、限位和命令发送 |
| CI | Python 3.10/3.12 安装包测试及源码目录之外的模拟运行 |

```bash
python3 -m pip install -e '.[can]'
python3 -m unittest discover -s tests -v
python3 -m sdk.selftest
python3 -m sdk.tui --selftest
python3 -m metal_arm_control --demo --seconds 8 --log-level ERROR
python3 -m metal_arm_control.sdk_adapter.demo --log-level ERROR
python3 -m arx_x5_control --demo --log-level ERROR
python3 -m arx_x5_control.sdk_adapter.demo --log-level ERROR
```

CAN 测试使用模拟传输，模拟器验证接口和控制状态流程。各型号的实机稳定性、负载能力和总线时序需要在目标设备上验证。
