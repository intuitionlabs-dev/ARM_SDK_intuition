# arx_x5_control

`arx_x5_control` 提供 PC Python 机械臂控制。控制逻辑以主分支 [`2bbb192`](https://github.com/Crystal-1438/ARX_X5_DMH7/tree/2bbb192) 为基线。

Python 版本支持双臂，每臂 6 关节及 1 夹爪。控制链路包含状态机、初始化、复位、夹爪找零、电机校准、PID、TD、0.16 s 五次样条、逆动力学补偿、速度前馈、粘性摩擦前馈、软限位及掉线处理。

上层通过 `Command` 输入命令，控制核心输出 `MotorRequest`、`ArmFeedback` 和 `Event`。`LocalLink` 将 `sdk.Arm` API 接入控制核心。`CANMotorIO` 通过 CANable2 SLCAN 或 Linux SocketCAN 连接真实电机，并提供 ENCOS 和达妙 MIT 帧编解码。

源文件对应关系和控制参数见 [控制链路](PC_PORT.md)，字段及调用契约见 [Python 接口](PC_API.md)，现有 SDK 程序的接入方法见 [SDK 本地后端](SDK_LOCAL.md)。

## 运行

Python 3.10 或更高版本；运行时只需要 Python 标准库。在 SDK 仓库根目录安装后执行：

```bash
python3 -m pip install .
python3 -m arx_x5_control --demo
```

默认执行 12 秒虚拟时间的双臂模拟：握手、初始化、夹爪找零、位置运动、重力模式、阻尼模式、复位。实际执行速度通常快于虚拟时间。结果 JSON 中的 `sequence_complete: true` 表示整个示例结束。

```bash
# 按墙钟时间播放同一模拟过程
python3 -m arx_x5_control --demo --realtime --seconds 12

# 可选：安装为 Python 包
# 安装包包含 SDK、控制核心和动力学模型。
python3 -m venv .venv
.venv/bin/python -m pip install .
.venv/bin/arx-x5-control --demo

# SDK 自测
python3 -m sdk.selftest
```

示例只使用内存中的测试电机模型，不访问硬件。模拟器用于验证状态与接口，未模拟真实惯性、摩擦、负载或稳定性。

## SDK 接入

```bash
# 运行 SDK 驱动的双臂示例。
python3 -m arx_x5_control.sdk_adapter.demo
```

```python
import sdk
from arx_x5_control import log
from arx_x5_control.sdk_adapter.link import LocalLink

log.configure(language="zh")
with LocalLink.simulated() as link:  # 实际电机程序改为 LocalLink(motor_io)
    arm = sdk.Arm(0, link)
    arm.connect()
    log.write_data(arm.snapshot())
```

现有的 `init_arm()`、`send_ctrl_pos_vel()`、模式切换、复位、设零和反馈接口继续使用 `sdk.Arm`；SDK 关节单位为 rad/rad/s，夹爪标定后使用 m。完整初始化及保活示例、`MotorIO` 两个接口、退出和异常处理见 [SDK 接入文档](SDK_LOCAL.md)。

双臂实机代码、设备准备和上层规划器接入见 [双臂实机接入例程](SDK_LOCAL.md#双臂实机接入例程)。

## CAN 连接

使用 SDK 终端界面控制双臂：

```bash
arx-x5-tui --channel /dev/ttyACM0 --channel /dev/ttyACM1
python3 -m arx_x5_control.sdk_adapter.tui --simulate
```

按 `i` 初始化，`s` 进入 RUN，`a` 切臂，`+`/`-` 点动，`o` 复位，`q` 退出并停止双臂。
完整按键和 nuc3 启动命令见 [SDK TUI](SDK_LOCAL.md#sdk-tui)。

```bash
python3 -m pip install '.[can]'
python3 -m arx_x5_control.can_check --channel /dev/ttyACM0 --channel /dev/ttyACM1 --seconds 10
```

该入口只发送电机读取请求。报告包含每个电机的回复数、位置、速度和错误码。正式配置使用 `/dev/serial/by-id/` 下的稳定设备路径。

双臂持续控制使用 `ProcessCANMotorIO`，可传给 `LocalLink`。
其独立进程内的 `CANMotorIO` 负责电机收发。
`CANControlIO` 提供 `runtime.run` 所需的命令和反馈回调。
CAN 收发遵守 200 μs 帧间隔和 501 ms ENCOS 静默期。
配置、持续控制示例和退出约定见 [CAN 接口](CAN_IO.md)。

## 集成入口

```python
from arx_x5_control import (
    ArmConfig, Command, CommandKind, ControlSystem, MotorFeedback, log,
)

controller = ControlSystem([ArmConfig()])  # 单臂；不传配置时默认双臂

# MotorFeedback 的角度为 rad，速度为 rad/s；这里模拟初始反馈。
feedback = tuple(
    MotorFeedback(arm_id=0, motor_index=i, timestamp_ms=0,
                  position=0.0, velocity=0.0, temperature=25.0)
    for i in range(7)
)
result = controller.step(
    0,
    commands=[Command(0, CommandKind.CONNECT)],
    motor_feedback=feedback,
)

# 电机适配器交付 result.motor_requests，并返回下一批反馈。
log.write_data({"state": result.feedback[0].state.name})  # WAIT_INIT

# shutdown 返回的停机请求也必须由通信程序交付。
final_requests = controller.shutdown().motor_requests
```

`ControlSystem.step()` 是确定性控制入口，应由一个线程以约 1 ms 周期调用。支持注入时间，便于回放和测试。`runtime.run()` 提供使用 `time.monotonic_ns()` 的循环，并通过 [ControlIO](interfaces.py) 调用通信适配器。没有绑定任何传输方式。

上层关节角使用 **deg**，速度使用 **deg/s**；电机接口使用 **rad、rad/s、N·m**。两种坐标单位不能混用。正常使能顺序是 `CONNECT → WAIT_INIT → INITIALIZE → NORMAL/DAMPING → START → POSITION`。初始化需要全部电机在线，且六个关节距复位角各不超过 20°。

## 包专属日志和语言

`arx_x5_control` 内的运行日志统一使用自己的 `log` 库，默认中文；整个包的已有日志对象和后台线程可通过一处配置切换语言：

```python
from arx_x5_control import log
log.configure(language="zh", level="INFO")
log.configure(language="en")  # 下一条日志开始使用英文
```

命令行使用 `--lang zh/en`、`--log-level INFO`，或设置 `ARX_X5_CONTROL_LOG_LANGUAGE`、`ARX_X5_CONTROL_LOG_LEVEL`。人类可读日志写入标准错误流，最终 JSON 结果写入标准输出。独立包分别拥有自己的日志库与配置，本次配置不会影响 SDK 或其他包。完整约定见 [日志文档](LOGGING.md)。

## 验证范围

测试覆盖双臂初始化、复位、校准、超时、掉线、模式切换、限位、调度、SDK 接入和日志语言。轨迹测试覆盖五次样条重规划及控制源衔接。动力学测试覆盖惯性项、科里奥利项、重力项和异常停机。C/Python 对照测试直接编译固件的 PID、TD、样条、命令层、重力和逆动力学源码。请安装主机 C 编译器 `cc`，并使用 H7 控制工程中的集成测试。

程序通过本地模拟和独立安装包验证。
nuc3 双臂实机验证覆盖初始化、夹爪找零、小幅运动、模式切换、复位及通信超时保护。
普通 PC Python 的调度受操作系统影响。
控制器提供漏拍计数。
主循环、命令更新和 CAN 周期统计见 [控制周期稳定性](CONTROL_TIMING.md)。

## 文件

| 位置 | 用途 |
| --- | --- |
| `arx_x5_control/system.py` | 单线程调度、双臂隔离、输入输出与退出 |
| `arx_x5_control/arm.py` | 总模式状态机、坐标转换、动力学与电机参考量 |
| `arx_x5_control/command.py` | 软限位、TD、五次样条、复位、夹爪找零 |
| `arx_x5_control/motor.py` | 电机反馈、PID 级联、限幅、启停及更新周期 |
| `arx_x5_control/calibration.py` | ENCOS/达妙设零状态机 |
| `arx_x5_control/numerics.py`、`trajectory.py` | 数值算法与多轴轨迹 |
| `arx_x5_control/config.py` | 标定量及 H7 默认参数 |
| `arx_x5_control/types.py`、`interfaces.py` | 控制器与适配器之间的类型及接口 |
| `arx_x5_control/can_io.py`、`can_protocol.py` | CAN 后台收发与电机帧编解码 |
| `arx_x5_control/usb2can/` | CAN 传输子包、后端配置与设备收发 |
| `arx_x5_control/can_process.py` | CAN 收发进程、控制器保活及进程通信 |
| `arx_x5_control/can_check.py` | CAN 只读连通性检查 |
| `arx_x5_control/runtime.py` | PC 单调时钟循环 |
| `arx_x5_control/log.py`、`log_messages.py` | 本包专属日志、全包配置及中英文词条 |
| `arx_x5_control/sdk_adapter/` | SDK 本地后端、终端界面、会话日志及调用示例 |
| `arx_x5_control/simulation.py`、`__main__.py` | 无硬件模拟及命令行入口 |
| `arx_x5_control/gravity_compensation/` | Python 重力与逆动力学、URDF 模型及许可证 |
| `tests/` | 行为、故障、C/Python 数值对比测试 |

动力学模块及模型使用各自的许可证，安装包包含许可证和来源文件。SDK 接入层使用仓库固定版本的 SDK。控制核心和 `--demo` 入口可独立运行。
