# 机械臂 SDK

SDK 通过 `sdk.Arm` 提供机械臂连接、初始化、关节控制、模式切换、复位和反馈读取接口。ARX X5 与 Metal Arm 共用调用方法及单位，设备参数由创建链路时选择的配置提供。

| 调用方式 | 用途 | 入口文档 |
| --- | --- | --- |
| Python API | 接入规划器、控制程序和自动化任务 | [Python 调用](docs/PYTHON.md)、[API 参考](API.md) |
| TUI | 查看反馈、初始化、切换模式和手动点动 | [TUI 调用](docs/TUI.md) |

## 安装

Python 要求 3.10 及以上。发行包 `arx-x5-control 0.6.0` 包含公共 SDK 和两个型号的 PC 控制实现。

```bash
git clone --branch feat/metal-arm-control https://github.com/intuitionlabs-dev/ARM_SDK_intuition.git
cd ARM_SDK_intuition
python3 -m pip install '.[can]'
```

`can` 扩展安装 H7 串口及 SLCAN 使用的 pySerial；SocketCAN 使用标准库。源码开发可使用 `python3 -m pip install -e '.[can]'`。

## 后端

| 后端类别 | 控制执行位置 | 链路创建 | 通信设备 |
| --- | --- | --- | --- |
| H7 单片机 | H7 固件 | `sdk.Link(port, robot_type=...)` | 板载 USB CDC 或 USB 转 RS485 |
| PC 本地 | PC Python 控制核心 | 对应型号包的 `LocalLink(motor_io, ...)` | 每台臂一条 SocketCAN 或 SLCAN 通道 |

H7 的 `robot_type` 支持 `arx_x5` 和 `metal_arm`，用于上位机预检和界面参数；H7 固件须支持实际机械臂。PC 本地使用 `arx_x5_control` 或 `metal_arm_control` 中的 `LocalLink`，两者均继承 `sdk.Link`。

链路创建后，应用通过 `sdk.Arm(arm_id, link)` 或 `link.arm(arm_id)` 操作。`arm_id` 选择同一连接中的臂，不表示型号。连接方式、重连及关闭行为见 [后端与链路](docs/BACKENDS.md)。

## Python 调用

```python
import sdk
from arx_x5_control.sdk_adapter import LocalLink

with LocalLink.simulated() as link:
    arm = sdk.Arm(0, link)
    arm.connect()
    print(arm.snapshot())
```

此例运行 PC 模拟器并读取反馈。不同设备的创建代码，以及包含初始化、保活、运动和退出的完整示例，见 [Python 调用](docs/PYTHON.md)。

## TUI 调用

| 连接 | 启动示例 |
| --- | --- |
| H7，ARX 参数 | `python3 -m sdk.tui --port /dev/ttyACM0 --robot-type arx_x5` |
| H7，Metal 参数 | `python3 -m sdk.tui --port /dev/ttyACM0 --robot-type metal_arm` |
| PC，ARX 模拟 | `arx-x5-tui --simulate` |
| PC，Metal 模拟 | `metal-arm-tui --simulate` |

界面中使用 `i` 初始化、`s` 进入 RUN、`a` 切臂、`+` / `-` 点动、`o` 复位、`q` 退出。实机 CAN 启动参数、按键和诊断选项见 [TUI 调用](docs/TUI.md)。

## 控制流程与单位

1. 创建链路和臂对象，连接后等待有效反馈及电机在线。
2. 在复位参考附近执行初始化，成功后进入正常运行 / 阻尼模式。
3. 请求 RUN，等待反馈确认后提交关节位置或位置速度目标。
4. 为每台臂持续提交目标或保活，命令间隔小于 3 s；发送返回值不代表运动完成。
5. 通过反馈判断结果，退出时按后端生命周期关闭设备。

| 数据 | 公共 SDK 单位或语义 |
| --- | --- |
| 关节角 / 角速度 | rad / rad/s |
| 夹爪开口 | m，由所选换算模型处理 |
| 温度 | ℃ |
| `snapshot()` | 同一帧的状态、模式及反馈 |
| `init_arm()` | 初始化，成功后为阻尼模式 |
| `reset()` | 运动至所选复位角，不改写电机零点 |
| `set_zero()` | 显式将六轴当前位姿写为电机零点 |
| `stop()` | 软停机，不提供位置保持或重力补偿 |

初始化内部动作、STOP 周期使能和关闭输出见 [机械臂行为表](docs/ROBOTS.md#控制行为)。退出前应放稳或支撑机械臂。

## 机械参数与夹爪

`arm.board_defs` / `link.board_defs` 提供所选型号的限位、复位角及辅助函数。公共控制接口使用相同方法，轨迹目标须满足所选机械臂的物理范围。

| 项目 | ARX X5 | Metal Arm |
| --- | --- | --- |
| 参数模块 | `sdk.board_defs.arx_x5` | `sdk.board_defs.metal_arm` |
| 六轴复位角 | `(0, 0.5, 0.5, 0, 0, 0)`° | `(0, 0, 0, 0, 0, 0)`° |
| PC 初始化夹爪动作 | 找零 | 保持入口反馈角 |
| 默认夹爪换算 | 文件两点标定 | 厂家查表，开口 `0..0.1 m` |

H7 与 PC 的默认夹爪换算都由所选型号给出：ARX X5 用文件两点标定，Metal Arm 用厂家查表。完整限位、电机协议、模型、调度及夹爪差异见 [机械臂参数与行为差异](docs/ROBOTS.md)，标定文件与工具见 [配置说明](config/README.md)。

## 文档索引

| 文档 | 内容 |
| --- | --- |
| [Python 调用](docs/PYTHON.md) / [TUI 调用](docs/TUI.md) | 两种调用方式、设备创建和公共操作 |
| [API 参考](API.md) | 方法、参数、反馈、预检和异常 |
| [后端与链路](docs/BACKENDS.md) | H7 / PC 执行位置、生命周期及诊断差异 |
| [机械臂差异](docs/ROBOTS.md) | 型号配置、参数和硬件动作对照 |
| [H7 接入与协议](docs/H7.md) | 接线、握手、串口帧与标定工具 |
| [PC 本地链路](docs/PC_LOCAL.md) / [控制核心](docs/PC_CONTROL.md) | 本地 SDK 适配及底层接口 |
| [CAN 接口](docs/CAN.md) / [USB-CAN 扩展](docs/USB_CAN.md) | 通道、进程、传输配置与设备扩展 |
| [日志](docs/LOGGING.md) / [控制周期](docs/TIMING.md) | 会话文件、指标及测量方法 |
| [ARX 型号说明](arx_x5_control/README.md) / [Metal 型号说明](metal_arm_control/README.md) | 各型号配置与实现入口 |
| [验证入口](HANDOFF_METAL_ARM.md#验证) | 离线测试、模拟运行和覆盖范围 |
| [ARX 来源与许可证](arx_x5_control/gravity_compensation/PROVENANCE.md) / [Metal 来源与许可证](metal_arm_control/gravity_compensation/PROVENANCE.md) | 算法、模型和参数来源 |
