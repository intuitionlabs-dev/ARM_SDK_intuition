# 机械臂 SDK

SDK 通过串口发送机械臂命令并接收反馈。`sdk.Arm` 提供机械臂连接、初始化、关节控制、模式切换、复位和反馈读取接口。ARX X5 与 Metal Arm 共用调用方法及单位，设备参数由创建链路时选择的配置提供。

| 调用方式 | 用途 | 入口文档 |
| --- | --- | --- |
| Python API | 接入规划器、控制程序和自动化任务 | [Python 调用](docs/PYTHON.md)、[API 参考](API.md) |
| TUI | 查看反馈、初始化、切换模式和手动点动 | [TUI 调用](docs/TUI.md) |

## 安装

Python 要求 3.10 及以上。发行包 `arx-x5-control 0.6.0` 提供 Python API、TUI 和离线协议模拟器。

```bash
git clone --branch master https://github.com/intuitionlabs-dev/ARM_SDK_intuition.git
cd ARM_SDK_intuition
python3 -m pip install .
```

`pySerial` 随安装提供，用于串口通信。源码开发可使用 `python3 -m pip install -e .`。

## 串口链路

| 后端类别 | 控制执行位置 | 链路创建 | 通信设备 |
| --- | --- | --- | --- |
| 串口 | 设备侧 | `sdk.Link(port, robot_type=...)` | USB CDC 或 USB 转 RS485 |

`robot_type` 支持 `arx_x5` 和 `metal_arm`，用于命令预检、参数读取和界面显示。

链路创建后，应用通过 `sdk.Arm(arm_id, link)` 或 `link.arm(arm_id)` 操作。`arm_id` 选择同一连接中的臂，不表示型号。连接方式、重连及关闭行为见 [链路与会话](docs/BACKENDS.md)。

## Python 调用

```python
import sdk
from sdk.sim import SimLink

with SimLink(robot_type="arx_x5") as link:
    arm = sdk.Arm(0, link)
    arm.connect()
    print(arm.snapshot())
```

此例运行协议模拟器并读取反馈。实机链路的创建代码，以及包含初始化、保活、运动和退出的完整示例，见 [Python 调用](docs/PYTHON.md)。

## TUI 调用

| 连接 | 启动示例 |
| --- | --- |
| ARX 参数 | `python3 -m sdk.tui --robot-type arx_x5` |
| Metal 参数 | `python3 -m sdk.tui --robot-type metal_arm` |
| 协议模拟，ARX 参数 | `python3 -m sdk.tui --fake --robot-type arx_x5` |
| 协议模拟，Metal 参数 | `python3 -m sdk.tui --fake --robot-type metal_arm` |

界面中使用 `i` 初始化、`s` 进入 RUN、`a` 切臂、`+` / `-` 点动、`o` 复位、`q` 退出。串口启动参数和按键见 [TUI 调用](docs/TUI.md)。

## 控制流程与单位

1. 创建链路和臂对象，连接后等待有效反馈及电机在线。
2. 在复位参考附近执行初始化，成功后进入正常运行 / 阻尼模式。
3. 请求 RUN，等待反馈确认后提交关节位置目标。
4. 为每台臂持续提交目标或保活，命令间隔小于 3 s；发送返回值不代表运动完成。
5. 通过反馈判断结果，退出时按链路生命周期关闭串口。

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

初始化、STOP 和关闭行为见 [机械臂行为表](docs/ROBOTS.md#控制行为)。退出前应放稳或支撑机械臂。

## 机械参数与夹爪

`arm.board_defs` / `link.board_defs` 提供所选型号的限位、复位角及辅助函数。公共控制接口使用相同方法，轨迹目标须满足所选机械臂的物理范围。

| 项目 | ARX X5 | Metal Arm |
| --- | --- | --- |
| 参数模块 | `sdk.board_defs.arx_x5` | `sdk.board_defs.metal_arm` |
| 六轴复位角 | `RESET_POSE_RAD`，单位 rad | `RESET_POSE_RAD`，单位 rad |
| 默认夹爪换算 | 文件两点标定 | 厂家查表 |

默认夹爪换算由所选型号给出：ARX X5 用文件两点标定，Metal Arm 用厂家查表。完整限位、复位参考及夹爪差异见 [机械臂参数与行为差异](docs/ROBOTS.md)，标定文件与工具见 [配置说明](config/README.md)。

## 文档索引

| 文档 | 内容 |
| --- | --- |
| [Python 调用](docs/PYTHON.md) / [TUI 调用](docs/TUI.md) | 两种调用方式、设备创建和公共操作 |
| [API 参考](API.md) | 方法、参数、反馈、预检和异常 |
| [链路与会话](docs/BACKENDS.md) | 链路生命周期及状态读取 |
| [机械臂差异](docs/ROBOTS.md) | 型号配置、参数及夹爪换算 |
| [串口接入与协议](docs/SERIAL.md) | 连接、握手、串口帧与标定工具 |
| [离线验证](#离线验证) | 协议、串口重连、型号参数及 TUI 自测 |
| [参数来源](docs/ROBOTS.md#参数来源) | 型号参数与夹爪查表的来源和许可证 |

## 离线验证

安装 SDK 后，在仓库根目录运行：

```bash
python3 -m unittest discover -s tests -v
python3 -m sdk.tui --selftest
```

`--selftest` 同时执行 SDK 协议、模拟状态机、串口重连与控制台回归检查，不连接硬件。
仅运行 SDK 检查可使用 `python3 -m sdk.selftest`。
