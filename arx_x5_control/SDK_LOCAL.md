# 通过 SDK 调用 PC Python 控制核心

`arx_x5_control.sdk_adapter.link.LocalLink` 实现了现有 `sdk.Link` 的调用接口。向 `sdk.Arm` 传入此对象后，`connect()`、初始化、模式切换、位置/速度控制、复位、设零、反馈读取及等待方法都会进入本机 `ControlSystem`。

```text
业务程序 → sdk.Arm（rad / rad/s / m）
                   ↓
              LocalLink（参数转换、命令队列、反馈分发）
                   ↓
              ControlSystem（状态机、轨迹、动力学、电机控制）
                   ↓
              MotorIO（MotorRequest / MotorFeedback）
                   ↓
              ProcessCANMotorIO → CANMotorIO → CANable2 / SocketCAN → 电机
```

SDK 的 `Arm`、单位换算和夹爪标定由同一仓库的 `sdk` 包提供。


## 目录结构

SDK 适配代码位于 `arx_x5_control/sdk_adapter/`。

| 文件 | 职责 |
| --- | --- |
| `link.py` | 转换 SDK 命令和反馈 |
| `tui.py` | 创建控制链路和 TUI 会话 |
| `tui_app.py` | 适配 SDK 终端界面 |
| `telemetry.py` | 记录 TUI 会话和运行指标 |
| `demo.py` | 提供 SDK 调用示例 |

导入本地链路：

```python
from arx_x5_control.sdk_adapter import LocalLink
```

## 运行完整示例

Python 3.10 及以上。在 SDK 仓库根目录执行：

```bash
python3 -m arx_x5_control.sdk_adapter.demo

# 本包所有运行日志切换为英文。
python3 -m arx_x5_control.sdk_adapter.demo --lang en
```

示例使用 SDK 完成双臂握手、初始化、夹爪找零、位置/速度下发、重力模式、阻尼模式及复位，按实际时间运行约 10 秒。末尾输出 `sequence_complete: true`，随后交付停机请求。它使用 `SimulatedMotors` 测试电机，控制算法运行的是 `ControlSystem`。

安装包同时包含 SDK、控制核心和动力学模型：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install .
.venv/bin/arx-x5-sdk-demo
```

安装 wheel 后可以从业务工程导入 SDK 和控制包。

## 现有 SDK 程序如何接入

将 `LocalLink(motor_io)` 传入 `sdk.Arm`。
无硬件时使用 `LocalLink.simulated()`：

```python
import sdk
from arx_x5_control import ArmConfig, log
from arx_x5_control.sdk_adapter.link import LocalLink
from arx_x5_control.sdk_adapter.demo import wait_arms

# 单臂示例；不传 configurations 时默认双臂。
log.configure(language="zh")
with LocalLink.simulated(configurations=[ArmConfig()]) as link:
    arm = sdk.Arm(0, link)             # 也可使用 link.arm(0)
    arm.connect()                    # 打开本地链路、握手、等待反馈
    wait_arms(link, [arm], lambda a: a.robot_state == sdk.board_defs.STATE_WAIT_INIT)
    arm.init_arm()
    wait_arms(link, [arm], lambda a: a.robot_state == sdk.board_defs.STATE_NORMAL)
    arm.start()
    wait_arms(link, [arm], lambda a: a.mode == sdk.board_defs.MODE_RUN)
    arm.send_ctrl_pos_vel(
        arm.joint_angles,            # 6 个关节角，rad
        (0.0,) * 6,                   # 6 个关节速度，rad/s
        gripper_raw_deg=-20.0,        # 未标定夹爪时可显式使用原始 deg
    )
    arm.wait_feedback(timeout=1)
    log.write_data(arm.snapshot())
# 离开 with 时，控制核心生成并交付停机请求。
```

夹爪完成 SDK 的两点标定后，可以继续使用 `gripper_m=...`，由 `sdk.GripperCalibStore` 和 `sdk.GripperCalibration` 换算为电机角。未标定时，米制控制仍按 SDK 原有行为报错。`joint_angles`、`joint_velocities` 和 `snapshot()` 仍提供 SI 单位；`arm.feedback` 是 SDK 的原始 deg 数据视图。

双臂共用一个 `LocalLink`，每台臂使用独立 `sdk.Arm`，并分别保活。`Arm.close()` 不关闭外部传入的共享链路；统一使用 `link.close()` 或 `with` 管理生命周期。

## 双臂实机接入例程

本例使用两条独立的 SocketCAN 通道。
`can0` 对应臂 A，`can1` 对应臂 B。
每条总线连接一台机械臂。
两台臂共享一个 `LocalLink`。

### 安装与准备

Python 版本须为 3.10 或更高。
在 SDK 仓库根目录执行：

```bash
python3 -m pip install '.[can]'
arx-x5-can-check --interface socketcan \
  --channel can0 --channel can1 --seconds 6
```

确认 `all_motors_responded` 为 `true`。
设备驱动须创建 `can0` 和 `can1`。
运行用户须有接口配置权限。
`ProcessCANMotorIO.open()` 自动配置 1 Mbps。
通道配置要求见 [CAN 接口](CAN_IO.md#安装和通道配置)。

初始化会执行复位和夹爪找零。
运行前，将两台臂放在复位位姿附近。
复位位姿为 `(0, 0.5, 0.5, 0, 0, 0)`°。
保证双臂运动空间无障碍。
同一通道由一个控制程序发送命令。

### 完整代码

将以下代码保存为 `dual_arm_control.py`：

```python
import math
import time

import sdk
from arx_x5_control import ArmConfig, CANChannel, ProcessCANMotorIO
from arx_x5_control.sdk_adapter import LocalLink


def wait_arms(link, arms, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        link.check_health()
        for arm in arms:
            arm.keepalive()
        if all(predicate(arm) for arm in arms):
            return
        time.sleep(0.05)
    raise TimeoutError("双臂状态等待超时")


def main():
    defs = sdk.board_defs
    channels = [
        CANChannel(0, "can0", interface="socketcan"),
        CANChannel(1, "can1", interface="socketcan"),
    ]
    with ProcessCANMotorIO(channels) as motor_io:
        with LocalLink(
            motor_io, configurations=[ArmConfig(), ArmConfig()]
        ) as link:
            arms = [sdk.Arm(i, link) for i in (0, 1)]
            for arm in arms:
                arm.connect()
            wait_arms(
                link, arms,
                lambda a: a.robot_state == defs.STATE_WAIT_INIT
                and a.motor_lost_mask == 0,
                timeout=5,
            )

            for arm in arms:
                arm.init_arm()
            wait_arms(
                link, arms,
                lambda a: a.robot_state == defs.STATE_NORMAL,
                timeout=30,
            )
            for arm in arms:
                arm.start()
            wait_arms(
                link, arms, lambda a: a.mode == defs.MODE_RUN,
                timeout=3,
            )

            origins = [tuple(arm.joint_angles) for arm in arms]
            started = time.monotonic()
            while True:
                elapsed = time.monotonic() - started
                if elapsed >= 5.0:
                    break
                link.check_health()
                for arm, origin in zip(arms, origins):
                    target = list(origin)
                    target[0] += 0.02 * math.sin(2 * math.pi * 0.2 * elapsed)
                    arm.send_ctrl_pos(target)
                time.sleep(0.01)

            for arm in arms:
                arm.reset()
            wait_arms(
                link, arms,
                lambda a: a.robot_state == defs.STATE_RESETTING,
                timeout=3,
            )
            wait_arms(
                link, arms,
                lambda a: a.robot_state == defs.STATE_NORMAL
                and a.mode == defs.MODE_DAMPING,
                timeout=10,
            )
            for arm in arms:
                arm.stop()
            wait_arms(
                link, arms, lambda a: a.mode == defs.MODE_STOP,
                timeout=3,
            )


if __name__ == "__main__":
    main()
```

执行程序：

```bash
python3 dual_arm_control.py
```

保留 `if __name__ == "__main__":` 入口。
CAN 工作进程使用 `spawn` 启动。
退出 `LocalLink` 上下文时交付双臂停机请求。
退出 `ProcessCANMotorIO` 上下文时关闭设备。
停机后，机械臂可能在重力下下落。

循环使双臂关节 1 做小幅正弦运动。
幅度为 0.02 rad，频率为 0.2 Hz。
循环运行 5 秒，目标下发间隔约为 10 ms。
其余关节保持起始目标，夹爪保持当前角度。
控制核心在后台执行目标周期为 1 ms 的控制。

### 接入上层控制程序

将循环中的 `target` 换为规划器输出。
每个目标包含六个关节角，单位为 rad。
确认关节顺序、正方向和零点一致。
位置速度控制使用以下调用：

```python
arm.send_ctrl_pos_vel(target_rad, velocity_rad_s)
```

`arm.joint_angles` 返回关节角，单位为 rad。
`arm.joint_velocities` 返回速度，单位为 rad/s。
`arm.snapshot()` 返回同一帧的反馈视图。
等待期间，例程对两台臂分别保活。
控制命令本身也构成对应臂的保活。
暂停目标下发时，每隔约 0.5 秒逐臂调用 `keepalive()`。
每台臂的有效命令间隔须小于 3 秒。

SLCAN 设备使用以下通道定义：

```python
channels = [
    CANChannel(0, "/dev/serial/by-id/臂A设备", interface="slcan"),
    CANChannel(1, "/dev/serial/by-id/臂B设备", interface="slcan"),
]
```

将设备路径替换为实际路径。
内置后端配置位于 `arx_x5_control.usb2can`。
外部设备接入见 [传输接口](usb2can/README.md)。

双臂 TUI 启动命令：

```bash
arx-x5-tui --interface socketcan --channel can0 --channel can1
```

TUI 自动保存会话日志。
上述 Python 例程通过状态和异常确认执行结果。
TUI 日志文件说明见 [运行日志](LOGGING.md#tui-运行日志)。

## 电机侧只实现两个接口

双臂持续控制使用 `ProcessCANMotorIO`。
通过 `LocalLink(motor_io)` 接入。
CAN 收发在独立进程内运行。
SLCAN 和 SocketCAN 的配置见 [CAN 接口](CAN_IO.md)。
双臂通道映射和启动代码见 [双臂实机接入例程](#双臂实机接入例程)。

通过 `LocalLink(motor_io, configurations=...)` 传入满足 `MotorIO` 协议的对象：

```python
from arx_x5_control import MotorFeedback, MotorRequest

class MyMotorIO:
    def poll_feedback(self, now_ms: int) -> tuple[MotorFeedback, ...]:
        # 取走新反馈，当前无数据时返回空元组。
        # 不得重复返回旧反馈，也不得将旧反馈的采样时间改成 now_ms。
        raise NotImplementedError

    def write_motor_requests(self, requests: tuple[MotorRequest, ...]) -> None:
        # 顺序交付控制、启停、设零请求；无法接收时抛出异常。
        # 包括 close() 时发出的 SOFT_STOP 和 DISABLE。
        raise NotImplementedError
```

两个方法都必须非阻塞，在控制所属线程中依次执行。反馈关节电机角为 rad、速度为 rad/s，输出转矩为 N·m；字段定义见 [Python 接口文档](PC_API.md)。设备打开、设备关闭、请求交付和设零保护间隔由电机适配器负责。

后台模式中 `now_ms` 是从 `LocalLink` **对象构造时刻**起经过的毫秒数；电机采样时间必须转换到同一时钟。`reopen()` 保留适配器对象和时钟起点。不要使用 Unix 时间戳，也不要在重开时把适配器时间清零。

`LocalLink` 在内存中转换 `sdk.Arm` 的 payload 参数。`CANMotorIO` 负责电机 CAN 帧编解码和设备收发。两者通过 `MotorRequest` 和 `MotorFeedback` 交换数据。

## SDK 方法映射

| SDK 方法 | 控制核心输入或输出 |
| --- | --- |
| `connect()`、`send_version()`、`keepalive()` | 校验 SDK 版本，提交 `CONNECT`；重复握手刷新该臂保活时间 |
| `init_arm()` | `INITIALIZE`，保留位姿、在线和状态门槛 |
| `set_zero()` | `CALIBRATE`，完成事件转换为 `sdk.SetZeroAck` |
| `start()` / `stop()` / `damping()` / `gravity()` | 对应模式命令 |
| `send_ctrl_pos()` | `POSITION`，关节 rad 转 deg，保留夹爪标定换算 |
| `send_ctrl_pos_vel()` | `POSITION_VELOCITY`，速度 rad/s 转 deg/s |
| `reset()` | `RESET`，执行复位轨迹 |
| `feedback`、`snapshot()`、`wait_feedback()`、`wait_for()` | 从对应臂的 SDK 通道读取控制核心反馈 |
| `last_setzero_ack`、`wait_setzero_ack()` | 读取该臂本轮设零的状态、7 个电机结果及耗时 |
| `send_raw()` | 支持上述命令的 SDK payload；未知类型、非法长度、非有限数及版本不符直接报错 |

`force=True` 仅跳过 SDK 的客户端预检，控制核心的状态和电机条件仍然生效。发送成功表示进入命令队列，执行结果应通过反馈或事件确认。每台臂队列容量为 7，满时抛出 `sdk.LinkError`，不会覆盖未执行的命令。

## 调度、退出和故障

- `open()` 启动目标周期为 1 ms 的线程，命令每 2 ms 处理，反馈约每 4 ms 发布；PC Python 不保证硬实时。
- 不自动发送保活。每台臂需在 3 s 内收到有效命令，反馈输出本身不算保活。等待初始化或设零时也要持续发送 `keepalive()`；示例的 `wait_arms()` 已处理此事。
- `close()` 停止控制线程并交付所有电机的停机请求；`reopen()` 新建控制器，清除旧反馈、旧设零结果和未执行命令，需要重新握手与初始化。
- 适配器抛异常时，后端记录故障并尝试交付停机请求。通过 `check_health()`、后续发送或 `wait_feedback()` 可取得 `sdk.LinkError`；`stats["link_err"]` 可读原因。SDK 的 `wait_for()` 和 `wait_setzero_ack()` 沿用原有轮询行为，应用长时间等待时也应调用 `check_health()`。
- 若停机请求也交付失败，`close()` 会报错。若适配器阻塞，超过 2 秒仍未退出会报错，并拒绝在旧线程运行时再开控制线程。适配器必须遵守非阻塞约定。
- `link.stats` 增加漏拍、旧反馈、核心拒绝命令、各臂队列长度和事件覆盖计数。`n_tx/n_rx` 为逻辑消息数量；没有真实串口状态包，`link_status` 为 `None`，CRC 计数为零。
- `drain_events()` 取走最近的控制事件，最多保留 512 条，历史覆盖数量记录在 `event_overruns` 中。`last_result` 提供最近一次控制结果；后台运行期间不得直接修改 `controller`。

## 确定性测试与回放

设置 `background=False` 后，使用调用者提供的严格递增毫秒时钟推进：

```python
from arx_x5_control.sdk_adapter.link import LocalLink
from arx_x5_control import log

log.configure(language="zh")
with LocalLink.simulated(background=False) as link:
    arm = link.arm(0)
    arm.send_version()
    link.step(0)
    log.write_data(arm.snapshot())
    arm.init_arm()
    for now_ms in range(1, 8001):
        if now_ms % 500 == 0:
            arm.keepalive()
        link.step(now_ms)
    log.write_data(arm.snapshot())
```

手动模式下，SDK 的阻塞等待不会自动推进控制时钟。可先提交命令再调用 `step()`，或让另一个所属线程持续推进。时间必须在 `reopen()` 前后保持递增。

后端运行日志使用 `arx_x5_control.log`，通过 `log.configure(language="zh"/"en")` 切换全包语言；不改变 SDK 自己的语言设置。配置、输出目标和事件数据的区别见 [日志文档](LOGGING.md)。

## 验证

安装后，在 SDK 仓库根目录运行：

```bash
python3 -m sdk.selftest
python3 -m sdk.tui --selftest
python3 -m arx_x5_control.sdk_adapter.demo
```

H7 控制工程的 `tests/` 提供集成测试。
在 H7 工程根目录安装 `./sdk` 后运行 `python3 -m unittest discover -s tests -v`。
集成测试使用 `sdk.Arm` 验证双臂初始化、模式、复位和控制单位。
测试还覆盖设零结果、在线掩码、保活、队列及退出停机。

## SDK TUI

`arx_x5_control.sdk_adapter.tui` 使用 SDK 的终端界面和按键。
双臂共用一个 `LocalLink`。
`ProcessCANMotorIO` 在独立进程中执行 CAN 收发。

nuc3 的快捷入口为 `/home/intuition-nuc3/arx-x5-tui`。
通过 SSH 打开交互终端并启动：

```bash
ssh -t intuition-nuc3 /home/intuition-nuc3/arx-x5-tui
```

快捷入口使用以下通道配置：

```bash
can_prefix=/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_
/home/intuition-nuc3/arx-connectivity-check/.venv/bin/arx-x5-tui \
  --channel "${can_prefix}208C339E5931-if00" \
  --channel "${can_prefix}207235C34831-if00"
```

通道顺序对应臂 0、臂 1。
此命令在启动时显示反馈并保持 STOP。
初始化和运动由按键触发。

仿真入口：

```bash
python3 -m arx_x5_control.sdk_adapter.tui --simulate
```

SocketCAN 使用 `--interface socketcan`。
每个 `--channel` 指定对应 CAN 网络接口。

| 按键 | 操作 |
| --- | --- |
| `a` | 切换当前臂 |
| `i` | 初始化当前臂并执行夹爪找零 |
| `s` | 进入 RUN |
| `1`～`6`、`0` | 选择关节或夹爪 |
| `+`、`-`、方向键 | 按当前步长调整目标 |
| `[`、`]` | 切换步长，关节单位为 rad |
| `c` | 将目标对齐当前反馈 |
| `d`、`v`、`x` | 阻尼、重力补偿、STOP |
| `o` | 复位当前臂 |
| `h` | 显示帮助 |
| `m` | 记录停顿标记，用于对齐诊断日志 |
| `q` | 退出，停止双臂并关闭 CAN |

默认目标下发频率为每臂 100 Hz。
控制核心独立执行电机控制周期。
切换当前臂时，两台臂继续各自保活。
退出和界面异常均触发双臂停机。
TUI 自动保存每次会话的运行日志。
指标和文件说明见 [运行日志](LOGGING.md#tui-运行日志)。
`--log-dir path` 指定日志根目录。
`--log path.log` 指定额外的界面事件文件。
`--lang en` 使用英文界面。
夹爪以 m 显示时，需要 SDK 两点标定。
CAN 故障后，退出控制台并重新启动。

TUI 按键测试使用模拟电机验证初始化、切臂、点动和复位。
nuc3 终端验证覆盖实机反馈显示、切臂和退出停机。
