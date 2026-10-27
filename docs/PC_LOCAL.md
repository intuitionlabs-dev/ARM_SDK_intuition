# PC 本地链路参考

本页适用于 ARX X5 和 Metal Arm 的 `LocalLink`。两者继承 `sdk.Link`，将公共 `sdk.Arm` 调用交给本地控制核心。完整应用示例见 [Python 调用](PYTHON.md)，型号参数见 [机械臂差异](ROBOTS.md)。

## 创建参数

```text
LocalLink(motor_io, *, configurations=None, background=True,
          collect_performance=False, collect_trace=False)
LocalLink.simulated(*, configurations=None, background=True,
                    collect_performance=False, collect_trace=False)
```

| 参数 | 含义 |
| --- | --- |
| `motor_io` | 非阻塞电机接口；设备打开和关闭由调用者负责 |
| `configurations` | 每台臂一个同包 `ArmConfig`；默认双臂，单臂传一个 |
| `background` | True 启动控制线程；False 由应用调用 `step(now_ms)` |
| `collect_performance` | 启用周期与耗时统计 |
| `collect_trace` | 启用每拍控制记录 |

| 型号 | `LocalLink` 导入 | `ArmConfig` 与电机类型导入 |
| --- | --- | --- |
| ARX X5 | `arx_x5_control.sdk_adapter` | `arx_x5_control` |
| Metal Arm | `metal_arm_control.sdk_adapter` | `metal_arm_control` |

下面的底层扩展示例使用 ARX 类型导入；Metal 在设备配置部分选用其对应包。状态机、方法和生命周期约定适用于两者。

## 电机侧只实现两个接口

双臂持续控制使用 `ProcessCANMotorIO`。
通过 `LocalLink(motor_io)` 接入。
CAN 收发在独立进程内运行。
SLCAN 和 SocketCAN 的配置见 [CAN 接口](CAN.md)。
双臂通道映射和启动代码见 [Python CAN 示例](PYTHON.md#pc-can-连接)。

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

两个方法都必须非阻塞，在控制所属线程中依次执行。反馈关节电机角为 rad、速度为 rad/s，输出转矩为 N·m；字段定义见 [Python 接口文档](PC_CONTROL.md)。设备打开、设备关闭、请求交付和设零保护间隔由电机适配器负责。

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
- 适配器抛异常时，后端记录故障并尝试交付停机请求。通过 `check_health()`、后续发送或 `wait_feedback()` 可取得 `sdk.LinkError`；`stats["link_err"]` 可读原因。SDK 的 `wait_for()` 和 `wait_setzero_ack()` 使用轮询等待，应用长时间等待时也应调用 `check_health()`。
- 若停机请求也交付失败，`close()` 会报错。若适配器阻塞，超过 2 秒仍未退出会报错，并拒绝在旧线程运行时再开控制线程。适配器必须遵守非阻塞约定。
- `link.stats` 增加漏拍、旧反馈、核心拒绝命令、各臂队列长度和事件覆盖计数。`n_tx/n_rx` 为逻辑消息数量；没有真实串口状态包，`link_status` 为 `None`，CRC 计数为零。
- `drain_events()` 取走最近的控制事件，最多保留 512 条，事件覆盖数量记录在 `event_overruns` 中。`last_result` 提供最近一次控制结果；后台运行期间不得直接修改 `controller`。

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
python3 -m unittest discover -s tests -v
python3 -m arx_x5_control.sdk_adapter.demo
```

本仓库测试覆盖后端接口对齐、独立参数、H7/TUI 选型、Metal 状态流程及 CAN 退出。测试命令和覆盖范围见 [验证说明](../HANDOFF_METAL_ARM.md#验证)。

