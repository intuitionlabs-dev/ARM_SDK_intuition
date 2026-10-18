# CAN 电机收发接口

适用包版本：`arx-x5-control 0.5.0`。
类型入口为 `CANChannel`、`CANMotorIO`、`ProcessCANMotorIO` 和 `CANControlIO`。

## 术语和单位

| 术语 | 含义 |
| --- | --- |
| CANable2 | 通过 USB 串口使用 SLCAN 的 CAN 适配器 |
| SLCAN | 串口上的 CAN 帧命令格式 |
| SocketCAN | Linux CAN 网络接口 |
| MotorIO | 控制核心使用的电机请求和反馈接口 |

每条 CAN 总线连接一台臂。两台臂分别使用独立通道。CAN 速率为 1 Mbit/s，帧类型为经典 CAN 标准数据帧。关节电机 ID 为 `1、2、4、5、6、7`，夹爪 ID 为 `8`。

`MotorRequest` 和 `MotorFeedback` 使用 rad、rad/s、N·m。ENCOS 电流反馈使用 A。温度使用 ℃。时间戳使用控制器单调时钟的整数毫秒。

## 安装和通道配置

CANable2 需要 pySerial：

```bash
python3 -m pip install '.[can]'
```

SocketCAN 收发使用 Python 标准库。
设备驱动须创建对应网络接口。
`CANMotorIO.open()` 自动调用传输后端的 `configure()`。
SocketCAN 固定使用 1 Mbit/s、正常收发和 16 帧队列。
配置完成后，后端将接口置为 UP。
配置需要 iproute2 和网络管理权限。
权限不足时，后端通过 `sudo -n` 调用 `ip`。
配置失败时，`open()` 关闭连接并抛出异常。

`CANChannel(arm_id, channel, interface="slcan", bitrate=1000000)`：

| 字段 | 类型和取值 | 含义 |
| --- | --- | --- |
| `arm_id` | int，0 或 1 | 控制核心的臂实例编号 |
| `channel` | 非空 str | 稳定串口路径或 SocketCAN 接口名 |
| `interface` | 非空 str | 传输配置中的接口名 |
| `bitrate` | int，1000000 | CAN 速率，bit/s |

示例：

```python
from arx_x5_control import CANChannel

channels = [
    CANChannel(0, "/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_208C339E5931-if00"),
    CANChannel(1, "/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_207235C34831-if00"),
]

socketcan_channels = [
    CANChannel(0, "can0", interface="socketcan"),
    CANChannel(1, "can1", interface="socketcan"),
]
```

臂编号按通道配置确定。两个通道的臂编号及设备路径必须分别唯一。CANable2 串口以独占模式打开。SocketCAN 通道须由本控制程序独占发送。

## MotorIO 方法

`CANMotorIO(channels, **options)` 创建适配器。构造函数保存配置。`open()` 打开通道，完成传输配置，再启动后台线程。一个实例支持一次打开周期。

| 选项 | 默认值 | 含义 |
| --- | --- | --- |
| `read_only` | False | True 时只允许反馈查询 |
| `tx_capacity` | 256 | 每臂发送队列的帧数上限 |
| `rx_capacity` | 4096 | 共享接收队列的样本数上限 |
| `max_tx_age_ms` | 20 | CONTROL、ENABLE 的最大排队时间，ms |
| `max_rx_age_ms` | 50 | 反馈最大接收年龄及接收线程最大间隔，ms |
| `command_timeout_ms` | 100 | 单电机持续控制流的最大间隔，ms |
| `transport_factory` | 配置工厂 | 自定义传输工厂，参数为 `CANChannel` |
| `transport_config` | None | JSON 路径或 `TransportRegistry` 配置快照 |

容量须为正整数。时间参数须为有限正数。

| 方法 | 行为 |
| --- | --- |
| `poll_feedback(now_ms)` | 排空有效新反馈，返回 `MotorFeedback` 元组 |
| `write_motor_requests(requests)` | 验证整批请求，然后按臂排队 |
| `query_feedback(arm_id=None)` | 查询指定臂或全部臂的位置、速度、电流和状态 |
| `check_health()` | 检查打开状态及后台故障 |
| `stats()` | 返回统计快照 |
| `flush(timeout=2.0)` | 等待发送队列和当前发送完成 |
| `close(timeout=2.0)` | 取消排队请求，停止电机输出并关闭通道 |

`poll_feedback` 和 `write_motor_requests` 由控制线程调用。两者通过内存队列交换数据。设备读写由后台线程完成。`open`、`flush` 和 `close` 是可等待的生命周期方法。

`poll_feedback` 首次调用建立控制器时钟映射。后续 `now_ms` 必须递增或保持相等。调用者须按实际单调经过时间推进。SocketCAN 使用内核接收时间。SLCAN 使用后台线程的主机接收时间。USB 适配器提供的帧格式包含 CAN ID、长度和数据。

反馈排队期间保留接收时间。过期反馈增加 `stale_rx`，随后丢弃。无效或未知反馈增加 `ignored_rx`。SocketCAN 排除本机发送回显及扩展帧、远程帧。总线错误帧触发适配器故障。

普通请求在每条总线上保持顺序。ENCOS 静默期间，达妙请求可继续发送。同一电机请求保持顺序。每条总线的发送调用起点至少间隔 200 μs。

ENCOS 设零和反馈查询之后，本臂 ENCOS 静默 501 ms。设零要求本臂 ENCOS 均处于适配器的停止状态。CONTROL 使该电机进入适配器的活动状态。SOFT_STOP 或 DISABLE 清除该状态。

`query_feedback` 使用 ENCOS 查询代码 1、2、3，以及达妙 `0xCC` 状态读取。提交电机输出后，该实例的查询方法会报错。只读诊断使用独立适配器实例。

## 传输后端配置

传输实现位于子包 `arx_x5_control.usb2can`。
配置格式见 [usb2can 接口说明](usb2can/README.md)。

配置选择顺序如下：

1. 使用 `transport_config` 参数。
2. 读取 `USB2CAN_CONFIG` 环境变量。
3. 使用包内 `arx_x5_control/usb2can/transports.json`。

外部配置定义完整的接口映射。
`CANChannel.interface` 指定映射名称。
配置中的 `class` 指定实现类路径。
`options` 指定实现类的附加构造参数。
`channel_kind` 指定通道标识的解析方式。

`CANMotorIO` 在构造时校验映射。
它在 `open()` 时创建设备连接。
两个通道不能指向同一后端类的同一端点。
路径类型通道按真实路径判断端点。
命名类型通道按名称判断端点。
`ProcessCANMotorIO` 向子进程传递配置快照。

TUI 示例：

```bash
arx-x5-tui --interface socketcan --channel can0 \
  --transport-config /path/to/transports.json
```

只读查询示例：

```bash
arx-x5-can-check --interface socketcan --channel can0 \
  --transport-config /path/to/transports.json --seconds 6
```

## 控制请求编码

| `MotorAction` | ENCOS | 达妙 |
| --- | --- | --- |
| CONTROL | 混合控制帧，KD 为 9 bit | MIT 控制帧，KD 为 12 bit |
| SOFT_STOP | 零增益、零速度、零力矩混合帧 | 固件软停机 MIT 帧 |
| DISABLE | 软停机混合帧 | `FF FF FF FF FF FF FF FD` |
| ENABLE | 由 CONTROL 提供使能语义 | `FF FF FF FF FF FF FF FC` |
| SAVE_ZERO | `0x7FF: 00 ID 00 03` | `FF FF FF FF FF FF FF FE` |

编码器要求臂号、索引、ID 和电机类型一致。位置、速度、力矩和增益须为有限数。增益须非负。编码器按固件量程限制输出。ENCOS ENABLE 单独请求会报错。

ENCOS 反馈使用电机 ID 路由。达妙反馈使用仲裁 ID 0，并按载荷首字节低 4 位路由。ENCOS 设置应答提供 `zero_ack`。该应答须匹配本总线的待确认设零请求，并在发送后 1500 ms 内到达。设置应答保留最近的电机错误码。部分字段应答仅更新对应字段。

## 持续控制和退出

SDK 双臂初始化、运动、复位及退出见 [双臂实机接入例程](SDK_LOCAL.md#双臂实机接入例程)。

双臂持续控制使用 `ProcessCANMotorIO`。
该适配器在独立进程内运行 `CANMotorIO`。
控制计算与 CAN 收发使用独立的 Python 解释器。

构造参数为 `ProcessCANMotorIO(channels, ipc_capacity=256, **options)`。
`options` 为 `CANMotorIO` 的选项。
`ipc_capacity` 是每条进程通信队列的消息数上限。
该值必须为正整数。

方法与 `CANMotorIO` 相同。
`open(timeout=5.0)` 等待工作进程就绪。
`close(timeout=3.0)` 等待停机和进程退出。
测试传输工厂必须支持 `spawn` 序列化。

按实际单调时间周期调用 `poll_feedback(now_ms)`。
该方法每 10 ms 发送进程保活。
工作进程的保活超时为 100 ms。
超时后，工作进程发送停机帧并关闭通道。
进程通信队列溢出也触发停机。
反馈经进程队列传递时保留采样时间。
进程反馈的有效期由 `max_rx_age_ms` 指定。
过期反馈增加 `ipc_stale_rx`。
进程接收缓存的上限由 `rx_capacity` 指定。
`flush` 等待期间持续发送进程保活。

SDK 接入使用 `__main__` 入口保护：

```python
from arx_x5_control import CANChannel, ProcessCANMotorIO
from arx_x5_control.sdk_adapter.link import LocalLink

def main():
    channels = [
        CANChannel(0, "arxcan0", interface="socketcan"),
        CANChannel(1, "arxcan1", interface="socketcan"),
    ]
    with ProcessCANMotorIO(channels) as motor_io:
        with LocalLink(motor_io) as link:
            arms = [link.arm(i).connect() for i in (0, 1)]
            # 在此运行应用的保活和关节控制循环。

if __name__ == "__main__":
    main()
```

`LocalLink` 负责控制周期和 SDK 命令转换。
`ProcessCANMotorIO` 负责进程通信和故障转发。
`CANMotorIO` 负责电机 CAN 收发。
打开 `LocalLink` 后，控制核心持续提交软停机和达妙使能保活请求。
夹爪设零时，六个关节继续保持控制周期。
设零流程独占参与电机的输出。

`runtime.run` 接入：

```python
from arx_x5_control import CANControlIO, ControlSystem
from arx_x5_control.runtime import run

with CANControlIO(channels, command_source=lambda now_ms: ()) as io:
    run(ControlSystem(), io, duration_s=1.0)
```

`command_source(now_ms)` 返回本拍命令。`feedback_sink(feedback)` 接收反馈元组。`event_sink(events)` 接收事件元组。回调由控制线程执行，须及时返回。

队列溢出、发送超时、持续控制流超时、读写异常和接收线程超时会锁存故障。控制接口随后抛出 `CANIOError`。后台线程对已提交电机输出的总线执行停机，再关闭通道。ENCOS 停机遵守静默期，达妙使用 DISABLE。

`close` 取消尚未发送的请求，并通过 `cancelled_tx` 计数。需要交付维护操作时，先调用 `flush`。关闭时的停机请求直接发送。故障时的停机发送失败会产生错误日志。

统计包含 `tx_frames`、`motor_tx_frames`、`query_tx_frames`、`rx_frames`、`feedback_samples`、`stale_rx`、`ignored_rx`、`shutdown_tx_frames` 和 `cancelled_tx`。`pending_tx`、`pending_rx` 为当前排队数，`failed` 为故障状态。帧数统计表示主机传输完成。电机应答用于确认设备连通。

`max_rx_poll_duration_ns` 记录接收调用的最大耗时。
`max_tx_duration_ns` 记录发送调用的最大耗时。
两者单位均为 ns。
进程适配器还提供 `worker_process`、`process_alive`、`pending_ipc_rx` 和 `ipc_stale_rx`。
工作进程每 20 ms 发布统计快照。

## 只读连通性检查

```bash
arx-x5-can-check --channel /dev/ttyACM0 --channel /dev/ttyACM1 --seconds 10
```

同一入口可通过 `python3 -m arx_x5_control.can_check` 执行。SocketCAN 使用 `--interface socketcan --channel can0 --channel can1`。

检查器使用 `read_only=True`。控制核心保持 STOP，产生的电机请求仅计数。硬件通道只发送读取请求。结果 JSON 写入标准输出。日志支持 `--lang zh/en` 和 `--log-level`。

退出码：0=全部电机有反馈，1=执行异常，2=至少一台电机缺少反馈，130=用户中断。报告包含每个电机的回复数、位置、速度、位置波动和错误码。

## 协议来源和验证

H7 控制工程的 ENCOS 编码以 `libs/encos_motor/encos_protocol.c` 为准。达妙编码与量化以 `libs/dm_motor/DM_motor.c` 和 `libs/common/common.c` 为准。

SLCAN 命令依据 [CANable2 固件](https://github.com/normaldotcom/canable2-fw/blob/main/README.md)。达妙状态读取依据 [厂商例程 refresh_motor_status](https://github.com/dmBots/motor-control-routine/blob/master/orin%E8%BD%BD%E6%9D%BFcan%E6%8E%A7%E5%88%B6%E8%BE%BE%E5%A6%99%E7%94%B5%E6%9C%BA%E4%BE%8B%E7%A8%8B/dm_hw/src/hardware_interface/damiao.cpp)。SocketCAN 时间戳和回显依据 [Linux 文档](https://docs.kernel.org/networking/can.html)。串口行为依据 [pySerial API](https://pyserial.readthedocs.io/en/latest/pyserial_api.html)。

在 H7 控制工程根目录安装 `./sdk`。
运行 `python3 -m unittest discover -s tests -v`。
测试覆盖帧编码、解码、队列、静默期、时间戳、进程保活及故障停机。
主循环和 CAN 周期统计见 [控制周期稳定性](CONTROL_TIMING.md)。
