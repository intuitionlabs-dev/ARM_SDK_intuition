# USB-CAN 传输接口

包路径：`arx_x5_control.usb2can`。
发行包：`arx-x5-control 0.6.0`，版本定义见 [pyproject.toml](../../pyproject.toml)。Metal 的同构传输子包见 [Metal USB-CAN](../../metal_arm_control/usb2can/README.md)。
Python 版本：3.10 或更高。
该子包提供经典 CAN 标准数据帧收发。

## 安装

在 SDK 仓库根目录执行：

```bash
python3 -m pip install .
```

SLCAN 需要 pySerial：

```bash
python3 -m pip install '.[can]'
```

安装包包含控制核心、SDK 适配层和 CAN 传输子包。

## 类型和单位

| 名称 | 类型或单位 | 含义 |
| --- | --- | --- |
| `CANFrame.arbitration_id` | int，0～2047 | 标准帧仲裁 ID |
| `CANFrame.data` | bytes，0～8 字节 | 数据载荷 |
| `channel` | 非空 str | 设备路径或接口名 |
| `bitrate` | int，bit/s | CAN 速率 |
| `acquired_ns` | int，ns | 主机单调时钟下的接收时刻 |

内置后端接收 `bitrate=1000000`。
SocketCAN 接口须由设备驱动创建。
`configure()` 将接口置为 DOWN 并配置参数。
配置完成后，该方法将接口置为 UP。
固定参数为 1 Mbps、正常收发和 16 帧发送队列。
配置阶段需要 iproute2。
方法先直接调用 `ip`。
权限不足时，方法使用 `sudo -n` 重试。
运行用户须有网络管理权限或对应免密 sudo 权限。
每个接口的配置总超时为 2 秒。

## JSON 配置

```json
{
  "version": 1,
  "transports": {
    "slcan": {
      "class": "arx_x5_control.usb2can.slcan:CANableTransport",
      "channel_kind": "path",
      "options": {}
    },
    "socketcan": {
      "class": "arx_x5_control.usb2can.socketcan:SocketCANTransport",
      "channel_kind": "name",
      "options": {}
    }
  }
}
```

| 字段 | 类型和取值 | 含义 |
| --- | --- | --- |
| `version` | int，1 | 配置格式版本 |
| `transports` | 非空对象 | 接口名到后端配置的映射 |
| 接口名 | 非空 str | 调用方使用的后端标识 |
| `class` | `模块:属性` | 可导入的实现类路径 |
| `channel_kind` | `path` 或 `name` | 路径标识或名称标识 |
| `options` | 对象 | 实现类的附加关键字参数 |

每个后端配置必须提供表中三个字段。
配置文件定义完整映射。
接口名不能包含首尾空白。
字段不能重复。
`options` 的键必须是 Python 标识符。
`options` 不得包含 `channel` 或 `bitrate`。
配置中的类路径决定要导入的 Python 模块。

配置选择顺序：显式路径、`USB2CAN_CONFIG`、包内默认配置。
配置快照可通过 `spawn` 传递。
快照不包含已打开的设备句柄。

## 配置调用

```python
from arx_x5_control.usb2can import CANFrame, TransportRegistry

registry = TransportRegistry("/path/to/transports.json")
transport = registry.create("socketcan", "can0", bitrate=1000000)
try:
    transport.configure()
    received = transport.receive()
finally:
    transport.close()
```

| 方法 | 返回值 | 行为 |
| --- | --- | --- |
| `TransportRegistry(config_path=None)` | 注册表 | 加载并校验配置 |
| `names` | tuple[str, ...] | 返回接口名 |
| `specification(name)` | `TransportSpec` | 返回后端配置副本 |
| `endpoint(interface, channel)` | str | 返回规范化端点 |
| `create(interface, channel, bitrate=...)` | 传输对象 | 导入后端类并创建连接 |

配置读取失败或字段无效时，抛出 `TransportConfigError`。
接口名未知或实现类不可用时，也抛出该异常。
设备打开失败时，保留后端异常。
构造失败的后端须释放已取得的资源。

## 后端接口

实现类须提供以下方法：

```python
class DeviceTransport:
    def __init__(self, channel, *, bitrate=1000000, **options):
        ...

    def configure(self):
        ...

    def send(self, frame):
        ...

    def receive(self):
        ...

    def close(self):
        ...
```

`send()` 接收一个 `CANFrame`。
发送失败时，方法须抛出异常。
发送成功表示设备或驱动接受该帧。

`receive()` 返回 `(CANFrame, acquired_ns)` 列表。
无数据时，返回空列表。
调用不得等待新帧。
每次处理的报文数须有上限。
接收时间戳须保留反馈的实际时效。
后端须过滤本机发送回显。
后端须上报断线、溢出和总线错误。

支持通道配置的后端实现 `ConfigurableTransport`。
该接口在 `Transport` 上增加 `configure()`。
`configure()` 在启动收发前完成设备配置。
同一实例重复调用时，内置后端只配置一次。
SLCAN 构造函数也调用此方法。
其固定配置为 `C`、`S8`、`M0`、`A0`。
`CANMotorIO.open()` 先配置全部通道。
随后，它调用可选的 `start()` 并启动收发线程。
SocketCAN 配置失败时，方法尝试将接口置为 DOWN。
上层关闭已创建的连接并报告错误。

调用方可调用可选的 `start()` 方法。
调用方须在同一收发线程中调用收发方法。
`close()` 释放设备资源。
停机请求由机械臂控制层生成并发送。

## 接入设备

1. 实现后端类。
2. 将后端模块加入 Python 导入路径。
3. 在 JSON 中登记类路径和通道类型。
4. 使用接口名选择后端。

例如，外部模块提供 `vendor_can.Transport`：

```json
{
  "version": 1,
  "transports": {
    "vendor": {
      "class": "vendor_can:Transport",
      "channel_kind": "name",
      "options": {"device_index": 0}
    }
  }
}
```

后端构造函数须接收 `device_index`。
应用程序用 `interface="vendor"` 选择该配置。
