# arx_x5_control 包专属日志库

`arx_x5_control` 使用 `arx_x5_control.log`。
每个独立包拥有自己的日志入口、词条和配置。
本库保持 SDK、重力模块和根日志器的配置。

## 一处配置，整个包生效

```python
from arx_x5_control import log

# 应用启动时配置一次；默认中文、INFO 级别，输出到标准错误流。
log.configure(language="zh", level="INFO")

# 运行中切换语言：包内已有日志对象和后台控制线程在下一条输出时使用英文。
log.configure(language="en")

# 切回中文，同时查看握手和保活等调试事件。
log.configure(language="zh", level="DEBUG")
```

这里的“全局”是**当前包内全局**：所有 `arx_x5_control.*` 模块读取同一份配置。另一个独立包应在其自己的 `log` 库中保存配置和词条，不能复用 `arx_x5_control.log`；本库拒绝创建 `sdk` 等其他包命名空间的日志对象。

支持的语言为 `zh`、`en`，级别为 `DEBUG`、`INFO`、`WARNING`、`ERROR`、`CRITICAL`。`log.get_config()` 返回不可变的 `LogConfig` 快照。非法配置会报错，原配置保持不变。重复配置不会重复添加输出处理器。

仅导入包不会输出日志。命令行入口自动调用 `configure()`；作为库使用时，由应用调用 `configure()` 启用输出，或自行安装日志处理器。

## 命令行和环境变量

两个运行入口使用相同的包级配置方式：

```bash
python3 -m arx_x5_control --demo --lang zh
python3 -m arx_x5_control --demo --lang en --log-level DEBUG
python3 -m arx_x5_control.sdk_adapter.demo --lang en

# 本包专属环境变量，只由本包的命令行入口读取。
ARX_X5_CONTROL_LOG_LANGUAGE=en ARX_X5_CONTROL_LOG_LEVEL=WARNING python3 -m arx_x5_control --demo
```

选择顺序为：显式命令行参数 > 本包环境变量 > 已有包级配置。新进程默认中文和 `INFO`。安装后的 `arx-x5-control`、`arx-x5-sdk-demo` 命令支持相同选项。普通 Python 应用通过 `log.configure()` 设置，不依赖命令行环境变量。

## 包内日志的写法

```python
# 以下示例应位于 arx_x5_control 包内。
from . import log

_LOG = log.get_logger(__name__)

def start_loop():
    _LOG.info("runtime.started")
```

业务模块传入**词条键和原始参数**。
消息词条定义在 [`log_messages.py`](log_messages.py) 中。
中文和英文词条使用相同的格式参数。例如：

```python
_LOG.warning("event.watchdog", arm_id=0)
# 中文：臂 0 命令超时，已切换到阻尼模式
# 英文：Arm 0 command timeout; switched to damping mode
```

业务模块通过 `log.py` 输出运行日志。
`tests/test_log.py` 检查本包的输出入口。

控制核心通过 `Event` 提供事件数据。运行循环、本地 SDK 后端和模拟入口调用 `_LOG.event(event)` 或 `_LOG.events(events)` 输出日志。日志语言只影响显示文本。握手和保活事件为 `DEBUG`，状态变化为 `INFO`，拒绝命令和保活超时为 `WARNING`。初始化失败、参考值错误和动力学错误为 `ERROR`。`dynamics_error` 按状态码翻译错误原因。

## 日志与机器数据

默认人类可读日志写入标准错误流，格式如下：

```text
INFO [arx_x5_control.sdk_adapter.link] 臂 0 初始化完成
INFO [arx_x5_control.sdk_adapter.link] Arm 0 initialization complete
```

模拟结果等机器可读数据统一使用 `log.write_data(value)` 输出到标准输出，保留 JSON 字段、数值和枚举表示，不随显示语言变化。这样可以分别保存结果和日志：

```bash
python3 -m arx_x5_control --demo --lang zh > result.jsonl 2> run.log
```

标准输出包含最终结果 JSON。
控制事件以所选语言写入日志。
通过 `demo(..., emit=callback)` 保存事件 JSON：

```python
from arx_x5_control.__main__ import demo
from arx_x5_control import log

log.configure(language="zh")
records = []
summary = demo(emit=records.append)
# records 保留每条原始事件及最终结果的 JSON 字符串。
```

外部适配器或 SDK 抛出的异常对象、未知事件详情等原始数据会保留原内容，本库不猜测或改写其他包的消息。日志的本包说明文案仍按配置语言输出。

## 接入应用自己的输出目标

```python
from arx_x5_control import log

with open("control.log", "a", encoding="utf-8") as stream:
    log.configure(language="zh", stream=stream)
    # 在此运行控制程序；结束后先退出控制线程，再关闭输出文件。
    log.configure(stream=None)
```

省略 `stream` 时保留原输出目标；首次调用默认选择当时的 `sys.stderr`。`stream=None` 禁用本库自带的输出处理器，不会移除应用自己添加的处理器。

应用也可在 `logging.getLogger("arx_x5_control")` 上安装自己的 `logging.Handler`，例如队列处理器。每条记录携带 `package`、`message_key`、`language`、`message_values` 字段；控制事件记录还带有原始 `control_event`。本包日志不向根日志器传播，避免与宿主应用的处理器重复输出。

默认处理器同步输出。
应用的日志处理器必须及时返回。
通过队列处理器异步写入文件。
每条事件使用同一份语言快照。

## TUI 运行日志

`arx-x5-tui` 自动记录实机和仿真会话。
每次启动创建独立的会话目录。
日志目录名包含 UTC 时间、进程 ID 和随机标识。
启动方法见 [SDK TUI](SDK_LOCAL.md#sdk-tui)。

默认根目录为 `~/.local/state/arx-x5-control/runs`。
设置 `XDG_STATE_HOME` 时，根目录为
`$XDG_STATE_HOME/arx-x5-control/runs`。
`--log-dir PATH` 指定日志根目录。
`latest` 链接指向最近创建的会话。
会话文件持续保留。

nuc3 默认日志位置为：

```text
/home/intuition-nuc3/.local/state/arx-x5-control/runs/
```

操作结束后，读取 `latest` 的目标目录：

```bash
readlink -f ~/.local/state/arx-x5-control/runs/latest
```

保留整个会话目录以供分析。
日志采用 UTF-8 编码。
JSONL 文件每行包含一个 JSON 对象。

| 文件 | 内容 | 写入周期 |
| --- | --- | --- |
| `metadata.json` | 主机、内核、Python、版本、源码哈希和启动参数 | 启动时 |
| `metrics.jsonl` | 周期直方图、CAN 计数、队列和 CPU 采样 | 约 1 s |
| `states.jsonl` | 双臂状态、模式、目标、角度、速度、温度和在线掩码 | 约 100 ms |
| `events.jsonl` | 会话配置和控制核心事件 | 事件发生时 |
| `runtime.log` | 本包运行消息和错误 | 消息发生时 |
| `ui.log` | TUI 操作和状态消息 | 消息发生时 |
| `summary.json` | 累计统计、最终状态和日志丢弃计数 | 约 1 s，退出时 |
| `control_trace.bin` | 每拍目标、参考值、反馈、在线状态和请求类型 | 控制线程每拍采集，后台批量写入 |
| `control_trace_schema.json` | 二进制记录格式、字段顺序和单位 | 启动时 |

后台日志线程执行文件写入。
控制线程和 CAN 线程在内存中累计指标。
事件队列容量为 4096 条。
队列满时，日志器累计丢弃计数。
TUI 显示日志写入错误。

### 停顿诊断

在感觉到停顿后按 `m`，记录事件标记。
标记包含按键接收时间、当前臂和所选关节。
标记时间表示操作员按键的时间。

`events.jsonl` 包含以下诊断事件：

| `kind` | 数据字段 | 含义 |
| --- | --- | --- |
| `input` | `key_code`、`received_ns`、`completed_ns`、`before`、`after` | 按键接收、处理耗时及目标变化 |
| `command` | `sequence`、`arm_id`、`started_ns`、`completed_ns`、`sent`、`target_rad` | SDK 位置命令下发 |
| `marker` | `received_ns`、`arm_id`、`axis`、`target_rad` | 操作员标记 |

事件数据中的 `*_ns` 使用主机单调时钟。
`input.received_ns` 表示 curses 返回按键后的时间。
`command.sent` 表示 SDK 命令下发调用的结果。
`axis` 的 0～5 表示六个关节，6 表示夹爪。
`target_rad` 的前六项为关节目标，单位为 rad。
第七项为夹爪原始角度，单位为 rad。
`before.step_unit` 为 `rad` 或 `m`。

TUI 自动启用逐拍控制记录。
记录的目标周期为 1 ms。
实际间隔由 `monotonic_ns` 和 `controller_ms` 给出。
后台日志线程批量写入二进制记录。
每条记录为 348 字节，采用小端格式。
格式字符串为 `<QQ7B7H7B38d`。
双臂按 1 kHz 采集时，数据量约为 42 MB/min。

| 顺序 | 字段 | 类型 | 单位或含义 |
| --- | --- | --- | --- |
| 1～2 | `monotonic_ns`、`controller_ms` | 2 个 uint64 | 主机单调时间 ns、控制器时间 ms |
| 3～9 | `arm_id`、`robot_state`、`requested_mode`、`actual_mode`、`lost_mask`、`soft_stop_mask`、`control_mask` | 7 个 uint8 | 状态枚举和位掩码 |
| 10～16 | `feedback_age_ms[7]` | 7 个 uint16 | 每台电机最后反馈的年龄，ms |
| 17～23 | `motor_error_code[7]` | 7 个 uint8 | 电机协议原始状态码 |
| 24～29 | `target_deg[6]` | 6 个 float64 | 控制层目标，° |
| 30～35 | `reference_position_deg[6]` | 6 个 float64 | 轨迹参考位置，° |
| 36～41 | `reference_velocity_deg_s[6]` | 6 个 float64 | 轨迹参考速度，°/s |
| 42～47 | `reference_acceleration_deg_s2[6]` | 6 个 float64 | 轨迹参考加速度，°/s² |
| 48～54 | `feedback_position_deg[7]` | 7 个 float64 | 六个关节及夹爪的反馈位置，° |
| 55～61 | `feedback_velocity_deg_s[7]` | 7 个 float64 | 六个关节及夹爪的反馈速度，°/s |

位掩码按 j1～j6、夹爪排列。
`soft_stop_mask` 和 `control_mask` 表示该拍生成的电机请求。
收到反馈前，`feedback_age_ms` 使用 65535。
已收到反馈的年龄最大记录为 65534 ms。
夹爪位置为原始角度。
原始电机状态码按对应电机协议解释。

控制记录缓冲区容量为 8192 条。
缓冲区满时，记录器丢弃最早记录并累计计数。
`metrics.jsonl` 的 `link.control_trace` 保存缓冲区统计。
其中 `captured` 为记录数，`dropped` 为丢弃数。
`pending` 为待写记录数，`error` 为采集错误。

库应用通过 `LocalLink(..., collect_trace=True)` 启用采集。
调用 `pop_control_trace()` 批量取出二进制记录。
`LocalLink.simulated()` 支持相同选项。

### 时间和性能字段

`schema_version=1` 标识数据格式。
`utc` 使用 UTC 的 ISO 8601 时间。
`monotonic_ns` 使用主机单调时钟，单位为 ns。
`metadata.json.monotonic_origin_ns` 提供会话时间原点。
控制器时间原点记录在会话事件和链路统计中。

性能窗口包含以下字段：

| 字段 | 类型或单位 | 含义 |
| --- | --- | --- |
| `source` | `host`、`can`、`tui` | 指标来源 |
| `start_ns`、`end_ns` | ns | 窗口起止时间 |
| `duration_s` | s | 窗口时长 |
| `timings` | 对象 | 各时间指标的分布 |
| `counters` | 非负整数映射 | 窗口内累计次数或字节数 |
| `peaks` | 非负整数映射 | 窗口内队列峰值 |

每个时间分布保存 `count`、`sum_ns`、`sum_squares_ns2`。
`min_ns`、`max_ns`、`mean_ns` 和 `stdev_ns` 描述实际样本。
`histogram` 使用 10 μs 的分桶宽度。
桶编号乘以桶宽得到样本上界。
编号 `1001` 保存大于 10 ms 的样本数。
`overflow_count` 保存该桶的计数。
`p50_upper_ns`、`p95_upper_ns`、`p99_upper_ns` 和
`p999_upper_ns` 保存分位数所在桶的上界。
分位数超过 10 ms 时，对应值为 `null`。
退出时，完整累计分布保存到 `summary.json.timing_totals`。
运行中的累计分布由各性能窗口合并得到。

| 指标 | 单位 | 含义 |
| --- | --- | --- |
| `host.control.period` | ns | 主机控制线程两次执行的起点间隔 |
| `host.control.duration` | ns | 单次主机控制执行耗时 |
| `host.command.period` | ns | 控制核心命令周期的起点间隔 |
| `can.can_worker.period` | ns | CAN 收发线程的循环间隔 |
| `can.can_service.period` | ns | CAN 子进程服务循环的间隔 |
| `can.armN.tx_duration` | ns | 单帧主机发送调用耗时 |
| `can.armN.tx_queue_age` | ns | 请求创建到发送开始的间隔 |
| `can.armN.feedback_age` | ns | 接收时间到 CAN 子进程交付反馈的间隔 |
| `can.armN.motorM.tx_period` | ns | 指定电机的控制帧或软停帧发送间隔 |
| `can.armN.motorM.rx_period` | ns | 指定电机的有效反馈接收间隔 |
| `tui.ui.period`、`tui.ui.duration` | ns | 界面会话更新的间隔和耗时 |
| `tui.ui.key_input.period` | ns | 两次按键接收的间隔 |
| `tui.ui.key_handler.duration` | ns | 按键处理耗时，包含交互确认等待 |
| `tui.ui.draw.period`、`tui.ui.draw.duration` | ns | 界面绘制的间隔和耗时 |
| `tui.logger.period` | ns | 指标采样线程的执行间隔 |

`N` 为臂编号 0 或 1。
`M` 为 CAN 电机 ID。
电机 ID 为 1、2、4、5、6、7、8。

### CAN 与主机采样

CAN 窗口计数按臂保存。
`tx_frames` 保存成功发送的帧数，包含退出停机帧。
`rx_frames` 保存传输层交付的接收帧数。
`tx_payload_bytes` 和 `rx_payload_bytes` 保存载荷字节数。
`action.*` 按电机动作统计发送次数。
`shutdown_tx_frames` 保存直接退出停机的发送次数。
`motorM.feedback_samples` 保存有效电机反馈数。
各窗口计数除以 `duration_s` 得到平均速率。

每个 `sample` 包含 CAN 适配器累计统计。
其中 `can.tx_frames` 统计常规发送帧。
`can.shutdown_tx_frames` 单独统计直接停机帧。
`ignored_rx`、`stale_rx`、`ipc_stale_rx` 描述反馈丢弃。
`failed` 和 `error` 保存适配器故障信息。
`performance_dropped` 保存性能窗口丢弃数。

Linux CPU 数据取自 `/proc`。
`host_cpu_percent` 表示全机 CPU 使用率，范围为 0～100%。
`per_cpu_percent` 按逻辑 CPU 保存使用率。
`cpu_percent_one_core` 以一个逻辑 CPU 为 100%。
多线程进程的此值可以超过 100%。
初次采样的使用率为 `null`。
采样覆盖 TUI 主进程、CAN 子进程及各线程。
线程数据包含 TID、角色和最近使用的 CPU 编号。
`rss_bytes` 保存进程常驻内存，单位为字节。
`governors` 保存 CPU 调频策略的数量。
`load_average` 保存 1、5、15 分钟平均负载。

正常退出时，`summary.json.status` 为 `complete`。
异常退出时，该字段为 `error` 或 `interrupted`。
运行中的检查点使用 `running`。
指标和事件文件约每秒刷新一次。
分析时先检查状态、错误和丢弃计数。

## 验证

```bash
python3 -m unittest discover -s tests -p test_log.py -v
python3 -m unittest discover -s tests -v
```

日志测试覆盖包间隔离、缓存对象及后台线程切换语言、处理器重复配置、级别过滤、词条与占位符完整性、控制事件翻译、JSON 兼容性、命令行/环境变量优先级和导入时静默行为。
