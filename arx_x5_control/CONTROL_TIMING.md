# 控制周期稳定性

适用包版本：`arx-x5-control 0.5.0`。

## 环境和测量方法

| 项目 | 配置 |
| --- | --- |
| 主机 | nuc3，16 个 CPU 调频策略 |
| 内核 | `7.0.0-34-realtime` |
| Python | 3.12.3 |
| 调度策略 | 默认 SCHED_OTHER，保持系统现有 CPU 亲和性 |
| CPU 调频策略 | 全部为 `performance` |
| CPU 能效偏好 | 全部为 `performance` |
| 系统电源配置 | `performance` |
| CAN 接口 | 两只 CANable2，直接 SLCAN，1 Mbit/s |
| 控制后端 | `LocalLink` 和 `ProcessCANMotorIO` |
| 后台服务 | 摄像头、采集服务保持运行 |
| 预热和采样 | 每组预热 2 s，采样 60 s |

停止模式组运行真实双臂的 STOP 会话。
负载组同时运行一个双臂 RUN 影子控制器。
影子控制器包含轨迹、逆动力学、PID 和模拟反馈。
影子控制器的输出仅交付模拟电机。
真实电机接口只接受 SOFT_STOP、ENABLE 和 DISABLE。
SDK 会话每 5 ms 执行一次保活检查。

使用 `time.monotonic_ns()` 记录时间。
主循环周期为相邻 `LocalLink._step_once` 起点的间隔。
命令周期取发生命令更新的主循环起点间隔。
迭代耗时包含控制核心、进程通信和影子控制计算。
CAN 周期取同一电机的相邻周期帧发送调用起点间隔。
CAN 周期统计排除独立的使能保活帧。
这些时间戳位于主机传输层。
测量结果包含采样探针的开销。

## 主循环和命令更新

P99 表示 99% 的采样间隔不超过该值。
P99.9 的定义相同。
漏拍为控制器对应计数器在采样窗口内的增量。

| 场景 | 目标周期 | 平均周期 | P99 | P99.9 | 最大间隔 | 漏拍数 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 停止模式主循环 | 1 ms | 1.000000 ms | 1.055 ms | 1.112 ms | 1.795 ms | 0 |
| 停止模式命令更新 | 2 ms | 1.999996 ms | 2.050 ms | 2.118 ms | 2.817 ms | 0 |
| 双臂影子负载主循环 | 1 ms | 1.000100 ms | 1.055 ms | 1.142 ms | 3.488 ms | 6 |
| 双臂影子负载命令更新 | 2 ms | 2.000167 ms | 2.053 ms | 2.134 ms | 4.496 ms | 2 |

| 单次迭代耗时 | 停止模式 | 双臂影子负载 |
| --- | ---: | ---: |
| 平均值 | 0.185 ms | 0.434 ms |
| P99 | 0.384 ms | 0.785 ms |
| P99.9 | 0.442 ms | 0.947 ms |
| 最大值 | 0.838 ms | 2.962 ms |
| 超过 1 ms 的迭代数 | 0 | 25 |

负载组平均周期接近目标。
负载组存在偶发长尾延迟和漏拍。
最长主循环间隔的前一迭代耗时为 0.333 ms。
该相邻间隔中有 3.154 ms 位于迭代外。
因此需同时关注循环耗时和线程等待时间。

## CAN 发送周期

每条总线的平均发送速率约为 2013.3 帧/s。
其中包括周期帧和独立的使能保活帧。
每条总线采样约 12.08 万帧。
发送调用起点的最小间隔为 0.201 ms。

j1 的目标发送周期为 2 ms。
其余六台电机的目标发送周期为 4 ms。
表中“其余电机 P99”取六台电机的最大值。

| 场景 | 臂 | j1 P99 | j1 最大间隔 | 其余电机 P99 | 其余电机最大间隔 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 停止模式 | 0 | 3.250 ms | 5.644 ms | 4.671 ms | 7.051 ms |
| 停止模式 | 1 | 3.266 ms | 5.795 ms | 4.677 ms | 7.164 ms |
| 双臂影子负载 | 0 | 3.241 ms | 6.275 ms | 4.782 ms | 7.638 ms |
| 双臂影子负载 | 1 | 3.254 ms | 6.344 ms | 4.797 ms | 7.587 ms |

CAN 发送间隔存在抖动。
平均帧率与尾部间隔需要分别评估。
两组的 CAN 故障状态均为 False。
两组的电机丢失观测数、旧反馈数和 SDK 拒绝命令数均为 0。
控制核心收到的反馈年龄最大为 6 ms。
最高电机温度为 31℃。
关节位置波动不超过 0.022°。
实机全程保持 STOP，结束时发送 14 帧停机请求。

## CPU 配置

持久化服务为 `/etc/systemd/system/arx-cpu-performance.service`。
该服务在 `power-profiles-daemon.service` 之后启动。
服务设置系统电源配置和 CPU 调频策略。
服务状态为 active，开机状态为 enabled。
采样开始和结束时，16 个调频策略均为 performance。

## 运行日志采集

日志采集验证在 nuc3 上持续 20 s。
实机双臂保持 STOP。
控制线程附加双臂模拟 RUN 计算负载。
日志格式见 [运行日志](LOGGING.md#tui-运行日志)。

| 指标 | 测量值 |
| --- | ---: |
| 主机控制周期 P99 上界 | 1.08 ms |
| 主机控制周期最大值 | 1.907 ms |
| 命令周期 P99 上界 | 2.08 ms |
| 命令周期最大值 | 2.807 ms |
| 核心电机周期漏步计数 | 7 |
| 核心命令周期漏步计数 | 0 |
| 日志线程 CPU 峰值，单核为 100% | 0.98% |
| TUI 主进程 CPU 峰值，含模拟负载 | 60.26% |
| CAN 子进程 CPU 峰值 | 56.61% |
| 日志体积 | 约 1.66 MB |

14 个电机的离线掩码均为 0。
关节反馈波动不超过 0.022°。
CAN 适配器故障状态为 False。
CAN 统计包含 3 帧旧反馈。
IPC 旧反馈计数为 0。
日志记录和性能窗口的丢弃计数均为 0。
退出时，CAN 线程发送 14 帧直接停机请求。

验证日志位于
`build/run-logging/nuc3/20261017T123317Z-945186-daaba9/`。
nuc3 保留相同会话目录：
`~/.local/state/arx-x5-control/runs/20261017T123317Z-945186-daaba9/`。

## 复现和数据

运行周期检测：

```bash
python3 tests/hardware/timing.py \
  --channel /dev/ttyACM0 --channel /dev/ttyACM1 \
  --seconds 60 --output build/timing
```

实际部署使用 `/dev/serial/by-id/` 下的稳定设备路径。
`--scenario baseline` 运行停止模式组。
`--scenario dynamics` 运行双臂影子负载组。
默认依次运行两组。
`--simulate` 使用模拟电机验证检测脚本。

检测脚本位于 [timing.py](https://github.com/Crystal-1438/ARX_X5_DMH7/blob/feat/pc-python-arm-control/tests/hardware/timing.py)。
本机测量数据位于 `build/timing/nuc3/`。
每组保留 `report.json`、`control.bin`、`can-0.bin`、`can-1.bin` 和 `feedback.json`。
`summary.csv` 保存主循环与命令周期的汇总值。
`timing.png` 和 `timing.pdf` 保存周期分布图。
nuc3 原始数据位于 `/home/intuition-nuc3/arx-timing-test/performance/`。
