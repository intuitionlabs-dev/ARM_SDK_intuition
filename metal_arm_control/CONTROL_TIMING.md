# 控制周期

本页说明 Metal PC 控制器的目标时序，发行包版本为 `arx-x5-control 0.6.0`。H7 链路的控制周期由固件决定。

控制器使用单调毫秒调度：主循环目标 1 ms、命令/动力学更新 2 ms、反馈发布 4 ms；Metal 七台电机各 4 ms（250 Hz），按相位分散发送。

CAN 单帧间隔至少 200 us。迟到调用只执行当前拍，不补发积压轨迹；发送队列有过期上限。电机反馈新鲜度为 100 ms，CAN/进程命令流看门狗与上层 3 s 保活是不同保护层。

`PerformanceProbe`、`ControlTrace`、`RuntimeRecorder` 提供周期统计和运行记录。可在 LocalLink/CANMotorIO 构造时开启 `collect_performance`，并调用 `performance_snapshot()`。实际周期受 PC 负载和操作系统调度影响，需要在目标电脑测量；采集方法见 [日志说明](LOGGING.md)。

STOP 下每台无已知故障的电机以至少 75 ms 间隔重发使能；4 ms 电机调度下正常间隔为 76 ms。周期 SOFT_STOP 保持零增益、零力矩，标零参与电机暂停这项保活，关闭时全部禁用。
