# 控制周期

保留 ARX 的单调毫秒调度：主循环目标 1 ms、命令/动力学更新 2 ms、反馈发布 4 ms；Metal 七台电机各 4 ms（250 Hz），按相位分散发送。原厂 C++ 400 Hz 不作为本实现的保证。

CAN 单帧间隔至少 200 us。迟到调用只执行当前拍，不补发积压轨迹；发送队列有过期上限。电机反馈新鲜度为 100 ms，CAN/进程命令流看门狗与上层 3 s 保活是不同保护层。

`PerformanceProbe`、`ControlTrace`、`RuntimeRecorder` 保留相同统计接口。可在 LocalLink/CANMotorIO 构造时开启 `collect_performance`，并调用 `performance_snapshot()`。Python 和普通 Linux 不是硬实时平台，实际周期需在目标电脑测量。没有 Metal 真机时序测试结果。
