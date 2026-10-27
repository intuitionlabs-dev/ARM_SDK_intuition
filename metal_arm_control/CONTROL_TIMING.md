# Metal Arm 控制周期

控制线程、命令更新、反馈发布、CAN 帧间隔及测量方法见 [公共控制周期](../docs/TIMING.md)。本页列出本型号的电机调度。

| 项目 | Metal Arm PC |
| --- | --- |
| 电机输出周期 | 全部 4 ms |
| 达妙周期使能 | 无已知故障电机各至少间隔 75 ms，正常调度约 76 ms |
| 参数定义 | [config.py](config.py) |
| 测量入口 | `metal-arm-tui --simulate --log-dir ./logs/metal_arm` |

使能与控制帧分别调度，维护流程接管参与电机。实际周期受操作系统和设备影响，模拟器用于检查统计功能，设备时序需实机采样。
