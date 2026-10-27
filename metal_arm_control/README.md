# metal_arm_control

当前发行包为 `arx-x5-control 0.6.0`，包含公共 SDK、ARX 和 Metal 后端。安装方法见 [仓库手册](../README.md)。

Metal Arm 的 PC Python 控制后端，与 `arx_x5_control` 保持相同的 Python 模块布局、公开类/方法、状态编号和 `sdk.Arm` 接入方式。每条总线为 6 个关节加 1 个夹爪，支持单臂或双臂。

控制器使用 Python 实现。控制参数、MIT 帧布局、模型和夹爪查表来自 `metal-python-ros`，见 [来源](gravity_compensation/PROVENANCE.md)。

## 安装和离线运行

在仓库根目录执行：

```bash
python3 -m pip install '.[can]'
python3 -m metal_arm_control --demo --seconds 8
python3 -m metal_arm_control.sdk_adapter.demo
metal-arm-tui --simulate
python3 -m unittest discover -s tests -v
```

Python >=3.10；控制核心及 SocketCAN 使用标准库，SLCAN 需要 pySerial。Metal 命令行入口为 `metal-arm-control`、`metal-arm-sdk-demo`、`metal-arm-can-check`、`metal-arm-tui`。

## SDK 接入

```python
import sdk
from metal_arm_control.sdk_adapter import LocalLink

with LocalLink.simulated() as link:
    arm = sdk.Arm(0, link)
    arm.connect()
    print(arm.snapshot())
```

控制接口包括 `init_arm()`、`start()`、`send_ctrl_pos()`、`send_ctrl_pos_vel()`、`gravity()`、`damping()`、`stop()`、`reset()`、`set_zero()` 和反馈属性。见 [完整流程](SDK_LOCAL.md)。

`arm.board_defs` / `link.board_defs` 指向 `sdk.board_defs.metal_arm`，控制核心也引用这份限位和复位参考。使用 H7 串口时通过 `sdk.Link(robot_type="metal_arm")` 或 `python3 -m sdk.tui --robot-type metal_arm` 选择上位机参数；实际控制由 H7 固件执行，夹爪米制换算使用文件标定。

## Metal 硬件参数

| 项目 | Metal |
|---|---|
| 电机协议 | 全部 Damiao MIT，经典 CAN，1 Mbps |
| 发送 ID | `0x01..0x07` |
| 接收 ID | `0x11..0x17`，同时检查数据中的电机 ID |
| PMAX | 全部 `6.28 rad` |
| VMAX | J1–J4 为 10；J5/J6/夹爪为 30 rad/s |
| TMAX | 30、120、30、30、20、20、20 N·m |
| 原生 Kp | 300、500、500、400、25、20、20 |
| 原生 Kd | 5、5、5、4、0.2、0.2、0.1 |
| 关节下限（deg） | -160、-180、0、-123、-85、-145 |
| 关节上限（deg） | 160、0、180、81、85、145 |
| 夹爪原始角 | 0..2.4 rad |
| 复位参考（deg） | J1～J6 全 0 |
| 动力学链 | `base_link → Link6` |

PMAX/VMAX/TMAX 是协议归一化范围，不是额定性能。CAN 反馈已是关节输出轴坐标，不能再除以原厂配置里的机械减速比。

## 控制行为

- 提供状态机、2 ms 命令更新、TD/五次轨迹、CAN 线程/进程、日志、性能统计、TUI 和 SDK。
- 电机输出使用 Metal 原生 MIT PD 增益和逐轴编码范围，主机级联 PID 设为零。
- 初始化执行六轴复位轨迹，完成后进入正常阻尼模式；夹爪保持初始化入口的反馈角。
- `reset()` 的复位角为 J1～J6 全 0°。初始化共用这组参考角，要求与参考角偏差不超过 20°，复位轨迹为 2 s；复位不写入电机零点。
- 默认夹爪米制换算使用厂家最近邻查表，要求出厂零点有效；米制命令范围为 0..0.1 m。自定义 `calib_store` 可覆盖。
- `set_zero()` 明确请求为 J1–J6 保存当前零点，不包含夹爪；需要先物理摆正。它不会自动找到机械零位。
- 电机输出目标为每轴 250 Hz；实际周期受 PC 负载和操作系统调度影响。
- `stop()` 对无已知故障的电机发送周期使能帧（每台约 75 ms，受 4 ms 调度量化）和零增益 SOFT_STOP 帧，不产生位置保持或重力补偿；标零期间暂停参与电机的周期使能，链路关闭会给全部 7 台电机发送 disable。退出前应放稳或支撑机械臂。
- 上层规划器通过关节位置或位置速度接口提交目标。

## 验证边界

离线测试覆盖帧编解码、状态机、SDK 单位、双臂隔离、故障停机、夹爪出厂零点保留、模型数值和目录/API 对齐。模拟器是理想控制对象，不能证明实机稳定性、负载能力或碰撞安全。尚未进行 Metal 真机验证。

仓库测试包含 33 项，公共 SDK 和 TUI 提供独立自测入口。测试命令和覆盖范围见 [架构与运行说明](../HANDOFF_METAL_ARM.md#验证)。
