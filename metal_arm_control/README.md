# metal_arm_control

Metal Arm 的 PC Python 控制后端，与 `arx_x5_control` 保持相同的 Python 模块布局、公开类/方法、状态编号和 `sdk.Arm` 接入方式。每条总线为 6 个关节加 1 个夹爪，支持单臂或双臂。

这是在 ARM_SDK_intuition 的控制架构内实现的 Metal 后端，不加载原厂二进制、不依赖 ROS，也不直接运行原厂 C++ 控制线程。控制参数、MIT 帧布局、模型和夹爪查表来自 `metal-python-ros` 的固定提交，见 [来源](gravity_compensation/PROVENANCE.md)。

## 安装和离线运行

在仓库根目录执行：

```bash
python3 -m pip install '.[can]'
python3 -m metal_arm_control --demo --seconds 8
python3 -m metal_arm_control.sdk_adapter.demo
metal-arm-tui --simulate
python3 -m unittest discover -s tests -v
```

Python >=3.10；控制核心及 SocketCAN 使用标准库，SLCAN 需要 pySerial。安装包继续使用 `arx-x5-control` 这个分发名称，同时包含两个后端。四个新增命令为 `metal-arm-control`、`metal-arm-sdk-demo`、`metal-arm-can-check`、`metal-arm-tui`。

## SDK 接入

```python
import sdk
from metal_arm_control.sdk_adapter import LocalLink

with LocalLink.simulated() as link:
    arm = sdk.Arm(0, link)
    arm.connect()
    print(arm.snapshot())
```

业务代码继续使用 `init_arm()`、`start()`、`send_ctrl_pos()`、`send_ctrl_pos_vel()`、`gravity()`、`damping()`、`stop()`、`reset()`、`set_zero()` 和反馈属性。仅更换后端导入和设备配置。见 [完整流程](SDK_LOCAL.md)。

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
| 动力学链 | `base_link → Link6` |

PMAX/VMAX/TMAX 是协议归一化范围，不是额定性能。CAN 反馈已是关节输出轴坐标，不能再除以原厂配置里的机械减速比。

## 与 ARX 一致的结构，以及必须区分的行为

- 保留状态机、2 ms 命令更新、TD/五次轨迹、CAN 线程/进程、日志、性能统计、TUI 和 SDK。
- 电机输出使用 Metal 原生 MIT PD 增益，主机级联 PID 设为零；不使用 ENCOS 帧或 ARX 的 DM 编码范围。
- Metal 初始化保持夹爪出厂零点，只检查新鲜反馈和范围；不会顶住限位、后退 142° 或写夹爪零点。
- `reset()` 的参考为六轴零角，这是本适配层为保持接口设置的策略，不是厂家的收纳姿态。初始化仍要求与参考角偏差不超过 20°，复位轨迹为 2 s。
- 默认夹爪米制换算使用厂家最近邻查表，要求出厂零点有效；米制命令范围为 0..0.1 m。自定义 `calib_store` 可覆盖。Metal 不读取 ARX 的本机标定文件。
- `set_zero()` 明确请求为 J1–J6 保存当前零点，不包含夹爪；需要先物理摆正。它不会自动找到机械零位。
- Python 电机输出目标为每轴 250 Hz，原厂 C++ 为 400 Hz；普通 PC 不保证硬实时。
- `stop()` 不再周期性使能；链路关闭会给全部 7 台电机发送 disable。退出前应放稳/支撑机械臂，这与原厂“析构不禁用”的行为不同。
- 这里只实现 ARX SDK 已有的关节控制接口，不额外增加原厂 `SetArmEndPose` 的 IK API。

## 验证边界

离线测试覆盖帧编解码、状态机、SDK 单位、双臂隔离、故障停机、夹爪出厂零点保留、模型数值和目录/API 对齐。模拟器是理想控制对象，不能证明实机稳定性、负载能力或碰撞安全。尚未进行 Metal 真机验证。
