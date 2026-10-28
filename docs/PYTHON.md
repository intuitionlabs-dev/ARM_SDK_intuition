# Python 调用

应用先创建链路，再通过 `sdk.Arm` 连接、初始化、控制并读取反馈。关节角使用 rad、角速度使用 rad/s，夹爪开口使用 m。

## 创建链路

| 后端 | 创建入口 | 型号选择 |
| --- | --- | --- |
| H7 单片机 | `sdk.Link(port, robot_type=...)` | `arx_x5` 或 `metal_arm` |
| PC 本地 | `LocalLink(motor_io, configurations=...)` | 从对应机械臂包导入 `LocalLink`、`ArmConfig` 和 CAN 类 |

`sdk.Link` 直接构造 H7 串口链路。PC 本地链路是其 `LocalLink` 子类；各类的构造参数见 [API](../API.md) 和 [本地链路](PC_LOCAL.md)。

### H7 连接

```python
import sdk

link = sdk.Link("/dev/ttyACM0", robot_type="arx_x5")
```

连接 Metal 参数配置时，将 `robot_type` 设为 `"metal_arm"`，并使用匹配的 H7 固件。构造对象不打开串口，`open()` 或 `Arm.connect()` 才建立连接。

### PC 离线连接

```python
from arx_x5_control.sdk_adapter import LocalLink

link = LocalLink.simulated()
```

| 型号 | 导入 |
| --- | --- |
| ARX X5 | `from arx_x5_control.sdk_adapter import LocalLink` |
| Metal Arm | `from metal_arm_control.sdk_adapter import LocalLink` |

模拟器运行对应控制核心，用于验证调用和状态流程；实际动力学、负载和总线时序需要设备验证。

### PC CAN 连接

以下代码在设备配置部分选择机械臂包。CAN 对象和 `ArmConfig` 必须来自同一个包。两个通道对应臂 0 和臂 1；单臂只配置一个通道。

```python
from contextlib import ExitStack

from arx_x5_control import ArmConfig, CANChannel, ProcessCANMotorIO
from arx_x5_control.sdk_adapter import LocalLink


def main():
    channels = [
        CANChannel(0, "can0", interface="socketcan"),
        CANChannel(1, "can1", interface="socketcan"),
    ]
    with ExitStack() as stack:
        motor_io = stack.enter_context(ProcessCANMotorIO(channels))
        link = stack.enter_context(LocalLink(
            motor_io, configurations=[ArmConfig() for _ in channels]))
        # 将 link 传给下文的 operate(link, arm_ids=(0, 1))。
        # 退出时先关闭控制链路，再关闭 CAN 适配器。


if __name__ == "__main__":
    main()
```

Metal 使用 `metal_arm_control` 中同名的导入。SLCAN 使用 `interface="slcan"` 和 `/dev/ttyACM0` 等设备路径。CAN 配置、进程入口及只读检查见 [CAN 接口](CAN.md)。正常 PC 控制链路即使处于 STOP 也可能发送使能与零增益帧。

## 公共控制流程

下面的 `operate()` 接受已经打开的链路，与后端及型号无关。它执行初始化，短时间保持初始化后的关节目标，然后进入阻尼。运行前需确认机械臂处于复位参考附近、工作空间可运动；各型号初始化中的夹爪动作见 [行为表](ROBOTS.md#控制行为)。

```python
import time
import sdk


def wait_all(arms, predicate, timeout=20.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for arm in arms:
            arm.keepalive()
        if all(arm.feedback_ok and predicate(arm) for arm in arms):
            return
        time.sleep(0.02)
    raise TimeoutError("机械臂未在期限内达到目标状态")


def operate(link, arm_ids=(0,)):
    arms = [sdk.Arm(arm_id, link) for arm_id in arm_ids]
    try:
        for arm in arms:
            arm.connect()
        wait_all(arms, lambda a: a.robot_state == a.board_defs.STATE_WAIT_INIT)
        for arm in arms:
            arm.init_arm()
        wait_all(arms, lambda a: a.robot_state == a.board_defs.STATE_NORMAL)
        for arm in arms:
            arm.start()
        wait_all(arms, lambda a: a.mode == a.board_defs.MODE_RUN)
        targets = [arm.joint_angles for arm in arms]
        for _ in range(100):
            for arm, target in zip(arms, targets):
                arm.send_ctrl_pos(target)
            time.sleep(0.01)
    finally:
        # 在链路仍可用时提交退出模式；所有权归外层 with 管理。
        pending = []
        for arm in arms:
            try:
                if arm.feedback_ok and arm.robot_state == arm.board_defs.STATE_NORMAL:
                    arm.damping()
                    pending.append(arm)
            except sdk.ArError:
                pass
        if pending:
            wait_all(pending, lambda a: a.mode == a.board_defs.MODE_DAMPING, timeout=2.0)


# 离线完整运行；也可使用上文创建的 H7 或 CAN 链路。
from arx_x5_control.sdk_adapter import LocalLink

with LocalLink.simulated() as link:
    operate(link, arm_ids=(0, 1))
```

| 步骤 | 完成条件 |
| --- | --- |
| `connect()` | 收到握手后的第一帧反馈 |
| `init_arm()` | `STATE_NORMAL`，完成后为阻尼模式 |
| `start()` | 反馈 `MODE_RUN` 后再提交目标 |
| 持续运动或等待 | 每台臂的有效命令间隔小于 3 s；等待时使用 `keepalive()` |
| `reset()` | 先观察进入 `STATE_RESETTING`，再等待 `STATE_NORMAL`，避免把提交前的旧反馈当作完成 |
| `set_zero()` | 使用 `wait_setzero_ack()` 读取本轮结果，等待期间持续保活 |

示例中的目标取自反馈，不依赖某个型号的固定角度。轨迹规划器可使用 `arm.board_defs.JOINT_ANGLE_MIN_RAD`、`JOINT_ANGLE_MAX_RAD` 和 `RESET_POSE_RAD` 生成目标。

## 反馈、夹爪和退出

1. `arm.snapshot()` 返回同一帧的状态、模式、关节位置速度、温度和夹爪开口；连续读取多个属性可能跨帧。
2. `gripper_m` 使用链路对应的换算模型；`gripper_raw_deg` 为调试用原始电机角。两者都省略时保持当前夹爪角，两者同时提供会报错。
3. `GateRejected` 表示预检失败，`FeedbackStale` 表示预检反馈超过 100 ms 或缺失，`LinkError` 表示链路故障。`force=True` 只跳过上位机预检。
4. 共享链路由外层上下文统一关闭。退出前放稳或支撑机械臂；H7 和 PC 的关闭行为见 [后端差异](BACKENDS.md#生命周期与诊断差异)。

方法签名、单位、预检条件和异常见 [API 参考](../API.md)。
