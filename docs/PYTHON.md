# Python 调用

应用先创建链路，再通过 `sdk.Arm` 连接、初始化、控制并读取反馈。关节角使用 rad、角速度使用 rad/s，夹爪开口使用 m。

## 创建链路

`sdk.Link(port, robot_type=...)` 创建连接单片机的串口链路。`robot_type` 支持 `arx_x5` 和 `metal_arm`；完整构造参数见 [API](../API.md)。

### 串口连接

```python
import sdk

link = sdk.Link(robot_type="arx_x5")
```

使用 Metal 参数配置时，将 `robot_type` 设为 `"metal_arm"`。构造对象不打开串口，`open()` 或 `Arm.connect()` 才建立连接。

### 离线协议模拟

```python
from sdk.sim import SimLink

link = SimLink(robot_type="arx_x5")
```

`SimLink` 模拟单片机协议与状态机。可选择 `robot_type="metal_arm"` 验证对应的上位机参数与夹爪换算。

## 公共控制流程

下面的 `operate()` 接受已经打开的链路，两种型号共用此流程。它执行初始化，短时间保持初始化后的关节目标，然后进入阻尼。运行前需确认机械臂处于复位参考附近、工作空间可运动；初始化接口见 [行为表](ROBOTS.md#控制行为)。

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
                arm.send_ctrl_pos(target, gripper_torque_limit_nm=0.5)
            time.sleep(0.01)
    finally:
        # 关闭链路前请求阻尼模式；外层 with 负责关闭。
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

# 实机连接请改用 sdk.Link(robot_type="arx_x5")。
from sdk.sim import SimLink

with SimLink(robot_type="arx_x5") as link:
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
2. `gripper_m` 使用链路对应的换算模型；`gripper_raw_deg` 使用原始角度，单位为 deg。两者都省略时保持当前夹爪角，两者同时提供会报错。
3. `GateRejected` 表示预检失败，`FeedbackStale` 表示预检反馈超过 100 ms 或缺失，`LinkError` 表示链路故障。`force=True` 只跳过上位机预检。
4. 共享链路由外层上下文统一关闭。退出前放稳或支撑机械臂；串口关闭行为见 [链路生命周期](BACKENDS.md#生命周期)。

方法签名、单位、预检条件和异常见 [API 参考](../API.md)。
