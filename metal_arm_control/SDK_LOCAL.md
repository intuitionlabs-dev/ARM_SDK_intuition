# Metal 的 SDK 本地后端

`sdk.Arm` 对外关节单位为 rad/rad/s，`gripper_m` 为 m，`gripper_raw_deg` 为原始电机角（deg）。控制核心 `Command` 使用 deg/deg/s；`MotorRequest` 使用 rad/rad/s/N·m。参数转换在 SDK 和 LocalLink 边界完成。

## 完整离线示例

```python
import time
import sdk
from metal_arm_control.sdk_adapter import LocalLink
from metal_arm_control.sdk_adapter.demo import wait_arms

with LocalLink.simulated() as link:
    arms = [sdk.Arm(i, link) for i in (0, 1)]
    for arm in arms:
        arm.connect()
    wait_arms(link, arms, lambda a: a.robot_state == 2)
    for arm in arms:
        arm.init_arm()
    wait_arms(link, arms, lambda a: a.robot_state == 4)
    for arm in arms:
        arm.start()
    wait_arms(link, arms, lambda a: a.mode == 1)
    for _ in range(100):
        for arm in arms:
            arm.send_ctrl_pos((0, -0.1, 0.1, 0, 0, 0), gripper_m=0.02)
        time.sleep(0.01)
    for arm in arms:
        arm.damping()
    wait_arms(link, arms, lambda a: a.mode == 2)
```

`wait_arms` 会逐臂保活并检查后台异常。命令入队不表示执行成功或运动到位，需要读取反馈及 `link.drain_events()`。初始化只执行六轴复位，保留夹爪入口反馈角；完成后直接落在阻尼模式，必须 `start()` 后才能位控。

## 实机对象组合

每个通道一台臂。单臂传一个 `ArmConfig`；双臂传两个。使用进程收发时必须有 `__main__` 保护：

```python
import sdk
from metal_arm_control import ArmConfig, CANChannel, ProcessCANMotorIO
from metal_arm_control.sdk_adapter import LocalLink


def main():
    channels = [CANChannel(0, 'can0', interface='socketcan')]
    with ProcessCANMotorIO(channels) as motor_io:
        with LocalLink(motor_io, configurations=[ArmConfig()]) as link:
            arm = sdk.Arm(0, link)
            arm.connect()
            print(arm.snapshot())
            # 完成反馈、姿态检查后，按上方状态流程执行初始化和控制。
            # 内层退出发送停机请求，外层等待 CAN 交付并关闭设备。


if __name__ == '__main__':
    main()
```

此示例不请求轨迹运动，但正常控制后端在 STOP 下也会发送周期使能及零增益帧。严格只读查询使用 `metal-arm-can-check`。不得在另一个原厂驱动或 MakerMods Lab 正占用同一总线时启动本控制后端。

## 生命周期与配置

`LocalLink` 接受注入的 `MotorIO`，其 `poll_feedback(now_ms)` 和 `write_motor_requests(requests)` 不能阻塞控制线程。`background=False` 时使用严格递增的毫秒时钟调用 `step()`；阻塞式 `Arm.connect()` 等待期间必须有人推进控制循环。

应用须每台臂 3 s 内提交有效命令/保活。命令超时进入阻尼；反馈丢失/故障进入停止。停止不维持重力补偿，但继续对无已知故障的电机周期使能，并发送零增益帧。标零期间的参与电机不接收周期使能；关闭链路会发送禁用电机请求。

SDK 的默认 Metal 夹爪查表与 ARX 标定文件分离。`sdk.Arm(..., calib_store=...)` 可明确提供实测标定存储。`link.board_defs` 和 `arm.board_defs` 指向 `sdk.board_defs.metal_arm`，包含该后端的限位、复位常量和限幅函数。ARX 参数独立位于 `sdk.board_defs.arx_x5`；`sdk.board_defs` 仅作为包入口，不再直接提供某台机械臂的参数。
