"""通过 sdk.Arm 完成双臂模拟流程，不连接硬件。"""

import time

import sdk

from .link import LocalLink
from .. import log

_LOG = log.get_logger("arx_x5_control.sdk_adapter.demo")


def wait_arms(link, arms, predicate, timeout=15.0):
    """等待所有臂到达指定状态，期间逐臂保活并检查控制线程是否正常。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        link.check_health()
        for arm in arms:
            arm.keepalive()
        if all(predicate(arm) for arm in arms):
            return
        time.sleep(0.05)
    states = [(arm.arm_id, arm.robot_state, arm.mode) for arm in arms]
    raise TimeoutError(log.text("sdk_demo.timeout", states=states))


def demo():
    """运行 SDK 双臂模拟，日志配置由所属包统一提供。"""
    defs = sdk.board_defs
    with LocalLink.simulated() as link:
        arms = [sdk.Arm(i, link) for i in (0, 1)]
        for arm in arms:
            arm.connect()
        wait_arms(link, arms, lambda arm: arm.robot_state == defs.STATE_WAIT_INIT)
        _LOG.info("sdk_demo.initializing")
        for arm in arms:
            arm.init_arm()
        wait_arms(link, arms, lambda arm: arm.robot_state == defs.STATE_NORMAL)
        for arm in arms:
            arm.start()
        wait_arms(link, arms, lambda arm: arm.mode == defs.MODE_RUN)
        _LOG.info("sdk_demo.controlling")
        targets = [tuple(q + (0.08 if i < 3 else 0) for i, q in enumerate(arm.joint_angles))
                   for arm in arms]
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            link.check_health()
            for arm, target in zip(arms, targets):
                arm.send_ctrl_pos_vel(target, (0.0,) * 6, gripper_raw_deg=-20.0)
            time.sleep(0.01)
        for arm in arms:
            arm.gravity()
        wait_arms(link, arms, lambda arm: arm.mode == defs.MODE_GRAVITY)
        for arm in arms:
            arm.damping()
        wait_arms(link, arms, lambda arm: arm.mode == defs.MODE_DAMPING)
        for arm in arms:
            arm.reset()
        wait_arms(link, arms, lambda arm: arm.robot_state == defs.STATE_RESETTING)
        wait_arms(link, arms, lambda arm: arm.robot_state == defs.STATE_NORMAL)
        log.write_data({
            "simulated": True,
            "sequence_complete": True,
            "arms": [{"arm_id": arm.arm_id, "state": arm.robot_state,
                      "mode": arm.mode, "joint_angles_rad": arm.joint_angles}
                     for arm in arms],
            "missed_motor_ticks": link.stats["missed_motor_ticks"],
        })
    _LOG.info("sdk_demo.complete")


def main(argv=None):
    parser = log.argument_parser("cli.sdk_description", argv)
    parser.parse_args(argv)
    try:
        demo()
    except KeyboardInterrupt:
        _LOG.info("runtime.interrupted")
        return 130
    except Exception as exc:
        _LOG.error("cli.failed", error=exc)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
