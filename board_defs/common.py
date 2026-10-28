"""Shared wire state/mode identifiers, topology and SDK timing."""

# ==================== 拓扑 ====================

# App/inc/robot_def.h：一块板子上的机械臂台数。**arm_id 必须 < 这个数**，越界的帧被静默丢弃。
ARM_NUM = 2

ARM_JOINT_NUM = 6          # 关节数
GRIPPER = ARM_JOINT_NUM    # 6：夹爪是第 7 个电机，但不属于 6 个关节
ARM_MOTOR_NUM = 7          # 电机总数（6 关节 + 夹爪）

JOINT_NAMES = ("j1", "j2", "j3", "j4", "j5", "j6")
AXIS_NAMES = JOINT_NAMES + ("夹爪",)   # 索引 0..6，和 motor_lost_mask / motor_inited_mask 的位一一对应


def arm_label(arm_id):
    """面板 / 日志里标一台臂的统一写法。"""
    tag = "AB"[arm_id] if 0 <= arm_id < len("AB") else "?"
    return "臂%s(arm_id=%d, CAN%d)" % (tag, arm_id, arm_id + 1)


def expand_mask(mask):
    """`motor_lost_mask` / `motor_inited_mask` → 人看得懂的一串名字。"""
    if not mask:
        return "无"
    return " ".join(AXIS_NAMES[i] for i in range(ARM_MOTOR_NUM) if mask >> i & 1)


# ==================== 状态 / 模式名（App/inc/robot_def.h、robot_state.h） ====================

# Arm_mode。**0/1 的数值不能动** —— JLink 上那些老偏移依赖它。
MODE_STOP = 0
MODE_RUN = 1
MODE_DAMPING = 2
MODE_GRAVITY = 3

MODE_NAME = {
    MODE_STOP: "stop(停止/松劲)",
    MODE_RUN: "run(位控)",
    MODE_DAMPING: "damping(阻尼)",
    MODE_GRAVITY: "gravity(重力补偿)",
}

# Robot_state。**这些数的个数必须和 robot_state.h 的枚举一一对上**（0..ROBOT_STATE_NUM-1），
# 少一个后面全部错位、也不会报错 —— 自测会去数那个枚举的个数来核对。
STATE_WAIT_HANDSHAKE = 0
STATE_OFFLINE_WAIT = 1
STATE_WAIT_INIT = 2
STATE_INITING = 3
STATE_NORMAL = 4
STATE_COMM_LOST = 5
STATE_CALIBRATING = 6
STATE_RESETTING = 7

STATE_NAME = {
    STATE_WAIT_HANDSHAKE: "等待握手",
    STATE_OFFLINE_WAIT: "离线等待",
    STATE_WAIT_INIT: "待初始化",
    STATE_INITING: "初始化中",
    STATE_NORMAL: "正常运行",
    STATE_COMM_LOST: "通信离线",
    STATE_CALIBRATING: "校准零点",
    STATE_RESETTING: "复位中",
}

# Arm_zero_state / Arm_zero_result（App/inc/arm_zero.h），0x11 回复里用。
ZERO_STATE_NAME = {
    0: "空闲", 1: "失能中", 2: "等失能确认", 3: "发设零帧", 4: "等结果", 5: "完成", 6: "超时",
}
ZERO_STATE_DONE = 5            # "完成"：0x11 正常跑完就是它
ZERO_RES_NAME = {
    0: "未走到", 1: "未确认", 2: "成功", 3: "失败", 4: "超时", 5: "跳过",
}
ZERO_RES_NONE, ZERO_RES_UNCONFIRMED, ZERO_RES_OK = 0, 1, 2
ZERO_RES_FAIL, ZERO_RES_TIMEOUT, ZERO_RES_SKIP = 3, 4, 5


def state_name(v):
    return STATE_NAME.get(v, "未知(%d)" % v)


def mode_name(v):
    return MODE_NAME.get(v, "未知(%d)" % v)


# Shared SDK timing and initialization policy.
ROBOT_INIT_POSE_TOL_DEG = 20.0
ROBOT_INIT_POSE_TOL_RAD = 20.0 * 3.141592653589793 / 180.0
KEEPALIVE_TIMEOUT_S = 3.0
ARM_AUTO_RESET_TIME_S = 2


def all_close_deg(values6, ref6, tol):
    return all(abs(a - b) <= tol for a, b in zip(values6, ref6))


# ==================== 夹爪换算 ====================

def default_gripper_store():
    """这条链路默认的夹爪换算存储。`None` = 用文件两点标定（`GripperCalibStore`）。

    型号模块可以覆盖它，给出不需要标定文件的换算（如厂家查表）。
    """
    return None
