"""板侧常量的**镜像** —— 这些数的真值在固件里，这里只是抄一份给上位机用。

抄的是这几处：
  - `App/inc/robot_def.h`      ARM_NUM / ARM_*_NUM / Arm_mode / 电机索引
  - `App/inc/robot_state.h`    Robot_state（以及 ROBOT_STATE_NUM 的个数）
  - `App/inc/gimbal_cmd_def.h` 关节与夹爪的软限位（deg）
  - `App/inc/arm_auto.h`       ARM_RESET_ANGLE、ROBOT_INIT_POSE_TOL_DEG
  - `App/inc/arm_zero.h`       Arm_zero_state / Arm_zero_result

为什么要抄、而不是从帧里读：闸门预检（见 `arm.py`）必须在**发帧之前**就知道"该往哪个方向
掰"，那时候手上只有反馈帧。所以这些数是上位机的**本地副本**，固件改了这里就得跟着改 ——
`selftest.py` 里有对拍：仓库在的时候它会去 parse 上面那几个头文件，对不上就报红。
"""

import math

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


# ==================== 软限位（deg，App/inc/gimbal_cmd_def.h） ====================
#
# 这几个数配的是**关节角**（Arm_control_data.joint_angle 那一套 deg 坐标），不是电机角。
# 板子在 `Motor_Angle_Limit()`（App/gimbal_cmd.c）里做 fclip —— **而且是静默夹的**，
# 并且：
#   - 关节：只对 `motor_inited[i]` 为真的电机夹。没初始化过的电机**不受限位保护**。
#   - 夹爪：只在 `motor_inited[GRIPPER]` 为真时夹，而那个标志**只有一次成功的找零**才会置 1。
# 所以上位机不能指望板子替你兜住 —— 要守限位就自己调 `clamp_axis()`。
JOINT_ANGLE_MIN = (-115.0, 0.5, 0.5, -90.0, -90.0, -112.87)
JOINT_ANGLE_MAX = (90.0, 206.26, 171.88, 90.0, 90.0, 119.17)

# 夹爪：找零之后**零点在行程中点**，所以两端是 ∓284/2 = ∓142。
# +142 = 闭合端、−142 = 张开端（arm_auto.c 的找零流程就是这么定的）。
GRIPPER_ANGLE_MAX = 284.0 / 2.0    # 闭合
GRIPPER_ANGLE_MIN = -284.0 / 2.0   # 张开
GRIPPER_STROKE_DEG = GRIPPER_ANGLE_MAX - GRIPPER_ANGLE_MIN   # 284，整个行程

AXIS_MIN = JOINT_ANGLE_MIN + (GRIPPER_ANGLE_MIN,)
AXIS_MAX = JOINT_ANGLE_MAX + (GRIPPER_ANGLE_MAX,)


def axis_lo(axis):
    return AXIS_MIN[axis]


def axis_hi(axis):
    return AXIS_MAX[axis]


def clamp_axis(axis, value):
    """把一根轴的值夹进软限位，返回 `(夹住之后的值, 是否发生了夹)`。

    **SDK 不自动调它** —— 命令方法发出去的就是你给的值。理由：板子是静默夹的，上位机再
    悄悄夹一次，你看到的期望角和电机实际收到的角会不一致，排查时很难发现。要夹就自己夹，
    而且**最好自己知道夹了**（第二个返回值就是干这个的）。
    """
    lo, hi = axis_lo(axis), axis_hi(axis)
    if value < lo:
        return lo, True
    if value > hi:
        return hi, True
    return value, False


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


# ==================== 复位 / 初始化位姿（App/inc/arm_auto.h） ====================
#
# ARM_RESET_ANGLE = {0, JOINT_ANGLE_MIN[JOINT2], JOINT_ANGLE_MIN[JOINT3], 0, 0, 0}
# —— 板子上是拿软限位拼出来的，这里写结果。0x02（初始化）和 0x18（一键复位）都走到这个位姿。
RESET_POSE_DEG = (0.0, 0.5, 0.5, 0.0, 0.0, 0.0)

# 0x02 的姿态闸门：6 个关节（**不含夹爪**）的反馈角与 RESET_POSE_DEG 逐个比，
# 任何一个超过这个容差就**拒绝且不回包**。
ROBOT_INIT_POSE_TOL_DEG = 20.0

# 板子一个有效帧都没收到就进通信离线的时间（robot_state.h）。**隐式保活**：
# 不是"发心跳包"才算保活，任何有效帧都算；板子自己发的 0x80 **不算**。
KEEPALIVE_TIMEOUT_S = 0.5

# 一键复位的轨迹时长（arm_auto.h 的 ARM_AUTO_RESET_TIME_S），跑完停阻尼。仅作参考。
ARM_AUTO_RESET_TIME_S = 2


def pose_deviation_deg(joint_deg6):
    """6 个关节角相对复位位姿的最大偏差（deg）。0x02 的预检用它。"""
    return max(abs(a - b) for a, b in zip(joint_deg6, RESET_POSE_DEG))


def all_close_deg(values6, ref6, tol):
    return all(abs(a - b) <= tol for a, b in zip(values6, ref6))


# 单位换算是 `units.py` 的事，但软限位用 rad 表达更顺手 —— 这里顺手给一份。
JOINT_ANGLE_MIN_RAD = tuple(math.radians(v) for v in JOINT_ANGLE_MIN)
JOINT_ANGLE_MAX_RAD = tuple(math.radians(v) for v in JOINT_ANGLE_MAX)
GRIPPER_ANGLE_MIN_RAD = math.radians(GRIPPER_ANGLE_MIN)
GRIPPER_ANGLE_MAX_RAD = math.radians(GRIPPER_ANGLE_MAX)
RESET_POSE_RAD = tuple(math.radians(v) for v in RESET_POSE_DEG)
ROBOT_INIT_POSE_TOL_RAD = math.radians(ROBOT_INIT_POSE_TOL_DEG)
