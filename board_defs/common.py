"""公共协议状态、模式、臂编号和时间参数。"""

# 拓扑

# 有效臂编号为 0..ARM_NUM-1。
ARM_NUM = 2

ARM_JOINT_NUM = 6          # 关节数
GRIPPER = ARM_JOINT_NUM    # 夹爪索引，不计入六个关节
ARM_MOTOR_NUM = 7          # 电机总数（6 关节 + 夹爪）

JOINT_NAMES = ("j1", "j2", "j3", "j4", "j5", "j6")
AXIS_NAMES = JOINT_NAMES + ("夹爪",)   # 轴索引对应掉线和初始化掩码位


def arm_label(arm_id):
    """返回包含臂编号的显示名称。"""
    tag = "AB"[arm_id] if 0 <= arm_id < len("AB") else "?"
    return "臂%s(arm_id=%d)" % (tag, arm_id)


def expand_mask(mask):
    """将电机掩码转换为空格分隔的轴名。"""
    if not mask:
        return "无"
    return " ".join(AXIS_NAMES[i] for i in range(ARM_MOTOR_NUM) if mask >> i & 1)


# 状态和模式名称。

# 协议模式码。
MODE_STOP = 0
MODE_RUN = 1
MODE_DAMPING = 2
MODE_GRAVITY = 3

MODE_NAME = {
    MODE_STOP: "stop(软停机)",
    MODE_RUN: "run(位控)",
    MODE_DAMPING: "damping(阻尼)",
    MODE_GRAVITY: "gravity(重力补偿)",
}

# 协议状态码。
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

# 0x11 使用的设零状态与结果。
ZERO_STATE_NAME = {
    0: "空闲", 1: "失能中", 2: "等待失能确认", 3: "发送设零命令", 4: "等待结果", 5: "完成", 6: "超时",
}
ZERO_STATE_DONE = 5            # 设零完成状态
ZERO_RES_NAME = {
    0: "未执行", 1: "未确认", 2: "成功", 3: "失败", 4: "超时", 5: "跳过",
}
ZERO_RES_NONE, ZERO_RES_UNCONFIRMED, ZERO_RES_OK = 0, 1, 2
ZERO_RES_FAIL, ZERO_RES_TIMEOUT, ZERO_RES_SKIP = 3, 4, 5


def state_name(v):
    return STATE_NAME.get(v, "未知(%d)" % v)


def mode_name(v):
    return MODE_NAME.get(v, "未知(%d)" % v)


# 初始化容差与公共时限。
ROBOT_INIT_POSE_TOL_DEG = 20.0
ROBOT_INIT_POSE_TOL_RAD = 20.0 * 3.141592653589793 / 180.0
KEEPALIVE_TIMEOUT_S = 3.0
ARM_AUTO_RESET_TIME_S = 2


def all_close_deg(values6, ref6, tol):
    return all(abs(a - b) <= tol for a, b in zip(values6, ref6))


# 夹爪换算

def default_gripper_store():
    """默认返回 None，使用文件两点标定；型号可提供其他换算存储。"""
    return None
