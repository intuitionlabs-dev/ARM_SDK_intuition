# SDK API 参考

逐个符号的参考：每个类有哪些方法、参数什么单位、受什么闸门、抛什么异常。
**概念和踩坑在 [README.md](README.md)**（状态机、闸门、保活、夹爪标定、注意事项）。

三条贯穿全文的约定：

- **单位是 SI**：关节角 rad、角速度 rad/s、夹爪开口 m、温度 ℃。只有 `protocol` 模块和
  `Feedback` 对象是**线上单位**（deg）—— 那是给协议层对拍用的，日常不碰。
- **命令方法先预检、再发帧**：拿最近一帧反馈把单片机的闸门复算一遍，不通过就抛
  `GateRejected`，**这一帧根本没发出去**。`force=True` 跳过预检（单片机的闸门照旧生效），
  也是绕过"遥控器在线"这条客户端查不到的条件的唯一办法。
- **做闸门判断用的反馈超过 100 ms 就不再用了**（`arm.FEEDBACK_STALE_S`）→ 抛 `FeedbackStale`。
  读属性的路不抛，读不到就是 `None`。

## 1. 顶层入口

`import sdk` 拿到的（**不 import pyserial**）：

| 名字 | 是什么 |
|---|---|
| `sdk.Arm` | 一台臂。**主入口**，见第 3 节 |
| `sdk.Link` | 一条链路（真串口 / 假单片机）。见第 2 节 |
| `sdk.Snapshot` | 同一帧里换算好的一致视图（第 4 节） |
| `sdk.Feedback` / `sdk.SetZeroAck` | 原始帧数据（**线上单位**） |
| `sdk.GripperCalibration` / `sdk.GripperCalibStore` | 夹爪标定模型和文件存取（第 6 节） |
| `sdk.ArError` + 7 种下级异常 | 异常层级（第 5 节） |
| `sdk.pick_default_port` / `list_links` / `describe_port` / `usb_identity` | 挑口子（第 2 节） |
| `sdk.PROTOCOL_VERSION` / `sdk.VERSION_CODE` | 协议版本串 / 版本码 |
| `sdk.protocol` / `board_defs` / `units` / `gripper` | 子模块 |

最小的完整流程（单台臂；`Arm(0)` 会自己建一条 `Link`，`connect()` 时才开口子）：

```python
import sdk, time

arm = sdk.Arm(0)
arm.connect()                                  # 开串口 → 发版本帧 → 等第一帧反馈

def wait_state(want, timeout=10.0):
    """等到 state == want。**等的期间要一直保活** —— 见下面的说明。"""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        arm.keepalive()
        if arm.wait_for(lambda a: a.robot_state == want, timeout=0.05):
            return True
    return False

wait_state(2)                                  # 待初始化
arm.init_arm()                                 # 0x02 初始化（闸门见表），跑几秒
wait_state(4)                                  # 正常运行（落点是阻尼档）
arm.start()                                    # 0x12 进 run 档，控制帧只在这个档被接受
arm.wait_for(lambda a: a.mode == sdk.board_defs.MODE_RUN, timeout=1.0)   # 等反馈确认切档

while True:
    arm.send_ctrl_pos(arm.joint_angles)        # 关节照抄反馈（不动）、夹爪不给（保持）
    time.sleep(0.01)                           # 这一帧本身就算保活
```

两处容易漏的：

- **不能干等**：单片机 500 ms 收不到**任何**有效帧就切阻尼、退回「离线等待」——自己发出的
  反馈帧不算数。所以每次 `wait_*` 之前都得有东西在发（周期控制帧，或 `keepalive()`，
  它发的是幂等的版本帧）。
- **切档要等反馈确认**：`start()` 返回只代表帧发出去了，反馈里的 `mode` 还是旧的；紧接着发
  控制帧会吃 `GateRejected`（预检拿的就是那帧旧反馈）。等 `mode` 变过来再发。

## 2. `Link` —— 一条链路

一条链路跑着**所有**臂的帧（靠帧头 `arm_id` 分），所以两台臂必须**共享一个 `Link`**；
两个进程抢同一个口子会把字节流撕碎。

```python
link = sdk.Link()                     # port=None：open() 时自动挑
link.open()
a, b = sdk.Arm(0, link), sdk.Arm(1, link)   # 或 link.arm(0) / link.arms()
```

| 成员 | 说明 |
|---|---|
| `Link(port=None, baud=921600, *, timeout=0.1, verify_on_open=True)` | `timeout` 是串口读超时（收包线程多久醒一次）；`port=None` 时 `open()` 会调 `pick_default_port()` |
| `open(port=None)` | 打开串口 + 起 RX 线程。**幂等**。第一次在这里 `import pyserial`；打不开或查出是 J-Link 探针口 → `LinkError` |
| `close()` / `reopen(port=None)` | 关 / 关掉再开（拔线、适配器重枚举之后用 `reopen`） |
| `with sdk.Link() as link:` | 进 `with` 就 open，出来就 close |
| `is_open` | 串口开着 |
| `send(tid, payload=b"", arm_id=0)` | 组一帧发出去，返回**发出的字节数**。写有锁（多线程写会撕帧）。**返回只代表字节出去了，不代表受理** |
| `channel(arm_id)` | 拿这一台的收包槽 `ArmChannel`。越界 `arm_id` → `ValueError` |
| `arm(arm_id)` / `arms()` | 拿（必要时建）`Arm` / 列出已建的 `Arm`。同一 `arm_id` 只有一个对象 |
| `inject_bytes(chunk)` | 手喂字节进解析器 + 按 `arm_id` 分流。真链路是 RX 线程调它，假链路也走这一条 |
| `wait_feedback(arm_id, after_seq=0, timeout=3.0, poll=0.005)` | 等**序号大于** `after_seq` 的一帧，返回 `Feedback`；超时 `None`。按序号等 ⇒ 没有竞态 |
| `stats` | 诊断字典，键见下 |
| `describe()` | 一行描述（给日志） |

**属性**：`port`、`baud`、`kind`（`"USB"` / `"RS485"` / `"FAKE"` / `None`）、`link_up`、
`link_err`、`board_version`（单片机回的版本码）、`version_seen`、`n_tx`、`n_rx`、`n_bad_arm`。

**`link.stats` 的键**：

| 键 | 含义 |
|---|---|
| `port` / `kind` / `is_open` | 口子、链路类型、开着没 |
| `link_up` / `link_err` | 链路还活着 / 最后一条错误（读写出错会把它置假） |
| `n_tx` | 发出去的帧数（**不是**"单片机收到了"） |
| `n_rx` | 收到的**整链路段**帧数（两台臂之和） |
| `n_bad_arm` | `arm_id` 越界、被丢掉的帧数（单片机的口径也是丢） |
| `n_err_crc8` / `n_err_crc16` / `n_drop` | CRC 错帧数 / CRC16 错帧数 / 被跳过的字节数 |
| `board_version` | 版本帧里的版本码；没收到过是 `None`（对不上 → 单片机锁死，只能复位） |

**`ArmChannel`**（`link.channel(arm_id)`）：`snapshot_feedback()` → `(Feedback, 时刻, 序号)`、
`snapshot_ack()` → `(SetZeroAck, 时刻, 序号)`、`feedback_age()`、`n_feedback`、`unknown_tids`
（认不出的 type_id 计数表）。一般用 `Arm` 上的同名封装就够。

**口子挑选**（纯函数，随时可调，用来打日志或报错）：

| 函数 | 说明 |
|---|---|
| `pick_default_port()` | 自动挑：单片机 USB(`0483:5740`) → RS485 适配器(`1a86`) → 兜底路径。**永远排除 J-Link 探针(`1366`)** |
| `list_links()` | `[(path, kind, ident), ...]`：机器上现在的候选链路 |
| `describe_port(path)` | 一行描述（带"这是哪种链路"） |
| `usb_identity(path)` | 顺着 `/sys/class/tty` 往上找 `idVendor` / `idProduct` / `product` / `manufacturer`；查不到 `None` |

## 3. `Arm` —— 一台臂

```python
Arm(arm_id=0, link=None, *, calib_store=None)
```

`arm_id` 越界 → `ValueError`；`link=None` 时自己建一条（`close()` 会把它关掉，共享链路时
`close()` 什么都不做 —— 要关公共口子就在 `link.close()` 上关）。

### 3.1 生命周期

| 方法 | 说明 |
|---|---|
| `connect(timeout=3.0, port=None)` | 必要时开链路 → 发 `0x01` → 等**这一台**的第一帧反馈。等不到抛 `LinkError`，消息里带口子、链路统计、单片机回的版本码 |
| `close()` | 关掉自己建的那条链路（共享链路时无动作） |
| `describe()` | 日志用的一行；没收到反馈也出得来 |

### 3.2 读反馈

| 成员 | 返回 |
|---|---|
| `feedback` | 最近一帧 `Feedback`（**线上单位**，deg）；没收到过 `None` |
| `feedback_age` | 最近一帧距今**秒**；没收到过 `None` |
| `feedback_ok` | 手上有 ≤ 100 ms 的反馈 |
| `link_up` | 链路本身活着（**不代表单片机在回帧** —— 那个看 `feedback_ok`） |
| `n_feedback` | 累计收到多少帧反馈 |
| `snapshot()` | 同一帧里换算好的 `Snapshot`；没收到过 `None`（第 4 节） |

单属性（方便，但**两次独立的读可能跨帧**；要严格一致用 `snapshot()`）：

| 属性 | 单位 |
|---|---|
| `joint_angles` | rad，6 个 |
| `joint_velocities` | rad/s，6 个 |
| `temperatures` | ℃，7 个（索引 0..5 = j1..j6、6 = 夹爪）。**0 ℃ 可能是"真的 0 度"也可能是"没读到"** |
| `raw_gripper_angle_deg` / `raw_gripper_angle_rad` | 夹爪**原始电机角**，与标定无关，任何时候都能用 |
| `gripper_opening` | 夹爪开口 **m**；**没标定就抛 `GripperNotCalibrated`** |
| `mode` / `mode_name` | 档位码 / 名字（`run(位控)` 等） |
| `robot_state` / `robot_state_name` | 状态码 / 名字（`正常运行` 等） |
| `motor_lost_mask` / `lost_names` | 掉线掩码 / "j3 夹爪"这样的名字串 |
| `motor_inited_mask` / `inited_names` | 已初始化掩码 / 名字串 |

### 3.3 发命令

**闸门列写的是单片机的受理条件**（SDK 的预检逐条照算，不通过发都不发）。

| 方法 | 帧 | 闸门 | 备注 |
|---|---|---|---|
| `send_version()` | `0x01` | — | 也是保活 |
| `keepalive()` | `0x01` | — | `send_version` 的别名。**SDK 不会自动发**：单片机 500 ms 收不到任意有效帧就切阻尼、退回离线等待 |
| `init_arm(*, force=False)` | `0x02` | state 是**待初始化(2)**，**或**（正常运行(4) 且 mode ∈ 停/阻尼）；**且** 7 台电机全在线；**且** 6 个关节（不含夹爪）与复位位姿偏差 ≤ 20° | 跑完落 `4` + **阻尼**，要再 `start()` 才进 run |
| `set_zero(*, force=False)` | `0x10` | state 是**待初始化(2)** | **覆盖电机的原零点**，触发前先摆位姿；只管 j1~j6；结果走 `wait_setzero_ack()` |
| `start(*, force=False)` | `0x12` | state == 正常运行(4) | 进 run 档。切档会作废上一帧控制指令 ⇒ **切过来之后要重发控制帧** |
| `gravity(*, force=False)` | `0x13` | 同 `start` | 重力补偿档 |
| `stop(*, force=False)` | `0x14` | 同 `start` | **失能、松劲**（不是"停在这不动"） |
| `damping(*, force=False)` | `0x15` | 同 `start` | 阻尼档 |
| `reset(*, force=False)` | `0x18` | state == 正常运行(4) **且** 7 台全在线（**故意不查姿态** —— 臂歪着才要收） | 走约 2 s 轨迹回复位位姿，**不做夹爪找零**，跑完停在阻尼 |
| `send_ctrl_pos(joint_rad, gripper_m=None, *, gripper_raw_deg=None, force=False)` | `0x16` | state == 4 **且** mode == run | `joint_rad` 是 6 个数（rad） |
| `send_ctrl_pos_vel(joint_rad, joint_vel_rad_s, gripper_m=None, *, gripper_raw_deg=None, force=False)` | `0x17` | 同上 | 速度前馈的起点建议对齐 `arm.joint_velocities`，否则起步处会顿一下 |
| `send_raw(tid, payload=b"")` | 任意 | **无预检、无换算** | 总逃逸口（单片机加了新包时用）。`payload` 不是 `bytes` → `TypeError` |

**夹爪目标三选一**（`gripper_m` / `gripper_raw_deg` 都不给 = **保持当前**）：

```python
arm.send_ctrl_pos(joints, gripper_m=0.03)        # 按标定换算成原始角（要先标定）
arm.send_ctrl_pos(joints, gripper_raw_deg=-141.5)  # 直接给原始电机角（不用标定）
arm.send_ctrl_pos(joints)                        # 保持当前（拿最近一帧反馈里的原始角）
```

两个都给 → `ValueError`；都没给又没收到过反馈 → `FeedbackStale`。
**软限位不在这里夹** —— 要守限位自己调 `board_defs.clamp_axis()`（第 6 节）。

**公共异常**：`FeedbackStale`（手上没有 / 太旧的反馈，没法判闸门）、`GateRejected`（预检不
通过，**帧没发出去**）、`ValueError` / `TypeError`（参数本身不对）、`LinkError`
（含 `NotConnected`：链路没 open）。

```python
try:
    arm.start()
except sdk.GateRejected as e:
    print(e)         # 消息里写清了缺哪一条，直接给人看
except sdk.ArError as e:
    print("SDK 的错：", e)
```

### 3.4 等

| 方法 | 说明 |
|---|---|
| `wait_for(pred, timeout=5.0, poll=0.02)` | 轮询等 `pred(arm)` 为真。真了 `True`、超时 `False`，**不抛** |
| `wait_feedback(timeout=1.0)` | 等一帧**新的**反馈；超时 `None` |
| `wait_setzero_ack(timeout=25.0, *, after=None)` | 等这一轮设零的 `0x11`。超时抛 `TimeoutError`。`after=None` 用 `set_zero()` 记下的序号（"这一轮"） |
| `last_setzero_ack` | 最近一帧 `0x11`；没收到过 `None` |

一轮设零实测几秒到十几秒（`0x11` 是**跑完才发**）。`result=1（未确认）`**不是故障** ——
它表示"帧发了、位置本来就在容差内"，臂本来就停在零位时必然是它。

### 3.5 夹爪标定

| 成员 | 说明 |
|---|---|
| `calibration` | 这一台的 `GripperCalibration`；**没标过是 `None`**（不抛）。第一次访问才读文件 |
| `require_calibration()` | 同上，没标过抛 `GripperNotCalibrated`（消息里带着该跑哪条命令） |
| `reload_calibration()` | 重读标定文件。**在别的进程里刚标完**，这边要调它（或重启）才有米数 |

用法和注意事项见 README 的「夹爪标定」一节。

## 4. 数据结构

`Snapshot`（`arm.snapshot()`）—— **同一帧**里换算好的一致视图：

| 字段 | 单位 |
|---|---|
| `arm_id` / `taken_at` / `age` | 台号 / `time.monotonic()` 时刻 / 距今秒数 |
| `joint_angles` / `joint_velocities` | rad / rad·s⁻¹，6 个 |
| `joint_angles_deg` / `joint_velocities_deg_s` | 原始线上值（对照用） |
| `gripper_opening` | **m**；没标定时是 `None`（快照不抛异常） |
| `gripper_raw_deg` / `gripper_calibrated` | 原始电机角 / 这台标了没 |
| `temperatures` | ℃，7 个 |
| `mode` / `mode_name` / `robot_state` / `robot_state_name` | 档位与状态（码 + 名字） |
| `motor_lost_mask` / `lost_names` / `motor_inited_mask` / `inited_names` | 掩码 + 名字串 |
| `motors_online` | 属性：7 台全在线（`0x02` / `0x18` 的闸门看这个） |

`Feedback`（`arm.feedback`，**线上单位**）：

| 字段 | 单位 |
|---|---|
| `mode` / `robot_state` / `motor_lost_mask` / `motor_inited_mask` | 码 |
| `joint_angle` / `joint_vel` | deg / deg·s⁻¹，6 个 |
| `gripper_angle` | deg（原始电机角） |
| `motor_temp` | ℃，7 个 |

`SetZeroAck`（`arm.last_setzero_ack`）：

| 字段 | 说明 |
|---|---|
| `state` | 设零状态机的状态，名字表 `board_defs.ZERO_STATE_NAME`（正常跑完是 5「完成」） |
| `result` | 7 个结果码（索引 0..5 = j1..j6、6 = 夹爪），名字表 `board_defs.ZERO_RES_NAME`：0 未走到 / 1 未确认 / 2 成功 / 3 失败 / 4 超时 / 5 跳过 |
| `elapsed_ms` | 这一轮花了多少毫秒 |

## 5. 异常

```
ArError
├── LinkError              串口打不开 / 读写出错 / 等不到反馈帧
│   └── NotConnected       链路没 open（或已 close）就要收发
├── ProtocolError          帧不符合协议（长度、payload 解不出来）
├── GateRejected           客户端预检不通过（**帧没发出去**）
├── FeedbackStale          要判闸门，但手上的反馈太旧或一帧都没有
├── GripperNotCalibrated   夹爪没标定，没法换算成米
└── CalibrationError       标定数据本身不合法（FULL_OPEN ≤ 0、两点重合、JSON 坏了）
```

`except sdk.ArError` 兜得住 SDK 自己抛的一切。串口底层那条路（pyserial 的
`SerialException`）**原样透出，不包** —— 那是环境问题。`wait_*` 的超时用
`TimeoutError`（标准库），不在这棵树里。

## 6. 常量与工具模块

### `sdk.board_defs` —— 单片机侧常量的镜像

| 名字 | 值 / 说明 |
|---|---|
| `ARM_NUM` | 2（一台单片机上的臂数；`arm_id` 必须 < 它） |
| `ARM_JOINT_NUM` / `GRIPPER` / `ARM_MOTOR_NUM` | 6 / 6（夹爪的轴号）/ 7 |
| `JOINT_NAMES` / `AXIS_NAMES` | `("j1"..."j6")` / 前者 + `("夹爪",)`，索引与掩码的位一一对应 |
| `arm_label(arm_id)` | `"臂A(arm_id=0, CAN1)"` 这种统一写法 |
| `MODE_STOP/RUN/DAMPING/GRAVITY` + `MODE_NAME` | 档位码 0/1/2/3 与名字 |
| `STATE_WAIT_HANDSHAKE` … `STATE_RESETTING` + `STATE_NAME` | 状态码 0..7 与名字 |
| `ZERO_STATE_NAME` / `ZERO_RES_NAME` | 上面 `SetZeroAck` 两张名字表 |
| `JOINT_ANGLE_MIN` / `MAX`（deg）、`_RAD` 结尾的各一份 | 关节软限位，6 个 |
| `GRIPPER_ANGLE_MIN` / `MAX`（deg）= ∓142 | 夹爪口径（找零之后零点在行程中点；**只在找零成功后生效**） |
| `GRIPPER_STROKE_DEG` | 284（整个行程） |
| `AXIS_MIN` / `AXIS_MAX` / `axis_lo(i)` / `axis_hi(i)` | 上面两套拼起来的 7 轴版本 |
| `clamp_axis(axis, value)` | → `(夹住之后的值, 是否夹了)`。**SDK 不自动调它**：单片机是静默夹的，上位机再悄悄夹一次会让期望角和实际对不上 |
| `expand_mask(mask)` | 掩码 → `"j3 夹爪"` 这样的名字串 |
| `RESET_POSE_DEG` / `RESET_POSE_RAD` | `(0, 0.5, 0.5, 0, 0, 0)` 度 |
| `ROBOT_INIT_POSE_TOL_DEG` / `_RAD` | 20° —— 就是 `init_arm()` 那条姿态闸门 |
| `KEEPALIVE_TIMEOUT_S` | 0.5：单片机的隐式保活窗口 |
| `ARM_AUTO_RESET_TIME_S` | 2：一键复位的轨迹时长（参考） |
| `pose_deviation_deg(joint_deg6)` / `all_close_deg(v6, ref6, tol)` | 姿态偏差 / 逐个比 |
| `state_name(v)` / `mode_name(v)` | 码 → 名字（不认识的码给 `"未知(n)"`） |

### `sdk.units` —— deg ↔ rad

`DEG2RAD` / `RAD2DEG`；`deg_to_rad` / `rad_to_deg`；成串的 `deg_seq_to_rad` /
`rad_seq_to_deg` / `deg_s_seq_to_rad_s` / `rad_s_seq_to_deg_s`（返回 tuple）；
`check_joint_seq(values, what)`（只查个数，**不查限位**）；`fmt_deg(values)`（日志用）。

### `sdk.protocol` —— 线上协议（低层）

日常用 `Arm` 就够；这一层是"要自己组帧 / 对拍字节 / 写别的语言实现"时用的，**单位是 deg**。

| 名字 | 说明 |
|---|---|
| `VERSION_STRING` = `"Intuition-ARX-X5-2023-v0.2"` / `VERSION_CODE` = `0xF91DD3C9` | 握手用；对不上单片机一个字节都不回 |
| `DEFAULT_ARM_ID` = 0 | |
| `TID_VERSION`(0x01) `TID_ARM_INIT`(0x02) `TID_MOTOR_SET_ZERO`(0x10) `TID_MOTOR_SET_ZERO_ACK`(0x11) `TID_ARM_START`(0x12) `TID_ARM_GRAVITY`(0x13) `TID_ARM_STOP`(0x14) `TID_ARM_DAMPING`(0x15) `TID_ARM_CTRL_POS`(0x16) `TID_ARM_CTRL_POS_VEL`(0x17) `TID_ARM_RESET`(0x18) `TID_ARM_FEEDBACK`(0x80) | 码表 |
| `FEEDBACK` / `SET_ZERO_ACK` 等长度常量 | `FB_LEN`=63、`CTRL_POS_LEN`=28、`CTRL_POS_VEL_LEN`=52、`SET_ZERO_ACK_LEN`=12 |
| `HEADER_LEN`=6 / `OVERHEAD`=8 / `ARM_ID_OFFSET`=4 / `MAX_DATA`=255 / `MAX_FRAME`=263 | 帧结构 |
| `encode(tid, payload=b"", arm_id=0)` | 组一帧（`AA 55 │ tid │ len │ arm_id │ crc8 │ payload │ crc16`） |
| `encode_version(code)` / `encode_ctrl_pos(angles_deg7)` / `encode_ctrl_pos_vel(angles_deg6, vels_deg6, gripper_deg)` / `encode_feedback(...)` | 各 payload |
| `decode_feedback(payload)` / `decode_setzero_ack(payload)` / `decode_version(payload)` | → `Feedback` / `SetZeroAck` / 版本码。布局不符抛 `ProtocolError`（`link` 里会静默丢掉） |
| `crc8(data)` / `crc16(data)` / `fnv1a32(s)` | CRC-8/MAXIM、反射多项式 `0x8408`、版本码用的 FNV-1a 32 |
| `FrameParser` | 字节流 → 帧。`feed(chunk)` 吐 `(type_id, arm_id, payload)`；粘包 / 前导噪声 / 半帧 / 坏 CRC 都能重同步。统计 `n_ok` / `n_err_crc8` / `n_err_crc16` / `n_drop` |

### `sdk.gripper` —— 夹爪两点标定

| 名字 | 说明 |
|---|---|
| `GripperCalibration(full_open_m, deg_at_zero, deg_at_full_open, recorded_at="", note="")` | 一台的标定。`opening_from_raw_deg(d)` / `raw_deg_from_opening(m)` 双向换算；`separation_deg` / `direction`（±1，只用来提示装反没有）；`validate()` 不合法抛 `CalibrationError`、记录点太近只 `warnings.warn` |
| `GripperCalibration.from_points(full_open_m, deg_at_zero, deg_at_full_open, note="")` | 建一份并当场校验 |
| `GripperCalibStore(path=None)` | JSON 文件读写（默认 `sdk/config/gripper_calib.json`）。`get(arm_id)`（没标过 `None`，那一项坏了抛）、`has`、`put(arm_id, calib)`（**原子替换**）、`require(arm_id, hint="")`、`missing_message(...)` |
| `calib_path()` / `default_calib_path()` / `ENV_CALIB_PATH` | 实际用的路径（环境变量 `ARX_GRIPPER_CALIB` 优先） |

## 7. 不接硬件跑

```bash
python3 -m sdk.selftest               # 离线自测（165 项，不需要 pyserial）
python3 -m sdk.calibrate_gripper --arm-id 0 --full-open-m 0.085   # 交互式标定
python3 -m sdk.tui --fake             # 上机控制台（curses 面板），假单片机；真机去掉 --fake
```

`sdk.tui` 就是控制台，`--fake` 等于给下面这个假单片机配了个交互界面；`--selftest` 跑 sdk
165 项 + 控制台 44 项。**面板自己不会使能电机**（`0x16` 只在板子已经在「正常运行 + run」
时才发，那要用户自己按键）。

`sdk.calibrate_gripper` 的选项：`--arm-id`（0=CAN1，1=CAN2）、`--full-open-m`（完全张开的
开口距离，米）、`--port` / `--baud`、`--calib`（标定文件路径）、`--note`、`--timeout`。

**假单片机 `sdk.sim.SimLink`** —— `Link` 的子类，走的是**真的**解析、分流、统计，发出去的帧
也按单片机的闸门被模拟状态机消化（该静默丢弃的照样丢弃），反馈帧用同一份 `encode_feedback`
组。不 import pyserial，`--fake` 全程不碰硬件。

```python
from sdk.sim import SimLink
import sdk

link = SimLink()                       # 默认两台臂，一台上电 ~0.6 s 后进「离线等待」
arm = sdk.Arm(0, link)
arm.connect()                          # 假单片机照抄握手闸门：版本码对不上就一帧不发
```

```python
SimLink(port=None, baud=921600, *, arm_ids=(0, 1), lost_mask=0, lost_arm=0,
        dt=None, speed=1.0)
```

`lost_mask` 非 0 时每 5 s 翻转 `lost_arm` 那台的掉线掩码（等价面板的 `--fake-lost 0x41`）；
`speed > 1` 让它整台时钟走快（时间缩放，运动也跟着快）。

时间常数（对拍固件用）：上电 0.6 s → 离线等待、0.8 s → 待初始化、`0x02` 3 s、`0x10` 1.2 s、
`0x18` 2 s、通信看门狗 0.5 s（**全链路一份**：给一台发的帧也把另一台喂了）、`--fake-lost`
翻转周期 5 s。

## 8. 速查

```python
import sdk, time

link = sdk.Link(); link.open()
a, b = sdk.Arm(0, link), sdk.Arm(1, link)
a.connect(); b.connect()

def wait_state(arm, want, timeout=10.0):       # 等的期间要一直保活
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        arm.keepalive()
        if arm.wait_for(lambda x: x.robot_state == want, timeout=0.05):
            return True
    return False

wait_state(a, 2)                               # 待初始化
a.init_arm()
wait_state(a, 4)                               # 正常运行（落点是阻尼档）
a.start()                                      # 进 run 档，才能发控制帧
a.wait_for(lambda x: x.mode == sdk.board_defs.MODE_RUN, timeout=1.0)   # 等反馈确认切档

s = a.snapshot()                               # 一帧里的一致视图（`__str__` 已排版好）
print(s, s.gripper_opening)                    # 未标定时 gripper_opening 是 None

a.send_ctrl_pos(s.joint_angles)                      # 关节照抄反馈、夹爪保持
a.send_ctrl_pos(s.joint_angles, gripper_m=0.02)      # 按标定给开口（要标定过）
a.send_ctrl_pos_vel(s.joint_angles, [0.0]*6)         # 位置 + 速度前馈
a.keepalive()                                        # 没在跑控制循环时别忘

a.damping(); a.stop()                          # 阻尼 / 失能
link.close()
```
