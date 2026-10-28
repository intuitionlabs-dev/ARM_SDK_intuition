# SDK API 参考

`sdk.Arm` 为 Python 程序与 TUI 提供公共控制方法。两类后端分别为 H7 单片机和 PC 本地；机械臂型号在创建链路时选择，之后的常规操作使用本页接口。

| 约定 | 内容 |
| --- | --- |
| 单位 | 关节 rad / rad/s，夹爪 m，温度 ℃；`Feedback` 与协议 payload 使用 deg / deg/s |
| 预检 | 命令使用最近反馈和 `arm.board_defs`；反馈缺失或超过 100 ms 抛 `FeedbackStale` |
| 拒绝 | 预检不满足条件抛 `GateRejected`，命令不提交 |
| `force=True` | 只跳过上位机预检，后端执行条件仍生效 |
| 结果 | 提交成功不代表执行完成，通过状态、模式与反馈判断 |

完整调用见 [Python 示例](docs/PYTHON.md)，按键与启动参数见 [TUI](docs/TUI.md)。

## 控制后端

| 类别 | 创建入口 | 配置 |
| --- | --- | --- |
| H7 单片机 | `sdk.Link(port, robot_type=...)` | 上位机参数选 `arx_x5` / `metal_arm`；固件须与机械臂匹配 |
| PC 本地 | 机械臂包中的 `LocalLink(motor_io, ...)` | 从 `arx_x5_control` 或 `metal_arm_control` 导入成套配置与 CAN 类 |

PC 的 `LocalLink` 是 `sdk.Link` 子类。构造代码见 [创建链路](docs/PYTHON.md#创建链路)，本地扩展方法见 [PC 链路参考](docs/PC_LOCAL.md)。通信、关闭、重连和诊断差异见 [后端对照](docs/BACKENDS.md#生命周期与诊断差异)，硬件相关差异见 [型号对照](docs/ROBOTS.md)。

## 1. 顶层入口

`import sdk` 导出的名字（**不 import pyserial**）：

| 名字 | 是什么 |
|---|---|
| `sdk.Arm` | 一台臂。**主入口**，见第 3 节 |
| `sdk.Link` | H7 串口链路及 PC `LocalLink` 的基类。见第 2 节 |
| `sdk.Snapshot` | 同一帧里换算好的一致视图（第 4 节） |
| `sdk.Feedback` / `sdk.SetZeroAck` / `sdk.LinkStatus` | 原始帧数据（**线上单位**） |
| `sdk.GripperCalibration` / `sdk.GripperCalibStore` | 夹爪标定模型和文件存取（第 6 节） |
| `sdk.ArError` + 7 种下级异常 | 异常层级（第 5 节） |
| `sdk.pick_default_port` / `list_links` / `describe_port` / `usb_identity` | 串口选择（第 2 节） |
| `sdk.protocol` / `board_defs` / `units` / `gripper` | 子模块 |

链路创建后，使用同一个 `sdk.Arm(arm_id, link)` 接口。单臂和双臂的初始化、保活、运动及退出示例见 [公共控制流程](docs/PYTHON.md#公共控制流程)。

## 2. `Link` —— 一条链路

本节描述 `sdk.Link` 串口后端。
PC 本地后端的同名方法及扩展参数见 [PC 链路参考](docs/PC_LOCAL.md)。

一条链路承载**所有**臂的帧（靠帧头 `arm_id` 分），因此两台臂必须**共享一个 `Link`**；
两个进程争用同一个串口会导致字节流交错。

```python
link = sdk.Link()                     # port=None：open() 时自动选择
link.open()
a, b = sdk.Arm(0, link), sdk.Arm(1, link)   # 或 link.arm(0) / link.arms()
```

| 成员 | 说明 |
|---|---|
| `Link(port=None, baud=921600, *, timeout=0.1, verify_on_open=True, auto_reconnect=True, reconnect_wait=0.5, reconnect_max_wait=2.0, reconnect_max_tries=10, robot_type=None)` | `timeout` 是串口读超时；`port=None` 时自动选口。重连参数依次控制是否重连、首次等待、退避上限、最多尝试次数（`<=0` = 不限次）。`robot_type` 见下文 |
| `board_defs` | 本链路所选的机械臂参数模块，供所有 `Arm` 和 TUI 使用 |
| `open(port=None)` | 打开串口 + 起 RX 线程。**幂等**。第一次在这里 `import pyserial`；打不开或查出是 J-Link 探针口 → `LinkError`（连上之后再掉线才走自动重连） |
| `close()` / `reopen(port=None)` | 关闭 / 关闭后重新打开（拔线、适配器重枚举之后用 `reopen`）。`reopen` 打开失败照样抛 `LinkError`，但会重新启动 RX 线程继续退避重试 |
| `with sdk.Link() as link:` | 进入 `with` 时 open，退出时 close |
| `is_open` | 串口已打开 |
| `send(tid, payload=b"", arm_id=0)` | 组一帧发出，返回**发出的字节数**。写操作有锁（多线程写会交错）。**返回值只代表字节已发出，不代表已受理** |
| `channel(arm_id)` | 取这一台的收包槽 `ArmChannel`。越界 `arm_id` → `ValueError` |
| `arm(arm_id)` / `arms()` | 获取（必要时创建）`Arm` / 列出已创建的 `Arm`。同一 `arm_id` 只有一个对象 |
| `inject_bytes(chunk)` | 手动写入字节进解析器 + 按 `arm_id` 分流。真链路由 RX 线程调用，假链路也走这一条 |
| `wait_feedback(arm_id, after_seq=0, timeout=3.0, poll=0.005)` | 等**序号大于** `after_seq` 的一帧，返回 `Feedback`；超时 `None`。按序号等待 ⇒ 没有竞态 |
| `stats` | 诊断字典，键见下 |
| `describe()` | 一行描述（用于日志） |

`robot_type` 支持 `"arx_x5"` 和 `"metal_arm"`，未知值在构造时抛出 `ValueError`。`None` 沿用类的默认配置：`sdk.Link` 默认 ARX，两个 PC 子类保留各自配置。同一 H7 链路的两台臂共用所选参数，不同链路互不影响。此选择只影响上位机预检与界面限位、参考角，不下发参数或改变 H7 固件。

```python
link = sdk.Link("/dev/ttyACM0", robot_type="metal_arm")
arm = link.arm(0)
assert arm.board_defs is sdk.board_defs.metal_arm
```

**属性**：`port`、`baud`、`kind`（`"USB"` / `"RS485"` / `"FAKE"` / `None`）、`link_up`、
`link_err`、`board_version`（单片机回的版本码）、`version_seen`、`link_status`（最近一帧 `0x81`）、
`n_tx`、`n_rx`、`n_bad_arm`、`auto_reconnect`、`open_gen`（每成功 open 一次 +1）、
`n_reconnect`（自动重连成功过的次数）、`reconnect_tries`（这一轮掉线试到第几次）、
`reconnect_err`、`reconnect_gave_up`。

**掉线自动重连**（默认开）：读或写出错就把 `link_up` 置假，RX 线程接着**退避重开**
（`reconnect_wait` 起，逐次翻倍，封顶 `reconnect_max_wait`）。试满 `reconnect_max_tries`
次还开不上就停止重试（`reconnect_gave_up=True`），之后 `reopen()` 可以再开一轮。断开期间
`is_open` 是假，`send()` 抛 `NotConnected`。设备重枚举后 tty 名字会变，所以重连**按 USB
身份重新识别串口**（`_reconnect_port()` 内部用 `match_ident()`）。`auto_reconnect=False`
时不执行重连，断开后保持断开状态。

**`link.stats` 的键**：

| 键 | 含义 |
|---|---|
| `port` / `kind` / `is_open` | 串口路径、链路类型、是否已打开 |
| `link_up` / `link_err` | 链路是否正常 / 最后一条错误（读写出错会把它置假） |
| `n_tx` | 发出的帧数（**不代表**单片机已收到） |
| `n_rx` | 收到的**整链路段**帧数（两台臂之和） |
| `n_bad_arm` | `arm_id` 越界、被丢弃的帧数（单片机侧同样丢弃） |
| `n_err_crc8` / `n_err_crc16` / `n_drop` | CRC8 错帧数 / CRC16 错帧数 / 被丢弃的字节数 |
| `board_version` | 版本帧里的版本码；没收到过是 `None`（单片机未握手时每 100 ms 主动发一帧，因此链路一通就立刻有值） |
| `link_status` | 最近一帧 `LinkStatus`（`0x81` 链路状态帧）。**握手之后才有**，每 100 ms 一帧；没收到过是 `None` |
| `auto_reconnect` / `reconnect_tries` / `reconnect_max_tries` | 自动重连开不开 / 这一轮试到第几次 / 这一轮的上限 |
| `reconnect_err` | 最近一次重连**开串口**失败的原因（一行）；没失败过是 `None` |
| `reconnect_gave_up` | 这一轮试满上限、已停止重试（等 `reopen()` 再开一轮） |
| `n_reconnect` | 自动重连成功过的次数（累计） |

**`ArmChannel`**（`link.channel(arm_id)`）：`snapshot_feedback()` → `(Feedback, 时刻, 序号)`、
`snapshot_ack()` → `(SetZeroAck, 时刻, 序号)`、`feedback_age()`、`n_feedback`、`unknown_tids`
（认不出的 type_id 计数表）。通常用 `Arm` 上的同名封装即可。

**串口选择**（纯函数，可随时调用，用于日志或报错）：

| 函数 | 说明 |
|---|---|
| `pick_default_port()` | 自动选择：单片机 USB(`0483:5740`) → RS485 适配器(`1a86`) → 回退路径。**始终排除 J-Link 探针(`1366`)** |
| `list_links()` | `[(path, kind, ident), ...]`：机器上当前的候选链路 |
| `describe_port(path)` | 一行描述（含链路类型） |
| `usb_identity(path)` | 顺着 `/sys/class/tty` 向上查找 `idVendor` / `idProduct` / `serial` / `product` / `manufacturer`；查不到返回 `None` |
| `same_usb_ident(a, b)` / `match_ident(ident, candidates)` | 两个身份是否属于同一台设备 / 在候选中匹配同一台（自动重连识别串口用）。**纯函数** |

## 3. `Arm` —— 一台臂

```text
Arm(arm_id=0, link=None, *, calib_store=None)
```

`arm_id` 越界 → `ValueError`；`link=None` 时自行创建一条（`close()` 会把它关闭）。共享链路时
`close()` 不执行操作，要关公共链路就在 `link.close()` 上关。`arm.board_defs` 在构造时取自链路；`calib_store` 显式传入时优先，否则使用链路按型号提供的存储（Metal Arm 为厂家查表），未提供时才回退到文件标定。

### 3.1 生命周期

| 方法 | 说明 |
|---|---|
| `connect(timeout=3.0, port=None)` | 必要时打开链路，提交握手并等待本臂第一帧反馈；超时抛 `LinkError`，附链路诊断信息 |
| `close()` | 关闭自己创建的那条链路（共享链路时无动作） |
| `describe()` | 日志用的一行；没有收到反馈时也能输出 |

### 3.2 读反馈

| 成员 | 返回 |
|---|---|
| `feedback` | 最近一帧 `Feedback`（**线上单位**，deg）；没收到过 `None` |
| `feedback_age` | 最近一帧距今**秒**；没收到过 `None` |
| `feedback_ok` | 手上有 ≤ 100 ms 的反馈 |
| `link_up` | 链路本身正常；机械臂反馈的新鲜度以 `feedback_ok` 为准 |
| `n_feedback` | 累计收到的反馈帧数 |
| `snapshot()` | 同一帧里换算好的 `Snapshot`；没收到过 `None`（第 4 节） |

单属性（使用方便，但**两次独立的读可能跨帧**；要严格一致用 `snapshot()`）：

| 属性 | 单位 |
|---|---|
| `joint_angles` | rad，6 个 |
| `joint_velocities` | rad/s，6 个 |
| `temperatures` | ℃，7 个（索引 0..5 = j1..j6、6 = 夹爪）。**0 ℃ 可能表示真实的 0 度，也可能表示没有数据** |
| `raw_gripper_angle_deg` / `raw_gripper_angle_rad` | 夹爪**原始电机角**，与标定无关，任何时候都能用 |
| `gripper_opening` | 夹爪开口 **m**；**没标定就抛 `GripperNotCalibrated`** |
| `mode` / `mode_name` | 档位码 / 名字（`run(位控)` 等） |
| `robot_state` / `robot_state_name` | 状态码 / 名字（`正常运行` 等） |
| `motor_lost_mask` / `lost_names` | 掉线掩码 / "j3 夹爪"这样的名字串 |
| `motor_inited_mask` / `inited_names` | 已初始化掩码 / 名字串 |

### 3.3 发命令

下表列出公共 SDK 的预检条件。H7 使用串口帧，PC 本地链路将同样的 payload 转为核心命令；帧号用于协议对照。H7 的遥控器接管和固件执行条件另见 [H7 接入](docs/H7.md)。

| 方法 | 帧 | 闸门 | 备注 |
|---|---|---|---|
| `send_version()` | `0x01` | — | 也是保活 |
| `keepalive()` | `0x01` | — | `send_version` 的别名。**SDK 不会自动发送**：后端 3 s 收不到有效命令时，已初始化的臂切回阻尼（每台各算各的；没初始化过的落到通信离线，收到下一帧才进离线等待） |
| `init_arm(*, force=False)` | `0x02` | state 是**待初始化(2)**，**或**（正常运行(4) 且 mode ∈ 停/阻尼）；**且** 7 台电机全在线；**且** 6 个关节（不含夹爪）与复位位姿偏差 ≤ 20° | 完成后落 `4` + **阻尼**，要再 `start()` 才进 run |
| `set_zero(*, force=False)` | `0x10` | state 是**待初始化(2)** | **覆盖电机的原零点**，触发前先调整位姿；只管 j1~j6；结果走 `wait_setzero_ack()` |
| `start(*, force=False)` | `0x12` | state == 正常运行(4) | 进 run 档。切档会作废上一帧控制指令 ⇒ **切档完成后必须重发控制帧** |
| `gravity(*, force=False)` | `0x13` | 同 `start` | 重力补偿档 |
| `stop(*, force=False)` | `0x14` | 同 `start` | STOP / 软停机，不保持位置或重力补偿；电机输出策略见 [行为表](docs/ROBOTS.md#控制行为) |
| `damping(*, force=False)` | `0x15` | 同 `start` | 阻尼档 |
| `reset(*, force=False)` | `0x18` | state == 正常运行(4) **且** 7 台全在线（**不检查位姿** —— 位姿偏离时才需要复位） | 走约 2 s 轨迹回复复位位姿，**不做夹爪找零**，完成后停在阻尼 |
| `send_ctrl_pos(joint_rad, gripper_m=None, *, gripper_raw_deg=None, force=False)` | `0x16` | state == 4 **且** mode == run | `joint_rad` 是 6 个数（rad） |
| `send_ctrl_pos_vel(joint_rad, joint_vel_rad_s, gripper_m=None, *, gripper_raw_deg=None, force=False)` | `0x17` | 同上 | 速度前馈的起点建议对齐 `arm.joint_velocities`，否则起步处会出现速度突变 |
| `send_raw(tid, payload=b"")` | 任意 | **无预检、无换算** | 底层 payload 入口；H7 支持透传，PC 仅支持已定义命令。`payload` 不是 `bytes` → `TypeError` |

**夹爪目标三选一**（`gripper_m` / `gripper_raw_deg` 都不给 = **保持当前**）：

```python
arm.send_ctrl_pos(joints, gripper_m=0.03)        # 使用当前换算模型：文件标定或厂家查表
arm.send_ctrl_pos(joints, gripper_raw_deg=arm.raw_gripper_angle_deg)  # 直接给原始电机角（不需要标定）
arm.send_ctrl_pos(joints)                        # 保持当前（取最近一帧反馈里的原始角）
```

两个都给 → `ValueError`；都没给又没收到过反馈 → `FeedbackStale`。
**软限位不在此处限幅**。需要限幅时由调用方调用 `arm.board_defs.clamp_axis()`（第 6 节）。

**公共异常**：`FeedbackStale`（没有反馈或反馈过旧，无法判断闸门）、`GateRejected`
（预检不通过，**帧未发出**）、`ValueError` / `TypeError`（参数本身不合法）、`LinkError`
（含 `NotConnected`：链路未 open）。

```python
try:
    arm.start()
except sdk.GateRejected as e:
    print(e)         # 消息里写清了缺哪一条，可直接展示给操作者
except sdk.ArError as e:
    print("SDK 的错：", e)
```

### 3.4 等

| 方法 | 说明 |
|---|---|
| `wait_for(pred, timeout=5.0, poll=0.02)` | 轮询等 `pred(arm)` 成立。成立返回 `True`、超时返回 `False`，**不抛异常** |
| `wait_feedback(timeout=1.0)` | 等一帧**新的**反馈；超时 `None` |
| `wait_setzero_ack(timeout=25.0, *, after=None)` | 等这一轮设零的 `0x11`。超时抛 `TimeoutError`。`after=None` 用 `set_zero()` 记下的序号（"这一轮"） |
| `last_setzero_ack` | 最近一帧 `0x11`；没收到过 `None` |

设零结果在流程结束后返回。结果码依赖电机确认机制，应用应逐项检查 ACK；等待时持续保活，流程和协议差异见 [型号对照](docs/ROBOTS.md#控制行为)。

### 3.5 夹爪标定

| 成员 | 说明 |
|---|---|
| `calibration` | 当前夹爪换算对象；具体模型由所选存储提供，见夹爪换算表。没有可用换算时为 `None`，第一次访问时加载 |
| `require_calibration()` | 同上，未标定抛 `GripperNotCalibrated`（消息里带着该执行哪条命令） |
| `reload_calibration()` | 从所选存储重新加载换算；文件标定被其他进程更新后需要调用它或重启 |

文件标定与厂家查表均提供 `opening_from_raw_deg()` 和 `raw_deg_from_opening()`。适用连接、范围和模型特有字段见 [夹爪换算表](docs/ROBOTS.md#夹爪换算)，文件管理见 [配置说明](config/README.md)。

## 4. 数据结构

`Snapshot`（`arm.snapshot()`）—— **同一帧**里换算好的一致视图：

| 字段 | 单位 |
|---|---|
| `arm_id` / `taken_at` / `age` | 臂号 / `time.monotonic()` 时刻 / 距今秒数 |
| `joint_angles` / `joint_velocities` | rad / rad·s⁻¹，6 个 |
| `joint_angles_deg` / `joint_velocities_deg_s` | 原始线上值（对照用） |
| `gripper_opening` | **m**；未标定时是 `None`（快照不抛异常） |
| `gripper_raw_deg` / `gripper_calibrated` | 原始电机角 / 这台是否已标定 |
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
| `state` | 设零状态机的状态，名字表 `arm.board_defs.ZERO_STATE_NAME`（正常完成是 5「完成」） |
| `result` | 7 个结果码（索引 0..5 = j1..j6、6 = 夹爪），名字表 `arm.board_defs.ZERO_RES_NAME`：0 未走到 / 1 未确认 / 2 成功 / 3 失败 / 4 超时 / 5 跳过 |
| `elapsed_ms` | 这一轮花了多少毫秒 |

`LinkStatus`（`link.link_status`）—— **链路级，不挂到某台臂上**：

| 字段 | 说明 |
|---|---|
| `rx_total` | 这一窗口里解出来的整帧数（**含**后面那些被丢弃的） |
| `rx_by_tid` | 12 个分类计数，下标同 `LINK_TID_ORDER`，最后一格 `LINK_TID_OTHER` 是"表外的码"（正常恒 0）。查某个码：`st.rx_by_tid[protocol.LINK_TID_ORDER.index(protocol.TID_ARM_CTRL_POS_VEL)]` |
| `crc8_err` / `crc16_err` | 校验失败被丢弃的帧数 |
| `drop_bytes` | 写入解析器但**未能组成任何有效帧**的字节数 |
| `bad_arm` / `over_len` | arm_id 越界 / payload 超长被丢弃的帧数（都正常恒 0） |
| `ring_drop` | 收包环满被丢弃的帧数（单片机处理能力不足） |
| `gated` | 被握手闸门拒绝的帧数（握手前发的非 `0x01` 帧都算） |
| `tx_fail` | 单片机**发**帧失败的次数 |
| `dt_ms` | 距上一帧 `0x81` 的实际毫秒数（标称 100） |

**每个数都是窗口增量**（窗口 = 单片机发上一帧 `0x81` 到发这一帧之间），不是累计值。两个窗口
不能直接比较大小，要按窗口看；需要累计值则自行累加。

## 5. 异常

```
ArError
├── LinkError              串口打不开 / 读写出错 / 等不到反馈帧
│   └── NotConnected       链路没 open（或已 close）就要收发
├── ProtocolError          本地后端拒绝未知命令、非法长度或 payload；
│                          串口解码器的长度错误为 `ValueError`（由 `link` 丢弃）
├── GateRejected           上位机预检不通过（**帧未发出**）
├── FeedbackStale          要判断闸门，但手上的反馈过旧或一帧都没有
├── GripperNotCalibrated   夹爪未标定，无法换算为米
└── CalibrationError       标定数据本身不合法（FULL_OPEN ≤ 0、两点重合、JSON 损坏）
```

`except sdk.ArError` 可捕获上述 SDK 异常。串口打开或写入失败封装为 `LinkError`，读取失败更新链路状态并按配置重连。参数校验还可能抛出 `ValueError`；等待 ACK 等超时使用 `TimeoutError`（标准库），不在这棵树里。`wait_for()` 超时返回 `False`。

## 6. 常量与工具模块

### `sdk.board_defs` —— 按机械臂拆分的参数包

使用 `from sdk.board_defs import arx_x5 as board_defs` 或 `metal_arm as board_defs`。已创建机械臂对象时直接使用 `arm.board_defs`。下面列出的参数和辅助函数均属于所选机械臂模块。

包入口提供 `ROBOT_TYPES = ("arx_x5", "metal_arm")` 和 `get_profile(robot_type)`；后者返回对应模块，未知名称抛出 `ValueError`。

| 名字 | 值 / 说明 |
|---|---|
| `ARM_NUM` | 2（SDK 臂编号上限；PC 还需符合实际配置的通道数） |
| `ARM_JOINT_NUM` / `GRIPPER` / `ARM_MOTOR_NUM` | 6 / 6（夹爪的轴号）/ 7 |
| `JOINT_NAMES` / `AXIS_NAMES` | `("j1"..."j6")` / 前者 + `("夹爪",)`，索引与掩码的位一一对应 |
| `arm_label(arm_id)` | `"臂A(arm_id=0, CAN1)"` 的统一写法 |
| `MODE_STOP/RUN/DAMPING/GRAVITY` + `MODE_NAME` | 档位码 0/1/2/3 与名字 |
| `STATE_WAIT_HANDSHAKE` … `STATE_RESETTING` + `STATE_NAME` | 状态码 0..7 与名字 |
| `ZERO_STATE_NAME` / `ZERO_RES_NAME` | 上面 `SetZeroAck` 两张名字表 |
| `JOINT_ANGLE_MIN` / `MAX`（deg）、`_RAD` 结尾的各一份 | 关节软限位，6 个 |
| `GRIPPER_ANGLE_MIN` / `MAX`（deg） | 所选型号的夹爪原始角范围，见 [参数对照](docs/ROBOTS.md#关节与夹爪参数) |
| `GRIPPER_STROKE_DEG` | 所选配置的 `MAX - MIN` |
| `AXIS_MIN` / `AXIS_MAX` / `axis_lo(i)` / `axis_hi(i)` | 上面两套拼起来的 7 轴版本 |
| `clamp_axis(axis, value)` | → `(限幅后的值, 是否发生限幅)`。**SDK 不自动调用它**：控制核心按其参数限幅；调用方可显式检查限幅标志 |
| `expand_mask(mask)` | 掩码 → `"j3 夹爪"` 这样的名字串 |
| `VERSION_STRING` / `VERSION_CODE` | 握手用的协议版本串与其 FNV-1a 32 版本码，各型号独立；不匹配时单片机不回复版本帧（它自身每 100 ms 报一次码） |
| `RESET_POSE_DEG` / `RESET_POSE_RAD` | 所选型号的六轴复位参考，分别提供度与弧度形式 |
| `ROBOT_INIT_POSE_TOL_DEG` / `_RAD` | 20° —— 即 `init_arm()` 的位姿闸门 |
| `KEEPALIVE_TIMEOUT_S` | 3：每台臂的有效命令保活窗口 |
| `ARM_AUTO_RESET_TIME_S` | 2：一键复位的轨迹时长（参考） |
| `pose_deviation_deg(joint_deg6)` / `all_close_deg(v6, ref6, tol)` | 位姿偏差 / 逐元素比较 |
| `default_gripper_store()` | 链路默认的夹爪换算存储：ARX X5 返回 `None`（用文件两点标定），Metal Arm 返回 `FactoryGripperStore` |
| `GRIPPER_DISTANCES_MM` / `GRIPPER_ANGLES_RAD`（Metal） | 厂家查表的开口（mm）/ 电机角（rad）两列，最近邻插值 |
| `FactoryGripperCalibration` / `FactoryGripperStore`（Metal） | 厂家查表换算对象及其存储；`opening_from_raw_deg()` / `raw_deg_from_opening()`，开口范围 0..0.1 m |
| `state_name(v)` / `mode_name(v)` | 码 → 名字（无法识别的码返回 `"未知(n)"`） |

### `sdk.units` —— deg ↔ rad

`DEG2RAD` / `RAD2DEG`；`deg_to_rad` / `rad_to_deg`；成串的 `deg_seq_to_rad` /
`rad_seq_to_deg` / `deg_s_seq_to_rad_s` / `rad_s_seq_to_deg_s`（返回 tuple）；
`check_joint_seq(values, what)`（只检查个数，**不检查限位**）；`fmt_deg(values)`（日志用）。

### `sdk.protocol` —— 线上协议（低层）

日常使用 `Arm` 即可；这一层用于自行组帧 / 对拍字节 / 实现其他语言版本，**单位是 deg**。

| 名字 | 说明 |
|---|---|
| `DEFAULT_ARM_ID` = 0 | |
| `TID_VERSION`(0x01) `TID_ARM_INIT`(0x02) `TID_MOTOR_SET_ZERO`(0x10) `TID_MOTOR_SET_ZERO_ACK`(0x11) `TID_ARM_START`(0x12) `TID_ARM_GRAVITY`(0x13) `TID_ARM_STOP`(0x14) `TID_ARM_DAMPING`(0x15) `TID_ARM_CTRL_POS`(0x16) `TID_ARM_CTRL_POS_VEL`(0x17) `TID_ARM_RESET`(0x18) `TID_ARM_FEEDBACK`(0x80) `TID_LINK_STATUS`(0x81) | 码表 |
| `FEEDBACK` / `SET_ZERO_ACK` 等长度常量 | `FB_LEN`=63、`CTRL_POS_LEN`=28、`CTRL_POS_VEL_LEN`=52、`SET_ZERO_ACK_LEN`=12、`LINK_STATUS_LEN`=88 |
| `LINK_TID_ORDER` / `LINK_TID_OTHER` / `LINK_TID_SLOT_NUM` | `0x81` 里 `rx_by_tid` 的分类顺序（11 个下行码）、"其他"那格的下标、数组长度 |
| `HEADER_LEN`=6 / `OVERHEAD`=8 / `ARM_ID_OFFSET`=4 / `MAX_DATA`=255 / `MAX_FRAME`=263 | 帧结构 |
| `encode(tid, payload=b"", arm_id=0)` | 组一帧（`AA 55 │ tid │ len │ arm_id │ crc8 │ payload │ crc16`） |
| `encode_version(code)` / `encode_ctrl_pos(angles_deg7)` / `encode_ctrl_pos_vel(angles_deg6, vels_deg6, gripper_deg)` / `encode_feedback(...)` / `encode_link_status(...)` | 各 payload |
| `decode_feedback(payload)` / `decode_setzero_ack(payload)` / `decode_version(payload)` / `decode_link_status(payload)` | → `Feedback` / `SetZeroAck` / 版本码 / `LinkStatus`。长度不符抛 `ValueError`（由 `link` 静默丢弃） |
| `crc8(data)` / `crc16(data)` / `fnv1a32(s)` | CRC-8/MAXIM、反射多项式 `0x8408`、版本码用的 FNV-1a 32 |
| `FrameParser` | 字节流 → 帧。`feed(chunk)` 吐 `(type_id, arm_id, payload)`；粘包 / 前导噪声 / 半帧 / 坏 CRC 都能重同步。统计 `n_ok` / `n_err_crc8` / `n_err_crc16` / `n_drop` |

### `sdk.gripper` —— 夹爪两点标定

这里描述文件标定接口。`robot_type=metal_arm` 的链路默认使用 `sdk.board_defs.metal_arm.FactoryGripperStore`（厂家最近邻查表，H7 与 PC 相同），`robot_type=arx_x5` 默认使用本节的 JSON 两点标定；链路通过 `board_defs.default_gripper_store()` 取默认存储。

| 名字 | 说明 |
|---|---|
| `GripperCalibration(full_open_m, deg_at_zero, deg_at_full_open, recorded_at="", note="")` | 一台的标定。`opening_from_raw_deg(d)` / `raw_deg_from_opening(m)` 双向换算；`separation_deg` / `direction`（±1，仅用于提示是否装反）；`validate()` 不合法抛 `CalibrationError`、记录点过近只 `warnings.warn` |
| `GripperCalibration.from_points(full_open_m, deg_at_zero, deg_at_full_open, note="")` | 建一份并当场校验 |
| `GripperCalibStore(path=None)` | JSON 文件读写（默认 `sdk/config/gripper_calib.json`）。`get(arm_id)`（未标定返回 `None`，该项损坏时抛异常）、`has`、`put(arm_id, calib)`（**原子替换**）、`require(arm_id, hint="")`、`missing_message(...)` |
| `calib_path()` / `default_calib_path()` / `ENV_CALIB_PATH` | 实际使用的路径（环境变量 `ARX_GRIPPER_CALIB` 优先） |

## 7. 离线运行（不接硬件）

离线测试命令见 [验证入口](HANDOFF_METAL_ARM.md#验证)，终端模拟命令见 [TUI 入口表](docs/TUI.md#启动入口)。

| 模拟对象 | 用途 |
| --- | --- |
| `sdk.sim.SimLink` | H7 协议解析、分流、握手和模拟状态机 |
| 对应型号的 `LocalLink.simulated()` | PC 控制核心、MotorIO 和型号行为 |

`SimLink` 是 `Link` 子类，不访问串口。用法与公共 `sdk.Arm` 相同：

```python
import sdk
from sdk.sim import SimLink

with SimLink(robot_type="arx_x5") as link:
    arm = sdk.Arm(0, link)
    arm.connect()
    print(arm.snapshot())
```

```text
SimLink(port=None, baud=921600, *, arm_ids=(0, 1), lost_mask=0, lost_arm=0,
        dt=None, speed=1.0, robot_type=None)
```

`lost_mask` 非 0 时每 5 s 翻转 `lost_arm` 那台的掉线掩码（等价于面板的 `--fake-lost 0x41`）。
`speed > 1` 使其整体时钟加速（时间缩放，运动同步加快）。
`robot_type` 与 `Link` 相同，假板子的复位目标跟随所选参数；它仍是 H7 协议状态机模拟器。验证 Metal PC 初始化和 STOP 行为应使用 `metal_arm_control.sdk_adapter.LocalLink.simulated()`。

时间常数（对拍固件用）：上电 0.6 s → 离线等待、0.8 s → 待初始化、`0x02` 3 s、`0x10` 1.2 s、
`0x18` 2 s、通信看门狗 3 s（**每台一份**：发往某台的帧只为该台保活，另一台照常超时）、
`--fake-lost` 翻转周期 5 s。

## 8. 操作速查

| 任务 | 公共调用 |
| --- | --- |
| 创建设备 | [链路创建](docs/PYTHON.md#创建链路) |
| 创建臂对象 | `sdk.Arm(arm_id, link)` |
| 连接与初始化 | `connect()` → 等待待初始化 → `init_arm()` → 等待正常运行 |
| 开始运动 | `start()` → 等待 RUN → `send_ctrl_pos()` / `send_ctrl_pos_vel()` |
| 读取一致反馈 | `snapshot()` |
| 等待时保活 | 为每台臂周期调用 `keepalive()` |
| 复位 | `reset()`，观察复位中再等待正常运行 |
| 模式 | `damping()` / `gravity()` / `stop()` |
| 退出 | [生命周期与诊断差异](docs/BACKENDS.md#生命周期与诊断差异) |
