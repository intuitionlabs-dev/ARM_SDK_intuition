# 串口接入与协议

Python 和 TUI 通过串口协议向单片机提交命令，并读取单片机反馈。
单片机执行控制状态机、轨迹、动力学和电机控制。
公共操作见 [Python 调用](PYTHON.md) 和 [TUI 调用](TUI.md)。

## 连接与臂编号

使用 `sdk.Link(robot_type=...)` 自动选择端口。
也可通过 `port` 参数指定当前系统的串口路径。
两台臂共享一个 `sdk.Link`，通过 `arm_id` 区分。
有效编号范围为 `0..ARM_NUM-1`。

`robot_type` 支持 `arx_x5` 和 `metal_arm`。
所选参数用于命令预检、限位、复位参考、夹爪换算及协议握手。

## 握手与重连

1. `Arm.connect()` 打开链路并发送协议版本帧。
2. 收到第一帧机械臂反馈后连接成功。握手要求 SDK 与单片机协议版本匹配。
3. 读写失败时，串口链路按 USB 身份寻找设备并退避重连；默认最多 10 次，初始等待 0.5 s，上限 2 s。
4. 恢复连接后重新检查状态、在线掩码和反馈年龄，再决定初始化或切换模式。重连成功不代表机械臂已可运动。

串口参数、统计和端口选择函数见 [Link API](../API.md#2-link--一条链路)。协议码和发行包版本的含义见 [版本显示](TUI.md#版本显示)。

## 帧接口

本节描述 SDK 与单片机之间的串口帧。

payload 全是小端。头部 6 字节 =
`AA 55 | type_id | datalen | arm_id | crc8`，尾 2 字节 crc16，整帧长 = `8 + datalen`。
`crc8` = CRC-8/MAXIM，盖头部 5 字节。`crc16` 用反射多项式 `0x8408`、初值 `0xFFFF`，
盖头部和 payload。

| type_id（简称 tid） | 帧名 | 发送方 | payload | 线上单位 | SDK 方法 |
|---|---|---|---|---|---|
| `0x01` | 版本帧 | 上位机、单片机 | 4 B：u32 版本码 | — | `send_version()` / `keepalive()` |
| `0x02` | 初始化帧 | 上位机 | 0 | — | `init_arm()` |
| `0x10` | 设零帧 | 上位机 | 0 | — | `set_zero()` |
| `0x11` | 设零回复帧 | 单片机 | 12 B：u8 state + u8[7] result + u32 用时 ms | — | `wait_setzero_ack()` / `last_setzero_ack` |
| `0x12` | 启动帧 | 上位机 | 0 | — | `start()`（进 run 档） |
| `0x13` | 重力补偿帧 | 上位机 | 0 | — | `gravity()` |
| `0x14` | 停止帧 | 上位机 | 0 | — | `stop()`（STOP / 软停机） |
| `0x15` | 阻尼帧 | 上位机 | 0 | — | `damping()` |
| `0x16` | 位置控制帧 | 上位机 | 80 B，依次为 f32[6] 关节角、f32 夹爪角、f32 夹爪力矩上限、f32[6] 关节速度、f32[6] 关节加速度 | deg、N·m、deg/s、deg/s² | `send_ctrl_pos()` |
| `0x18` | 一键复位帧 | 上位机 | 0 | — | `reset()` |
| `0x80` | 反馈帧 | 单片机 | 63 B：4×u8 + f32[6] + f32[6] + f32 + u8[7] | deg, deg/s, ℃ | `arm.feedback` / `snapshot()` |
| `0x81` | 链路状态帧 | 单片机 | 84 B：21×u32（见下） | — | `link.link_status` |
| 其它 | （自定义） | 上位机 | 任意 | — | `send_raw(tid, payload)` —— **通用透传入口**，无预检无换算 |

`0x80` 的 4 个 u8 依次是 `mode`（档位）、`robot_state`（状态）、`motor_lost_mask`（掉线掩码）、
`motor_inited_mask`（已初始化掩码），后面是 6 个关节角、6 个关节角速度、夹爪原始角和 7 个
温度。掩码 **bit0..5 = j1..j6、bit6 = 夹爪**。

`0x81` 是握手后的链路状态帧，链路级只有一份，不按台分。它报的是"距上一帧 `0x81`
之间，单片机收到了你多少帧"，**每个数都是窗口增量**，不是累计值。字段有 `rx_total`（解出来的
整帧数）、`rx_by_tid[11]`（按 tid 分，下标同 `protocol.LINK_TID_ORDER`，最后一格是"表外的
码"）、`crc8_err`、`crc16_err`、`drop_bytes`（没能组成任何有效帧的字节数）、`bad_arm`、
`over_len`、`ring_drop`、`gated`（被握手闸门挡下的）、`tx_fail`（单片机**发**帧失败）、
`dt_ms`。`link.link_status` 存最近一帧，`link.stats["link_status"]` 同款；没收到过是 `None`。

## 夹爪标定工具

此工具使用默认型号配置。其他配置的标定存储见 [机械臂差异](ROBOTS.md#夹爪换算)。

将环境变量 `FULL_OPEN_M` 设为完全张开的实测开口，单位 m。
运行时通过 `--full-open-m` 传入。

```bash
python3 -m sdk.calibrate_gripper --arm-id 0 --full-open-m "$FULL_OPEN_M"
```

工具会连上、显示状态。然后 `d` 切阻尼（或 `x` 切 STOP），**手动把夹爪移动到两个端点**，各按一次键
记录，回车存进 `sdk/config/gripper_calib.json`：

- `FULL_OPEN` 是**预先配置的参数**：完全张开时的开口距离，单位米，必须大于 0。代码中不固定
  量程。
- `0` 记「这里 = 0 m」，`f` 记「这里 = FULL_OPEN」，之后按这两点线性映射。
  **方向和尺度完全由这两个点决定**，工具不预设哪端是张开，装反了也不需要改代码。
- 也可以用 `r` 进 run 档后按 `←`/`→` **点动**。点动方向按原始角，不是"张开/闭合"。run 档
  整条臂都使能，不便手动移动时再用。

标定前须完成初始化，并确认夹爪反馈有效。
夹爪零点变化后须重新标定。

**标定之后怎么用**（下面三条用的是同一份文件里 `arm_id` 那一项）：

- **控制开口** `arm.send_ctrl_pos(joints, gripper_m=0.03, gripper_torque_limit_nm=0.5)`。`gripper_m=` 按标定换算成原始角。
  原始角目标可传入 `gripper_raw_deg=arm.raw_gripper_angle_deg`。**两个都不给 = 保持当前**；两个都给直接
  报错。
- **读开口**：`arm.gripper_opening`，单位米。**没标定就抛 `GripperNotCalibrated`**，不返回推测
  值。不希望被异常打断就用 `snapshot()`：同一帧里换算好的一套，`gripper_calibrated` 为假时
  `gripper_opening` 是 `None`。和标定无关的读法是 `arm.raw_gripper_angle_deg`。
- **生效时机**：第一次访问 `arm.calibration` 才读文件，读了就缓存。因此在别的进程里刚标完，
  这边要 `arm.reload_calibration()` 或重启才有米数。控制台面板按 `r` 重开链路时会自动重读
  两台。标定存在时，面板上夹爪那一行显示为米，步长也换成米档。

SDK 在 `FULL_OPEN` 和 `0` 两点之间线性换算。

标定文件是本机实测数据。仓库 `.gitignore` 忽略 `*.json`，**不要提交**。
