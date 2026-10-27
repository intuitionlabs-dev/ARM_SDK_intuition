# 上位机 SDK 手册

SDK 提供 `sdk.Arm` 机械臂控制接口。
关节角使用 rad，关节速度使用 rad/s。
夹爪开口使用 m，需要夹爪标定。

## Metal Arm 后端

新增 [`metal_arm_control`](metal_arm_control/README.md)，与 `arx_x5_control` 保持相同模块布局及 `sdk.Arm` 接口。Metal 使用独立的 CAN 编码、限位、模型和夹爪出厂查表。完整接入见 [Metal SDK 流程](metal_arm_control/SDK_LOCAL.md)。原有 ARX/H7 默认值不变；两个后端可在同一 Python 进程中导入。

接手维护见 [Metal Arm SDK 交接文档](HANDOFF_METAL_ARM.md)，包含改动边界、验证记录和待真机确认事项。

```bash
python3 -m metal_arm_control --demo --seconds 8
metal-arm-tui --simulate
```

## 每种机械臂的参数

`sdk.board_defs` 现在是按机械臂拆分的包，不再提供默认的全局限位：

```python
from sdk.board_defs import arx_x5, metal_arm

# 已创建 arm 时，使用它所属链路的配置。
defs = arm.board_defs
angle_deg, clipped = defs.clamp_axis(1, -30.0)
```

ARX/H7 和 ARX PC 后端使用 `sdk.board_defs.arx_x5`；Metal PC 后端使用 `sdk.board_defs.metal_arm`。控制核心和 SDK 引用同一份限位与复位角。旧的 `sdk.board_defs.MODE_RUN` 等访问请改为 `arm.board_defs.MODE_RUN`，或显式选择机械臂模块。

Metal 的非零复位角待用户从 TUI 读取后提供；目前仍保留旧的六轴零角，不能把它当成已确认的厂家复位姿态。

## 选择控制后端

| 连接方式 | 链路对象 | 电机控制执行位置 |
| --- | --- | --- |
| PC → H7 → CAN → 机械臂 | `sdk.Link` | H7 控制板 |
| PC → USB-CAN → 机械臂 | `arx_x5_control.sdk_adapter.LocalLink` | PC 控制核心 |

上层将链路对象传给 `sdk.Arm(arm_id, link)`。
使用 PC 本地后端时，安装 `arx-x5-control`。
安装包包含 SDK、控制核心和 `arx_x5_control.usb2can`。

双臂实机接入见 [完整例程](arx_x5_control/SDK_LOCAL.md#双臂实机接入例程)。
例程包含初始化、保活、运动、复位及退出。
设备配置见 [CAN 接口](arx_x5_control/CAN_IO.md)。
传输扩展见 [USB-CAN 接口](arx_x5_control/usb2can/README.md)。

第 1 节说明两种后端的硬件配置。
第 3、10、12 节描述 H7 串口协议和收发。
PC 本地后端的接入方式见第 11 节。

命令发送成功后，通过反馈确认执行结果。
PC 本地后端还提供 `link.drain_events()` 控制事件。
命令执行条件见第 8 节。

## 1. 硬件构成

### H7 控制板后端

```text
PC 上层程序 → sdk.Arm → sdk.Link
                           │ USB CDC 或 USB 转 RS485
                           ▼
                 CtrBoard-H7（STM32H723）
                           ├─ CAN1 → 机械臂 A
                           └─ CAN2 → 机械臂 B
```

H7 执行状态机、轨迹、动力学和电机控制。
PC 通过一条链路发送双臂目标并接收反馈。
两台臂共享一个 `sdk.Link`。

固件中的 `ROBOT_LINK` 选择上位机接口：

| 固件配置 | PC 接线 |
| --- | --- |
| `ROBOT_LINK_USB` | PC USB 口连接 H7 板载 USB CDC 口 |
| `ROBOT_LINK_UART` | PC USB 转 RS485 适配器连接 H7 的 USART3 RS485 端子 |

接线须与设备运行的固件配置一致。
H7 将 `arm_id=0` 映射到 CAN1。
H7 将 `arm_id=1` 映射到 CAN2。

### PC 本地控制后端

```text
PC 上层程序 → sdk.Arm → LocalLink → arx_x5_control
                                        │
                                ProcessCANMotorIO
                                        │
                            arx_x5_control.usb2can
                                        ├─ USB-CAN A → CAN → 机械臂 A
                                        └─ USB-CAN B → CAN → 机械臂 B
```

PC 控制核心执行状态机、轨迹、动力学和电机控制。
USB-CAN 适配器负责 CAN 帧收发。
两台臂使用两条独立 CAN 通道。
两台臂共享一个 `LocalLink`。
`CANChannel.arm_id` 将臂编号映射到设备通道。
例如，`0` 对应 `can0`，`1` 对应 `can1`。

支持的设备接口包括 SocketCAN 和 SLCAN。
通道打开时，传输后端自动配置通信参数。
设备驱动、权限和配置见 [CAN 接口](arx_x5_control/CAN_IO.md)。

### 电机与总线

每台臂包含六个关节电机和一个夹爪电机。
关节按根部到末端编号为 j1～j6。
每条 CAN 总线连接一台臂。
CAN 速率为 1 Mbps，使用经典 CAN 标准帧。
同一通道由一个控制程序发送命令。

## 2. 术语表

**粗体**是后面正文里最常出现的写法。

| 词 | 一句话解释 |
|---|---|
| **臂** | 一台六轴机械臂。臂 A 的 `arm_id=0`，臂 B 的 `arm_id=1`。 |
| **关节** | 臂的 6 个转动轴，编号 j1…j6；SDK 里轴号 `0..5`。 |
| **夹爪** | 装在臂末端的抓手，自己一台电机；SDK 里轴号 `6`。 |
| **使能 / 失能** | 使能 = 电机加力（能按指令运动、能承重）；失能 = 电机不加力（臂在重力下下落）。 |
| **档位**（mode） | 整台臂的工作方式：**run**（位置控制，跟随下发指令）／**阻尼**（可被外力推动，阻力随速度增大）／**重力补偿**（不下发指令也保持托举）／**停**（等价于失能）。 |
| **状态**（robot_state） | 控制核心的流程状态，共 8 个：等待握手 / 离线等待 / 待初始化 / 初始化中 / 正常运行 / 通信离线 / 校准零点（设零期间）/ 复位中。见第 4 节的状态转移表。 |
| **初始化**（`0x02`） | 控制核心将臂收回参考位姿并完成夹爪找零；完成后进入「正常运行」。闸门见第 8 节。 |
| **复位位姿** | 臂的参考位姿：6 个关节 `(0, 0.5, 0.5, 0, 0, 0)` 度。与「零点」是两个独立的量。 |
| **零点** | 角度 0 的位置。出厂时写入电机，`0x10` 可以覆盖它。 |
| **找零** | **夹爪专用**的动作，在初始化中执行：推到一侧限位 → 退回半个行程 → 把中点记为夹爪的 0 位。 |
| **标定** | 夹爪的电机角度与实际开口（米）之间的实测换算，见第 9 节。与设零、找零是两个独立的概念。 |
| **命令帧 / 反馈帧** | H7 串口协议中的命令数据帧和反馈数据帧。 |
| **type_id** | 帧类型编号，例如 `0x02` = 初始化、`0x16` = 位置控制。 |
| **payload** | 帧里除头尾之外的有效数据（也可能是空的）。 |
| **arm_id** | 这一帧的目标臂（`0`/`1`）。 |
| **链路** | 承载 SDK 命令和反馈的对象。两台臂共享一个 `Link` 或 `LocalLink`。 |
| **闸门** | 命令的执行条件。SDK 预检和控制核心分别检查，见第 8 节。 |
| **保活** | 每台臂须在 3 s 内收到有效命令。应用周期发送目标或调用 `keepalive()`。 |
| **原始角 / 开口** | 夹爪电机的角度（线上单位是度）/ 两爪之间的实际距离（米，需要标定）。 |
| **SI** | SDK 对外一律用国际单位：rad、rad/s、m、℃。线上是度，由 SDK 换算（第 7 节）。 |
| **软限位** | 控制核心对已初始化电机执行的角度限幅。见第 13 节第 6 条。 |
| **遥控器** | H7 后端的手持控制设备。通过板载接收机接管档位，其在线状态未包含在 SDK 反馈中。 |

## 3. H7 串口协议交互

**两种帧，一条链路。** 上位机发命令帧，单片机回反馈帧。

- **握手**：连上以后先发一帧版本帧（`0x01`，带 4 字节版本码）。单片机**只认**
  `Intuition-ARX-X5-2023-v0.3` 这一个版本，版本码 `0xF81DD236`。版本码不匹配的输入它
  **不回**：那一帧直接丢弃，状态不变。握手完成之前它**每 100 ms 主动发一帧 `0x01`**，
  所以上位机可以直接从它报的版本码判断是上位机侧不匹配。把版本串改对再发一帧即可握手，
  不需要复位单片机。不要手工修改这个版本码；握手成功表示双方使用相同的协议版本码。
- **收到第一帧反馈（`0x80`）才算链路打通**：握手成功之前单片机不发反馈帧，`0x01` 是唯一的
  例外（见上一条）。`Arm.connect()` 等的就是 `0x80`。
- **帧结构**（由 SDK 处理，这里列出是为了便于解读日志）：头部 6 字节是
  `AA 55 │ type_id │ payload 长度 │ arm_id │ crc8`，然后是 payload，最后 2 字节 crc16 收尾。
  整帧长 = `8 + payload 长度`。字节流会粘连，也会从中间断开；重同步由 CRC 校验完成，这部分
  由 `sdk.protocol.FrameParser` 处理。
- **速率**：单片机正常工作后**每秒回 250 帧反馈**，每台臂各 250。反馈的使用方式见第 12 节。

具体每一帧的编号、payload 布局、线上单位，见第 10 节的表。

## 4. 一次会话的生命周期

两种后端使用相同的机械臂状态编号。
H7 在固件中执行状态机。
PC 本地后端在 `ControlSystem` 中执行状态机。
表中的接收帧行为适用于 H7 串口后端。
本地后端将 SDK 命令转换为控制核心命令。

状态一共 8 个，编号就是反馈帧里的 `robot_state` 字段（名字表见 `board_defs.STATE_NAME`）。
一次会话只会沿着下面这些转移走：

| 起点 | 触发 | 终点 | 同时发生 |
|---|---|---|---|
| `0 等待握手` | 收到版本码相符的版本帧 `0x01` | `1 离线等待` | 单片机从这一刻起才开始发反馈帧 |
| `0 等待握手` | 收到版本码不符的 `0x01` | 停在 `0` | **不回这一帧**（丢弃，状态不变；板子自身的 10 Hz `0x01` 仍持续发送）；改对版本码再发即可握手 |
| `1 离线等待` | 7 台电机全部上线 | `2 待初始化` | 档位落 `停` |
| `2 待初始化` | 收到初始化 `0x02` | `3 初始化中` | 档位落 `run` |
| `4 正常运行`（档位在 停/阻尼） | 收到初始化 `0x02` | `3 初始化中` | 直接重做初始化；档位在 `run`/`gravity` 时这个帧被丢掉 |
| `3 初始化中` | 初始化完成（夹爪同时完成找零） | `4 正常运行` | 档位落**阻尼**（要再发 `0x12` 才进 `run`） |
| `4 正常运行` | 收到复位 `0x18`（且 7 台全在线） | `7 复位中` | 走一段约 2 s 的轨迹 |
| `7 复位中` | 轨迹执行完毕 | `4 正常运行` | 档位落阻尼；关节停在复位位姿 |
| `2 待初始化` | 收到设零 `0x10` | `6 校准零点` | 只管 j1~j6，不含夹爪 |
| `6 校准零点` | 设零完成 | `2 待初始化` | 几秒后补发一帧 `0x11` 反馈帧 |
| 收到过有效帧之后的**任意状态**（`0` 和 `5` 除外） | 3 s 内没再收到任何有效帧 | `4 正常运行`（留在原地）/ `5 通信离线` | 档位一律切阻尼；**总模式取决于初始化是否完成**：`4` 里的留在 `4`，其余落 `5` |
| `5 通信离线` | 收到任意有效帧 | `1 离线等待` | **要重新初始化**，不会自行回到 `4` |

三个容易出错的环节：

- **只有收到第一帧反馈才算握手成功**：单片机握手前不发帧，`connect()` 等的就是它。
- 上电后的常规路径是 `0 → 1 → 2`。**要到 `2` 才能发初始化**，闸门见第 8 节。
- 链路中断（拔线或串口异常）后的行为：**初始化完成过的臂**停在 `4` + 阻尼，链路恢复后补一帧
  `0x12`（`start()`）即恢复运行。**未完成过的**落到 `5 → 1`，链路恢复后按 `2` 那一步重新执行。
  H7 串口后端提供自动重连，见第 12 节。
  PC 本地后端的故障处理见第 11 节。

## 5. 安装

### 完整控制包

Python 版本须为 3.10 或更高。
在 SDK 仓库根目录执行：

```bash
python3 -m pip install '.[can]'
```

安装后可以从业务程序中导入 `sdk`。
`can` 扩展安装 SLCAN 所需的 pySerial。
SocketCAN 使用 Python 标准库收发。
内置后端由 `arx_x5_control.usb2can` 提供。

### H7 串口依赖

SDK 源码支持 Python 3.8 或更高。
使用 H7 串口后端时，在 SDK 仓库根目录执行：

```bash
python3 -m pip install -r requirements.txt
```

SDK 在打开串口时加载 pySerial。
源码开发使用 `python3 -m pip install -e '.[can]'`。

## 6. 快速开始

本节使用 H7 串口后端。
USB-CAN 双臂控制见 [实机例程](arx_x5_control/SDK_LOCAL.md#双臂实机接入例程)。

```python
import sdk, time

arm = sdk.Arm(0)                 # arm_id 0 = CAN1 上的 A 臂，1 = CAN2 上的 B 臂
arm.connect()                    # 开串口 → 发版本帧（握手）→ 等这一台的第一帧反馈
print(arm.robot_state_name, arm.joint_angles)    # 关节角，单位 rad

def wait_state(want, timeout=10.0):
    """等到 state == want。**等待期间必须持续保活** —— 见下面的说明。"""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        arm.keepalive()
        if arm.wait_for(lambda a: a.robot_state == want, timeout=0.05):
            return True
    return False

wait_state(2)                                    # 等到「待初始化」
arm.init_arm()                                   # 初始化（先看第 8 节的闸门！）
wait_state(4)                                    # 等到「正常运行」（落点是阻尼档）
arm.start()                                      # 切进 run 档；控制帧只在这个档被接受
arm.wait_for(lambda a: a.mode == a.board_defs.MODE_RUN)   # 等反馈确认切档（见下）

while True:
    # 夹爪目标三选一：gripper_m=0.03（要标定过）/ gripper_raw_deg=-141.5（不用标定）/ 都不给 = 保持当前
    arm.send_ctrl_pos(arm.joint_angles, gripper_m=0.0)   # 沿用当前角度、夹爪闭合
    time.sleep(0.01)                                     # 这一帧本身就算保活
```

两项必须遵守：

- **等待期间必须保活**（`wait_state` 那个循环）：单片机 3 s 收不到**任何**有效帧就切阻尼，没初始化
  过的还会退回「离线等待」，它自身发出的反馈帧不计入。因此 `wait_for` 之外必须有帧在持续发出：
  周期控制帧，或者 `keepalive()`（幂等的 `0x01`）。等待期间不发帧的结果是永远等不到 `state`，
  之后 `init_arm()` 直接抛 `GateRejected`。
- **切档要等反馈确认**：`start()` 返回只代表帧已发出，反馈里的 `mode` 仍是旧值。紧接着发
  控制帧会抛 `GateRejected: Arm_mode 不是 run：现在 damping(阻尼)`，因为预检使用的就是那帧
  旧反馈。

两台臂（**必须共享一个 `Link`**，原因见第 11 节）：

```python
link = sdk.Link(); link.open()
a, b = sdk.Arm(0, link), sdk.Arm(1, link)
a.connect(); b.connect()
```

全部帧接口都在 `Arm` 上，签名和闸门见第 10 节。

## 7. 单位约定

| 量 | SDK 对外 | 线上 |
|---|---|---|
| 关节角 | **rad** | deg |
| 关节角速度 | **rad/s** | deg/s |
| 夹爪开口 | **m**（要标定，见第 9 节） | — |
| 夹爪原始角 | `raw_gripper_angle_deg` / `gripper_raw_deg=` | deg |
| 温度 | ℃ | ℃（uint8） |

**原始夹爪角是唯一的例外**：它是电机的原始角，双向都不做"电机角 → 关节角"的换算，
因为那几张换算表只有 6 个关节有。所以想要"米"就只能标定。`snapshot()` 提供**同一帧**里
换算好的一致视图；单独读 `arm.joint_angles` 再读 `arm.mode` 是两次读，中间可能插入一帧新的。

## 8. 命令预检与执行条件

`Arm` 命令方法根据最近反馈执行预检。
条件不满足时，SDK 抛出 `GateRejected`。
反馈年龄超过 100 ms 时，SDK 抛出 `FeedbackStale`。
`force=True` 跳过 SDK 预检。
控制核心仍检查命令执行条件。

H7 拒绝命令时，不发送专用拒绝应答。
应用通过后续反馈确认状态和模式。
PC 本地后端通过 `link.drain_events()` 提供控制事件。
本地命令发送成功表示命令已进入队列。
应用通过反馈和事件确认执行结果。

| 命令 | 控制核心执行条件 |
|---|---|
| `init_arm()` 初始化 `0x02` | 状态是**待初始化(2)** **或**（正常运行(4) 且档位在 停/阻尼），**且** 7 台电机全在线，**且** 6 个关节（不算夹爪）与复位位姿偏差 ≤ 20° |
| `set_zero()` 设零 `0x10` | 状态是**待初始化(2)** |
| `start/gravity/stop/damping` 四个档位 `0x12..0x15` | 状态是**正常运行(4)** |
| `send_ctrl_pos/pos_vel` 控制 `0x16/0x17` | 状态是**正常运行(4)** **且** 档位是 **run** |
| `reset()` 复位 `0x18` | 状态是**正常运行(4)** **且** 7 台全在线（**不检查位姿** —— 位姿偏离时才需要复位） |

H7 后端还要求遥控器离线。
遥控器在线状态未包含在反馈帧中。
使用 H7 上位机控制前，关闭遥控器。
PC 本地后端直接接收应用提交的控制命令。

初始化要求的参考姿态是 `board_defs.RESET_POSE_DEG = (0, 0.5, 0.5, 0, 0, 0)` 度，容差
`ROBOT_INIT_POSE_TOL_DEG = 20` 度，就是上面表里那个 20°。发初始化之前先检查
`arm.joint_angles`。

## 9. 夹爪标定

夹爪使用达妙 DM4310，CAN 电机 ID 为 8。
电机收发层使用 MIT 协议。
反馈中的夹爪角为电机原始角，单位为度。
开口距离与电机角度的换算需要实测标定。

以下标定工具使用 H7 串口后端：

```bash
python3 -m sdk.calibrate_gripper --arm-id 0 --full-open-m 0.085
```

工具会连上、显示状态。然后 `d` 切阻尼（或 `x` 失能），**手动把夹爪移动到两个端点**，各按一次键
记录，回车存进 `sdk/config/gripper_calib.json`：

- `FULL_OPEN` 是**预先配置的参数**：完全张开时的开口距离，单位米，必须大于 0。代码中不固定
  量程。
- `0` 记「这里 = 0 m」，`f` 记「这里 = FULL_OPEN」，之后按这两点线性映射。
  **方向和尺度完全由这两个点决定**，工具不预设哪端是张开，装反了也不需要改代码。
- 也可以用 `r` 进 run 档后按 `←`/`→` **点动**。点动方向按原始角，不是"张开/闭合"。run 档
  整条臂都使能，不便手动移动时再用。

**必须在一次成功的初始化之后标定**：夹爪的零点是行程中点（∓142°），而这个零点只有找零才会
建立。找零之前，原始角的零点没有含义。工具会查反馈帧里夹爪的"已初始化"标志位，并在开头警告。
**之后再执行任何 `0x10` 设零或再次找零，零点会移动，标定作废，需要重新标定。**

**标定之后怎么用**（下面三条用的是同一份文件里 `arm_id` 那一项）：

- **控制开口**：`arm.send_ctrl_pos(joints, gripper_m=0.03)`。`gripper_m=` 按标定换算成原始角；
  不标定就走 `gripper_raw_deg=-141.5`（原始电机角）。**两个都不给 = 保持当前**；两个都给直接
  报错。
- **读开口**：`arm.gripper_opening`，单位米。**没标定就抛 `GripperNotCalibrated`**，不返回推测
  值。不希望被异常打断就用 `snapshot()`：同一帧里换算好的一套，`gripper_calibrated` 为假时
  `gripper_opening` 是 `None`。和标定无关的读法是 `arm.raw_gripper_angle_deg`。
- **生效时机**：第一次访问 `arm.calibration` 才读文件，读了就缓存。因此在别的进程里刚标完，
  这边要 `arm.reload_calibration()` 或重启才有米数。控制台面板按 `r` 重开链路时会自动重读
  两台。标定存在时，面板上夹爪那一行显示为米，步长也换成米档。

SDK 在 `FULL_OPEN` 和 `0` 两点之间线性换算。
夹爪标定与控制核心软限位分别生效。

标定文件是本机实测数据。仓库 `.gitignore` 忽略 `*.json`，**不要提交**。

## 10. 帧接口参考

本节描述 PC 与 H7 之间的串口协议。
PC 本地后端使用 SDK payload 转换命令参数。
USB-CAN 传输层收发电机协议 CAN 帧。

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
| `0x14` | 停止帧 | 上位机 | 0 | — | `stop()`（**失能**） |
| `0x15` | 阻尼帧 | 上位机 | 0 | — | `damping()` |
| `0x16` | 位置控制帧 | 上位机 | 28 B：f32[6] 关节角 + f32 夹爪角 | deg | `send_ctrl_pos()` |
| `0x17` | 位置+速度控制帧 | 上位机 | 52 B：f32[6] 关节角 + f32[6] 关节角速度 + f32 夹爪角 | deg, deg/s | `send_ctrl_pos_vel()` |
| `0x18` | 一键复位帧 | 上位机 | 0 | — | `reset()` |
| `0x80` | 反馈帧 | 单片机 | 63 B：4×u8 + f32[6] + f32[6] + f32 + u8[7] | deg, deg/s, ℃ | `arm.feedback` / `snapshot()` |
| `0x81` | 链路状态帧 | 单片机 | 88 B：22×u32（见下） | — | `link.link_status` |
| 其它 | （自定义） | 上位机 | 任意 | — | `send_raw(tid, payload)` —— **通用透传入口**，无预检无换算 |

`0x80` 的 4 个 u8 依次是 `mode`（档位）、`robot_state`（状态）、`motor_lost_mask`（掉线掩码）、
`motor_inited_mask`（已初始化掩码），后面是 6 个关节角、6 个关节角速度、夹爪原始角和 7 个
温度。掩码 **bit0..5 = j1..j6、bit6 = 夹爪**。`0x80` 单片机侧**实测 250 Hz**。

`0x81` 是**握手后 10 Hz** 的链路状态帧，链路级只有一份，不按台分。它报的是"距上一帧 `0x81`
之间，单片机收到了你多少帧"，**每个数都是窗口增量**，不是累计值。字段有 `rx_total`（解出来的
整帧数）、`rx_by_tid[12]`（按 tid 分，下标同 `protocol.LINK_TID_ORDER`，最后一格是"表外的
码"）、`crc8_err`、`crc16_err`、`drop_bytes`（没能组成任何有效帧的字节数）、`bad_arm`、
`over_len`、`ring_drop`、`gated`（被握手闸门挡下的）、`tx_fail`（单片机**发**帧失败）、
`dt_ms`。`link.link_status` 存最近一帧，`link.stats["link_status"]` 同款；没收到过是 `None`。

几个容易出错的点：

- `0x14 stop()` 是**失能**，电机不加力。
- `0x18 reset()` 走一段若干秒的轨迹回复复位位姿。它**不做夹爪找零**，完成后停在**阻尼**。
- `0x10 set_zero()` **覆盖电机的原零点**，把"当前位姿"定义成零点。它只管 j1~j6，夹爪不在
  默认参与集里。**没有即时反馈帧**，结果走几秒后的 `0x11`。
- `0x11` 的 `result = 1（未确认）`**不是故障**：它表示"帧已发出、位置本来就在容差内"。臂本来
  就停在零位上时，结果必然是这个。

## 11. 多臂

### H7 串口后端

```python
link = sdk.Link(); link.open()
a, b = sdk.Arm(0, link), sdk.Arm(1, link)     # 或 link.arm(0) / link.arms()
```

- **必须共享一个 `Link`**。两个进程、或者一个进程里两个 `Link` 争用同一个串口设备，会导致帧交错
  （`device reports readiness to read but returned no data`）。
- 帧头的 `arm_id` 决定目标臂：`0` = CAN1 = A 臂，`1` = CAN2 = B 臂。**没有广播**。越界的
  `arm_id` 单片机**直接丢弃**，SDK 侧也会丢弃并计数 `link.stats["n_bad_arm"]`。
- 一个 `Link` 可以同时驱动两台。各自的反馈槽、序号、`wait_*` 完全独立。

### PC 本地后端

两台臂使用两条独立 CAN 通道。
两台臂共享一个 `LocalLink`。
`arm_id` 与 `CANChannel.arm_id` 对应。

下面的程序连接双臂并读取反馈：

```python
import sdk
from arx_x5_control import ArmConfig, CANChannel, ProcessCANMotorIO
from arx_x5_control.sdk_adapter import LocalLink


def main():
    channels = [
        CANChannel(0, "can0", interface="socketcan"),
        CANChannel(1, "can1", interface="socketcan"),
    ]
    with ProcessCANMotorIO(channels) as motor_io:
        with LocalLink(
            motor_io, configurations=[ArmConfig(), ArmConfig()]
        ) as link:
            arms = [sdk.Arm(i, link) for i in (0, 1)]
            for arm in arms:
                arm.connect()
            for arm in arms:
                print(arm.arm_id, arm.snapshot())


if __name__ == "__main__":
    main()
```

通道打开时自动配置为 1 Mbps。
SocketCAN 驱动须创建接口，运行用户须有配置权限。
初始化和运动步骤见 [双臂完整例程](arx_x5_control/SDK_LOCAL.md#双臂实机接入例程)。

`LocalLink` 在后台执行控制核心。
`ProcessCANMotorIO` 在独立进程中执行 CAN 收发。
周期命令对对应臂保活。
等待期间须对两台臂分别调用 `keepalive()`。
每台臂的有效命令间隔须小于 3 秒。

业务循环调用 `link.check_health()` 检查控制故障。
`link.drain_events()` 返回控制事件。
发送成功表示命令进入队列。
执行结果通过反馈状态和控制事件确认。

使用上下文管理设备生命周期。
先退出 `LocalLink`，再退出 `ProcessCANMotorIO`。
关闭链路时，控制核心交付双臂停机请求。
`Arm.close()` 保留外部传入的共享链路。
`LocalLink` 的故障处理和重开要求见 [SDK 本地后端](arx_x5_control/SDK_LOCAL.md#调度退出和故障)。

## 12. 线程与反馈帧

本节描述 `sdk.Link` 的 H7 串口收发。
PC 本地后端的线程和故障处理见第 11 节。

- `Link` 开一根 **RX 后台线程**：`read → FrameParser → 按 arm_id 分发`。**RX 线程里不调
  用户回调**，因为慢回调会降低收帧速率。上层只能主动读或 `wait_*` 轮询。
- 每臂一把锁，反馈是整对象替换的（`Feedback` 是不可变对象），所以拿到手的那份永远不会被
  改写。但**两次独立的读可能跨帧**。要严格一致就用 `snapshot()`。
- 发帧持一把**写锁**，因为 pyserial 的 `write` 多线程不保证整帧原子。
- **保活不自动发送**：SDK 不启动保活线程。周期控制帧本身即构成保活；其他场景由上层调用
  `keepalive()`。**单片机自身发出的反馈帧不算"收到的帧"。**
- `0x11` 没有请求 id，只能按序号等。`set_zero()` 会记下发送前的序号，`wait_setzero_ack()`
  等序号更大的那一帧（默认 25 s，超时抛 `TimeoutError`）。
- 读写失败后属性会停在旧值。用 `link_up` / `feedback_age` / `feedback_ok` 判断链路状态，也可以
  `link.reopen()`。
- **掉线自动重连（默认开）**：读或写一失败，链路就置成断开，RX 线程跟着**退避重开**
  （0.5 s → 1 → 2 → 2 s…）。试满 `reconnect_max_tries` 次（默认 10）还开不上就停止重试
  （`reconnect_gave_up`），停在断开状态，等排查线缆或调用 `reopen()`。**一轮掉线最多重试
  10 次。** 断开期间 `is_open` 是假，`send()` 抛 `NotConnected`（`LinkError` 的子类，
  `except LinkError` 照样接住）。**设备重枚举后 tty 名字会变，所以重连按 USB 身份
  （VID:PID + 序列号）重新认口**，不认名字。要"断了就立刻失败"的脚本传
  `Link(auto_reconnect=False)`。次数和退避是构造参数：`reconnect_max_tries` /
  `reconnect_wait` / `reconnect_max_wait`，`<=0` 表示不限次。

## 13. 注意事项

1. **确认命令执行结果。** H7 串口发送成功表示数据已写入链路。
   PC 本地后端发送成功表示命令已进入队列。
   应用通过反馈确认状态和模式，本地后端还提供控制事件。
2. **H7 遥控器接管。** 遥控器在线时，H7 使用遥控输入。
   使用 H7 上位机控制前，关闭遥控器。
3. **通过 `Arm` 提交机械臂命令。** `Arm` 提供参数换算和预检。
   H7 自定义帧可以使用 `send_raw()`。
   本地后端支持的命令见 [方法映射](arx_x5_control/SDK_LOCAL.md#sdk-方法映射)。
4. **双臂分别保活。** 每台臂的有效命令间隔须小于 3 s。
   周期控制命令和 `keepalive()` 都构成保活。
   已初始化的臂超时后进入阻尼模式。
   未初始化的臂按通信离线状态处理。
5. **切换模式后重新提交目标。** 等待反馈确认进入 RUN，再下发控制目标。
6. **上层目标须满足关节限位。** 控制核心对已初始化电机执行限幅。
   `board_defs.clamp_axis(axis, value)` 接收度，返回限幅值和限幅标志。
   六关节最小值为 `(-115, 0.5, 0.5, -90, -90, -112.87)`°。
   最大值为 `(90, 206.26, 171.88, 90, 90, 119.17)`°。
   `Arm` 的位置控制方法接收 rad。
7. **夹爪控制目标是角度或标定开口。** 夹爪找零后，控制核心执行 ±142° 软限位。
   初始化状态由反馈中的 `motor_inited_mask` 表示。
8. **设零会改变电机零点。** 执行 `set_zero()` 前，将关节放到标定基准位姿。
   夹爪零点改变后，重新测量开口标定。
9. **停机后电机失能。** 调用 `stop()` 或关闭本地控制链路后，机械臂可能因重力下落。
10. **检查反馈时效和在线状态。** 使用 `snapshot().age` 和 `motor_lost_mask`。
    温度值为 0 时，结合在线状态判断数据是否有效。
11. **按后端检查通信。** SDK 保存 H7 反馈的主机到达时间。
    应用通过 `feedback_age` 判断时效。
    本地后端的 CAN、控制周期和日志指标见 [运行日志](arx_x5_control/LOGGING.md)。
12. **按后端选择设备。** H7 串口由 `sdk.Link` 选择。
    `list_links()` 列出候选串口，USB CDC 身份为 `0483:5740`。
    PC 本地后端通过 `CANChannel` 指定设备路径或 SocketCAN 接口名。
    SLCAN 设备使用 `/dev/serial/by-id/` 稳定路径。
13. **夹爪标定文件保存本机实测结果。** 每台臂使用对应的 `arm_id` 标定项。

## 14. 离线自测

```bash
python3 -m sdk.selftest          # 不需要硬件、不需要 pyserial
python3 -m sdk.tui --selftest    # 再带上控制台那 48 项（同样不接硬件、不起 curses）
```

205 项：CRC 参考值（位运算参考实现逐字节比表）、整帧字节锚点、编解码往返、切帧重同步、
按 `arm_id` 分流、单位往返、夹爪标定模型、**闸门矩阵**（逐条断言该拒绝的已拒绝、该发送的发送内容）、
**假单片机状态机**（`sdk.sim`：握手闸门、时序、两台互不干扰、看门狗、`--fake-lost`）、
**掉线自动重连**（把 `serial` 换成假的：退避、试满上限后停止、重连后残留半帧作废、按 USB 身份
认口），以及**与固件头文件对拍**（码表、限位、状态个数、档位顺序）。对拍项只在仓库存在时运行，
因此 `sdk/` 单独拷出使用时也不会失败。改协议或改板侧常量之后先运行它。

`python3 -m sdk.tui --selftest` 先运行上面那 205 项，再运行**控制台后端**的 48 项：两台臂的链路
（一条链路、两个 `Arm`、状态机各自独立）、单位与软限位、步长档、闸门是否确实拦截、命令是否
确实写出。这些同样用 `sdk.sim` 的仿真板运行。

### 界面语言（`--lang`）

面板默认中文，`--lang en` 换成**英文面板、英文事件、英文 `--help`**。`--selftest` 也跟着换，
因此英文模板会被实际执行一次。中英文共用同一份代码和一张语言表（`sdk/i18n.py`，键名
`<模块>.<名字>`）；zh/en 的键集必须相等，语言表自检就挂在 `--selftest` 里。
`set_lang()` 只认 `zh`/`en`，其余值（含 `None`）一律回落 `zh`，永不抛异常。

```bash
python3 -m sdk.tui --lang en                 # 英文面板（真机）
python3 -m sdk.tui --fake --lang en          # 英文面板（假板子）
python3 -m sdk.tui --selftest --lang en      # 英文模板实际执行一次自测
```

英文只覆盖**面向读者的文本**。下面这些**保持中文**，这是有意设计：

* `sdk/board_defs/common.py` 的名字表。它是公开字段 `Snapshot.robot_state_name` / `mode_name` /
  `lost_names` / `inited_names` 的来源，保持中文；英文名并列在 `i18n` 里；
* `python3 -m sdk.selftest` 自己的小节标题与汇总行，以及 `--selftest` 里那一段 sdk 自测的
  同名部分；
* `calibrate_gripper.py` 自身打印的交互提示。它抛出的 arm/link 异常会跟着变英文。

## 15. 文件清单

| 文档入口 | 内容 |
| --- | --- |
| [PC 控制包](arx_x5_control/README.md) | 安装、运行和模块索引 |
| [双臂实机接入](arx_x5_control/SDK_LOCAL.md#双臂实机接入例程) | 双通道、初始化、运动和退出 |
| [CAN 接口](arx_x5_control/CAN_IO.md) | 通道配置和电机收发 |
| [USB-CAN 传输](arx_x5_control/usb2can/README.md) | 后端接口和 JSON 配置 |
| [动力学来源与许可证](arx_x5_control/gravity_compensation/PROVENANCE.md) | Python 动力学实现和 URDF 模型 |
| [运行日志](arx_x5_control/LOGGING.md) | TUI 日志和性能指标 |


| 文件 | 职责 |
|---|---|
| [`README.md`](README.md) | **本文**：会话流程、状态机、闸门、注意事项 —— 先看它 |
| [`API.md`](API.md) | **逐符号参考**：每个类 / 方法 / 常量是什么、什么单位、受什么闸门、抛什么异常 |
| `__init__.py` | 包入口，只 re-export；`import sdk` 不碰 pyserial、不起线程 |
| `errors.py` | 异常层级（单独一个文件，避免各模块互相 import） |
| `protocol.py` | 协议层：CRC / 版本码的 FNV 哈希 / 码表 / 组帧 / `FrameParser` / payload 编解码（**线上单位**） |
| `board_defs/` | `arx_x5.py` / `metal_arm.py` 分别提供各机械臂限位与复位姿态；`common.py` 保存共同协议编号 |
| `units.py` | 度 ↔ rad 换算策略 |
| `link.py` | 串口 + RX 线程 + 按 `arm_id` 分流 + 写锁 + 串口选择 |
| `arm.py` | `Arm`：全部帧接口 + 闸门预检 + 单位换算（**SI**） |
| `sim.py` | `SimLink`：假单片机，两台臂的模拟状态机 + 闸门 + 看门狗 |
| `gripper.py` | 夹爪两点标定：m ↔ 原始度的映射 + JSON 存取 |
| `calibrate_gripper.py` | 交互式标定工具 |
| `i18n.py` | 语言表（`zh`/`en`）+ `tr()` + 名字表 + 表自检。中文是默认，也是 `board_defs` 的唯一来源 |
| `tui/` | **上机控制台**（curses 面板）：`python3 -m sdk.tui`，不接硬件用 `--fake`，英文面板用 `--lang en`。`session` = 后端/适配层、`app` = 面板、`selftest` = 控制台那 48 项 |
| `selftest.py` | 离线自测 |
| `arx_x5_control/` | PC 控制核心、SDK 适配层和 USB-CAN 传输 |
| `pyproject.toml` | SDK 与 PC 控制包的统一安装配置 |
| `config/` | 运行期数据（`gripper_calib.json`），由工具自动创建 |

异常层级（`errors.py`）：根是 `ArError`，下面分 `LinkError`（`NotConnected` 是它的子类）、
`ProtocolError`（**代码中从不抛出**，解码器长度不符时抛的是 `ValueError`）、`GateRejected`、
`FeedbackStale`、`GripperNotCalibrated`、`CalibrationError`。
`GateRejected` 的消息里会写清楚缺哪一条闸门，可直接 `print` 展示给操作者。
