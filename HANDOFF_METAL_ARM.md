# Metal Arm SDK 适配交接

本次已将 MakerMods Metal Arm 适配为 `ARM_SDK_intuition` 的独立 PC 控制后端 `metal_arm_control`，沿用 `arx_x5_control` 的模块结构、控制流程及 `sdk.Arm` 接口。实现和离线验证已提交推送，仓库也已迁移到 `intuitionlabs-dev`。尚未进行 Metal 真机验证，PR 保持草稿。

交接日期：2026-10-27。

后续更新：Metal STOP 已恢复周期使能加零增益输出；初始化仅执行六轴复位，夹爪校准阶段已删除，入口保留夹爪当前反馈角。设备参数已拆为 `sdk.board_defs.arx_x5` 和 `sdk.board_defs.metal_arm`，通过 `arm.board_defs` / `link.board_defs` 读取。包根不再暴露默认设备参数。用户将通过 TUI 读取并提供 Metal 复位角；当前仍保留旧六轴零角，复位角替换待该数据提供后完成。

本文记录的功能实现基线为 `5b76a9b`；后续交接文档提交不属于该功能基线。接手时先核对分支和远端最新状态。

## 任务背景和当前状态

用户的核心要求是：参考 Metal 原厂 SDK，将 Metal Arm 接入本仓库，使其拥有与 `arx_x5_control` 相同的结构，主要通过设备参数区分；随后要求将仓库迁移到 `intuitionlabs-dev`。

| 项目 | 当前值 |
| --- | --- |
| 仓库 | https://github.com/intuitionlabs-dev/ARM_SDK_intuition |
| 本机目录 | `/home/zhaosuya/ARM_SDK_intuition` |
| 任务分支 | `feat/metal-arm-control` |
| 合并目标 | `master` |
| 草稿 PR | https://github.com/intuitionlabs-dev/ARM_SDK_intuition/pull/1 |
| 本机 `origin` | `https://github.com/intuitionlabs-dev/ARM_SDK_intuition.git` |
| 原厂源码目录 | `/home/zhaosuya/metal-python-ros` |
| 原厂分支及固定版本 | `humble`，`bccf3c82a21453a7b1784a94966456e16ff0f008` |

仓库迁移已经完成，无需再次发起 transfer。旧的 `Crystal-1438/ARM_SDK_intuition` 地址会重定向；后续使用新组织地址。PR 尚未合并，`master` 尚未包含本次 Metal 实现。用户没有要求将 PR 转为正式评审或合并。

三个功能提交按依赖顺序为：

| 提交 | 内容 |
| --- | --- |
| `0687af3` | 为共享 SDK 和 TUI 增加按链路提供限位、复位参数及夹爪标定存储的能力 |
| `5c1311e` | 新增 Metal 镜像包、设备协议与参数、模型、文档、测试及安装入口 |
| `5b76a9b` | 增加原厂 C++ 编码对照、CAN 线程和进程生命周期测试及 CI |

这三个提交基于 `f6937f0`。最初三个提交未修改 `arx_x5_control/`；后续参数拆分也更新了 ARX 配置导入和本地链路，硬件参数与控制行为保持原值。不要为统一两个后端而覆盖 ARX 原有默认值。

## 代码入口和设计边界

先阅读 [Metal 概览](metal_arm_control/README.md)、[SDK 接入流程](metal_arm_control/SDK_LOCAL.md) 和 [来源与许可证](metal_arm_control/gravity_compensation/PROVENANCE.md)，然后按下表定位代码。

| 入口 | 职责或本次变化 |
| --- | --- |
| [arm.py](arm.py)、[tui/session.py](tui/session.py)、[tui/app.py](tui/app.py) | 读取链路选择的独立机械臂参数模块及可选的 `link.gripper_calib_store`；H7 链路通过 `Link(robot_type="arx_x5"/"metal_arm")` 或串口 TUI 的 `--robot-type` 选择上位机参数，默认 ARX，不修改 H7 固件 |
| [metal_arm_control/sdk_adapter/link.py](metal_arm_control/sdk_adapter/link.py) | `LocalLink`、共享 SDK 协议适配、单位转换、每臂配置、调度与反馈 |
| [metal_arm_control/system.py](metal_arm_control/system.py)、[arm.py](metal_arm_control/arm.py) | 单臂或双臂管理、状态机、命令门控和故障处理 |
| [command.py](metal_arm_control/command.py)、[trajectory.py](metal_arm_control/trajectory.py) | 目标处理、TD 和五次轨迹、初始化及复位流程 |
| [config.py](metal_arm_control/config.py)、[motor.py](metal_arm_control/motor.py) | 硬件参数、坐标、MIT 原生 PD 和电机输出策略 |
| [can_protocol.py](metal_arm_control/can_protocol.py) | Metal Damiao MIT 帧编解码及特殊命令 |
| [can_io.py](metal_arm_control/can_io.py)、[can_process.py](metal_arm_control/can_process.py)、[usb2can](metal_arm_control/usb2can/README.md) | CAN 线程/独立进程、SocketCAN/SLCAN、退出时交付禁用命令 |
| [calibration.py](metal_arm_control/calibration.py) | 显式六轴标零；夹爪厂家查表位于 SDK adapter，初始化已没有夹爪校准阶段 |
| [gravity_compensation/solver.py](metal_arm_control/gravity_compensation/solver.py) | Metal URDF、重力/科氏/摩擦补偿 |
| [pyproject.toml](pyproject.toml) | 分发版本 `0.6.0`，打包 SDK、两个后端、模型及命令行入口 |
| [tests](tests)、[CI](.github/workflows/control-tests.yml) | 协议、状态机、兼容性、动力学和传输生命周期验证 |

`metal_arm_control` 与 `arx_x5_control` 的相对 Python 文件集合一致，公开导出及关键类方法的参数名和默认值有测试约束。协议和硬件行为存在必要差异，不能将“结构相同”解释为逐行相同或仅替换常量。

Metal 后端直接实现于本仓库的控制架构内，不加载原厂二进制，也不依赖 ROS。未增加原厂 `SetArmEndPose` 一类笛卡尔 IK 接口，因为目标 ARX SDK 没有该接口。

安装分发名称仍是 `arx-x5-control`，不是新的 PyPI 包名。新增命令为 `metal-arm-control`、`metal-arm-sdk-demo`、`metal-arm-can-check`、`metal-arm-tui`。

## 控制流程和单位

业务命令依次经过 `sdk.Arm → Metal LocalLink → ControlSystem → metal_arm_control.arm.Arm → 轨迹及动力学 → MotorRequest → CAN 收发 → 电机`；电机反馈沿相反方向更新 SDK 快照和 TUI。

典型流程如下，完整可运行例子见 [SDK_LOCAL.md](metal_arm_control/SDK_LOCAL.md)：

1. 创建 `LocalLink`，为每条 CAN 通道配置一台臂；用 `sdk.Arm(arm_id, link)` 接入。离线可用 `LocalLink.simulated()`。
2. `connect()` 握手，等待电机反馈齐全并进入 `WAIT_INIT`。连接成功不表示初始化完成。
3. `init_arm()` 执行初始化。完成后进入 `NORMAL`，模式为 `DAMPING`。
4. `start()` 切换 `RUN`，再用 `send_ctrl_pos()` 或 `send_ctrl_pos_vel()` 提交目标。读取反馈和 `link.drain_events()` 判断结果；命令入队不代表运动完成。
5. 等待期间逐臂保活。示例中的 `wait_arms()` 同时处理保活和后台异常；不要用长时间 `sleep()` 代替。
6. 结束前处理机械臂支撑，再关闭链路。关闭会向七个电机发送禁用请求。

状态编号保持 ARX 约定：`WAIT_HANDSHAKE=0`、`OFFLINE_WAIT=1`、`WAIT_INIT=2`、`INITIALIZING=3`、`NORMAL=4`、`COMM_LOST=5`、`CALIBRATING=6`、`RESETTING=7`。模式为 `STOP=0`、`RUN=1`、`DAMPING=2`、`GRAVITY=3`。

| 层级 | 单位 |
| --- | --- |
| `sdk.Arm` 关节角和速度 | rad、rad/s |
| SDK `gripper_m` / `gripper_raw_deg` | m / deg |
| 控制核心 `Command` | deg、deg/s |
| `MotorRequest` | rad、rad/s、N·m |

调度保留 ARX 架构：1 ms 控制 tick、2 ms 命令更新、4 ms 反馈周期，每轴目标发送周期为 4 ms，即 250 Hz，并错开发送相位。这不是原厂 C++ 的 400 Hz 循环，普通 PC Python 环境也不保证硬实时。手动 `background=False` 模式必须以严格递增的毫秒时间调用 `step()`。

应用需要在每台臂的 3 s 命令超时之前发送有效命令或保活。正常运行时命令超时转阻尼；电机反馈超过 100 ms 或出现故障则使受影响整臂停止，另一臂独立处理。`STOP` 不维持重力补偿；无已知故障的电机会继续收到每台约 75 ms 的周期使能和零增益 SOFT_STOP。已知故障电机不发送使能，标零参与电机由标零流程独占，关闭仍禁用全部电机。

## 必须保留的 Metal 差异

### 电机协议和坐标

每臂六个关节加一个夹爪，七个电机均使用 Damiao MIT 协议，经典 CAN 1 Mbps。

| 参数 | J1 到 J6 及夹爪 |
| --- | --- |
| 发送 ID | `1, 2, 3, 4, 5, 6, 7` |
| 接收 ID | `0x11..0x17`，同时校验数据内电机 ID |
| PMAX | 全部 `6.28 rad` |
| VMAX | `10, 10, 10, 10, 30, 30, 30 rad/s` |
| TMAX | `30, 120, 30, 30, 20, 20, 20 N·m` |
| 原生 Kp | `300, 500, 500, 400, 25, 20, 20` |
| 原生 Kd | `5, 5, 5, 4, 0.2, 0.2, 0.1` |
| 六轴下限 | `(-160, -180, 0, -123, -85, -145) deg` |
| 六轴上限 | `(160, 0, 180, 81, 85, 145) deg` |
| 夹爪原始角范围 | `0..2.4 rad` |

PMAX/VMAX/TMAX 是协议编码范围，不能当作额定性能，也不能据此推断具体电机型号。现有源码确认 Damiao 协议和这些参数，尚不足以确认商业型号。

厂家 CAN 坐标已经是关节输出轴坐标，所以默认 `direction=1`、`reduction=1`；不要再次套用厂家机械减速比。Metal 使用电机原生 MIT PD，主机级联位置/速度 PID 置零。

编码保持与原厂 C++ 的 float32 算术一致，反馈归一化分母为 `65535/4095`。特殊命令为 `FC` 使能、`FD` 禁用、`FE` 保存零点。只读状态查询使用 `0x7ff`，不发送使能帧。反馈状态 `0` 为禁用、`1` 为使能，其余作为故障处理；不要套用 ARX 的 ENCOS、接收 ID 或归一化规则。

### 初始化和夹爪标定

Metal 初始化仅执行六轴复位，结束后直接进入正常阻尼模式；夹爪校准及范围核验阶段已删除。初始化入口锁存夹爪当前反馈角，不寻找限位或写入零点。正常控制的夹爪限位和整臂反馈健康检查仍然有效。

默认夹爪换算来自原厂 `can_manager.h` 的最近邻查表，要求出厂零点仍有效。SDK 米制范围为 `0..0.1 m`；表中虽有 `102.5 mm` 项，米制接口仍限制在 `0.1 m`。例如 `50 mm` 对应 `1.10383 rad`。显式传入 `sdk.Arm(..., calib_store=...)` 可以覆盖默认存储，Metal 不自动读取 ARX 本机标定文件。

`set_zero()` 仍是显式写入 J1–J6 当前零点的命令，不包含夹爪，也不会自动寻找机械零位。初始化参考和 `reset()` 目标仍临时保留旧六轴零角，等待用户从 TUI 读取并提供正确复位角后，更新 `board_defs/metal_arm.py`。初始化姿态容差为 20°，复位轨迹为 2 s；旧零角不是已确认的厂家收纳姿态。

### 动力学和退出行为

两份原厂 URDF 为 `metal_with_gripper`、`metal_no_gripper`，动力学链为 `base_link → Link6`，关节为 `JOINT1..JOINT6`，默认重力 `(0,0,-9.81)`。切换无夹爪模型只改变动力学参数，当前控制拓扑仍要求七台电机。

默认 `gravity_sdk_scale=True` 时，RUN 前馈为缩放的重力、科氏项与粘性摩擦之和，不含加速度前馈；GRAVITY 模式只用缩放重力。关闭 SDK 缩放后可计算原始 `M*qdd + C + G`。缩放和摩擦系数集中在 `config.py`，不要带入 ARX 的模型或经验系数。

本后端关闭时禁用全部七台电机，与原厂析构时保留电机使能的行为不同。需要先放稳或支撑机械臂，不能把退出理解为保持当前位置。

移植参数、模型、夹爪表及协议保留 MakerMods MIT 许可证；沿用的动力学算法保留 LGPL-2.1-or-later 许可证，详见 [PROVENANCE.md](metal_arm_control/gravity_compensation/PROVENANCE.md)。

## 已完成的验证

以下结果对应功能基线 `5b76a9b`。GitHub Actions 的 Python 3.10 和 3.12 两组均成功，可查看 [验证记录](https://github.com/intuitionlabs-dev/ARM_SDK_intuition/actions/runs/40483599307)。本次交接文档不改变控制代码。

| 验证 | 结果和覆盖范围 |
| --- | --- |
| `unittest discover -s tests -v` | 24 项通过；覆盖共享配置隔离、模块/API 对齐、状态机、协议、夹爪、SDK 单位、TUI 和动力学 |
| 原厂 C++ 编码对照 | 七个电机各 100 组固定随机输入，共 700 组命令字节一致；包含在上述 24 项中 |
| CAN 生命周期 | 真实线程及 spawn 子进程配合 `EchoTransport` 假传输，覆盖只读查询、双臂路由、故障退出及全部电机禁用 |
| 现有 SDK 自测 | 178 通过、0 失败、1 跳过；跳过项依赖仓库之外的 H7 固件头文件 |
| TUI 自测 | 48 通过、0 失败 |
| 安装后离线演示 | Metal 控制核心、Metal 双臂 SDK、原有 ARX 演示均 `sequence_complete: true` |
| 本机独立 wheel 验证 | `0.6.0` wheel 在独立环境、源码目录之外完成测试与资源加载检查 |

目录/API 测试比较全部相对 `.py` 路径和 `__all__`，并比较 `arm`、`system`、`motor`、`can_io`、`can_process`、`sdk_adapter.link` 中公开类方法的参数名与默认值；它不证明两个设备的硬件语义完全相同。

原厂对照测试需要 `c++` 编译器，缺失时会跳过；本机和上述 CI 运行时均具备编译器。CAN 测试使用模拟传输，不代表真实总线测试。模拟器和动力学数值测试不能证明实机稳定性、负载能力或碰撞安全。

## 接手后的复现步骤

先检查工作区，不要覆盖未提交修改。已有本地克隆使用以下只读检查和 fetch；确认在任务分支后再继续：

```bash
cd /home/zhaosuya/ARM_SDK_intuition
git status --short --branch
git remote -v
git fetch origin
git log -5 --oneline
git diff --stat origin/master...HEAD
gh pr view 1 --repo intuitionlabs-dev/ARM_SDK_intuition \
  --json url,state,isDraft,headRefName,baseRefName,statusCheckRollup
```

新环境可从新组织克隆 `feat/metal-arm-control` 分支。使用 Python >=3.10；已有 `.venv` 时可直接使用，无需重建：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[can]'
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python -m sdk.tui --selftest
.venv/bin/python -m metal_arm_control --demo --seconds 8 --log-level ERROR
.venv/bin/python -m metal_arm_control.sdk_adapter.demo --log-level ERROR
.venv/bin/python -m arx_x5_control --demo --log-level ERROR
```

以上演示均为离线模拟。需要交互检查时运行 `.venv/bin/metal-arm-tui --simulate`。真实设备对象组合见 [SDK_LOCAL.md](metal_arm_control/SDK_LOCAL.md)，CAN 参数和接口见 [CAN_IO.md](metal_arm_control/CAN_IO.md)。`ProcessCANMotorIO` 使用 spawn，调用脚本必须有 `if __name__ == '__main__':` 保护。

## 后续工作及交接约定

当前没有已确认但未修复的离线测试失败。接下来主要是代码评审和实机确认，不应将交接请求理解为已经授权新的硬件运动或 PR 合并。

1. 评审 Metal 必要行为差异，尤其是初始化、六轴零角复位、禁用电机退出，以及共享 SDK 配置不污染 ARX 的约束。
2. 在具备对应设备和调试条件后，先确认 CAN ID、固件协议范围、反馈单位、零点和夹爪出厂标定。不能与原厂驱动或 MakerMods Lab 同时控制同一总线。
3. 再安排实机初始化、低幅关节控制、夹爪换算、重力补偿、超时停机和退出行为验证；记录设备型号、固件版本、负载、参数和结果。重点确认零角复位策略、250 Hz 时序和真实负载下的补偿参数。
4. 按后续用户指示处理 PR 评审和合并。保留实机验证边界，不因模拟通过就将实现描述为已完成硬件验收。

用户已持续授权任务相关的小步提交和推送：每个完整小改动及相关检查完成后及时 commit/push，较长任务有可保存成果时约每 5–10 分钟保存一次。只提交本任务文件，检查 diff，保护无关工作和凭据；沿用任务分支及配置远端，不 force-push。每次验证推送结果；若受阻，保留本地提交并报告具体原因。
