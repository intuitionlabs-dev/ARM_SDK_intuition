# TUI 调用

TUI 通过公共 SDK 显示反馈、初始化、切换模式和发送关节目标。设备与型号在启动时选择，进入界面后的常规操作使用同一组按键。

## 启动入口

| 后端 | 型号 | 实机命令 | 离线命令 |
| --- | --- | --- | --- |
| H7 | ARX X5 | `python3 -m sdk.tui --port /dev/ttyACM0 --robot-type arx_x5` | `python3 -m sdk.tui --fake --robot-type arx_x5` |
| H7 | Metal 参数 | `python3 -m sdk.tui --port /dev/ttyACM0 --robot-type metal_arm` | `python3 -m sdk.tui --fake --robot-type metal_arm` |
| PC 本地 | ARX X5 | `arx-x5-tui --interface socketcan --channel can0` | `arx-x5-tui --simulate` |
| PC 本地 | Metal Arm | `metal-arm-tui --interface socketcan --channel can0` | `metal-arm-tui --simulate` |

H7 的设备动作由固件实现，参数选项用于上位机预检与显示。`--fake` 模拟 H7 协议状态机；PC 的 `--simulate` 运行各型号的本地控制核心。

PC 双臂在命令末尾增加 `--channel can1`；通道顺序对应臂 0、臂 1。SLCAN 使用 `--interface slcan --channel /dev/ttyACM0`，设备路径应对应 USB-CAN 适配器。

## 操作流程

1. 查看连接、反馈年龄、电机在线状态和当前臂编号。
2. 确认六轴处于复位参考附近，按 `i` 初始化，等待正常运行 / 阻尼状态。
3. 按 `s` 进入 RUN，选择关节或夹爪，调整目标。
4. 通过反馈判断运动结果；使用 `d`、`v`、`x` 切换模式，`o` 执行复位。
5. 放稳或支撑机械臂，按 `q` 退出。设备输出的退出行为见 [后端生命周期](BACKENDS.md#生命周期与诊断差异)。

初始化内部动作由型号和后端决定，见 [控制行为表](ROBOTS.md#控制行为)。界面中的限位、复位参考和夹爪换算读取链路配置。

## 公共按键

| 按键 | 操作 |
| --- | --- |
| `a` | 切换当前臂 |
| `1`～`6`、`0` | 选择关节或夹爪 |
| `+` / `-`、方向键 | 按步长调整目标 |
| `[` / `]` | 切换步长 |
| `c` | 将目标对齐当前反馈 |
| `i` | 初始化 |
| `s` | RUN 位控 |
| `d` / `v` / `x` | 阻尼 / 重力补偿 / STOP |
| `o` | 复位 |
| `z` | 显式设零，按界面提示确认 |
| `h` / `?` | 帮助 |
| `q` | 退出 |

TUI 为每台臂发送目标或保活，切换显示臂不会停止另一台臂的保活。Python 程序须自行安排保活；两者共用相同的后端超时规则。

## 参数与诊断差异

| 项目 | H7 TUI | PC TUI |
| --- | --- | --- |
| 设备参数 | `--port`、`--baud` | `--interface`、一个或两个 `--channel`、`--transport-config` |
| 型号参数 | `--robot-type` | 由命令入口确定 |
| 初始化容差 | `--init-tol`，默认读取所选参数 | 使用对应型号参数 |
| 故障恢复 | 自动串口重连；`r` 可重开 | 关闭控制台，排除故障后重新启动 |
| 日志 | `--log` 指定界面事件文件 | 自动保存运行记录，`--log-dir` 指定根目录，`m` 添加标记 |

两种入口均支持 `--lang en`、`--arm-id`、`--ctrl-hz`、`--fb-hz` 和 `--step`。默认控制目标频率为每臂 100 Hz；电机内部调度频率由后端决定。完整选项使用对应入口的 `--help` 查看。

界面默认中文，`--lang en` 切换面板和 SDK 消息语言。参数模块名字表、SDK 自测和夹爪标定工具的提示使用中文。PC 日志和文件格式见 [日志说明](LOGGING.md)。

## 版本显示

| 标识 | 来源 | 含义 |
| --- | --- | --- |
| `vcode 0xXXXXXXXX` | 所选型号的 `board_defs.VERSION_CODE`：`arx_x5` 为 `0xF81DD236`，`metal_arm` 为 `0xD8D044AE` | 握手用的协议版本码，随 `--robot-type` 变化 |
| `board_version` | H7 版本帧；PC 本地适配层填入协议码 | 握手协议版本 |
| `package_version` | `arx-x5-control` 安装元数据，当前 `0.6.0` | 发行包版本 |
| 源码提交 | `git rev-parse --short HEAD` | 当前源码版本 |

协议码不代表机械臂型号或电机固件版本。PC 运行记录还保存所选源码文件的哈希，用于定位采样时的实现。
