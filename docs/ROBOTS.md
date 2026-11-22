# 机械臂参数与行为差异

ARX X5 和 Metal Arm 使用相同的 `sdk.Arm` 控制方法、SI 单位、状态编号和 TUI 按键。公共流程见 [Python 调用](PYTHON.md) 与 [TUI 调用](TUI.md)。本页列出型号配置与夹爪换算。

## 参数选择

| 项目 | ARX X5 | Metal Arm |
| --- | --- | --- |
| `robot_type` | `arx_x5`，默认值 | `metal_arm` |
| 参数模块 | `sdk.board_defs.arx_x5` | `sdk.board_defs.metal_arm` |

型号选择集中在链路创建处。运行时通过 `link.board_defs` 或 `arm.board_defs` 读取所选参数；同一链路的两台臂共用参数模块。所选配置用于命令预检、单位换算及界面显示。

## 关节与夹爪参数

通过 `arm.board_defs` 或 `link.board_defs` 读取所选参数。

| 字段 | 含义 |
| --- | --- |
| `JOINT_ANGLE_MIN_RAD` / `JOINT_ANGLE_MAX_RAD` | 六轴角度上下限，单位 rad |
| `RESET_POSE_RAD` | 六轴复位参考，单位 rad |
| `GRIPPER_ANGLE_MIN` / `GRIPPER_ANGLE_MAX` | 夹爪原始角上下限，单位 deg |
| `ROBOT_INIT_POSE_TOL_DEG` | 初始化姿态预检容差，单位 deg |
| `DEFAULT_GRIPPER_TORQUE_LIMIT_NM` | 默认夹爪力矩上限，单位 N·m |
| `VERSION_STRING` / `VERSION_CODE` | 握手协议版本串和版本码 |

`reset()` 使用复位参考。
`set_zero()` 提交设零请求，结果通过 `SetZeroAck` 返回。

## 控制行为

SDK 负责命令预检、单位换算和反馈读取。连接和握手接口见 [串口接入](SERIAL.md)。

| 操作 | SDK 语义 |
| --- | --- |
| 初始化 | 检查电机在线及复位姿态容差；完成后为正常运行 / 阻尼 |
| 显式设零 | 将六轴当前位姿写为电机零点；逐项检查 `SetZeroAck` |
| 复位 | 使用所选复位参考，完成后进入阻尼；不改写零点 |
| STOP | 软停机，不提供位置保持或重力补偿 |
| 关闭串口 | 停止收发数据并关闭接收线程 |

退出前应放稳或支撑机械臂。

## 夹爪换算

| 连接 | 默认米制换算 | 使用条件 |
| --- | --- | --- |
| ARX X5 | 文件中的两点标定 | 初始化找零成功，并有该臂的有效标定 |
| Metal Arm | 厂家最近邻查表 | 出厂零点有效 |

公共 API 使用 `gripper_m`，换算由所选存储负责。`sdk.Arm(..., calib_store=...)` 可指定实测标定；未提供夹爪目标时保持当前反馈角。原始角接口 `gripper_raw_deg` 使用 deg，数值范围由所选参数给出。

文件标定提供 `full_open_m`、`direction` 等两点模型字段；厂家查表提供双向角度/开口换算。上层控制使用 `gripper_opening` 或 `snapshot()`，不依赖换算对象的特有字段。标定文件与工具见 [配置说明](../config/README.md)。

## 参数来源

ARX X5 参数定义在 `sdk.board_defs.arx_x5`。
Metal Arm 的限位与夹爪查表取自厂家 `metal-python-ros` 的 `bccf3c8` 版本，分别对应 `motor_config.cpp` 与 `can_manager.h`；版权及 MIT 许可保留于 [LICENSE.MakerMods](../board_defs/LICENSE.MakerMods)。六轴复位参考定义于 `sdk.board_defs.metal_arm.RESET_POSE_DEG`。
