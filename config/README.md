# 夹爪标定数据

`GripperCalibStore` 按 `arm_id` 保存实测的两点标定。公共控制接口使用 `gripper_m`，将开口转换为原始角；换算方式由链路默认存储或 `sdk.Arm(..., calib_store=...)` 指定。

## 默认换算

| 连接 | 默认方式 | 配置要求 |
| --- | --- | --- |
| H7，任一型号参数 | JSON 两点标定 | 与设备的实际夹爪零点一致 |
| PC，ARX X5 | JSON 两点标定 | 初始化找零后测量开口端点 |
| PC，Metal Arm | 厂家查表，0..0.1 m | 有效出厂零点；显式存储可覆盖 |

厂家查表和文件标定的对象差异见 [夹爪换算](../docs/ROBOTS.md#夹爪换算)。

## 文件配置

| 配置 | 含义 |
| --- | --- |
| 默认文件 | SDK 安装目录下的 `config/gripper_calib.json` |
| `ARX_GRIPPER_CALIB` | 自定义默认文件路径 |
| `GripperCalibStore(path=...)` | 显式指定可写文件 |
| `arm.reload_calibration()` | 重新加载被其他程序更新的标定 |

不同设备复用同一 `arm_id` 时，应选用各自的标定文件。夹爪拆装、找零或零点改变后需复核标定。生成的 JSON 是本机设备数据，仓库忽略该文件；电机传输配置 `usb2can/transports.json` 单独管理。

## 标定工具

`python3 -m sdk.calibrate_gripper` 使用 ARX H7 串口链路。工具参数、两点采样和保存步骤见 [H7 夹爪标定](../docs/H7.md#arx-h7-夹爪标定工具)。其他连接可通过公共 `GripperCalibration` 和 `GripperCalibStore` 接口提供实测结果，见 [API](../API.md#sdkgripper--夹爪两点标定)。
