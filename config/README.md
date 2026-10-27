# 夹爪标定数据

`gripper_calib.json` 保存本机实测的夹爪两点标定，按 `arm_id` 区分。默认路径相对于 SDK 安装位置，与当前工作目录无关；可用 `ARX_GRIPPER_CALIB` 或 `GripperCalibStore(path=...)` 指定可写位置。

`python3 -m sdk.calibrate_gripper` 连接原 ARX H7 链路，在首次保存时创建文件。该工具目前没有机械臂类型选项。H7 `Link(robot_type="metal_arm")` 仍使用这套文件存储；Metal PC `LocalLink` 默认使用厂家查表，可由显式的 `sdk.Arm(..., calib_store=...)` 覆盖。

这些文件是设备实测数据，仓库忽略生成的标定 JSON；不要提交。夹爪拆装或零点改变后应重新核验。电机配置使用的受版本管理的 `usb2can/transports.json` 不属于本目录的标定数据。
