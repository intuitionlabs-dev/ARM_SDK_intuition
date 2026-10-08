这个目录放**每台机器一份**的运行期数据，默认是 gripper_calib.json（夹爪标定）。

它由 `python3 -m sdk.calibrate_gripper` 在第一次存盘时自动创建；仓库的 .gitignore
全局忽略 *.json，所以标定内容不会进版本库 —— 那是实测数据，换一台夹爪或拆装一次就该重标。
