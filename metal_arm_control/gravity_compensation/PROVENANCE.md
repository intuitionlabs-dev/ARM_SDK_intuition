# Metal Arm 算法与模型来源

## 算法来源

| 实现 | 参考项目与版本 | 提交 | 许可证 |
| --- | --- | --- | --- |
| `solver.py`、`dynamics.py`、`spatial.py` 的递归牛顿欧拉算法 | [Orocos KDL v1.5.1](https://github.com/orocos/orocos_kinematics_dynamics) | `db25b7e480e068df068232064f2443b8d52a83c7` | LGPL-2.1-or-later，全文见 [LICENSE](LICENSE) |
| `urdf.py` 的坐标语义 | [ROS kdl_parser 1.14.2](https://github.com/ros/kdl_parser) | `74d4ee3bc6938de8ae40a700997baef06114ea1b` | 对应版权声明保留于源文件 |

Python 动力学实现由本仓库提供，源文件保留版权和 SPDX 标识。

## 模型与参数

厂家参数、协议与模型来自 [metal-python-ros bccf3c8](https://github.com/makermods-robotics/metal-python-ros/tree/bccf3c8)。原 SDK 的 MIT License（Copyright 2025 MakerMods）保留在 [models/LICENSE](models/LICENSE)，覆盖据其实现的参数、协议、模型和查表数据。

| 内容 | 原 SDK 路径 |
| --- | --- |
| CAN ID、MIT 范围、增益和软限位 | `metal_sdk/native/config/motor_config.cpp` |
| MIT 编码与特殊命令 | `metal_sdk/native/motor_writers/dm_motor_writer.cpp` |
| 反馈布局和归一化分母 | `metal_sdk/native/motor_readers/dm_motor_reader.cpp` |
| 夹爪行程与角度查表 | `metal_sdk/native/can_manager.h` |
| 重力、科氏和摩擦参数 | `metal_sdk/native/kinodynamic/kdl_solver.cpp` |
| 带夹爪 / 无夹爪负载 URDF | `metal_sdk/example/urdf/metal_{with,no}_gripper.urdf` |

URDF 保留于 [models](models)，动力学读取几何与惯量；网格资源不参与计算。模型链为 `base_link → Link6`，默认重力为 `(0, 0, -9.81)` m/s²。

限位与复位参考定义于 [board_defs/metal_arm.py](../../board_defs/metal_arm.py)。运行参数见 [Metal 控制参数](../PC_PORT.md)，跨型号对照见 [机械臂差异](../../docs/ROBOTS.md)。
