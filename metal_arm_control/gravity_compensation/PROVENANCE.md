# 动力学代码、模型与参数来源

Python `solver.py` / `dynamics.py` / `spatial.py` / `urdf.py` 沿用 ARM_SDK_intuition 的实现；原算法为 Orocos KDL v1.5.1（db25b7e480e068df068232064f2443b8d52a83c7），LGPL-2.1-or-later，许可证保留于本目录 LICENSE。URDF 解析参考 ROS kdl_parser 1.14.2（74d4ee3bc6938de8ae40a700997baef06114ea1b）。

Metal 参数与模型来自：
https://github.com/makermods-robotics/metal-python-ros/tree/bccf3c8

- `metal_sdk/native/config/motor_config.cpp`：CAN IDs、MIT 范围、增益和软限位。
- `metal_sdk/native/motor_writers/dm_motor_writer.cpp`：MIT 编码与特殊命令。
- `metal_sdk/native/motor_readers/dm_motor_reader.cpp`：反馈布局和归一化分母。
- `metal_sdk/native/can_manager.h`：夹爪行程/角度查表。
- `metal_sdk/native/kinodynamic/kdl_solver.cpp`：重力、科氏和摩擦参数。
- `metal_sdk/example/urdf/metal_{with,no}_gripper.urdf`：原样保留，无网格资源；动力学只读取几何/惯量。

原厂 SDK 的 MIT License（Copyright 2025 MakerMods）保留在 `models/LICENSE`，同样覆盖本包据其实现移植的参数、协议、模型和查表数据。

模型链为 `base_link → Link6`，世界重力为 `(0,0,-9.81)`。SDK 缩放时前馈为 scaled G + scaled C + viscous friction，不使用加速度前馈。纯模型模式仍可计算 M*qdd+C+G。没有把 ARX 的 0.8/1.32 缩放带入 Metal。

复位六轴零角、250 Hz 调度、0.16 s 关节参考轨迹和状态机是本仓库接口/控制策略，不是厂家的收纳姿态或 400 Hz 控制循环。镜像结构来自 ARM_SDK_intuition 的 f6937f0。
