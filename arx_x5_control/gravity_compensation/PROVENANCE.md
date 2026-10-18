# 动力学代码与模型来源

## Python 动力学实现

`solver.py` 和 `dynamics.py` 提供重力与逆动力学计算。
算法使用 Orocos KDL 的递归牛顿欧拉方法。
参考版本为 Orocos KDL v1.5.1。
参考提交为 `db25b7e480e068df068232064f2443b8d52a83c7`。
项目地址：<https://github.com/orocos/orocos_kinematics_dynamics>。

Python 实现采用 LGPL-2.1-or-later。
许可证全文位于 `LICENSE`。
源文件保留版权和 SPDX 标识。

`urdf.py` 使用 Python XML 标准库解析模型。
坐标语义参考 ROS kdl_parser 1.14.2。
参考提交为 `74d4ee3bc6938de8ae40a700997baef06114ea1b`。
项目地址：<https://github.com/ros/kdl_parser>。
源文件保留对应的 BSD 版权声明。

## ARX X5 模型和参数

URDF 模型位于 `models/`。
来源为 ARX X5 仓库：<https://gitee.com/li-bozha0/ARX_X5>。
参考提交为 `c78328785ea23a81d908e1dcc551eca992f2f1e9`。
模型许可证位于 `models/LICENSE`。

计算使用 `base_link` 到 `link6` 的六轴链。
默认重力为 `(0, 0, -9.81)` m/s²。
SDK 力矩缩放系数为前三轴 0.8、后三轴 1.32。
参数定义见 `solver.py`。
