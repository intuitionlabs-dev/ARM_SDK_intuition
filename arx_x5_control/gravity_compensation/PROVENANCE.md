# ARX X5 算法与模型来源

## 算法来源

| 实现 | 参考项目与版本 | 提交 | 许可证 |
| --- | --- | --- | --- |
| `solver.py`、`dynamics.py` 的递归牛顿欧拉算法 | [Orocos KDL v1.5.1](https://github.com/orocos/orocos_kinematics_dynamics) | `db25b7e480e068df068232064f2443b8d52a83c7` | LGPL-2.1-or-later，全文见 [LICENSE](LICENSE) |
| `urdf.py` 的坐标语义 | [ROS kdl_parser 1.14.2](https://github.com/ros/kdl_parser) | `74d4ee3bc6938de8ae40a700997baef06114ea1b` | 对应 BSD 版权声明保留于源文件 |

Python 实现使用标准库解析 XML。源文件保留版权和 SPDX 标识。

## 模型与参数

| 项目 | ARX X5 来源或配置 |
| --- | --- |
| URDF 来源 | [ARX X5](https://gitee.com/li-bozha0/ARX_X5)，提交 `c78328785ea23a81d908e1dcc551eca992f2f1e9` |
| 模型目录及许可 | [models](models)、[models/LICENSE](models/LICENSE) |
| 动力学链 | `base_link → link6` |
| 默认重力 | `(0, 0, -9.81)` m/s² |
| SDK 重力系数 | 前三轴 0.8，后三轴 1.32，见 `solver.py` |
| 限位与复位角 | [board_defs/arx_x5.py](../../board_defs/arx_x5.py) |

运行参数见 [ARX 控制参数](../PC_PORT.md)，跨型号对照见 [机械臂差异](../../docs/ROBOTS.md)。
