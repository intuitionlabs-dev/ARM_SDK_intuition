# ARX X5 日志配置

日志 API、会话文件、统计字段及二进制记录格式统一见 [PC 日志说明](../docs/LOGGING.md)。

| 项目 | ARX X5 配置 |
| --- | --- |
| 日志模块 | `arx_x5_control.log` |
| 语言环境变量 | `ARX_X5_CONTROL_LOG_LANGUAGE` |
| 日志级别环境变量 | `ARX_X5_CONTROL_LOG_LEVEL` |
| 默认目录 | `~/.local/state/arx-x5-control/runs` |
| 自定义目录 | `arx-x5-tui --log-dir PATH` |

设置 `XDG_STATE_HOME` 时替换默认目录中的 `~/.local/state`。配置只作用于所选型号包；协议码与安装包版本的含义见 [版本显示](../docs/TUI.md#版本显示)。

## TUI 运行日志

启动命令见 [公共 TUI 入口](../docs/TUI.md#启动入口)，文件内容及性能采样见 [会话记录格式](../docs/LOGGING.md#tui-运行日志)。
