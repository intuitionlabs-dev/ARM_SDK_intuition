# 日志

```python
from metal_arm_control import log
log.configure(language="zh", level="INFO")
log.configure(language="en")
```

日志配置只作用于 `metal_arm_control`。环境变量为 `METAL_ARM_CONTROL_LOG_LANGUAGE` / `METAL_ARM_CONTROL_LOG_LEVEL`，CLI 接受 `--lang`、`--log-level`。JSON 输出不随语言改变字段。

TUI 使用 `sdk_adapter.telemetry.RuntimeRecorder`，运行记录默认保存在 `$XDG_STATE_HOME/metal-arm-control/runs`（未设置时使用 `~/.local/state`）。不会复用 ARX 的日志目录。
