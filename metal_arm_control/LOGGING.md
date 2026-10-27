# 日志

```python
from metal_arm_control import log
log.configure(language="zh", level="INFO")
log.configure(language="en")
```

日志配置只作用于 `metal_arm_control`。环境变量为 `METAL_ARM_CONTROL_LOG_LANGUAGE` / `METAL_ARM_CONTROL_LOG_LEVEL`，CLI 接受 `--lang`、`--log-level`。JSON 输出不随语言改变字段。

TUI 使用 `sdk_adapter.telemetry.RuntimeRecorder`，运行记录默认保存在 `$XDG_STATE_HOME/metal-arm-control/runs`（未设置时使用 `~/.local/state`）。不会复用 ARX 的日志目录。

```bash
metal-arm-tui --simulate --lang zh --log-dir /tmp/metal-arm-runs
```

每次运行创建独立目录，包含 `metadata.json`、`metrics.jsonl`、`events.jsonl`、`runtime.log`、`ui.log`、`summary.json`、`control_trace.bin` 和对应 schema。字段及单位与 [ARX 记录格式](../arx_x5_control/LOGGING.md#tui-运行日志) 对应，包名、环境变量和默认路径使用 Metal 自己的配置。

`metadata.json` 的 `package_version` 读取 `arx-x5-control` 的安装版本（当前 `0.6.0`，无发行元数据时为 `source`）；`source_sha256` 保存记录器选取的源码文件哈希。TUI 顶部 `vcode` 是公共 SDK 协议哈希，PC 后端本地填充握手版本，它不代表电机固件版本。确认完整源码提交使用 `git rev-parse --short HEAD`，版本说明见 [公共手册](../README.md#tui-版本显示)。
