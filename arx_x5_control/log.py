"""arx_x5_control 专属日志入口，配置仅在本包内全局生效。

各模块通过 get_logger(__name__) 获取日志对象，传入词条键及参数；语言在每次
输出时读取。导入包不会输出日志或改动根日志器。调用 configure 后启用输出。
JSON 结果通过 write_data 输出，保留机器接口，不随显示语言改动字段或枚举。
"""

import argparse
from contextlib import contextmanager
from dataclasses import asdict, dataclass, is_dataclass
from enum import Enum
import json
import logging
import os
import sys
import threading

from .log_messages import MESSAGES, REASON_KEYS

PACKAGE = "arx_x5_control"
LANGUAGES = ("zh", "en")
LEVELS = ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL")
LANGUAGE_ENV = "ARX_X5_CONTROL_LOG_LANGUAGE"
LEVEL_ENV = "ARX_X5_CONTROL_LOG_LEVEL"
_UNSET = object()
_lock = threading.RLock()
_owner = logging.getLogger(PACKAGE)
_owner.propagate = False
_owner.addHandler(logging.NullHandler())
_owner.setLevel(logging.INFO)
_handler = None


@dataclass(frozen=True)
class LogConfig:
    """包级不可变配置快照；修改配置必须调用 configure。"""

    language: str = "zh"
    level: str = "INFO"


_config = LogConfig()


def get_config():
    """返回当前包级配置，不包含其他包的设置。"""
    with _lock:
        return _config


def configure(*, language=None, level=None, stream=_UNSET):
    """设置全包日志；已创建的日志对象和后台线程在下一次输出时使用新配置。

    首次配置默认写入当时的 sys.stderr；后续省略 stream 时保留原输出目标。
    stream=None 禁用本库自带的输出处理器，便于调用者安装自己的 logging.Handler。
    不调用 basicConfig，不修改根日志器和其他包；重复配置不会叠加输出处理器。
    """
    global _config, _handler
    with _lock:
        language = _config.language if language is None else language
        level = _config.level if level is None else (level.upper() if isinstance(level, str) else level)
        if language not in LANGUAGES:
            raise ValueError(f"language must be one of {LANGUAGES}")
        if level not in LEVELS:
            raise ValueError(f"level must be one of {LEVELS}")
        if stream is not _UNSET and stream is not None and not callable(getattr(stream, "write", None)):
            raise TypeError("stream must provide write(), or be None")
        if stream is not _UNSET or _handler is None:
            if _handler is not None:
                _owner.removeHandler(_handler)
                _handler.close()
            _handler = logging.NullHandler() if stream is None else logging.StreamHandler(
                sys.stderr if stream is _UNSET else stream)
            _handler.setFormatter(logging.Formatter("%(levelname)s [%(name)s] %(message)s"))
            _owner.addHandler(_handler)
        _owner.setLevel(getattr(logging, level))
        _config = LogConfig(language, level)
        return _config


def configure_console(*, language=None, level=None):
    """启用当前标准错误流上的包级日志输出。"""
    return configure(language=language, level=level, stream=sys.stderr)


@contextmanager
def subscribe(callback):
    """订阅本包日志；回调必须非阻塞。"""
    class CallbackHandler(logging.Handler):
        def emit(self, record):
            callback(dict(logger=record.name, level=record.levelname, message=record.getMessage(),
                          message_key=getattr(record, "message_key", None),
                          thread=record.threadName, native_thread_id=threading.get_native_id()))
    handler = CallbackHandler()
    _owner.addHandler(handler)
    try:
        yield
    finally:
        _owner.removeHandler(handler)
        handler.close()


def text(key, *, language=None, **values):
    """渲染本包词条；可指定语言，通常由当前包级配置决定。"""
    chosen = get_config().language if language is None else language
    return MESSAGES[chosen][key].format(**values)


class PackageLogger:
    """只接受本包词条的日志对象；模块名称必须属于 arx_x5_control。"""

    def __init__(self, name):
        if name != PACKAGE and not name.startswith(PACKAGE + "."):
            raise ValueError(f"logger must belong to {PACKAGE}: {name}")
        self.name = name
        self._logger = logging.getLogger(name)

    def _write(self, level, key, values, *, language=None, event=None):
        if not self._logger.isEnabledFor(level):
            return
        chosen = get_config().language if language is None else language
        message = text(key, language=chosen, **values)
        self._logger.log(level, message, extra={
            "package": PACKAGE, "message_key": key, "language": chosen,
            "message_values": dict(values), "control_event": event,
        }, stacklevel=3)

    def debug(self, key, **values):
        self._write(logging.DEBUG, key, values)

    def info(self, key, **values):
        self._write(logging.INFO, key, values)

    def warning(self, key, **values):
        self._write(logging.WARNING, key, values)

    def error(self, key, **values):
        self._write(logging.ERROR, key, values)

    def critical(self, key, **values):
        self._write(logging.CRITICAL, key, values)

    def event(self, event):
        """输出控制事件，保持原事件不变，整条消息使用同一份语言快照。"""
        level = _EVENT_LEVELS.get(event.kind, logging.INFO)
        if not self._logger.isEnabledFor(level):
            return
        chosen = get_config().language
        key, values = _event_message(event, chosen)
        self._write(level, key, values, language=chosen, event=event)

    def events(self, events):
        for event in events:
            self.event(event)


def get_logger(name=PACKAGE):
    return PackageLogger(name)


_EVENT_LEVELS = {
    "connection_required": logging.DEBUG,
    "connected": logging.DEBUG,
    "rejected": logging.WARNING,
    "auto_aborted": logging.WARNING,
    "watchdog": logging.WARNING,
    "initialization_failed": logging.ERROR,
    "gravity_error": logging.ERROR,
    "dynamics_error": logging.ERROR,
    "reference_error": logging.ERROR,
}


def _event_message(event, language):
    catalog = MESSAGES[language]
    key = "event." + event.kind
    values = {"arm_id": event.arm_id}
    if key not in catalog:
        return "event.unknown", dict(values, kind=event.kind, detail=event.detail)
    if event.kind == "state":
        values["state"] = catalog.get("state." + event.detail, event.detail)
    elif event.kind in ("rejected", "auto_aborted"):
        reason = event.detail
        if event.kind == "rejected":
            command, _, reason = reason.partition(": ")
            values["command"] = catalog.get("command." + command, command)
        values["reason"] = catalog.get(REASON_KEYS.get(reason), reason)
    elif event.kind == "calibrated":
        state = getattr(event.zero_state, "name", str(event.zero_state))
        values["state"] = catalog.get("zero_state." + state, state)
        values["results"] = ", ".join(catalog.get("zero_result." + r.name, r.name)
                                      for r in event.zero_results)
        values["elapsed_ms"] = event.elapsed_ms
    elif event.kind == "dynamics_error":
        values["status"] = catalog.get("dynamics_status." + event.detail, event.detail)
    return key, values


def event_text(event, *, language=None):
    """将控制事件转换为显示文本。"""
    chosen = get_config().language if language is None else language
    key, values = _event_message(event, chosen)
    return text(key, language=chosen, **values)


def _json_default(value):
    if isinstance(value, Enum):
        return value.name
    if is_dataclass(value):
        return asdict(value)
    raise TypeError(f"not JSON serializable: {type(value).__name__}")


def write_data(value, *, sink=None):
    """输出一行机器可读 JSON；sink 接收完整字符串，兼容原 demo 的 emit 回调。"""
    line = json.dumps(value, default=_json_default, ensure_ascii=False, allow_nan=False)
    if sink is not None:
        sink(line)
    else:
        sys.stdout.write(line + "\n")
        sys.stdout.flush()


class ArgumentParser(argparse.ArgumentParser):
    """命令行运行错误统一通过本包日志库输出；帮助文本仍由 argparse 排版。"""

    def error(self, message):
        get_logger(PACKAGE + ".cli").error("cli.invalid_arguments", error=message)
        self.exit(2)


def argument_parser(description_key, argv=None):
    """先读取语言和级别，再生成本包入口的帮助文本。

    优先级：命令行参数 > 包专属环境变量 > 已有包级配置。返回 parser，业务入口
    随后添加自身选项并调用 parse_args；本函数不改变其他包的语言或输出设置。
    """
    current = get_config()
    configure()
    preliminary = ArgumentParser(add_help=False, allow_abbrev=False)
    preliminary.add_argument("--lang", choices=LANGUAGES,
                             default=os.environ.get(LANGUAGE_ENV, current.language))
    preliminary.add_argument("--log-level", choices=LEVELS,
                             default=os.environ.get(LEVEL_ENV, current.level))
    options, _ = preliminary.parse_known_args(argv)
    try:
        configure(language=options.lang, level=options.log_level)
    except (TypeError, ValueError) as exc:
        preliminary.error(str(exc))
    parser = ArgumentParser(description=text(description_key), allow_abbrev=False)
    parser.add_argument("--lang", choices=LANGUAGES, default=options.lang,
                        help=text("cli.language_help"))
    parser.add_argument("--log-level", choices=LEVELS, default=options.log_level,
                        help=text("cli.level_help"))
    return parser
