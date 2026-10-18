"""从 JSON 配置加载 CAN 传输后端。"""

from copy import deepcopy
from dataclasses import dataclass
from importlib import import_module, resources
import json
import os
from pathlib import Path


class TransportConfigError(ValueError):
    """传输配置无效或指定后端不可用。"""


@dataclass(frozen=True)
class TransportSpec:
    class_path: str
    channel_kind: str
    options: dict


class TransportRegistry:
    """持有一份已校验的传输配置快照。"""

    def __init__(self, config_path=None):
        selected = config_path if config_path is not None else os.environ.get("USB2CAN_CONFIG")
        self.config_path = str(Path(selected).expanduser().resolve()) if selected is not None else None
        try:
            source = (Path(self.config_path).read_text(encoding="utf-8") if self.config_path is not None
                      else resources.files(__package__).joinpath("transports.json").read_text(encoding="utf-8"))
            data = json.loads(source, object_pairs_hook=self._unique_keys, parse_constant=self._invalid_constant)
        except (OSError, ValueError) as error:
            raise TransportConfigError(f"cannot read transport configuration: {selected or 'arx_x5_control/usb2can/transports.json'}: {error}") from error
        if not isinstance(data, dict) or set(data) != {"version", "transports"} or type(data["version"]) is not int or data["version"] != 1:
            raise TransportConfigError("transport configuration requires version=1 and transports")
        entries = data["transports"]
        if not isinstance(entries, dict) or not entries:
            raise TransportConfigError("transports must be a nonempty object")
        self._specs = {}
        for name, entry in entries.items():
            if not isinstance(name, str) or not name or name.strip() != name:
                raise TransportConfigError("transport names must be nonempty and have no surrounding whitespace")
            if not isinstance(entry, dict) or set(entry) != {"class", "channel_kind", "options"}:
                raise TransportConfigError(f"transport {name}: requires class, channel_kind and options")
            class_path, kind, options = entry["class"], entry["channel_kind"], entry["options"]
            if not isinstance(class_path, str) or class_path.count(":") != 1 or not all(
                    part and all(piece.isidentifier() for piece in part.split(".")) for part in class_path.split(":")):
                raise TransportConfigError(f"transport {name}: class must use module:attribute syntax")
            if kind not in ("path", "name"):
                raise TransportConfigError(f"transport {name}: channel_kind must be path or name")
            if not isinstance(options, dict) or not all(isinstance(key, str) and key.isidentifier() for key in options):
                raise TransportConfigError(f"transport {name}: options must contain keyword arguments")
            if {"channel", "bitrate"} & options.keys():
                raise TransportConfigError(f"transport {name}: channel and bitrate belong to channel configuration")
            self._specs[name] = TransportSpec(class_path, kind, deepcopy(options))

    @staticmethod
    def _invalid_constant(value):
        raise TransportConfigError(f"nonfinite configuration value: {value}")

    @staticmethod
    def _unique_keys(pairs):
        output = {}
        for key, value in pairs:
            if key in output:
                raise TransportConfigError(f"duplicate configuration key: {key}")
            output[key] = value
        return output

    @property
    def names(self):
        return tuple(self._specs)

    def specification(self, name):
        try:
            spec = self._specs[name]
        except KeyError as error:
            raise TransportConfigError(f"unknown transport: {name}; configured: {', '.join(self.names)}") from error
        return TransportSpec(spec.class_path, spec.channel_kind, deepcopy(spec.options))

    def endpoint(self, interface, channel):
        spec = self.specification(interface)
        return os.path.realpath(channel) if spec.channel_kind == "path" else channel

    def create(self, interface, channel, *, bitrate=1000000):
        spec = self.specification(interface)
        module_name, attribute = spec.class_path.split(":")
        try:
            factory = import_module(module_name)
            for name in attribute.split("."):
                factory = getattr(factory, name)
        except (ImportError, AttributeError) as error:
            raise TransportConfigError(f"cannot load transport {interface}: {spec.class_path}: {error}") from error
        if not callable(factory):
            raise TransportConfigError(f"transport {interface}: configured class is not callable")
        for method in ("send", "receive", "close"):
            if not callable(getattr(factory, method, None)):
                raise TransportConfigError(f"transport {interface}: class requires {method}()")
        return factory(channel, bitrate=bitrate, **spec.options)
