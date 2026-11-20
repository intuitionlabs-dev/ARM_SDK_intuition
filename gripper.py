"""夹爪两点标定：原始电机角 deg ↔ 开口距离 m。

初始化成功后记录闭合点和完全张开点，方向与尺度由两点决定。
两点之间线性插值，范围外线性外推，不自动限幅。夹爪零点变化后须重新标定。
ARX X5 原始角负向闭合、正向张开；软限位不能代替实测开口标定。

默认文件为 sdk/config/gripper_calib.json，可用 ARX_GRIPPER_CALIB 覆盖。
标定文件属于本机设备数据，不提交到仓库。
"""

import json
import os
import tempfile
import warnings
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from .errors import CalibrationError, GripperNotCalibrated

CALIB_VERSION = 1

# 标定点过近会放大噪声，保留数据并发出警告。
MIN_POINT_SEPARATION_DEG = 10.0

ENV_CALIB_PATH = "ARX_GRIPPER_CALIB"


def default_calib_path():
    """返回 SDK 目录下的默认标定路径，与工作目录无关。"""
    return Path(__file__).resolve().parent / "config" / "gripper_calib.json"


def calib_path():
    """优先使用环境变量指定的路径，否则使用默认路径。"""
    env = os.environ.get(ENV_CALIB_PATH)
    return Path(env).expanduser() if env else default_calib_path()


@dataclass(frozen=True)
class GripperCalibration:
    """夹爪两点标定：full_open_m 为最大开口，两个 deg 字段为端点原始角。"""

    full_open_m: float
    deg_at_zero: float
    deg_at_full_open: float
    recorded_at: str = ""
    note: str = ""

    # 校验

    def validate(self):
        """无效数据抛 CalibrationError；端点过近时发出警告。"""
        if not self.full_open_m > 0:
            raise CalibrationError(
                "FULL_OPEN 必须 > 0（收到 %r）—— 它是「完全张开时的开口距离」，"
                "单位米。" % (self.full_open_m,))
        sep = self.separation_deg
        if sep < 1e-9:
            raise CalibrationError(
                "两个记录点重合（deg_at_zero=%.6f，deg_at_full_open=%.6f）—— 这样映射不出"
                "任何尺度，标定无效。请把夹爪分别摆到 0 m 和 FULL_OPEN 两处重新记录。"
                % (self.deg_at_zero, self.deg_at_full_open))
        if sep < MIN_POINT_SEPARATION_DEG:
            warnings.warn(
                "夹爪两个记录点只差 %.2f°（< %.0f°）—— 尺度和方向都会被噪声放大，"
                "建议重新标定。" % (sep, MIN_POINT_SEPARATION_DEG))
        return self

    @property
    def separation_deg(self):
        return abs(self.deg_at_full_open - self.deg_at_zero)

    @property
    def direction(self):
        """原始角增大时的开口方向：+1 张开，-1 闭合。"""
        return 1 if self.deg_at_full_open > self.deg_at_zero else -1

    # 换算

    def opening_from_raw_deg(self, raw_deg):
        """原始角 deg 转开口 m；范围外线性外推。"""
        return (self.full_open_m * (raw_deg - self.deg_at_zero)
                / (self.deg_at_full_open - self.deg_at_zero))

    def raw_deg_from_opening(self, opening_m):
        """开口 m 转原始角 deg；范围外线性外推。"""
        return (self.deg_at_zero + (opening_m / self.full_open_m)
                * (self.deg_at_full_open - self.deg_at_zero))

    # 存取

    def to_dict(self):
        return {
            "full_open_m": round(float(self.full_open_m), 9),
            "deg_at_zero": round(float(self.deg_at_zero), 6),
            "deg_at_full_open": round(float(self.deg_at_full_open), 6),
            "recorded_at": self.recorded_at,
            "note": self.note,
        }

    @classmethod
    def from_dict(cls, d):
        if not isinstance(d, dict):
            raise CalibrationError("标定项不是一个 JSON 对象：%r" % (d,))
        try:
            calib = cls(
                full_open_m=float(d["full_open_m"]),
                deg_at_zero=float(d["deg_at_zero"]),
                deg_at_full_open=float(d["deg_at_full_open"]),
                recorded_at=str(d.get("recorded_at", "")),
                note=str(d.get("note", "")),
            )
        except KeyError as e:
            raise CalibrationError("标定项缺字段 %s：%r" % (e, d)) from None
        except (TypeError, ValueError) as e:
            raise CalibrationError("标定项字段不是数：%s（%r）" % (e, d)) from None
        return calib

    @classmethod
    def from_points(cls, full_open_m, deg_at_zero, deg_at_full_open, note="",
                    recorded_at=None):
        """由两点建立标定并校验。"""
        if recorded_at is None:
            recorded_at = datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")
        return cls(full_open_m=float(full_open_m),
                   deg_at_zero=float(deg_at_zero),
                   deg_at_full_open=float(deg_at_full_open),
                   recorded_at=recorded_at,
                   note=note).validate()


class GripperCalibStore:
    """按 arm_id 读写 JSON 标定文件。"""

    def __init__(self, path=None):
        self.path = Path(path) if path is not None else calib_path()

    # 原始 JSON

    def load_raw(self):
        """读取完整标定文件；文件不存在时返回空结构。"""
        if not self.path.exists():
            return {"version": CALIB_VERSION,
                    "units": {"opening": "m", "raw_angle": "deg"},
                    "arms": {}}
        try:
            text = self.path.read_text(encoding="utf-8")
        except OSError as e:
            raise CalibrationError("读不了标定文件 %s：%s" % (self.path, e)) from None
        try:
            data = json.loads(text)
        except ValueError as e:
            raise CalibrationError(
                "标定文件 %s 不是合法 JSON：%s\n（它坏了的话直接删掉重标一次最省事）"
                % (self.path, e)) from None
        if not isinstance(data, dict):
            raise CalibrationError("标定文件 %s 的顶层不是一个 JSON 对象" % self.path)

        data.setdefault("version", CALIB_VERSION)
        data.setdefault("units", {"opening": "m", "raw_angle": "deg"})
        arms = data.setdefault("arms", {})
        if not isinstance(arms, dict):
            raise CalibrationError("标定文件 %s 的 arms 不是一个 JSON 对象" % self.path)
        return data

    # 读一台

    def get(self, arm_id):
        """返回指定臂的标定；缺失返回 None，无效数据抛 CalibrationError。"""
        arms = self.load_raw()["arms"]
        entry = arms.get(str(int(arm_id)))
        if entry is None:
            return None
        calib = GripperCalibration.from_dict(entry)
        calib.validate()
        return calib

    def has(self, arm_id):
        return str(int(arm_id)) in self.load_raw()["arms"]

    # 写一台

    def put(self, arm_id, calib):
        """更新指定臂并原子替换文件；不支持多个进程并发写入。"""
        calib.validate()
        data = self.load_raw()
        data["version"] = CALIB_VERSION
        data.setdefault("units", {"opening": "m", "raw_angle": "deg"})
        data["arms"][str(int(arm_id))] = calib.to_dict()
        self._write_atomic(data)
        return calib

    def _write_atomic(self, data):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=str(self.path.parent),
                                   prefix=self.path.name + ".", suffix=".tmp")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
                f.write("\n")
            os.replace(tmp, self.path)   # 原子替换，避免留下不完整文件。
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    # 对外接口

    def require(self, arm_id, hint=""):
        """返回指定臂的标定；缺失抛 GripperNotCalibrated。"""
        calib = self.get(arm_id)
        if calib is None:
            raise GripperNotCalibrated(self.missing_message(arm_id, hint))
        return calib

    def missing_message(self, arm_id, hint=""):
        tail = ("\n" + hint + "\n") if hint else "\n"
        return ("夹爪还没标定（期望的标定文件是 %s，里面没有 arm_id=%d 那一项）。%s"
                "标定一次：\n"
                "    python3 -m sdk.calibrate_gripper --arm-id %d --full-open-m <完全张开时的"
                "开口距离，米>\n"
                "标定要在**一次成功的初始化之后**做（夹爪的零点由找零给出，"
                "见 sdk/README.md 的「夹爪标定」一节）。"
                % (self.path, arm_id, tail, arm_id))
