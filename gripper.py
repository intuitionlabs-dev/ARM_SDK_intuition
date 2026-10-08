"""夹爪：**电机原始角（deg） ↔ 开口距离（m）** 的两点标定模型 + JSON 存取。

## 为什么是两点标定

板子线上的 `gripper_angle` 是**电机原始角**（deg），双向都不做 `pos_to_angle` /
`MOTOR_DIR` / `MOTOR_REDUCTION` 那套换算 —— 那几张表只有 6 个关节有，夹爪是直接透传的
（见 `App/gimbal_arm.c` 的 `Set_Motor_ref()`）。所以"单位是电机的 deg"这件事是确定的，
而"1 deg 对应多少米开口"**全仓库没有任何规格**：翻遍了只有一个 ±142° 的角度口径
（`GRIPPER_ANGLE_*`，找零之后零点落在行程中点），没有行程的毫米/米数。

既然没有真值可抄，就用**实测**定：`FULL_OPEN`（正向完全张开的距离）是用户预先配置的参数，
用户在臂初始化完成后手动把夹爪摆到 `FULL_OPEN` 和 `0 m` 两处，程序把这两处的原始电机角记
下来、存进配置文件。之后按这两点线性映射：

    opening_m = full_open_m * (raw_deg - deg_at_zero) / (deg_at_full_open - deg_at_zero)
    raw_deg   = deg_at_zero + (opening_m / full_open_m) * (deg_at_full_open - deg_at_zero)

**方向和尺度完全由这两个记录点决定**：代码里不预设哪端是闭合、也不假设 deg 是减小还是增大
对应张开。记录点反了就自动反号，所以换一台夹爪、甚至把夹爪装反了都不用改代码，重标一次即可。

## 三个必须知道的边界

1. **两点之间是线性假设**。真实行程可能非线性（连杆式夹爪尤其），超出两点范围会**线性外推
   而不是夹住** —— 外推出来的数只是工程近似，别当精密测量用。
2. **原始角的零点由"找零"给出**。夹爪的零点是行程中点（∓142），而那个零点只有
   "推到限位 → 退回半个行程"那次找零（0x02 初始化里带的、或一键夹爪找零）才会建立 ——
   在那之前 `raw_deg` 的零点是没有含义的。所以**标定必须在一次成功的初始化之后做**；
   之后再跑任何设零（0x10）或再找零，零点会挪，**标定作废，要重标**。
3. **未标定时 `opening_m` 这条路直接抛异常**，不返回一个猜的数（见 `GripperNotCalibrated`）；
   原始角那条路（`raw_gripper_angle_deg` / `gripper_raw_deg=`）任何时候都能用。

## 文件

默认 `sdk/config/gripper_calib.json`，可用环境变量 `ARX_GRIPPER_CALIB` 覆盖（测试用）。
仓库的 `.gitignore` 全局忽略 `*.json`，所以这个文件天然不进版本库 —— 它是**每台机器一份**
的实测数据，本来就不该提交（换一台夹爪、拆装一次就可能变）。
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

# 两个记录点离得太近时映射会放大噪声 —— 不禁止，但要吵一句。
MIN_POINT_SEPARATION_DEG = 10.0

ENV_CALIB_PATH = "ARX_GRIPPER_CALIB"


def default_calib_path():
    """`sdk/config/gripper_calib.json`（相对本文件定位，跟当前工作目录无关）。"""
    return Path(__file__).resolve().parent / "config" / "gripper_calib.json"


def calib_path():
    """实际用的路径：环境变量优先，其次默认。"""
    env = os.environ.get(ENV_CALIB_PATH)
    return Path(env).expanduser() if env else default_calib_path()


@dataclass(frozen=True)
class GripperCalibration:
    """一台臂的夹爪标定：两个记录点 + 预先配置的 FULL_OPEN。

    - `full_open_m`     完全张开时的开口距离（m，> 0）。**预先配置的参数**，不是测出来的。
    - `deg_at_zero`     开口 = 0 m 处记录的原始电机角（deg）。
    - `deg_at_full_open` 开口 = FULL_OPEN 处记录的原始电机角（deg）。
    - `recorded_at` / `note` 记账用，不参与换算。
    """

    full_open_m: float
    deg_at_zero: float
    deg_at_full_open: float
    recorded_at: str = ""
    note: str = ""

    # ---- 校验 ----

    def validate(self):
        """不合法就抛 `CalibrationError`；可疑但不致命只 `warnings.warn`。"""
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
        """原始角增大是张开还是闭合：+1 = 角度越大越张开，-1 = 角度越大越闭合。

        只用来在工具/日志里提示"装反了没有"，换算本身不依赖它（两点已经把方向定死了）。
        """
        return 1 if self.deg_at_full_open > self.deg_at_zero else -1

    # ---- 换算 ----

    def opening_from_raw_deg(self, raw_deg):
        """原始电机角（deg）→ 开口距离（m）。两点之外是**线性外推**，不夹。"""
        return (self.full_open_m * (raw_deg - self.deg_at_zero)
                / (self.deg_at_full_open - self.deg_at_zero))

    def raw_deg_from_opening(self, opening_m):
        """开口距离（m）→ 原始电机角（deg）。"""
        return (self.deg_at_zero + (opening_m / self.full_open_m)
                * (self.deg_at_full_open - self.deg_at_zero))

    # ---- 存取 ----

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
        """从两个记录点建一份，并**当场校验**（不合法直接抛）。"""
        if recorded_at is None:
            recorded_at = datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")
        return cls(full_open_m=float(full_open_m),
                   deg_at_zero=float(deg_at_zero),
                   deg_at_full_open=float(deg_at_full_open),
                   recorded_at=recorded_at,
                   note=note).validate()


class GripperCalibStore:
    """`sdk/config/gripper_calib.json` 的读写。一台臂一项，键是 arm_id 的字符串形式。"""

    def __init__(self, path=None):
        self.path = Path(path) if path is not None else calib_path()

    # ---- 原始 JSON ----

    def load_raw(self):
        """整份文件。文件不存在 → 空骨架（不是错误：还没标过而已）。"""
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

    # ---- 读一台 ----

    def get(self, arm_id):
        """这一台臂的标定。**没标过返回 None**；文件在但这一项坏掉 → 抛 `CalibrationError`。

        坏掉的那一项不静默跳过：那是"标定存在但不可用"，和"没标过"是两回事，糊在一起会让
        人以为标定丢了。
        """
        arms = self.load_raw()["arms"]
        entry = arms.get(str(int(arm_id)))
        if entry is None:
            return None
        calib = GripperCalibration.from_dict(entry)
        calib.validate()
        return calib

    def has(self, arm_id):
        return str(int(arm_id)) in self.load_raw()["arms"]

    # ---- 写一台 ----

    def put(self, arm_id, calib):
        """写一台的标定（读-改-写整份文件，**原子替换**，所以不会写坏别人的那一项）。

        两个并发进程同时写会互相覆盖 —— 标定是手工操作，不值得为它加锁。
        """
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
            os.replace(tmp, self.path)   # 原子：要么旧内容、要么新内容，不会是半份
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    # ---- 给外部用的合成入口 ----

    def require(self, arm_id, hint=""):
        """拿一台的标定，**没标过就抛 `GripperNotCalibrated`**（消息里带怎么标）。

        `arm.py` 的 `gripper_opening` 走的就是这条路。
        """
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
