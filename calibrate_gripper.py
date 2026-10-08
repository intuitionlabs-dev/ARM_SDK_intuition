"""夹爪标定小工具：把"开口距离（m）"和"电机原始角（deg）"这两把尺子对起来。

    python3 -m sdk.calibrate_gripper --arm-id 0 --full-open-m 0.085

为什么需要人肉标定，见 `sdk/gripper.py` 的模块注释：板子线上的夹爪角是**电机原始角**，
仓库里没有任何"多少度对应多少毫米"的规格，所以只能实测 —— 把夹爪摆到 `FULL_OPEN` 和
`0 m` 两处，各记一次原始角，之后按这两点线性映射。

## 前提：标定必须在**一次成功的初始化之后**做

夹爪的零点落在行程中点（∓142°），而这个零点只有找零（`0x02` 里带的夹爪找零）才会建立。
在那之前 `raw_deg` 的零点没有含义，标出来的两点是废的。工具会去查
`motor_inited` 里夹爪那一位，为 0 就警告。之后**再跑任何 `0x10` 或再找零，零点会挪，
标定作废要重标**。

## 怎么把夹爪挪到那两处

两条路，按 `r` / `d` / `x` 在三个档位之间切：

- **手掰**（推荐，`d`=阻尼档 / `x`=失能档）：夹爪松劲或只剩粘滞，直接上手掰到位置。
  夹爪不受重力，所以不存在"一松手就掉下来"的问题。
- **点动**（`r`=run 档）：`←` `→` 每次挪一个步长，工具以 50 Hz 发控制帧让位置环跟过去。
  run 档下**整条臂都是使能的**，关节会试图保持当前姿态 —— 只在手不好够的时候用。

`[` `]` 改步长。到了位置按 `0` 记"这里是 0 m"、按 `f` 记"这里是 FULL_OPEN"。两点都记齐了
按**回车**存盘（写到 `sdk/config/gripper_calib.json`，可用 `--calib` 或环境变量
`ARX_GRIPPER_CALIB` 改）。`q` 不存盘退出。

⚠ 这个工具的**按键会真的让电机使能/失能**（`r`/`d`/`x` 发 0x12/0x15/0x14），并且 run 档下
的 `←`/`→` 会真的驱动夹爪。所以它是个"要看着机器"的工具，别在无人看管的时候挂后台。
"""

import argparse
import os
import select
import sys
import termios
import time

from . import board_defs as bd
from .arm import FEEDBACK_STALE_S, Arm
from .errors import ArError, CalibrationError
from .gripper import GripperCalibStore, GripperCalibration
from .link import Link

# 点动时控制帧的发帧率。板子上夹爪的位置环很软，50 Hz 足够跟得平顺；
# 再快只是白占链路。
JOG_HZ = 50.0

# 不点动时也要喂链路：板子 500 ms 收不到有效帧就判通信离线、切阻尼。
ALIVE_HZ = 5.0

# 画面刷新率（别每圈都重画，闪）。
DRAW_HZ = 5.0

STEP_MIN_DEG = 0.1
STEP_MAX_DEG = 45.0
STEP_DEFAULT_DEG = 2.0

# 点动时对"反馈有多新"的要求。比 SDK 闸门（100 ms）稍紧一点，否则会卡在边界上：
# 闸门是 `age > 100 ms 就抛 FeedbackStale`，正好等于 100 ms 的那一拍还是会抛。
JOG_FRESH_S = FEEDBACK_STALE_S * 0.8

# 单字节按键。方向键是 Vt100 的 ESC 序列，单列出来。
SINGLES = {
    b"0": "zero",
    b"f": "full",
    b"[": "step_smaller",
    b"]": "step_bigger",
    b"r": "mode_run",
    b"d": "mode_damping",
    b"x": "mode_stop",
    b"p": "status",
    b"?": "help",
    b"h": "help",
    b"\r": "save",
    b"\n": "save",
    b"q": "quit",
    b"\x03": "quit",      # Ctrl-C：cbreak 下它是普通字节，自己处理才收得到
    b"\x04": "quit",      # Ctrl-D
}
ESCAPES = {
    b"\x1b[C": "right", b"\x1b[D": "left",     # 标准 CSI
    b"\x1bOC": "right", b"\x1bOD": "left",     # 某些终端发 SS3
    b"\x1b[A": "up", b"\x1b[B": "down",
}

HELP = """按键：
  ←  →      点动夹爪：← 让原始角**变大**一个步长，→ 变小（只在 run 档有效）
  [  ]      步长 减半 / 加倍          p  打印完整状态一屏
  0         **记下当前位置 = 0 m**     f   **记下当前位置 = FULL_OPEN**
  回车      存盘（两点都记齐了才行）  q   不存盘退出
  r / d / x 切到 run / damping / stop 档（r 之前请确认臂周围是安全的）
  ?  h      显示这段帮助

哪边是"张开"由板子上的实际装配决定，工具不预设 —— 所以点动按的是**原始角**的方向，
不是"张开/闭合"的方向。记完两点按 p 看一眼方向对不对（角差和方向都会打出来）。
"""


class Calibrator:
    def __init__(self, arm, store, full_open_m, note=""):
        self.arm = arm
        self.store = store
        self.full_open_m = full_open_m
        self.note = note

        self.step_deg = STEP_DEFAULT_DEG
        self.target_deg = None      # 点动目标（只认原始 deg）
        self.deg_at_zero = None
        self.deg_at_full_open = None

        self._last_tx = 0.0
        self._last_draw = 0.0
        self._stale_warned = False
        self._buf = b""
        self._line_shown = False
        self._note_line = ""        # 显示在画面下方的一次性提示（错误、保存结果……）
        self.quitting = False

    # ==================== 输入 ====================

    def _read_keys(self):
        """把 stdin 上现在能读到的字节全读出来，切成按键记号。"""
        while True:
            try:
                r, _, _ = select.select([sys.stdin], [], [], 0)
            except InterruptedError:
                continue
            if not r:
                break
            try:
                chunk = os.read(sys.stdin.fileno(), 64)
            except OSError:
                chunk = b""
            if not chunk:           # EOF：管道的另一头关了
                self.quitting = True
                return []
            self._buf += chunk

        keys = []
        while self._buf:
            for seq, name in ESCAPES.items():
                if self._buf.startswith(seq):
                    keys.append(name)
                    self._buf = self._buf[len(seq):]
                    break
            else:
                if self._buf[:1] == b"\x1b" and len(self._buf) < 4:
                    break           # 序列还没收全，留着下圈再拼
                one, self._buf = self._buf[:1], self._buf[1:]
                keys.append(SINGLES.get(one, "other"))
        return keys

    # ==================== 动作 ====================

    def _say(self, text):
        self._note_line = text

    def _raw_deg(self):
        return self.arm.raw_gripper_angle_deg

    def _jog(self, direction):
        if self.target_deg is None:
            self._say("先按 r 进 run 档，点动才有位置环跟着。")
            return
        before = self.target_deg
        self.target_deg = _clip(self.target_deg + direction * self.step_deg,
                                bd.GRIPPER_ANGLE_MIN, bd.GRIPPER_ANGLE_MAX)
        if self.target_deg == before:
            self._say("到头了（软限位 %+.1f..%+.1f°）。"
                      % (bd.GRIPPER_ANGLE_MIN, bd.GRIPPER_ANGLE_MAX))

    def _set_mode(self, tid_name):
        try:
            getattr(self.arm, tid_name)()
        except ArError as e:
            self._say("%s 没发出去：%s" % (tid_name, _first_line(e)))
            return
        self._say("已发 %s（板子接受与否看下一帧的 mode）。" % tid_name)
        if tid_name == "start":
            self.target_deg = self._raw_deg()      # 进 run 档：目标对齐当前，别突然跳

    def _record(self, which):
        raw = self._raw_deg()
        if raw is None:
            self._say("还没收到反馈帧，读不到夹爪原始角。")
            return
        if which == "zero":
            self.deg_at_zero = raw
        else:
            self.deg_at_full_open = raw
        self._say("记下 %s = %+.3f°（原始电机角）"
                  % ("0 m" if which == "zero" else "FULL_OPEN", raw))

    def _save(self):
        if self.deg_at_zero is None or self.deg_at_full_open is None:
            self._say("两点还没记齐（0 m=%s，FULL_OPEN=%s）—— 都记了才能存。"
                      % (_fmt_pt(self.deg_at_zero), _fmt_pt(self.deg_at_full_open)))
            return False
        if self.full_open_m is None:
            self._say("不知道 FULL_OPEN 是多少米：加 --full-open-m，或让文件里先有一份标定。")
            return False
        try:
            calib = self.store_put()
        except CalibrationError as e:
            self._say("存不下：%s" % _first_line(e))
            return False
        except OSError as e:
            self._say("写文件失败：%s" % e)
            return False
        self._say("已存到 %s：两点的原始角差 %.2f°（方向 %s）。" %
                  (self.store.path, calib.separation_deg,
                   "角大→张开" if calib.direction > 0 else "角小→张开"))
        return True

    def store_put(self):
        calib = GripperCalibration.from_points(
            self.full_open_m, self.deg_at_zero, self.deg_at_full_open,
            note=self.note)
        return self.store.put(self.arm.arm_id, calib)

    def _status(self):
        s = self.arm.snapshot()
        if s is None:
            print("\n（还没收到任何反馈帧 —— 板子在发吗？口子选对了吗？）")
            return
        print("\n" + "-" * 68)
        print("  %s" % s)
        print("  夹爪原始角   %+.3f°（= %+.4f rad）"
              % (s.gripper_raw_deg, self.arm.raw_gripper_angle_rad or 0.0))
        print("  夹爪电机     motor_inited=%s   lost=%s"
              % ("是" if s.motor_inited_mask >> bd.GRIPPER & 1 else "**否**", s.lost_names))
        print("  记录点       0 m = %s   FULL_OPEN(%s) = %s"
              % (_fmt_pt(self.deg_at_zero),
                 ("%.4f m" % self.full_open_m) if self.full_open_m else "**没给**",
                 _fmt_pt(self.deg_at_full_open)))
        print("-" * 68)

    # ==================== 周期 ====================

    def _tick(self):
        now = time.monotonic()
        period = 1.0 / (JOG_HZ if self._jogging() else ALIVE_HZ)
        if now - self._last_tx < period:
            return
        self._last_tx = now
        try:
            self._send_once()
        except ArError as e:
            self._say("%s" % _first_line(e))

    def _jogging(self):
        """这一拍该发**控制帧**了吗（否则只喂保活）。

        反馈新不新也要查：夹爪目标是拿**当前关节角**当基准发的，反馈断了就不知道该把关节
        定在哪、而且 `send_ctrl_pos` 自己也会抛 `FeedbackStale`。与其每拍去撞那个异常，
        不如自己先停手 —— 停手时降回保活的节奏，别拿 50 Hz 白占链路。
        """
        if self.target_deg is None:
            return False
        if self.arm.mode != bd.MODE_RUN or not self.arm.link_up:
            return False
        return self.arm.feedback is not None and self.arm.feedback_age <= JOG_FRESH_S

    def _send_once(self):
        if self._jogging():
            s = self.arm.snapshot()
            self.arm.send_ctrl_pos(s.joint_angles, gripper_raw_deg=self.target_deg)
            self._stale_warned = False
            return
        # 不点动也要喂：0x01 是板子认的有效帧，纯保活用（500 ms 收不到就判通信离线）。
        self.arm.keepalive()
        age = self.arm.feedback_age
        if (self.target_deg is not None and age is not None
                and age > JOG_FRESH_S and not self._stale_warned):
            self._stale_warned = True
            self._say("反馈已经 %.0f ms 没更新了 —— 暂停点动、只喂保活。先看链路"
                      "（link_up=%s），别拿着旧姿态继续发位置。"
                      % (age * 1000.0, self.arm.link_up))

    # ==================== 画面 ====================

    def _draw(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_draw < 1.0 / DRAW_HZ:
            return
        self._last_draw = now
        s = self.arm.snapshot()
        if s is None:
            left = "[%s] 等反馈…" % bd.arm_label(self.arm.arm_id)
            right = ""
        else:
            raw = s.gripper_raw_deg
            opening = ("%.4f m" % (self.full_open_m * (raw - self.deg_at_zero)
                                   / (self.deg_at_full_open - self.deg_at_zero))
                       if (self.deg_at_zero is not None
                           and self.deg_at_full_open is not None
                           and self.deg_at_full_open != self.deg_at_zero
                           and self.full_open_m) else "--")
            tgt = ""
            if self.target_deg is not None:
                tgt = "  目标 %+.3f°%s" % (self.target_deg,
                                           "" if self._jogging() else " ⏸")
            left = ("[%s] %s / %s  raw %+8.3f°  ≈%s  step %.2f°%s"
                    % (bd.arm_label(self.arm.arm_id), s.robot_state_name, s.mode_name,
                       raw, opening, self.step_deg, tgt))
            right = "  0m:%s  F:%s" % (_fmt_pt(self.deg_at_zero),
                                       _fmt_pt(self.deg_at_full_open))
        sys.stdout.write("\r\x1b[K" + left + right)
        sys.stdout.flush()
        self._line_shown = True

    def _clear_line(self):
        if self._line_shown:
            sys.stdout.write("\r\x1b[K")
            sys.stdout.flush()
            self._line_shown = False

    def _flush_note(self):
        if not self._note_line:
            return
        self._clear_line()
        print("  · %s" % self._note_line)
        self._note_line = ""

    # ==================== 主循环 ====================

    def run(self):
        print(HELP)
        self._warn_preconditions()
        self._draw(force=True)
        while not self.quitting:
            for key in self._read_keys():
                self._handle(key)
                if self.quitting:
                    break
            if not self.quitting:
                self._tick()
                self._flush_note()
                self._draw()
            time.sleep(0.005)
        self._flush_note()      # 最后一条提示（"已存到 <路径>"）别被退出吞掉
        self._clear_line()
        return 0

    def _handle(self, key):
        if key == "left":
            self._jog(+1)
        elif key == "right":
            self._jog(-1)
        elif key == "step_smaller":
            self.step_deg = max(STEP_MIN_DEG, self.step_deg / 2.0)
            self._say("步长 %.2f°" % self.step_deg)
        elif key == "step_bigger":
            self.step_deg = min(STEP_MAX_DEG, self.step_deg * 2.0)
            self._say("步长 %.2f°" % self.step_deg)
        elif key == "mode_run":
            self._set_mode("start")
        elif key == "mode_damping":
            self._set_mode("damping")
        elif key == "mode_stop":
            self._set_mode("stop")
        elif key == "zero":
            self._record("zero")
        elif key == "full":
            self._record("full")
        elif key == "save":
            if self._save():
                self.quitting = True
        elif key == "status":
            self._clear_line()
            self._status()
            self._draw(force=True)
        elif key == "help":
            self._clear_line()
            print(HELP)
            self._draw(force=True)
        elif key == "quit":
            self._clear_line()
            print("退出（没存盘）。")
            self.quitting = True

    def _warn_preconditions(self):
        s = self.arm.snapshot()
        if s is None:
            self._say("还没收到反馈帧 —— 板子在发吗？口子选对了吗？")
            return
        if not (s.motor_inited_mask >> bd.GRIPPER) & 1:
            self._say("⚠ 夹爪还没找零（motor_inited 里夹爪那位是 0）—— 现在记录的原始角"
                      "**零点是没有含义的**。先做一次成功的 0x02 初始化再回来标定。")
        if s.robot_state != bd.STATE_NORMAL:
            self._say("⚠ 板子现在在「%s」，不是正常运行 —— 标定应在一次成功的初始化之后做。"
                      % s.robot_state_name)
        if self.full_open_m is None:
            self._say("⚠ 没给 --full-open-m，文件里也没有旧值 —— 最后存不下。")


def _clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _fmt_pt(v):
    return "未记" if v is None else "%+.3f°" % v


def _first_line(e):
    return str(e).strip().splitlines()[0]


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        prog="python3 -m sdk.calibrate_gripper",
        description="交互式标定夹爪：把开口距离（m）和夹爪的原始电机角（deg）对起来。",
        epilog="标定必须在一次成功的 0x02 初始化之后做（夹爪零点由找零给出）。",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--arm-id", type=int, default=0,
                   help="标哪一台臂：0=CAN1，1=CAN2（默认 0）")
    p.add_argument("--full-open-m", type=float, default=None, metavar="M",
                   help="完全张开时的开口距离，米（>0）。不传就用文件里已有的值。")
    p.add_argument("--port", default=None, help="串口设备；不传就自动挑（板子 USB 优先）")
    p.add_argument("--baud", type=int, default=921600, help="波特率（默认 921600）")
    p.add_argument("--calib", default=None, help="标定文件路径（默认 sdk/config/gripper_calib.json）")
    p.add_argument("--note", default="", help="记一句备注进标定文件")
    p.add_argument("--timeout", type=float, default=3.0, help="等第一帧反馈的超时，秒")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    if not sys.stdin.isatty():
        print("这个工具要交互按键，stdin 得是终端。", file=sys.stderr)
        return 2

    if args.arm_id < 0 or args.arm_id >= bd.ARM_NUM:
        print("--arm-id 只能是 0..%d（板子就 %d 台臂）。" % (bd.ARM_NUM - 1, bd.ARM_NUM),
              file=sys.stderr)
        return 2

    store = GripperCalibStore(args.calib)
    full_open_m = args.full_open_m
    if full_open_m is None:
        old = store.get(args.arm_id)
        if old is not None:
            full_open_m = old.full_open_m
    if full_open_m is not None and not full_open_m > 0:
        print("--full-open-m 必须 > 0（它是「完全张开时的开口距离」，单位米）。", file=sys.stderr)
        return 2

    link = Link(port=args.port, baud=args.baud)
    arm = Arm(args.arm_id, link, calib_store=store)
    try:
        arm.connect(timeout=args.timeout)
    except ArError as e:
        print(str(e), file=sys.stderr)
        link.close()
        return 1

    print("已连上 %s。" % link.describe())
    print("标定文件：%s%s" % (store.path,
                             "" if store.path.exists() else "（还没有，存盘时创建）"))

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    tty_cbreak(fd)
    try:
        return Calibrator(arm, store, full_open_m, note=args.note).run()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        # **不替用户切档**。切阻尼听着"安全"，但它会把电机使能起来 —— 本来在 stop 档（失能）
        # 的臂会突然被通上电，这是没人要求过的动作。退出就退出：不再发帧之后板子 500 ms
        # 收不到有效帧，自己进「通信离线」切阻尼，这才是板子上已经验证过的那条兜底路径。
        print("退出。最后看到的档位是「%s」—— 不再发帧之后板子 500 ms 就会自己进"
              "「通信离线」切阻尼，不用再做什么。" % (arm.mode_name or "未知"))
        link.close()


def tty_cbreak(fd):
    """cbreak：按键立刻到手，但 Ctrl-C 之类的信号不被吞掉、回显也不乱。"""
    import tty
    tty.setcbreak(fd)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print()
        sys.exit(130)
