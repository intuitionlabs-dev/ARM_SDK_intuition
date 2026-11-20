"""ARX H7 夹爪两点标定工具：记录闭合和完全张开时的原始角。

运行：python3 -m sdk.calibrate_gripper --arm-id 0 --full-open-m 0.085。
须先完成初始化；夹爪零点变化后须重新标定。
按 d/x 切阻尼/STOP 后手动调整，或按 r 进入 RUN 后用方向键点动。
RUN 会使能整条臂，点动会实际驱动夹爪。按 0/f 记录两点，回车保存，q 放弃。
配置路径可由 --calib 或 ARX_GRIPPER_CALIB 指定。
"""

import argparse
import os
import select
import sys
import termios
import time

from .board_defs import arx_x5 as bd
from .arm import FEEDBACK_STALE_S, Arm
from .errors import ArError, CalibrationError
from .gripper import GripperCalibStore, GripperCalibration
from .link import Link

# 点动目标发送频率。
JOG_HZ = 50.0

# 空闲时仍保活，避免 3 s 通信超时。
ALIVE_HZ = 5.0

# 界面刷新频率。
DRAW_HZ = 5.0

STEP_MIN_DEG = 0.1
STEP_MAX_DEG = 45.0
STEP_DEFAULT_DEG = 2.0

# 点动反馈时限略小于 SDK 的 100 ms，留出处理余量。
JOG_FRESH_S = FEEDBACK_STALE_S * 0.8

# 单字节按键与方向键 ESC 序列。
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
    b"\x03": "quit",      # Ctrl-C 输入字节
    b"\x04": "quit",      # Ctrl-D
}
ESCAPES = {
    b"\x1b[C": "right", b"\x1b[D": "left",     # 标准 CSI
    b"\x1bOC": "right", b"\x1bOD": "left",     # 某些终端发 SS3
    b"\x1b[A": "up", b"\x1b[B": "down",
}

HELP = """按键：
  ← / →     夹爪原始角增加 / 减少一个步长，仅 RUN 模式有效
  [ / ]     步长减半 / 加倍          p   显示完整状态
  0         记录闭合点（0 m）        f   记录全开点（FULL_OPEN）
  回车      保存两点标定             q   不保存退出
  r / d / x 切换 RUN / 阻尼 / STOP
  ? / h     显示帮助

RUN 会使能整条臂，切换前请确认运动范围内无人和障碍。
开合方向由装配决定。记录两点后按 p 核对原始角，保存时会显示开合方向。
"""


class Calibrator:
    def __init__(self, arm, store, full_open_m, note=""):
        self.arm = arm
        self.store = store
        self.full_open_m = full_open_m
        self.note = note

        self.step_deg = STEP_DEFAULT_DEG
        self.target_deg = None      # 夹爪点动目标，单位 deg
        self.deg_at_zero = None
        self.deg_at_full_open = None

        self._last_tx = 0.0
        self._last_draw = 0.0
        self._stale_warned = False
        self._link_gen = None        # 上次连接代数，用于检测重连
        self._buf = b""
        self._line_shown = False
        self._note_line = ""        # 底部单次提示
        self.quitting = False

    # 输入

    def _read_keys(self):
        """读取当前可用的输入字节并解析按键。"""
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
            if not chunk:           # 输入流已结束
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
                    break           # 保留未完整接收的按键序列
                one, self._buf = self._buf[:1], self._buf[1:]
                keys.append(SINGLES.get(one, "other"))
        return keys

    # 动作

    def _say(self, text):
        self._note_line = text

    def _raw_deg(self):
        return self.arm.raw_gripper_angle_deg

    def _jog(self, direction):
        if self.target_deg is None:
            self._say("请先按 r 进入 RUN，再点动夹爪。")
            return
        before = self.target_deg
        self.target_deg = _clip(self.target_deg + direction * self.step_deg,
                                bd.GRIPPER_ANGLE_MIN, bd.GRIPPER_ANGLE_MAX)
        if self.target_deg == before:
            self._say("已到软限位（%+.1f..%+.1f°）。"
                      % (bd.GRIPPER_ANGLE_MIN, bd.GRIPPER_ANGLE_MAX))

    def _set_mode(self, tid_name):
        try:
            getattr(self.arm, tid_name)()
        except ArError as e:
            self._say("%s 未发送：%s" % (tid_name, _first_line(e)))
            return
        self._say("已发送 %s；请查看后续模式反馈。" % tid_name)
        if tid_name == "start":
            self.target_deg = self._raw_deg()      # 进入 RUN 时对齐当前反馈，避免目标跳变。

    def _record(self, which):
        raw = self._raw_deg()
        if raw is None:
            self._say("尚无反馈，无法读取夹爪原始角。")
            return
        if which == "zero":
            self.deg_at_zero = raw
        else:
            self.deg_at_full_open = raw
        self._say("已记录 %s = %+.3f°（原始角）"
                  % ("0 m" if which == "zero" else "FULL_OPEN", raw))

    def _save(self):
        if self.deg_at_zero is None or self.deg_at_full_open is None:
            self._say("标定点不完整：0 m=%s，FULL_OPEN=%s；请补齐后保存。"
                      % (_fmt_pt(self.deg_at_zero), _fmt_pt(self.deg_at_full_open)))
            return False
        if self.full_open_m is None:
            self._say("缺少最大开口，请用 --full-open-m 指定米数。")
            return False
        try:
            calib = self.store_put()
        except CalibrationError as e:
            self._say("标定无效：%s" % _first_line(e))
            return False
        except OSError as e:
            self._say("写入失败：%s" % e)
            return False
        self._say("已保存到 %s：端点角差 %.2f°，%s。" %
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
            print("\n尚无反馈，请检查串口、接线和固件。")
            return
        print("\n" + "-" * 68)
        print("  %s" % s)
        print("  夹爪原始角   %+.3f°（= %+.4f rad）"
              % (s.gripper_raw_deg, self.arm.raw_gripper_angle_rad or 0.0))
        print("  夹爪电机     motor_inited=%s   lost=%s"
              % ("是" if s.motor_inited_mask >> bd.GRIPPER & 1 else "否", s.lost_names))
        print("  记录点       0 m = %s   FULL_OPEN(%s) = %s"
              % (_fmt_pt(self.deg_at_zero),
                 ("%.4f m" % self.full_open_m) if self.full_open_m else "未设置",
                 _fmt_pt(self.deg_at_full_open)))
        print("-" * 68)

    # 周期

    def _tick(self):
        now = time.monotonic()
        self._watch_link()
        period = 1.0 / (JOG_HZ if self._jogging() else ALIVE_HZ)
        if now - self._last_tx < period:
            return
        self._last_tx = now
        try:
            self._send_once()
        except ArError as e:
            self._say("%s" % _first_line(e))

    def _watch_link(self):
        """检测重连并在状态行显示结果。"""
        gen = getattr(self.arm.link, "open_gen", 0)
        if self._link_gen is None:
            self._link_gen = gen
            return
        if gen != self._link_gen:
            self._link_gen = gen
            self._say("链路已自动重连，请检查反馈后继续操作。")

    def _jogging(self):
        """判断是否可发送点动目标；反馈过期时仅发送保活。"""
        if self.target_deg is None:
            return False
        if self.arm.mode != bd.MODE_RUN or not self.arm.link_up:
            return False
        return self.arm.feedback is not None and self.arm.feedback_age <= JOG_FRESH_S

    def _send_once(self):
        if self._jogging():
            s = self.arm.snapshot()
            self.arm.send_ctrl_pos(s.joint_angles, gripper_raw_deg=self.target_deg,
                                   gripper_torque_limit_nm=self.arm.board_defs.DEFAULT_GRIPPER_TORQUE_LIMIT_NM)
            self._stale_warned = False
            return
        # 空闲时发送 0x01 保活。
        self.arm.keepalive()
        age = self.arm.feedback_age
        if (self.target_deg is not None and age is not None
                and age > JOG_FRESH_S and not self._stale_warned):
            self._stale_warned = True
            self._say("反馈已过期 %.0f ms，暂停点动并保活；请检查链路（link_up=%s）。"
                      % (age * 1000.0, self.arm.link_up))

    # 画面

    def _draw(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_draw < 1.0 / DRAW_HZ:
            return
        self._last_draw = now
        s = self.arm.snapshot()
        if s is None:
            left = "[%s] 等待反馈…" % bd.arm_label(self.arm.arm_id)
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
            left = ("[%s] %s / %s  原始角 %+8.3f°  ≈%s  步长 %.2f°%s"
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

    # 主循环

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
        self._flush_note()      # 退出前显示最终操作结果。
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
            print("已退出，未保存。")
            self.quitting = True

    def _warn_preconditions(self):
        s = self.arm.snapshot()
        if s is None:
            self._say("尚无反馈，请检查串口、接线和固件。")
            return
        if not (s.motor_inited_mask >> bd.GRIPPER) & 1:
            self._say("夹爪尚未找零（夹爪初始化标志为 0），请先完成 0x02 初始化再标定。")
        if s.robot_state != bd.STATE_NORMAL:
            self._say("当前状态为 %s，请完成初始化并进入正常运行后标定。"
                      % s.robot_state_name)
        if self.full_open_m is None:
            self._say("缺少最大开口，无法保存；请用 --full-open-m 指定。")


def _clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _fmt_pt(v):
    return "未记录" if v is None else "%+.3f°" % v


def _first_line(e):
    return str(e).strip().splitlines()[0]


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        prog="python3 -m sdk.calibrate_gripper",
        description="夹爪两点标定：建立开口距离（m）与原始角（deg）的对应关系。",
        epilog="请先完成 0x02 初始化，确定夹爪零点后再标定。",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--arm-id", type=int, default=0,
                   help="机械臂编号：0=CAN1，1=CAN2（默认 0）")
    p.add_argument("--full-open-m", type=float, default=None, metavar="M",
                   help="最大开口距离，单位 m，须 >0；默认读取已有标定")
    p.add_argument("--port", default=None, help="串口路径；默认自动选择，板载 USB 优先")
    p.add_argument("--baud", type=int, default=921600, help="波特率（默认 921600）")
    p.add_argument("--calib", default=None, help="标定文件路径（默认 sdk/config/gripper_calib.json）")
    p.add_argument("--note", default="", help="标定备注")
    p.add_argument("--timeout", type=float, default=3.0, help="首帧反馈超时，单位 s")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    if not sys.stdin.isatty():
        print("标定需要键盘交互，请在终端运行。", file=sys.stderr)
        return 2

    if args.arm_id < 0 or args.arm_id >= bd.ARM_NUM:
        print("--arm-id 应为 0..%d（臂数 %d）。" % (bd.ARM_NUM - 1, bd.ARM_NUM),
              file=sys.stderr)
        return 2

    store = GripperCalibStore(args.calib)
    full_open_m = args.full_open_m
    if full_open_m is None:
        old = store.get(args.arm_id)
        if old is not None:
            full_open_m = old.full_open_m
    if full_open_m is not None and not full_open_m > 0:
        print("--full-open-m 为最大开口距离（m），须 >0。", file=sys.stderr)
        return 2

    link = Link(port=args.port, baud=args.baud)
    arm = Arm(args.arm_id, link, calib_store=store)
    try:
        arm.connect(timeout=args.timeout)
    except ArError as e:
        print(str(e), file=sys.stderr)
        link.close()
        return 1

    print("已连接 %s。" % link.describe())
    print("标定文件：%s%s" % (store.path,
                             "" if store.path.exists() else "（将在保存时创建）"))

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    tty_cbreak(fd)
    try:
        return Calibrator(arm, store, full_open_m, note=args.note).run()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        # 退出不主动切换模式；关闭串口后由固件处理通信超时。
        print("已退出，最后反馈模式为 %s；停止发帧约 3 s 后，固件触发通信超时并切换阻尼。" % (arm.mode_name or "未知"))
        link.close()


def tty_cbreak(fd):
    """启用即时按键输入，保留终端信号处理，退出时恢复设置。"""
    import tty
    tty.setcbreak(fd)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print()
        sys.exit(130)
