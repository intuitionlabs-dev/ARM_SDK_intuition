"""The curses panel: `Ui` (draw / keys / dialogs). It only knows `session`'s interface
contract.

The backend is in [session.py](session.py): the link, the gate pre-checks, the unit
conversion, the per-beat tick and the event policy all live in `ConsoleSession`. Here we
only draw the "one consistent view per frame" it hands us, plus **keep feeding tick on the
blocking path** (the one inside `input_line`) — the arm falls back to damping if it does
not get a valid frame within 3 s, and sitting in a dialog would knock it out of `run`.

The command-line entry point is in [__main__.py](__main__.py): `python3 -m sdk.tui`.
"""

import curses
import time

from .. import i18n, protocol
from ..board_defs import arx_x5 as board_defs
from .session import GRIPPER, N_AXIS, axis_hi_rad, axis_lo_rad, short_label


# ============================================================================
# Layer D: the curses panel (knows only the sess.snapshot() contract; everything
# else lives in the backend)
# ============================================================================

C_OK, C_WARN, C_ERR, C_DIM, C_HEAD = 1, 2, 3, 4, 5

# Key order of the help page ("" is a separating blank line, it does not go into the
# language table). **The text must not be looked up at import time** — `set_lang` only
# runs once `__main__` has parsed `--lang`.
HELP_KEYS = ("app.help.select", "app.help.jog", "app.help.step", "app.help.arm",
             "app.help.align", "app.help.modes", "app.help.init", "app.help.reset",
             "app.help.setzero", "app.help.hold", "app.help.misc", "",
             "app.help.si1", "app.help.si2", "app.help.si3", "",
             "app.help.gate1", "app.help.gate2", "app.help.gate3", "",
             "app.help.two1", "app.help.two2", "",
             "app.help.exit")


def help_lines():
    return [i18n.tr(k) if k else "" for k in HELP_KEYS]


class Ui:
    def __init__(self, stdscr, sess, opts):
        self.scr = stdscr
        self.sess = sess
        self.opts = opts
        self.show_help = False
        self.quit = False
        self.last_draw = 0.0
        self.draw_period = 1.0 / max(1.0, opts.fb_hz)
        self._logged = set()
        self.esc_until = 0.0   # swallow keys for 50 ms after ESC (so an arrow-key
                               # sequence is not split into single keys)
        self.logf = None
        if opts.log:
            try:
                self.logf = open(opts.log, "a", encoding="utf-8")
                self.logf.write(i18n.tr("app.log_head", time.strftime("%F %T"),
                                        opts.port, opts.fake, opts.arm_id))
                self.logf.flush()
            except OSError as e:
                self.logf = None
                print(i18n.tr("app.log_warn", e))

    # ---- drawing ----
    def init_colors(self):
        try:
            curses.start_color()
            curses.use_default_colors()
            curses.init_pair(C_OK, curses.COLOR_GREEN, -1)
            curses.init_pair(C_WARN, curses.COLOR_YELLOW, -1)
            curses.init_pair(C_ERR, curses.COLOR_RED, -1)
            curses.init_pair(C_DIM, curses.COLOR_CYAN, -1)
            curses.init_pair(C_HEAD, curses.COLOR_WHITE, -1)
        except curses.error:
            pass

    def attr(self, pair, extra=0):
        try:
            return curses.color_pair(pair) | extra
        except curses.error:
            return extra

    def put(self, y, x, s, a=0):
        try:
            maxy, maxx = self.scr.getmaxyx()
            if y < 0 or y >= maxy or x >= maxx:
                return
            self.scr.addstr(y, x, s[:max(0, maxx - x - 1)], a)
        except curses.error:
            pass

    def draw(self):
        scr = self.scr
        scr.erase()
        try:
            maxy, maxx = scr.getmaxyx()
        except curses.error:
            return
        if maxy < 12 or maxx < 60:
            self.put(0, 0, i18n.tr("app.too_small", maxy, maxx), self.attr(C_ERR))
            scr.noutrefresh()
            curses.doupdate()
            return
        snap = self.sess.snapshot()
        fb = snap["fb"]
        arm_id = snap["arm_id"]
        y = 0

        title = i18n.tr("app.title", i18n.arm_label(arm_id),
                        i18n.tr("app.fake_tag") if self.opts.fake
                        else self.opts.port.split("/")[-1],
                        "%d bps" % self.opts.baud)
        self.put(y, 0, title, self.attr(C_HEAD, curses.A_BOLD))
        self.put(y, maxx - 24, "vcode 0x%08X" % self.sess.board_defs.VERSION_CODE,
                 self.attr(C_DIM))
        y += 1

        link = i18n.tr("app.link_line",
                       "OK" if snap["link_up"] else i18n.tr("app.link_down"))
        if not snap["link_up"]:
            # What the link is doing about it right now (it retries on its own, and it stops
            # once it has used up its tries -- then pressing r starts another round).
            if snap["link_give_up"]:
                link += i18n.tr("app.link_giveup", snap["link_retry"])
            elif snap["link_retry"]:
                link += i18n.tr("app.link_retry", snap["link_retry"], snap["link_retry_max"])
            elif snap["link_auto"]:
                link += i18n.tr("app.link_retry_on")
            else:
                link += i18n.tr("app.link_manual")
            if snap["link_err"]:
                link += " " + snap["link_err"]
        stats = i18n.tr("app.stats", snap["n_rx"], snap["n_rx_fb"], snap["rx_rate"],
                        snap["n_tx"], snap["n_drop"], snap["n_crc_err"])
        self.put(y, 0, link + stats,
                 self.attr(C_ERR) if not snap["link_up"] else self.attr(C_OK))
        y += 1

        # Two-arm overview: one link drives both arms, so "what is the other one doing
        # right now" has to be visible at a glance (> is the current arm; whether its arm
        # is still streaming and whether anything is lost are all on this line)
        seg = ["%s%s %s/%s %s %.0fHz%s" % (
            ">" if a["arm_id"] == arm_id else " ", short_label(a["arm_id"]),
            a["state_name"], a["mode_name"],
            i18n.tr("app.streaming") if a["ctrl_live"] else i18n.tr("app.idle"),
            a["rx_rate"], i18n.tr("app.lost_tag") if a["lost"] else "")
            for a in snap["arms"]]
        self.put(y, 0, i18n.tr("app.arms_line", "  |  ".join(seg)),
                 self.attr(C_WARN) if any(a["lost"] for a in snap["arms"]) else self.attr(C_DIM))
        y += 1

        if fb is None:
            self.put(y, 0, i18n.tr("app.no_fb"), self.attr(C_WARN))
            y += 1
            if not snap["link_up"]:
                self.put(y, 0, i18n.tr("app.link_dead"), self.attr(C_ERR))
            elif time.monotonic() - self.sess.t0 > self.opts.handshake_timeout:
                self.put(y, 0, i18n.tr("app.hs_timeout",
                                       int(self.opts.handshake_timeout)), self.attr(C_ERR))
                for i, line in enumerate([
                    i18n.tr("app.hs1"),
                    i18n.tr("app.hs2", self.sess.board_defs.VERSION_CODE,
                            self.sess.board_defs.VERSION_STRING),
                    i18n.tr("app.hs3"),
                    i18n.tr("app.hs4"),
                    i18n.tr("app.hs5"),
                ]):
                    self.put(y + 1 + i, 0, line, self.attr(C_WARN))
        else:
            missing = ~fb["inited"] & 0x7F
            extra = ""
            if fb["lost"]:
                extra += i18n.tr("app.lost_prefix") + i18n.expand_mask(fb["lost"])
            if missing:
                # The gripper's ∓142 soft limits only take effect **after one successful
                # zeroing** (motor_inited[gripper] is maintained by the zeroing itself,
                # nobody else writes it), so an uncalibrated gripper has to be visible.
                extra += i18n.tr("app.uninit_prefix") + i18n.expand_mask(missing)
                if missing >> GRIPPER & 1:
                    extra += i18n.tr("app.grip_limit_note")
            self.put(y, 0, "robot_state %s(%d)   arm_mode %s   inited 0x%02X   lost 0x%02X%s" % (
                i18n.state_name(fb["state"]), fb["state"],
                i18n.mode_name(fb["mode"]), fb["inited"], fb["lost"], extra),
                self.attr(C_ERR) if fb["lost"] else self.attr(C_WARN) if missing else 0)
            y += 1
            stream = i18n.tr("app.ctrl_prefix",
                             i18n.tr("app.streaming") if self.sess.ctrl_live
                             else i18n.tr("app.idle"))
            if self.sess.hold:
                stream += i18n.tr("app.hold_tag")
            stream += i18n.tr("app.ctrl_meta", self.opts.ctrl_hz, self.sess.step_str,
                              i18n.axis_name(self.sess.sel))
            if self.sess.init_t0 is not None:
                stream = i18n.tr("app.initing", time.monotonic() - self.sess.init_t0)
            elif self.sess.reset_t0 is not None:
                stream = i18n.tr("app.resetting", time.monotonic() - self.sess.reset_t0,
                                 " ".join("%+.4f" % v for v in self.sess.board_defs.RESET_POSE_RAD))
            self.put(y, 0, stream,
                     self.attr(C_WARN) if not self.sess.ctrl_live else self.attr(C_OK))
            y += 1
            self.put(y, 0, i18n.tr("app.table_head"),
                     self.attr(C_HEAD, curses.A_UNDERLINE))
            y += 1
            tgt = self.sess.target
            grip = self.sess.gripper_row(fb)
            for i in range(N_AXIS):
                mark = ">" if i == self.sess.sel else " "
                if i < GRIPPER:
                    fv = fb["joint"][i]
                    line = "%s%-4s %11.4f %13.4f %+11.4f   [%8.4f,%8.4f] %3d°C" % (
                        mark, i18n.axis_name(i), tgt[i], fv, tgt[i] - fv,
                        axis_lo_rad(i, self.sess.board_defs), axis_hi_rad(i, self.sess.board_defs), fb["temp"][i])
                else:
                    fv = grip["fb"]
                    unit = "m" if grip["calibrated"] else "raw rad"
                    if fv is None:      # no feedback (or the calibration has no metres):
                                        # draw only the target
                        line = "%s%-4s %11.4f %13s %11s   [%8.4f,%8.4f] %s" % (
                            mark, i18n.axis_name(GRIPPER), grip["tgt"], "-", "-",
                            grip["lo"], grip["hi"], unit)
                    else:
                        line = "%s%-4s %11.4f %13.4f %+11.4f   [%8.4f,%8.4f] %s" % (
                            mark, i18n.axis_name(GRIPPER), grip["tgt"], fv, grip["tgt"] - fv,
                            grip["lo"], grip["hi"], unit)
                a = curses.A_REVERSE if i == self.sess.sel else 0
                if i == GRIPPER:
                    a |= self.attr(C_DIM)
                self.put(y, 0, line, a)
                y += 1
            self.put(y, 0, "jvel: " + " ".join("%+8.3f" % v for v in fb["jvel"]) + "  rad/s",
                     self.attr(C_DIM))
            y += 1
            if not grip["calibrated"]:
                # This line is the answer to "why do the numbers in the gripper row look
                # odd" (useless once calibrated)
                self.put(y, 0, i18n.tr("app.grip_uncal_hint",
                                       self.sess.board_defs.GRIPPER_ANGLE_MAX_RAD, arm_id),
                         self.attr(C_DIM))
                y += 1

        # event area
        if snap["ack"] is not None and self.sess.have_ever_rx:
            ack = snap["ack"]
            self.put(y, 0, i18n.tr("app.setzero_ack",
                                   i18n.zero_state_name(ack["state"], ack["state"]),
                                   ack["elapsed_ms"],
                                   " ".join("%s=%s" % (i18n.axis_name(i),
                                                       i18n.zero_res_name(ack["result"][i]))
                                            for i in range(N_AXIS))), self.attr(C_DIM))
            y += 1
            if 1 in ack["result"]:
                # "Unconfirmed" is normal for DM motors (the position is already near 0,
                # so whether the write landed cannot be told apart); without saying so,
                # every zeroing would look like it failed.
                self.put(y, 0, i18n.tr("app.setzero_unconfirmed"), self.attr(C_WARN))
                y += 1

        y += 1
        if self.show_help:
            for line in help_lines():
                self.put(y, 2, line, self.attr(C_DIM))
                y += 1
        else:
            room = max(0, maxy - y - 2)
            shown = list(self.sess.events)[-room:]
            for ts, level, text in shown:
                a = {"info": 0, "warn": self.attr(C_WARN), "err": self.attr(C_ERR)}[level]
                self.put(y, 0, "%s %s" % (time.strftime("%H:%M:%S", time.localtime(ts)), text), a)
                y += 1
                if y >= maxy - 2:
                    break

        self.put(maxy - 2, 0, i18n.tr("app.hotkeys"), self.attr(C_DIM))
        scr.noutrefresh()
        curses.doupdate()

    # ---- input ----
    def input_line(self, prompt):
        """One-line input at the bottom (used by the double confirmation for set-zero). It
        draws itself, eats keys itself, returns the string on Enter and None on ESC."""
        try:
            maxy, maxx = self.scr.getmaxyx()
        except curses.error:
            return None
        buf = ""
        while True:
            self.put(maxy - 1, 0, (" " * (maxx - 2)))
            self.put(maxy - 1, 0, prompt + buf, self.attr(C_ERR, curses.A_BOLD))
            self.scr.noutrefresh()
            curses.doupdate()
            ch = self.scr.getch()
            if ch == -1:
                # getch timed out (the beat is set by --ctrl-hz, 2~17 ms) — **the link must
                # be fed here**: this function blocks, so the main loop's tick() does not
                # run for the whole time, and both the 0x16 and the 0x01 keepalive are sent
                # by tick() — with not a single byte going out, the arm falls back to
                # damping when the board gets no valid frame for 3 s (robot_state.c's
                # Fsm_Step / Comm_Lost_Enter), i.e. leaving the o confirmation up for over
                # 3 s knocks the arm out of `run` and it has to be put back with s.
                # tick() advances both arms, so both arms' keepalives are on this path.
                self.sess.tick(time.monotonic())
                continue
            if ch in (10, 13, curses.KEY_ENTER):
                return buf
            if ch == 27:  # ESC
                # The swallow window has to be set here too: the tail of a sequence after
                # ESC lands in on_key, and 'D' there = 0x15 damping
                self.esc_until = time.monotonic() + 0.05
                return None
            if ch in (curses.KEY_BACKSPACE, 127, 8):
                buf = buf[:-1]
            elif 32 <= ch < 127:
                buf += chr(ch)

    def on_key(self, ch):
        s = self.sess
        # Bytes after an ESC have to be swallowed: a terminal arrow-key sequence (CSI/SS3)
        # that terminfo cannot match is split by ncurses into the three keys ESC '[' 'D',
        # and 'D' in our key table = 0x15 damping! So "press an arrow key and knock the arm
        # into damping along the way" really does happen; hence any key within 50 ms of an
        # ESC is discarded as the tail of a sequence.
        if ch == 27:
            self.esc_until = time.monotonic() + 0.05
            if self.show_help:
                self.show_help = False
                s.dirty = True
            return
        if time.monotonic() < self.esc_until:
            return
        if self.show_help:
            # The help is an overlay but **used as a modal**: pressing keys in it should not
            # send control frames along the way
            if ch in (ord("h"), ord("?"), ord("e"), ord("q")):
                self.show_help = False
                if ch == ord("e"):
                    s.events.clear()
                    s.info(i18n.tr("app.events_cleared"))
                elif ch == ord("q"):
                    self.quit = True
            s.dirty = True
            return
        if ch in (ord("q"), ord("Q")):
            if s.streaming_any():
                if self.input_line(i18n.tr("app.quit_confirm")).strip().lower() == "y":
                    self.quit = True
            else:
                self.quit = True
        elif ch in (ord("h"), ord("?")):
            self.show_help = not self.show_help
        elif ch in (ord("a"), ord("A")):
            s.switch_arm()
        elif ord("1") <= ch <= ord("6"):
            s.sel = ch - ord("1")
            s.dirty = True
        elif ch == ord("0"):
            s.sel = GRIPPER
            s.dirty = True
        elif ch == 9:  # Tab
            s.sel = (s.sel + 1) % N_AXIS
            s.dirty = True
        elif ch in (curses.KEY_LEFT, curses.KEY_DOWN, ord("-"), ord("_")):
            s.jog(s.sel, -1)
        elif ch in (curses.KEY_RIGHT, curses.KEY_UP, ord("+"), ord("=")):
            s.jog(s.sel, +1)
        elif ch == ord("["):
            s.cycle_step(-1)
        elif ch == ord("]"):
            s.cycle_step(+1)
        elif ch == ord("c"):
            fb = s.fb()
            if fb is None:
                s.warn(i18n.tr("app.no_fb_align"))
            else:
                s.seed_from_feedback(fb, i18n.tr("app.why_manual"))
        elif ch == ord(" "):
            s.hold = not s.hold
            s.info(i18n.tr("app.hold_toggle", i18n.tr("app.hold_paused") if s.hold
                           else i18n.tr("app.hold_resumed")))
        elif ch in (ord("s"), ord("S")):
            s.send_empty_mode(protocol.TID_ARM_START, i18n.tr("app.tid.start"))
        elif ch in (ord("d"), ord("D")):
            s.send_empty_mode(protocol.TID_ARM_DAMPING, i18n.tr("app.tid.damping"))
        elif ch in (ord("v"), ord("V")):
            s.send_empty_mode(protocol.TID_ARM_GRAVITY, i18n.tr("app.tid.gravity"))
        elif ch in (ord("x"), ord("X")):
            s.send_empty_mode(protocol.TID_ARM_STOP, i18n.tr("app.tid.stop"))
        elif ch in (ord("i"), ord("I")):
            s.send_arm_init()
        elif ch in (ord("o"), ord("O")):
            self.reset_confirm()
        elif ch in (ord("z"), ord("Z")):
            self.setzero_confirm()
        elif ch == ord("e"):
            s.events.clear()
            s.info(i18n.tr("app.events_cleared"))
        elif ch == ord("r"):
            self.reopen()
        elif ch == curses.KEY_RESIZE:
            s.dirty = True

    def reset_confirm(self):
        """0x18's pose gate lives on **the host** side (the board deliberately does not
        check it: an arm in motion is supposed to be off the reset pose, otherwise there
        would be no point resetting). When the deviation exceeds the tolerance, a y/n is
        popped once for a human to decide; when it passes, the frame goes out directly."""
        s = self.sess
        fb = s.fb()
        if fb is not None and fb["state"] == self.sess.board_defs.STATE_NORMAL and not fb["lost"]:
            bad = [i18n.axis_name(i)
                   for i, want in enumerate(self.sess.board_defs.RESET_POSE_RAD)
                   if abs(fb["joint"][i] - want) > self.opts.init_tol]
            if bad:
                s.warn(i18n.tr("app.reset_pose_warn", self.opts.init_tol,
                               i18n.tr("app.pose_dev_sep").join(bad)))
                if self.input_line(i18n.tr("app.reset_confirm")).strip().lower() != "y":
                    s.info(i18n.tr("app.reset_cancelled"))
                    return
        # The remaining self-checks (no feedback / wrong state / motors lost) are left to
        # send_arm_reset to report, so that two places do not fight over the wording.
        s.send_arm_reset()

    def setzero_confirm(self):
        """0x10 is the only frame that moves the "zero", hence the double confirmation
        (typed out in full, to guard against a slip of the hand)."""
        s = self.sess
        fb = s.fb()
        if fb is None:
            s.warn(i18n.tr("app.setzero_no_fb"))
            return
        if fb["state"] != self.sess.board_defs.STATE_WAIT_INIT:
            s.warn(i18n.tr("app.setzero_state", i18n.state_name(fb["state"])))
            return
        s.warn(i18n.tr("app.setzero_warn1"))
        s.warn(i18n.tr("app.setzero_warn2",
                       " ".join("%+.4f" % v for v in fb["joint"]),
                       " ".join("%+.4f" % v for v in self.sess.board_defs.RESET_POSE_RAD)))
        if self.input_line(i18n.tr("app.setzero_step1")) != "SETZERO":
            s.info(i18n.tr("app.setzero_cancel1"))
            return
        if self.input_line(i18n.tr("app.setzero_step2")) != "CONFIRM":
            s.info(i18n.tr("app.setzero_cancel2"))
            return
        s.send_setzero()

    def reopen(self):
        """Reopen the link (under `--fake` this is "power-cycle the fake board"). Errors are
        collected into the event stream by the backend."""
        self.sess.reopen()
        self.sess.dirty = True

    def log_events(self):
        if self.logf is None:
            return
        for ts, level, text in self.sess.events:
            key = (ts, text)
            if key in self._logged:
                continue
            self._logged.add(key)
            self.logf.write("%s %-4s %s\n" % (time.strftime("%F %T", time.localtime(ts)), level, text))
        self.logf.flush()

    def run(self):
        s = self.sess
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        self.scr.keypad(True)
        # Panel beat = half the control period: sending is done in tick, and tick only runs
        # after a getch timeout, so a beat that is not at least half the control period
        # cannot keep up with --ctrl-hz (5 ms for 100 Hz). Floor of 2 ms, so that a large
        # --ctrl-hz does not peg the CPU. Screen refresh is governed separately by --fb-hz
        # and is unrelated to this.
        self.scr.timeout(max(2, int(round(500.0 / max(1.0, self.opts.ctrl_hz)))))
        curses.noecho()
        curses.cbreak()
        self.init_colors()
        self.sess.dirty = True
        while not self.quit:
            ch = self.scr.getch()
            if ch != -1:
                self.on_key(ch)
                self.sess.dirty = True
            now = time.monotonic()
            s.tick(now)
            if s.dirty or now - self.last_draw >= self.draw_period:
                self.draw()
                self.last_draw = now
                s.dirty = False
                self.log_events()
        self.log_events()
