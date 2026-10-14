"""Backend for the interactive terminal: "everything but curses" from `sdk/tui/app.py`.

Why a separate file: the panel's few hundred lines only draw and handle keys; the link,
the protocol, the gate pre-checks and the unit conversion **all come from `sdk/`**. This
module is the only seam, and the panel knows just these few things:

    sess.snapshot()                one consistent view per frame (**SI**), fixed key
                                   names, draw() depends on it entirely
    sess.tick(now)                 advance one beat: send 0x16, or top up with a 0x01
                                   keepalive
    sess.jog/cycle_step/send_*     key actions (gating is left to sdk.Arm's pre-checks)
    sess.target/sel/step/hold/...  the "current arm"'s state; the panel reads/writes it
    sess.switch_arm(i)             the a key switches the "current arm"

**Two arms**: one `Link` + one `sdk.Arm` per arm. **Both are being driven** — each has its
own target, its own 0x16 stream, its own keepalive timer, and the `a` key only switches
*which arm the panel is looking at*. So the arm you switched away from is not left alone
(it keeps following its own target).

The panel **does not know there are two arms**: `ConsoleSession.__getattr__/__setattr__`
below forwards the `target/sel/step/...` names to the "current arm", so every `sess.xxx`
in the panel stays untouched.
"""

import argparse
import time
from collections import deque

from .. import board_defs, i18n, protocol, units
from ..errors import ArError, FeedbackStale, GateRejected
from ..link import Link
from ..sim import SimLink

# ============================================================================
# Layer A: constants and helpers (protocol / limits / state names all come from
# sdk; only "the conventions the panel wants" live here)
# ============================================================================

DEFAULT_ARM_ID = protocol.DEFAULT_ARM_ID
N_ARM = board_defs.ARM_NUM                 # 2
N_AXIS = board_defs.ARM_MOTOR_NUM          # 7 (6 joints + gripper)
GRIPPER = board_defs.GRIPPER               # 6

# Jog step presets. **rad is native** — the panel and the session always use rad; only
# once the gripper is calibrated does it get its own metric presets (it has a real
# travel, and rad presets are not intuitive for "how far open").
STEP_CHOICES_RAD = [0.005, 0.01, 0.02, 0.05, 0.1, 0.2]
STEP_CHOICES_M = [0.0005, 0.001, 0.002, 0.005, 0.01, 0.02]
DEFAULT_STEP_RAD = 0.02                    # the preset close to the old 1.0°
DEFAULT_STEP_M = 0.002

# Threshold for "is this feedback frame fresh enough": the same convention as
# `sdk.arm.FEEDBACK_STALE_S` (100 ms). The control stream checks it once itself so it
# does not enter sdk's pre-check every beat (at 100 Hz that is pure waste).
FEEDBACK_FRESH_S = 0.1

MODE_TID_NAME = {protocol.TID_ARM_START: ("session.tid.start", "run"),
                 protocol.TID_ARM_GRAVITY: ("session.tid.gravity", "gravity"),
                 protocol.TID_ARM_STOP: ("session.tid.stop", "stop"),
                 protocol.TID_ARM_DAMPING: ("session.tid.damping", "damping")}


def short_label(arm_id):
    """Short label for the log prefix (`ArmA`). For the full name (with the CAN bus)
    use `i18n.arm_label()`."""
    return i18n.arm_label(arm_id).split("(")[0]


def parse_arm_id(s):
    """Value check for `--arm-id` (**the "current arm" at startup**; the a key can switch
    it at run time).

    The board side only accepts `arm_id < ARM_NUM`; an out-of-range frame is **silently
    dropped** (it does not even answer with a version frame), so the symptom is "nothing
    moves and no bytes come back" — hence it is better to fail loudly at startup than to
    turn this into a silent failure you can only diagnose with a packet capture.
    """
    try:
        v = int(s, 0)
    except ValueError:
        raise argparse.ArgumentTypeError(i18n.tr("session.arm_id.type", N_ARM - 1))
    if not 0 <= v < N_ARM:
        raise argparse.ArgumentTypeError(i18n.tr("session.arm_id.range", N_ARM - 1, N_ARM))
    return v


def axis_lo_rad(axis):
    """Lower soft limit of one axis (rad). The gripper uses the raw angle's ∓142°, not
    metres."""
    return (board_defs.GRIPPER_ANGLE_MIN_RAD if axis == GRIPPER
            else board_defs.JOINT_ANGLE_MIN_RAD[axis])


def axis_hi_rad(axis):
    return (board_defs.GRIPPER_ANGLE_MAX_RAD if axis == GRIPPER
            else board_defs.JOINT_ANGLE_MAX_RAD[axis])


def clamp_axis_rad(axis, value):
    """Clamp one axis into its soft limits (rad); returns `(clamped value, was it clamped)`.

    `sdk.board_defs` only has the deg version (`clamp_axis`); the session always uses rad,
    so this composes one out of its rad limit mirrors — composed, not a second copy of the
    limit table.
    """
    lo, hi = axis_lo_rad(axis), axis_hi_rad(axis)
    if value < lo:
        return lo, True
    if value > hi:
        return hi, True
    return value, False


def gripper_m_limits(calib):
    """The gripper's metric soft limits `(lo, hi)` once it is calibrated.

    Which way the two points map is decided by the calibration itself (neither end is
    assumed to be "open"), so both ends are computed here and then sorted.
    """
    a = calib.opening_from_raw_deg(board_defs.GRIPPER_ANGLE_MIN)
    b = calib.opening_from_raw_deg(board_defs.GRIPPER_ANGLE_MAX)
    return (a, b) if a <= b else (b, a)


# ============================================================================
# Layer C: the session (all of it runs on the UI thread, single writer for sends)
# ============================================================================

class ArmView:
    """**One arm**'s piece of panel state. The `a` key switches "which piece the panel is
    looking at".

    The canonical domain of the target `target`: 6 joints in **rad** + the gripper in
    **raw-angle rad** (the gripper gets no pos_to_angle conversion; metres are what
    calibration produces, not a quantity in the protocol). Converted on demand for
    display.
    """

    __slots__ = ("arm_id", "arm", "target", "sel", "hold", "seeded", "prev",
                 "last_tx", "last_ctrl", "ctrl_live", "init_t0", "init_told_reset",
                 "reset_t0", "force_init", "have_ever_rx", "handshake_reported",
                 "grip_hint_shown", "ctrl_err_reported", "step_idx_rad", "step_idx_m",
                 "_rate_n", "_rate_t", "_rate_hz")

    def __init__(self, arm, opts):
        self.arm_id = arm.arm_id
        self.arm = arm
        self.target = [0.0] * N_AXIS
        self.sel = 0
        self.hold = False              # pause this arm's 0x16 stream (freezes the target,
                                       # it is not a stop)
        self.seeded = False
        self.prev = None               # previous frame (state, mode, lost)
        self.last_tx = 0.0             # when the last frame went out (0x16 counts too)
        self.last_ctrl = 0.0
        self.ctrl_live = False         # the last beat really sent a 0x16
        self.init_t0 = None
        self.init_told_reset = False   # did the init sequence pass through "reset"
                                       # (display only)
        self.reset_t0 = None
        self.force_init = False        # the i key was stopped by the pre-check once; the
                                       # second press force-sends
        self.have_ever_rx = False
        self.handshake_reported = False
        self.grip_hint_shown = False   # the uncalibrated-gripper hint is printed once
        self.ctrl_err_reported = False # a periodic-stream send failure is reported once
                                       # (at 100 Hz it would flood)
        self.step_idx_rad = (STEP_CHOICES_RAD.index(opts.step)
                             if opts.step in STEP_CHOICES_RAD
                             else STEP_CHOICES_RAD.index(DEFAULT_STEP_RAD))
        self.step_idx_m = STEP_CHOICES_M.index(DEFAULT_STEP_M)
        self._rate_n = 0
        self._rate_t = time.monotonic()
        self._rate_hz = 0.0

    # ---- step preset: gripper calibrated AND the gripper row is selected → metric ----

    @property
    def step_in_m(self):
        return self.sel == GRIPPER and self.arm.calibration is not None

    @property
    def step(self):
        return (STEP_CHOICES_M[self.step_idx_m] if self.step_in_m
                else STEP_CHOICES_RAD[self.step_idx_rad])

    @property
    def step_unit(self):
        return "m" if self.step_in_m else "rad"

    @property
    def step_str(self):
        """The text of the panel's "step xx" cell (the unit follows the row)."""
        return ("%.4f m" % self.step) if self.step_in_m else ("%.3f rad" % self.step)


# Forwarding table for the "current arm": every name the panel reads/writes directly
# means the current arm's copy.
_PER_ARM_ATTRS = frozenset({
    "arm", "arm_id", "target", "sel", "hold", "seeded", "prev", "last_tx",
    "last_ctrl", "ctrl_live", "init_t0", "init_told_reset", "reset_t0",
    "force_init", "have_ever_rx", "handshake_reported", "grip_hint_shown",
    "ctrl_err_reported", "step", "step_unit", "step_str", "step_in_m",
})


class ConsoleSession:
    """The panel's session: derives board state (from feedback frames, never invented),
    sends commands, keeps the event log.

    `tick()` is called every 2~5 ms (`getch` timeout):
      - when a 0x16 can be sent (normal operation + run + not paused) it sends the current
        target at `--ctrl-hz`;
      - when it cannot, it tops up with a 0x01 keepalive every 150 ms — otherwise those
        7 s of init would drop into comm_lost.
    **This runs once per arm**, so the arm you switched away from is kept alive and
    position-controlled too.
    """

    KEEPALIVE_S = 0.15

    def __init__(self, link, opts, arm_ids=None):
        # events / dirty / t0 / link / opts / cur_arm_id / views are **session-level** and
        # must be written with object.__setattr__: the forwarding rules below would
        # intercept a same-named attribute (views above all — a wrong write there is
        # infinite recursion).
        object.__setattr__(self, "link", link)
        object.__setattr__(self, "opts", opts)
        object.__setattr__(self, "events", deque(maxlen=400))
        object.__setattr__(self, "dirty", True)
        object.__setattr__(self, "t0", time.monotonic())
        object.__setattr__(self, "version_reported", False)
        object.__setattr__(self, "cur_arm_id", opts.arm_id)
        # The link's three edges are watched in tick() (`_watch_link`): None = "not seen yet",
        # so the first beat just records the current values instead of reporting them.
        object.__setattr__(self, "_link_gen", None)
        object.__setattr__(self, "_link_up", False)
        object.__setattr__(self, "_link_gave_up", False)
        arms = {}
        views = {}
        for arm_id in (range(N_ARM) if arm_ids is None else arm_ids):
            arm = link.arm(arm_id)
            arms[arm_id] = arm
            views[arm_id] = ArmView(arm, opts)
        object.__setattr__(self, "arms", arms)
        object.__setattr__(self, "views", views)

    # ---- indirection for the "current arm": the panel reading sess.target reads the
    # ---- current arm's

    @property
    def cur(self):
        return self.views[self.cur_arm_id]

    def __getattr__(self, name):
        if name in _PER_ARM_ATTRS:
            return getattr(self.views[self.cur_arm_id], name)
        raise AttributeError(name)

    def __setattr__(self, name, value):
        if name in _PER_ARM_ATTRS:
            setattr(self.cur, name, value)
        else:
            object.__setattr__(self, name, value)

    def switch_arm(self, arm_id=None):
        """Switch the "current arm" (the a key). This only changes which one the panel
        looks at — both arms are being driven all along."""
        if arm_id is None:
            arm_id = (self.cur_arm_id + 1) % N_ARM
        if arm_id == self.cur_arm_id:
            return
        self.cur_arm_id = arm_id
        self.dirty = True
        self.info(i18n.tr("session.switch_arm", i18n.arm_label(arm_id)))

    # ---- events ----

    def event(self, level, text):
        self.events.append((time.time(), level, text))
        self.dirty = True

    def streaming_any(self):
        """Is any arm streaming (or paused in) a control stream — used by the exit
        confirmation.

        **Both arms have to be checked**: judging by the current arm alone, pressing q on
        the arm that is not running would exit right away while the other arm's 0x16 is
        still going."""
        return any(v.ctrl_live or v.hold for v in self.views.values())

    def info(self, text):
        self.event("info", text)

    def warn(self, text):
        self.event("warn", text)

    def err(self, text):
        self.event("err", text)

    # ---- reads (used by the panel) ----

    def snapshot(self):
        """The one consistent view per frame the panel wants — the key names are the
        panel's contract, the values are **SI**.

        `fb` comes straight from `arm.snapshot()`: that is already a "converted within one
        frame" consistent view and will not be changed by the next frame. The gripper
        cell: **metres** once calibrated, None when not (the panel falls back to showing
        the raw angle).
        """
        view = self.cur
        arm = view.arm
        link = self.link
        st = link.stats
        sp = arm.snapshot()
        now = time.monotonic()

        if sp is None:
            fb = None
            fb_t = 0.0
        else:
            fb = {"mode": sp.mode, "state": sp.robot_state,
                  "lost": sp.motor_lost_mask, "inited": sp.motor_inited_mask,
                  "joint": list(sp.joint_angles),                        # rad
                  "jvel": list(sp.joint_velocities),                     # rad/s
                  "gripper": sp.gripper_opening,                         # m / None
                  "gripper_raw_rad": units.deg_to_rad(sp.gripper_raw_deg),
                  "gripper_calibrated": sp.gripper_calibrated,
                  "temp": list(sp.temperatures)}
            fb_t = sp.taken_at

        chan = link.channel(self.cur_arm_id)
        ack, ack_t, _ = chan.snapshot_ack()

        return {
            "fb": fb,
            "fb_t": fb_t,
            "ack": ({"state": ack.state, "result": list(ack.result),
                     "elapsed_ms": ack.elapsed_ms} if ack is not None else None),
            "ack_t": ack_t,
            "board_version": st["board_version"],
            "link_up": st["link_up"],
            "link_err": st["link_err"],
            "link_auto": st["auto_reconnect"],        # will it come back on its own
            "link_retry": st["reconnect_tries"],      # try number in this round (0 = none yet)
            "link_retry_max": st["reconnect_max_tries"],
            "link_give_up": st["reconnect_gave_up"],  # used up the tries, stopped
            "n_rx": st["n_rx"],                     # link level (sum of both arms)
            "n_rx_fb": arm.n_feedback,              # **this arm**'s 0x80 count
            "rx_rate": self._rate(view, now),       # this arm's 0x80 rate
            "n_tx": st["n_tx"],
            "n_drop": st["n_drop"],
            "n_crc_err": st["n_err_crc8"] + st["n_err_crc16"],
            "n_bad_arm": st["n_bad_arm"],
            "unknown_tids": dict(chan.unknown_tids),
            "arm_id": self.cur_arm_id,
            "arms": [self._arm_line(arm_id, now) for arm_id in sorted(self.views)],
        }

    def _arm_line(self, arm_id, now):
        """The short description used by the "two-arm overview" line under the status
        line."""
        view = self.views[arm_id]
        sp = view.arm.snapshot()
        return {"arm_id": arm_id,
                "state": None if sp is None else sp.robot_state,
                # Rendered from the **numeric code** through i18n: `sp.robot_state_name`
                # / `sp.mode_name` are the Chinese that `board_defs` pre-rendered (public
                # API, must not change), so the English panel has to go around them here.
                "state_name": i18n.tr("session.no_feedback") if sp is None
                else i18n.state_name(sp.robot_state),
                "mode_name": "-" if sp is None else i18n.mode_name(sp.mode),
                "lost": None if sp is None else sp.motor_lost_mask,
                "ctrl_live": view.ctrl_live,
                "have_ever_rx": view.have_ever_rx,
                "n_feedback": view.arm.n_feedback,
                "rx_rate": self._rate(view, now)}

    def _rate(self, view, now):
        """This arm's 0x80 arrival rate (0.5 s window). The SDK's stats are link-level
        aggregate counts with no rate, and link level would add both arms together —
        while the panel's `0x80 %u` is this arm's, so the two numbers have to use the same
        convention."""
        dt = now - view._rate_t
        if dt >= 0.5:
            view._rate_hz = (view.arm.n_feedback - view._rate_n) / dt
            view._rate_n = view.arm.n_feedback
            view._rate_t = now
        return view._rate_hz

    def fb(self):
        """The current arm's feedback view (used by the panel's key handling; None when
        there is no feedback)."""
        return self.snapshot()["fb"]

    def gripper_row(self, fb):
        """The quantities the gripper row shows (the panel only lays them out; unit
        conversion stays in the backend).

        **Metres once calibrated** (target/feedback/soft limits all map through those two
        calibration points), **the raw angle (rad) when not** — the raw angle is the
        motor's quantity and has no "how many millimetres" spec, so it can only come out of
        a calibration (see sdk/README "gripper calibration"). `fb` may be None (no
        feedback frame yet).
        """
        view = self.cur
        calib = view.arm.calibration
        if calib is not None:
            lo, hi = gripper_m_limits(calib)
            return {"calibrated": True, "unit": "m",
                    "tgt": calib.opening_from_raw_deg(units.rad_to_deg(view.target[GRIPPER])),
                    "fb": (fb["gripper"] if fb else None), "lo": lo, "hi": hi}
        return {"calibrated": False, "unit": "rad",
                "tgt": view.target[GRIPPER],
                "fb": (fb["gripper_raw_rad"] if fb else None),
                "lo": axis_lo_rad(GRIPPER), "hi": axis_hi_rad(GRIPPER)}

    def state_mode(self):
        snap = self.snapshot()
        fb = snap["fb"]
        if fb is None:
            return None, None, snap
        return fb["state"], fb["mode"], snap

    # ---- target / step ----

    def seed_from_feedback(self, fb, why):
        view = self.cur
        view.target = list(fb["joint"]) + [fb["gripper_raw_rad"]]
        view.seeded = True
        self.info(i18n.tr("session.seeded", short_label(view.arm_id), why))

    def _seed(self, view, sp, why):
        """Seed from `sdk`'s snapshot (the session's internal automatic re-seeding goes
        through this; the value domain matches `target` exactly)."""
        view.target = list(sp.joint_angles) + [units.deg_to_rad(sp.gripper_raw_deg)]
        view.seeded = True
        self.info(i18n.tr("session.seeded", short_label(view.arm_id), why))

    def jog(self, axis, sign):
        """A key only changes target[]; the actual send is done by tick at its period."""
        view = self.cur
        if axis is None:
            return
        sp = view.arm.snapshot()
        if sp is None:
            self.warn(i18n.tr("session.no_feedback_yet"))
            return
        lbl = short_label(view.arm_id)

        if axis == GRIPPER and view.step_in_m:
            # metric preset: add/subtract the target in metres, then map back to the raw
            # angle (the mapping is decided by the two calibration points)
            calib = view.arm.calibration
            cur_m = calib.opening_from_raw_deg(units.rad_to_deg(view.target[axis]))
            want_m = cur_m + sign * view.step
            lo_m, hi_m = gripper_m_limits(calib)
            if want_m < lo_m or want_m > hi_m:
                got_m = min(max(want_m, lo_m), hi_m)
                self.warn(i18n.tr("session.grip_limit_m", lbl, want_m, lo_m, hi_m, got_m))
                view.target[axis] = units.deg_to_rad(calib.raw_deg_from_opening(got_m))
                return
            view.target[axis] = units.deg_to_rad(calib.raw_deg_from_opening(want_m))
            self.dirty = True
            return

        want = view.target[axis] + sign * view.step
        got, clamped = clamp_axis_rad(axis, want)
        view.target[axis] = got
        if clamped:
            self.warn(i18n.tr("session.axis_limit", lbl, i18n.axis_name(axis),
                              units.rad_to_deg(want),
                              units.rad_to_deg(axis_lo_rad(axis)),
                              units.rad_to_deg(axis_hi_rad(axis)),
                              units.rad_to_deg(got)))
        else:
            self.dirty = True

    def cycle_step(self, direction=1):
        view = self.cur
        lbl = short_label(view.arm_id)
        if view.step_in_m:
            view.step_idx_m = (view.step_idx_m + direction) % len(STEP_CHOICES_M)
        else:
            view.step_idx_rad = (view.step_idx_rad + direction) % len(STEP_CHOICES_RAD)
        self.info(i18n.tr("session.step", lbl, view.step_str))

    # ---- sends: commands (pre-checking is left to sdk.Arm; here we only turn exceptions
    # ---- into plain words in the event stream)

    def _report_send_failure(self, view, what, e):
        """A command that cannot be sent goes into the event stream in one place. Send
        errors are err, gate rejections are warn (the board did not accept it either)."""
        lbl = short_label(view.arm_id)
        if isinstance(e, (GateRejected, FeedbackStale)):
            self.warn(i18n.tr("session.send_gated", lbl, what, e))
        else:
            self.err(i18n.tr("session.send_error", lbl, what, e))

    def send_empty_mode(self, tid, name):
        view = self.cur
        arm = view.arm
        fn = {protocol.TID_ARM_START: arm.start, protocol.TID_ARM_GRAVITY: arm.gravity,
              protocol.TID_ARM_STOP: arm.stop, protocol.TID_ARM_DAMPING: arm.damping}[tid]
        try:
            fn()
        except ArError as e:
            self._report_send_failure(view, name, e)
            return
        self.info(i18n.tr("session.sent_mode", short_label(view.arm_id), name,
                          MODE_TID_NAME[tid][1]))
        view.last_tx = time.monotonic()

    def send_arm_init(self):
        """0x02. The board first compares the pose and the lost mask and **silently drops**
        anything that fails, so the pre-check is left to `sdk.Arm`.

        **"Press i again to force" is given to the pose gate only** (that is how the old
        one behaved): the other gates (wrong state, motors lost) would also be silently
        dropped by the board if forced, but the log would still say "sent 0x02, about
        7 s" — that is lying about the init having started, exactly the trap this panel
        exists to prevent. So the pose is re-checked here, and only when it is exceeded does
        the two-press force path open; every other gate is reported straight by the SDK
        with no force option.
        """
        view = self.cur
        arm = view.arm
        lbl = short_label(view.arm_id)
        sp = arm.snapshot()
        if sp is None:
            self.warn(i18n.tr("session.init_no_fb", lbl))
            return

        bad = [(board_defs.JOINT_NAMES[i],
                units.rad_to_deg(sp.joint_angles[i] - want))
               for i, want in enumerate(board_defs.RESET_POSE_RAD)
               if abs(sp.joint_angles[i] - want) > self.opts.init_tol]
        if bad:
            if not view.force_init:
                view.force_init = True
                self.warn(i18n.tr(
                    "session.init_pose_bad", lbl, units.rad_to_deg(self.opts.init_tol),
                    i18n.tr("session.pose_dev_sep").join(
                        i18n.tr("session.pose_dev_item", n, d) for n, d in bad),
                    " ".join("%+.4f" % v for v in board_defs.RESET_POSE_RAD)))
                return
            self.warn(i18n.tr("session.init_force", lbl))
        try:
            arm.init_arm(force=bool(bad))
        except ArError as e:
            view.force_init = False
            self._report_send_failure(view, i18n.tr("session.what.init"), e)
            return
        view.force_init = False
        self.info(i18n.tr("session.init_sent", lbl))
        view.last_tx = time.monotonic()

    def send_arm_reset(self):
        """0x18 one-key reset. The gate is "normal operation + nothing lost" (the board
        **deliberately does not check the pose**)."""
        view = self.cur
        lbl = short_label(view.arm_id)
        try:
            view.arm.reset()
        except ArError as e:
            self._report_send_failure(view, i18n.tr("session.what.reset"), e)
            return
        self.info(i18n.tr("session.reset_sent", lbl,
                          " ".join("%+.4f" % v for v in board_defs.RESET_POSE_RAD)))
        now = time.monotonic()
        view.last_tx = now
        view.reset_t0 = now

    def send_setzero(self):
        view = self.cur
        try:
            view.arm.set_zero()
        except ArError as e:
            self._report_send_failure(view, i18n.tr("session.what.setzero"), e)
            return
        self.err(i18n.tr("session.setzero_sent", short_label(view.arm_id)))
        view.last_tx = time.monotonic()

    def stop_all(self):
        """`--stop-on-exit`: send one extra 0x14 to **both arms** (disable, go slack)."""
        done = []
        for arm_id in sorted(self.views):
            try:
                self.views[arm_id].arm.stop(force=True)
                done.append(short_label(arm_id))
            except ArError as e:
                self.err(i18n.tr("session.stop_all_fail", short_label(arm_id), e))
        return done

    # ---- link ----

    def reopen(self):
        """Reopen the link (the r key). Under `--fake` this is "power-cycle the fake
        board".

        The SDK's `Link.open()` **raises `LinkError`** on failure (not SystemExit), so this
        catches it and reports an event as usual without tearing the panel down. Behind the
        scenes the link keeps retrying on its own after a failed reopen, so pressing r with
        the cable still out just starts another round.

        The re-seeding itself lives in `_on_link_back`, which the automatic reconnect also
        goes through.
        """
        try:
            self.link.reopen()
        except ArError as e:
            self.err(i18n.tr("session.reopen_fail", e))
            self.dirty = True
            return False
        self._link_gen = getattr(self.link, "open_gen", 0)
        self._link_up = True
        self._link_gave_up = False
        self._on_link_back("session.reopened")
        return True

    def _on_link_back(self, key):
        """The per-arm reset for a link that has (re)connected — the r key and the automatic
        reconnect both land here.

        **The targets have to be re-aligned with the feedback**: the arm may have been
        dragged while the link was down, and sending a 0x16 with the old target would yank it
        back. The gripper calibration is re-read too (that is usually done in another
        process).
        """
        for view in self.views.values():
            view.handshake_reported = False
            view.ctrl_err_reported = False
            view.prev = None
            view.seeded = False
        self.t0 = time.monotonic()
        for view in self.views.values():
            try:
                view.arm.reload_calibration()
            except ArError:
                pass
        self.info(i18n.tr(key, self.link.describe() if not self.opts.fake
                          else i18n.tr("session.fake_board")))
        self.dirty = True

    def _watch_link(self):
        """The link's edges: down / came back on its own / auto-reconnect gave up.

        The retrying runs in the RX thread; the panel only sees the state on `Link`, so it is
        compared once per beat (tick runs every 2~5 ms, dense enough). A changed `open_gen`
        means "a new link is up" — pressing r changes it too, so `reopen()` syncs the
        baseline itself and this only reports the automatic kind.
        """
        link = self.link
        gen = getattr(link, "open_gen", 0)
        up = bool(link.link_up)
        gave_up = bool(getattr(link, "reconnect_gave_up", False))
        if self._link_gen is None:
            self._link_gen, self._link_up, self._link_gave_up = gen, up, gave_up
            return
        if gen != self._link_gen:
            self._link_gen = gen
            if up:
                self._on_link_back("session.relinked")
        if up != self._link_up:
            self._link_up = up
            if not up:
                err = link.link_err or ""
                self.warn(i18n.tr("session.link_lost" if link.auto_reconnect
                                 else "session.link_lost_manual", err))
        if gave_up and not self._link_gave_up:
            self.warn(i18n.tr("session.relink_giveup", link.reconnect_tries,
                              link.reconnect_err or link.link_err or ""))
        self._link_gave_up = gave_up

    # ---- per beat ----

    def tick(self, now):
        self._watch_link()
        for arm_id in sorted(self.views):
            view = self.views[arm_id]
            sp = view.arm.snapshot()
            if sp is not None:
                self._on_feedback(view, sp, now)
            elif now - self.t0 > self.opts.handshake_timeout \
                    and not view.handshake_reported:
                view.handshake_reported = True
                self.dirty = True

            can_ctrl = (sp is not None
                        and sp.robot_state == board_defs.STATE_NORMAL
                        and sp.mode == board_defs.MODE_RUN
                        and not view.hold
                        and self.link.link_up
                        and sp.age <= FEEDBACK_FRESH_S)
            if can_ctrl:
                if now - view.last_ctrl >= 1.0 / max(1.0, self.opts.ctrl_hz):
                    view.last_ctrl = now
                    view.ctrl_live = self._send_ctrl(view, now)
            else:
                view.ctrl_live = False
                if now - view.last_tx > self.KEEPALIVE_S:
                    self._send_keepalive(view, now)

            if view.init_t0 is not None and sp is not None \
                    and sp.robot_state != board_defs.STATE_INITING:
                view.init_t0 = None
            if view.reset_t0 is not None and sp is not None \
                    and sp.robot_state != board_defs.STATE_RESETTING:
                view.reset_t0 = None
            self._rate(view, now)

    def _send_ctrl(self, view, now):
        """The periodic 0x16. Why `force=True`: the local gate above and `sdk.Arm`'s
        pre-check test **exactly the same things** (state/mode/age/link_up), so running the
        pre-check again every beat at 100 Hz is pure waste — but the conversion and the
        encoding still go through `Arm`, we do not build the frame here."""
        try:
            view.arm.send_ctrl_pos(view.target[:6],
                                   gripper_raw_deg=units.rad_to_deg(view.target[GRIPPER]),
                                   force=True)
        except ArError as e:
            if not view.ctrl_err_reported:
                view.ctrl_err_reported = True
                self.err(i18n.tr("session.ctrl_tx_fail", short_label(view.arm_id), e))
            return False
        view.ctrl_err_reported = False
        view.last_tx = now
        return True

    def _send_keepalive(self, view, now):
        try:
            view.arm.keepalive()
        except ArError as e:
            if not view.ctrl_err_reported:
                view.ctrl_err_reported = True
                self.err(i18n.tr("session.keepalive_tx_fail", short_label(view.arm_id), e))
            return False
        view.ctrl_err_reported = False
        view.last_tx = now
        return True

    def _report_handshake(self):
        """The version code is **link level** (one link, one host), so the whole session
        reports it once."""
        if self.version_reported:
            return
        self.version_reported = True
        bv = self.link.board_version
        if bv is None:
            self.info(i18n.tr("session.hs_fb_only", protocol.VERSION_CODE))
        elif bv == protocol.VERSION_CODE:
            self.info(i18n.tr("session.hs_ok", bv, protocol.VERSION_STRING))
        else:
            self.err(i18n.tr("session.hs_bad", bv, protocol.VERSION_CODE))

    def _on_feedback(self, view, sp, now):
        """Feedback frame → events / automatic re-seeding. **The policy is carried over
        clause by clause** from the old single-arm version; only the units became SI and
        the log lines carry an arm label. Angles are converted back to deg before being
        written to the log (the log's convention is deg)."""
        lbl = short_label(view.arm_id)
        if not view.have_ever_rx:
            view.have_ever_rx = True
            self._report_handshake()
            self._seed(view, sp, i18n.tr("session.why.first_fb"))
            if not sp.gripper_calibrated and view.arm_id == self.cur_arm_id \
                    and not view.grip_hint_shown:
                view.grip_hint_shown = True
                self.info(i18n.tr("session.grip_uncal", lbl, view.arm_id))

        st, mode, lost = sp.robot_state, sp.mode, sp.motor_lost_mask
        cur = (st, mode, lost)
        if view.prev is None:
            view.prev = cur
        elif cur != view.prev:
            pst, pmode, plost = view.prev
            if st != pst:
                self.info(i18n.tr("session.state_changed", lbl,
                                  i18n.state_name(pst), i18n.state_name(st)))
                if st == board_defs.STATE_INITING:
                    view.init_t0 = now
                    view.init_told_reset = False
                if st == board_defs.STATE_RESETTING:
                    view.reset_t0 = now
                # Coming back from comm_lost / finishing init into normal — these two
                # must re-seed, otherwise a stale target yanks the arm on the very frame
                # that enters run (the board's own joint_src seeding only follows the
                # feedback while there is no valid control frame).
                if st == board_defs.STATE_WAIT_INIT and pst in (
                        board_defs.STATE_INITING, board_defs.STATE_COMM_LOST,
                        board_defs.STATE_CALIBRATING):
                    self._seed(view, sp, i18n.tr("session.why.wait_init"))
                if st == board_defs.STATE_NORMAL and pst == board_defs.STATE_RESETTING:
                    # End of a reset. **Must re-seed**: the arm really moved during the
                    # reset, so the target from before the excursion went stale long ago
                    # (the board's Ctrl_Invalidate cleared it too); without re-seeding the
                    # next 0x16 would yank the arm back to the old target once more.
                    self._seed(view, sp, i18n.tr("session.why.reset_tail"))
                    bad = [(board_defs.JOINT_NAMES[i],
                            units.rad_to_deg(sp.joint_angles[i] - want))
                           for i, want in enumerate(board_defs.RESET_POSE_RAD)
                           if abs(sp.joint_angles[i] - want) > self.opts.init_tol]
                    if bad:
                        # That silent failure on the board: when the temporary planning
                        # pool is full the reset starts an empty plan and "finishes" on the
                        # next beat, so the state machine thinks it is done but the arm
                        # never moved.
                        self.err(i18n.tr(
                            "session.reset_skipped", lbl,
                            i18n.tr("session.pose_dev_sep").join(
                                i18n.tr("session.pose_dev_item", n, d) for n, d in bad),
                            units.rad_to_deg(self.opts.init_tol)))
                    else:
                        self.info(i18n.tr("session.reset_done", lbl))
                    view.reset_t0 = None
            if mode != pmode:
                good = {1: "info", 2: "warn", 0: "warn", 3: "info"}.get(mode, "warn")
                text = i18n.tr("session.mode_changed", lbl, i18n.mode_name(pmode),
                               i18n.mode_name(mode))
                if pst in (board_defs.STATE_INITING, board_defs.STATE_RESETTING) \
                        and st == board_defs.STATE_NORMAL \
                        and mode == board_defs.MODE_DAMPING:
                    # Both the end of init and the end of a reset go run → damping; that
                    # is a **normal finish**, not an anomaly (the anomalous one is a loss
                    # knocking run back to stop, the line below). Do not scare anyone.
                    good = "info"
                    text += i18n.tr("session.mode_tail_normal")
                if mode == board_defs.MODE_RUN:
                    self._seed(view, sp, i18n.tr("session.why.enter_run"))
                    self.event("info", text)
                elif pmode == board_defs.MODE_RUN:
                    # The most typical one: Motor_Lost_Handle knocks arm_mode back to stop
                    # because motors were lost.
                    self.event("err" if lost else good,
                               text + i18n.tr("session.ctrl_stopped"))
                else:
                    self.event(good, text)
            if lost != plost:
                if lost:
                    self.err(i18n.tr("session.motor_lost", lbl, i18n.expand_mask(lost), lost))
                else:
                    self.info(i18n.tr("session.motor_recovered", lbl, plost))
            view.prev = cur
        elif not view.seeded:
            self._seed(view, sp, i18n.tr("session.why.reseed"))

        if st == board_defs.STATE_INITING and view.init_t0 is not None:
            self.dirty = True      # the panel wants to watch the seconds tick
            if now - view.init_t0 > 4.0 and not view.init_told_reset:
                view.init_told_reset = True
                self.info(i18n.tr("session.initing_note", lbl))


# ============================================================================
# Shell: build the session (used by the panel and main)
# ============================================================================

def build_session(opts):
    """Build the link + session from opts. **Both arms are built**: one link, one sdk.Arm
    each."""
    if opts.fake:
        # the --fake-lost mask lands on the arm in opts.arm_id (the other stays as a
        # control)
        link = SimLink(lost_mask=opts.fake_lost, lost_arm=opts.arm_id)
    else:
        link = Link(opts.port, opts.baud)
    return ConsoleSession(link, opts)
