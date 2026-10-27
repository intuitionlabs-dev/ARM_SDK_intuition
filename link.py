"""一条上位机链路：串口 + 后台收包线程 + 按 arm_id 分流 + 口子挑选。

板子上**只有一条**上位机链路（走串口 USART3/RS485 还是走板载 USB CDC，由固件里的
`ROBOT_LINK` 编译期二选一），而链路上跑着**所有**臂的帧 —— 每一帧的 `arm_id` 说明它是
哪台臂的。所以"两台臂一个进程"的正确形状是**一条 `Link` + 每台一个 `Arm`**：

    link = Link(); link.open()
    a = Arm(0, link); b = Arm(1, link)

反过来（两个 `Link` / 两个进程抢同一个 tty）会把字节流撕碎 —— 每一端都只读到一部分帧，
pyserial 报的错是 `device reports readiness to read but returned no data`，很容易被误读成
"臂死了"。**一个口子只能一个进程**。

这一层只做三件事：把字节搬进来切成帧、按 arm_id 投进各自的收包槽、把帧组好发出去。
**它不认识状态机、不做任何闸门**（闸门在 `arm.py`），也不主动发任何帧 —— 心跳要显式发
（`Arm.keepalive()`），SDK 不起自动保活线程：板子的保活判据是**每台臂各自**"3 s 内收到过
**任意**有效帧"（按帧头的 `arm_id` 算），所以每一台都得自己喂 —— 你的周期控制帧本身就是
保活，再自动加一条只会和它抢时序。

**掉线由收包线程自己接**（`auto_reconnect`，默认开）：读或写失败就把链路置成断开，然后
退避着重新开串口，试满 `reconnect_max_tries`（默认 10）次还开不上就停手，等你去查线或
按面板的 `r`。断开期间 `is_open` 是假、`send()` 抛 `NotConnected`（`LinkError` 的子类），
链路在背后自己恢复。设备重枚举之后 tty 名字会变，所以重连**按 USB 身份重新认口**，
不认名字。

`import sdk` 不会 import pyserial（`serial` 只在 `open()` 里 import），所以没装 pyserial
也能用 SDK 的协议层和离线自测。
"""

import glob
import os
import threading
import time

from . import i18n, protocol
from .board_defs import arx_x5 as board_defs
from .errors import LinkError, NotConnected

# ==================== 口子挑选 ====================
#
# **按 USB 身份找，不按名字找。** 名字靠不住：适配器的 by-id 名字随型号变
# （本机就换过一次：`usb-1a86_USB_Single_Serial_*` → 换 RS485 适配器之后是
# `usb-1a86_USB_Serial-if00-port0`），写死一个必然过一阵就失效。

BY_ID_DIR = "/dev/serial/by-id"
DEFAULT_PORT = BY_ID_DIR + "/usb-1a86_USB_Serial-if00-port0"

SEGGER_VID = "1366"     # J-Link 探针，**永远不碰**（占着它就连不上、烧不了）
ADAPTER_VID = "1a86"    # CH340/CH343 那类 USB 转串口/转 RS485 适配器（沁恒）
BOARD_VID = "0483"      # 板子自己的 USB CDC（STM32 虚拟串口的标准 VID:PID）
BOARD_PID = "5740"

FORBIDDEN_HINTS = ("segger", "j-link", "jlink")

# ==================== 掉线自动重连 ====================
#
# 拔线、适配器重枚举、串口自己崩 —— 这些都会让 `read` 抛异常。一次掉线就是一轮重试，
# 退避着来，**试满上限就停手**（不无限打），停在断开状态等人工处理。

RECONNECT_WAIT_S = 0.5        # 每次重试前等多久（第一轮）
RECONNECT_MAX_WAIT_S = 2.0    # 退避的封顶
RECONNECT_MAX_TRIES = 10      # 一轮掉线最多试几次（<=0 = 不限）

# 链路类型 → 人话。板子那侧对应 `App/inc/robot_state.h` 的 ROBOT_LINK。
# 两边选反了的现象是"一切都对但一个字节都没有"，所以选中了哪种要写出来。
# 存的是 i18n 的 key（不是文案）：模块级常量在 import 期就求值，那时候 `set_lang`
# 还没跑，存中文会锁死语言。
KIND_NAMES = {"USB": "link.kind_usb", "RS485": "link.kind_rs485",
              "FAKE": "link.kind_fake"}


def kind_name(kind):
    """链路类型 → 人话（走 i18n；未知/null 给个兜底）。"""
    return i18n.tr(KIND_NAMES.get(kind) or "link.kind_unknown")


def _read_sys(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def usb_identity(path):
    """这个 tty 是哪个 USB 设备：往上找到有 idVendor 的那一层。查不到返回 None。

    `serial` 有的设备没有（不是所有适配器都烧了序列号），拿不到就是空串。
    """
    name = os.path.basename(os.path.realpath(path))
    d = os.path.realpath("/sys/class/tty/%s/device" % name)
    for _ in range(8):
        vid = _read_sys(os.path.join(d, "idVendor"))
        if vid:
            return {
                "idVendor": vid,
                "idProduct": _read_sys(os.path.join(d, "idProduct")) or "?",
                "serial": _read_sys(os.path.join(d, "serial")) or "",
                "product": _read_sys(os.path.join(d, "product")) or "",
                "manufacturer": _read_sys(os.path.join(d, "manufacturer")) or "",
            }
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


def same_usb_ident(a, b):
    """两个 USB 身份是不是同一台设备。**纯函数，不碰 /sys**。

    先比 VID:PID；序列号两边都有时还必须相等 —— 机器上插两个同型号适配器时，只看
    VID:PID 会在重枚举后把另一根线当成原来那根。序列号是空的（设备没烧）就只认 VID:PID。
    """
    if not a or not b:
        return False
    if a["idVendor"].lower() != b["idVendor"].lower() or \
            a["idProduct"].lower() != b["idProduct"].lower():
        return False
    sa, sb = a.get("serial") or "", b.get("serial") or ""
    return not (sa and sb) or sa == sb


def match_ident(ident, candidates):
    """在 `[(path, ident), ...]` 里找和 `ident` 同一台设备的那条路径，找不到返回 None。

    同一台设备会有好几条路径（by-id 那条和 `/dev/ttyACM*` 指的是同一个 realpath），
    按路径字典序取第一条，保证可复现。**纯函数，不碰 /sys**。
    """
    for path, other in sorted(candidates, key=lambda c: c[0]):
        if same_usb_ident(ident, other):
            return path
    return None


def classify_port(ident):
    """USB 身份 → 这是哪条链路（`"USB"` / `"RS485"` / None）。**纯函数，不碰 /sys**。"""
    if ident is None:
        return None
    vid = ident["idVendor"].lower()
    if vid == BOARD_VID and ident["idProduct"].lower() == BOARD_PID:
        return "USB"
    if vid == ADAPTER_VID:
        return "RS485"
    return None


def _pick_from(candidates):
    """`[(path, ident), ...]` → 该用哪条。**纯函数，优先级就在这里**。

    板子 USB 优先、RS485 退回（用户定的）。同一档里按路径字典序，保证可复现 ——
    插两个同型号适配器时"随便挑一个"会让日志没法复现。
    """
    for want in ("USB", "RS485"):
        for path, ident in sorted(candidates, key=lambda c: c[0]):
            if classify_port(ident) == want:
                return path
    return None


def _port_candidates():
    """机器上所有可能是链路的 tty，带上各自的 USB 身份。

    `/dev/serial/by-id/*` 都是指向 ttyUSB*/ttyACM* 的符号链接，所以按 realpath 去重，
    免得同一条设备出现两次（by-id 那条名字更好读，保留先扫到的）。
    """
    out, seen = [], set()
    paths = []
    if os.path.isdir(BY_ID_DIR):
        paths += sorted(os.path.join(BY_ID_DIR, n) for n in os.listdir(BY_ID_DIR))
    for pat in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        paths += sorted(glob.glob(pat))
    for p in paths:
        real = os.path.realpath(p)
        if real in seen:
            continue
        seen.add(real)
        out.append((p, usb_identity(p)))
    return out


def list_links():
    """`[(path, kind, ident), ...]`：机器上现在看得到的候选链路，给报错时列菜单用。"""
    out = []
    for path, ident in _port_candidates():
        out.append((path, classify_port(ident), ident))
    return out


def pick_default_port():
    """自动找那条链路。**按 USB 身份找，不按名字找。**

    优先级：板子 USB(0483:5740) → RS485 适配器(1a86) → 写死那条兜底（好让报错里有具体路径）。
    **永远排除 SEGGER(1366)** —— 它不属于这两类，`classify_port` 直接返回 None。
    """
    got = _pick_from(_port_candidates())
    return got if got else DEFAULT_PORT


def describe_port(path):
    """给日志用的一行描述。带上"这是哪种链路" —— 插错口子时一眼看得出来。"""
    ident = usb_identity(path)
    if ident is None:
        return i18n.tr("link.desc_no_ident", path)
    kind = classify_port(ident)
    tag = (i18n.tr("link.desc_kind_tag", kind_name(kind)) if kind
           else i18n.tr("link.not_link_dev"))
    return i18n.tr("link.desc_port", path, ident["idVendor"], ident["idProduct"],
                   ident["product"], ident["manufacturer"], tag)


def link_hint():
    """没找到合适口子时给的提示：两条链路各要配什么，一起说清楚。"""
    lines = [i18n.tr("link.hint_uart"),
             i18n.tr("link.hint_usb", BOARD_VID, BOARD_PID)]
    cands = list_links()
    if cands:
        lines.append(i18n.tr("link.hint_cands"))
        for path, kind, _ in cands:
            # 缩进和箭头是排版不是文案，两边一样，所以不进表
            lines.append("    %s%s" % (path, ("  ← %s" % kind_name(kind)) if kind else ""))
    else:
        lines.append(i18n.tr("link.hint_no_tty"))
    lines.append(i18n.tr("link.hint_port_arg"))
    return "\n".join(lines)


def assert_port_allowed(path):
    """用户手给的口子可能写在探针那条上。**按 USB 身份判，不按名字判**。

    （名字判不出来：适配器那条 by-id 路径 realpath 出来也是 /dev/ttyACM*，
    一刀切"含 ttyacm 就拒"会把好口子也拒掉。）
    """
    if not path:
        return
    ident = usb_identity(path)
    if ident is not None:
        blob = (ident["manufacturer"] + " " + ident["product"]).lower()
        if ident["idVendor"].lower() == SEGGER_VID or any(h in blob for h in FORBIDDEN_HINTS):
            raise LinkError(i18n.tr("link.reject_probe_ident", path, describe_port(path)))
        return
    # 查不到身份（没有 /sys、设备还没枚举出来）：名字像 SEGGER 的直接拒；
    # 名字是裸 ttyACM* 的也拒（宁可不连，也不赌它是探针）
    real = os.path.realpath(path)
    blob = (path + " " + real).lower()
    if any(h in blob for h in FORBIDDEN_HINTS) or \
            os.path.basename(real).lower().startswith("ttyacm"):
        raise LinkError(i18n.tr("link.reject_probe_name", path, link_hint()))


# ==================== 每台臂的收包槽 ====================

class ArmChannel:
    """一条链路上**某一台臂**的那份收包状态。一台一个，互不串台。

    `Link` 建、`Arm` 用。所有字段都在 `lock` 里改；读侧拿一个不可变对象（`Feedback` /
    `SetZeroAck`）的引用，所以拿到手之后不会再变。
    """

    __slots__ = ("arm_id", "lock", "feedback", "feedback_t", "feedback_seq",
                 "n_feedback", "ack", "ack_t", "ack_seq", "ack_event", "unknown_tids")

    def __init__(self, arm_id):
        self.arm_id = arm_id
        self.lock = threading.Lock()
        self.feedback = None       # protocol.Feedback
        self.feedback_t = 0.0      # time.monotonic()，0 = 还没收到过
        self.feedback_seq = 0      # 每收到一帧 +1，用来做无竞态的"等下一帧"
        self.n_feedback = 0
        self.ack = None            # protocol.SetZeroAck
        self.ack_t = 0.0
        self.ack_seq = 0
        self.ack_event = threading.Event()
        self.unknown_tids = {}     # tid -> 计数

    # ---- 写侧（只有 RX 线程调） ----

    def put_feedback(self, fb, ts):
        with self.lock:
            self.feedback = fb
            self.feedback_t = ts
            self.feedback_seq += 1
            self.n_feedback += 1

    def put_ack(self, ack, ts):
        with self.lock:
            self.ack = ack
            self.ack_t = ts
            self.ack_seq += 1
            self.ack_event.set()

    def note_unknown(self, tid):
        with self.lock:
            self.unknown_tids[tid] = self.unknown_tids.get(tid, 0) + 1

    # ---- 读侧 ----

    def snapshot_feedback(self):
        """`(feedback, 收到时刻, 序号)`。一帧都没收到过时 feedback 是 None。"""
        with self.lock:
            return self.feedback, self.feedback_t, self.feedback_seq

    def feedback_age(self):
        """最近一帧反馈距今多少秒。没收到过返回 None。"""
        with self.lock:
            t = self.feedback_t
        return None if t == 0.0 else time.monotonic() - t

    def snapshot_ack(self):
        with self.lock:
            return self.ack, self.ack_t, self.ack_seq


# ==================== 链路 ====================

class Link:
    """真串口。写加锁（多线程往同一个口写会把帧拆散），读在后台线程。

    统计量（`stats()`）：`n_tx` / `n_rx` / `n_bad_arm` / `n_err_crc8` / `n_err_crc16` /
    `n_drop`。注意 **`n_tx` 是"字节写出去了"，不是"板子收到了"** —— 板子对所有命令帧都是
    静默的，受理与否只能看反馈帧里的状态有没有变。

    **掉线自动重连**：`read`/`write` 出错就把 `link_up` 置假，收包线程接着退避重开，试满
    `reconnect_max_tries` 次（默认 10）还开不上就停手（`reconnect_gave_up`），之后按面板的
    `r`（`reopen()`）可以再开一轮。`auto_reconnect=False` 就退回"断了就断着"的老样子。
    """

    board_defs = board_defs

    def __init__(self, port=None, baud=921600, *, timeout=0.1, verify_on_open=True,
                 auto_reconnect=True, reconnect_wait=RECONNECT_WAIT_S,
                 reconnect_max_wait=RECONNECT_MAX_WAIT_S,
                 reconnect_max_tries=RECONNECT_MAX_TRIES):
        self.port = port                  # None = open() 时自动挑
        self.baud = baud
        self.ser_timeout = timeout        # 读超时，决定收包线程多久醒一次
        self.verify_on_open = verify_on_open
        self.auto_reconnect = auto_reconnect
        self.reconnect_wait = reconnect_wait
        self.reconnect_max_wait = reconnect_max_wait
        self.reconnect_max_tries = reconnect_max_tries

        self.ser = None
        self.kind = None
        self.link_up = False
        self.link_err = None
        self.board_version = None         # 板子回过来的 0x01 里的版本码
        self.version_seen = False
        self.link_status = None           # 板子报的 0x81（链路级，不挂到某台臂上）
        self.link_status_t = None         # 收到它的时刻（time.monotonic）

        self.open_gen = 0                 # 每成功 open 一次 +1（上层靠它发现"链路换了一条"）
        self.n_reconnect = 0              # 自动重连成功过的次数（累计）
        self.reconnect_tries = 0          # 这一轮掉线已经试到第几次
        self.reconnect_err = None         # 最近一次重连开串口失败的原因（一行）
        self.reconnect_gave_up = False    # 这一轮试满上限、停手了

        self.parser = protocol.FrameParser()
        self._channels = {}
        self._chan_lock = threading.Lock()   # 管 _channels/_arms 和 link_up/link_err 这一组
        self._arms = {}
        self._write_lock = threading.Lock()
        self._alive = False
        self._thr = None
        self._port_ident = None           # 打开时记下的 USB 身份，重连认口用
        self._stop_ev = threading.Event() # close() 用它把正在退避的重连立刻叫醒

        self.n_tx = 0
        self.n_rx = 0
        self.n_bad_arm = 0

    # ---- 生命周期 ----

    def open(self, port=None):
        """打开串口并起收包线程。已经开着就直接返回（幂等）。

        打不开**抛 `LinkError`**（不静默留在断开上）：第一次连不上是配错了口子，当场报
        出来比在后台一遍遍重试有用。连上之后再掉线才走自动重连。

        **收包线程还在跑（= 正在重连）时也直接返回**：那说明链路有人在管，再开一次会多出
        第二个线程读同一个口，把字节流撕碎。断开期间该抛的是 `send()` 的 `NotConnected`。
        """
        if self.is_open or (self._thr is not None and self._thr.is_alive()):
            return self
        if port is not None:
            self.port = port
        if self.port is None:
            self.port = pick_default_port()
        self._open_serial(self.port)
        self._after_open()
        return self

    def close(self):
        """停收包线程、关串口。幂等。

        自动重连也停在这一下：正在退避等待的那一轮会被 `_stop_ev` 立刻叫醒，所以不会出现
        "关掉了还偷偷把口子开回来"。
        """
        with self._chan_lock:
            self._alive = False
        self._stop_ev.set()
        if self._thr is not None:
            self._thr.join(timeout=1.0)
            self._thr = None
        self._drop_serial()
        with self._chan_lock:
            self.link_up = False

    def reopen(self, port=None):
        """关掉再开。读写出错（拔线、适配器重枚举）之后用这个复位。

        开不上照样抛 `LinkError`（脚本侧看得见）；自动重连开着的话，还会把收包线程重新起
        起来接着退避重试 —— 于是"自动重连已经放弃、线插回来后按 r"永远有下文。
        """
        self.close()
        try:
            return self.open(port)
        except LinkError:
            self._restart_reader()
            raise

    # ---- 串口句柄（自动重连要用，单独拆出来）----

    def _open_serial(self, path):
        """开一条串口、换上去。不动线程、不动 link_up。失败抛 `LinkError`。"""
        assert_port_allowed(path)
        import serial  # 只有真要连的时候才需要 pyserial

        try:
            # 板子 USB(CDC) 那条上波特率是**没有意义的** —— CDC 是包传输，tcsetattr 会把
            # SET_LINE_CODING 交给设备，STM32 那边 CDC_Control_HS 对它是 no-op 直接返回 OK。
            # 所以照设不误，只是别据此以为它有用。串口那条(RS485)上它才是真的。
            ser = serial.Serial(path, self.baud, timeout=self.ser_timeout)
        except Exception as e:
            raise LinkError(i18n.tr("link.open_failed", path, e,
                                    link_hint())) from None
        ident = usb_identity(path)
        self.kind = classify_port(ident)
        self._port_ident = ident
        with self._write_lock:
            self.ser = ser
        self.port = path

    def _drop_serial(self):
        """丢掉当前句柄（有的话）。先摘引用再关，免得有人正拿着它写。"""
        with self._write_lock:
            ser, self.ser = self.ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _after_open(self):
        """一次成功的 open 之后要做的几步（起收包线程）。"""
        self._on_serial_open()
        self._stop_ev.clear()
        with self._chan_lock:
            self._alive = True
            self.reconnect_tries = 0
            self.reconnect_gave_up = False
        self._thr = threading.Thread(target=self._reader, name="sdk-rx", daemon=True)
        self._thr.start()

    def _on_serial_open(self):
        """一条新的字节流接上了（首次 open、`r` 重开、自动重连都走这里）。

        上一段的残留半帧和"上一段报的"板级状态都作废 —— 新的流不该接着旧半帧解，也不该
        拿着拔线前的版本码当现在的。累计量（`n_rx`/`n_tx`/各级错误）不清，它们是累计的。
        """
        self.parser.reset_stream()
        self.board_version = None
        self.version_seen = False
        self.link_status = None
        self.link_status_t = None
        self.open_gen += 1
        with self._chan_lock:
            self.link_up = True
            self.link_err = None

    def _restart_reader(self):
        """把收包线程重新起起来（只在上一个已经不在了的时候）。

        它的第一圈就会看见 `link_up` 是假，于是走重连那条路。
        """
        if self._thr is not None and self._thr.is_alive():
            return          # 上一个还活着（别让两个线程读同一个口，那会把字节流撕碎）
        self._stop_ev.clear()
        with self._chan_lock:
            self._alive = True
            self.reconnect_tries = 0
            self.reconnect_gave_up = False
        self._thr = threading.Thread(target=self._reader, name="sdk-rx", daemon=True)
        self._thr.start()

    # ---- 自动重连 ----

    def _reconnect_port(self):
        """这一轮该试哪条路径：原来那条还在就用它，否则**按 USB 身份重新认**。

        重枚举之后 tty 名字会变（`ttyACM0` → `ttyACM1`，by-id 那条名字也跟着变），所以
        "记住的那条路径"不能当唯一依据；认不到就还试原路径，下一轮再认一次。
        """
        path = self.port
        if path and os.path.exists(path):
            return path
        ident = self._port_ident or (usb_identity(path) if path else None)
        if ident is not None:
            got = match_ident(ident, _port_candidates())
            if got:
                return got
        return path

    def _reconnect(self):
        """掉线后自己重新开：退避着试，试上为止、或被 `close()` 打断、或试满上限。

        返回 True = 链路回来了（`link_up` 已置真）；False = 收包线程该退出了。
        """
        if not self.auto_reconnect:
            return False
        with self._chan_lock:
            self.reconnect_tries = 0
            self.reconnect_gave_up = False
        self._drop_serial()          # 死句柄先摘掉：断开期间 send() 要立刻抛，别往里写
        delay = self.reconnect_wait
        while self._alive:
            if self._stop_ev.wait(delay):    # close() 一置就醒，不用等满这一轮
                return False
            if not self._alive:
                return False
            with self._chan_lock:
                self.reconnect_tries += 1
            try:
                self._open_serial(self._reconnect_port())
            except LinkError as e:
                with self._chan_lock:
                    self.reconnect_err = str(e).splitlines()[0]
                if 0 < self.reconnect_max_tries <= self.reconnect_tries:
                    with self._chan_lock:
                        self.reconnect_gave_up = True
                    return False
                delay = min(delay * 2.0, self.reconnect_max_wait)
                continue
            if not self._alive:              # 刚开上就有人 close() 了
                self._drop_serial()
                return False
            self._on_serial_open()
            with self._chan_lock:
                self.n_reconnect += 1
                self.reconnect_err = None
            return True
        return False

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()
        return False

    @property
    def is_open(self):
        return self.ser is not None

    # ---- 发送 ----

    def send(self, tid, payload=b"", arm_id=protocol.DEFAULT_ARM_ID):
        """组一帧发出去。**返回只代表字节写出去了**，不代表板子受理了。

        加写锁是因为 pyserial 的 `write` 不保证整帧原子 —— 两个线程同时写会把两帧的字节
        交错在一起，板子两帧都解不出来（而且是静默的）。判空也在锁里：自动重连会在背后
        换 `self.ser`，锁外看一眼再进锁写就可能在中间被换成 None。

        断开期间抛 `NotConnected`（`LinkError` 的子类）；写失败还把链路置成断开，收包线程
        那边会接着去重连。
        """
        frame = protocol.encode(tid, payload, arm_id)
        with self._write_lock:
            ser = self.ser
            if ser is None:
                # 断开期间要能分清"忘了 open"和"掉了、正在自己重连"：后者是等一会儿的事
                if not self.link_up and self.auto_reconnect \
                        and self._thr is not None and self._thr.is_alive():
                    raise NotConnected(i18n.tr("link.down_reconnecting",
                                               self.reconnect_tries,
                                               self.reconnect_max_tries))
                raise NotConnected(i18n.tr("link.not_open"))
            try:
                ser.write(frame)
            except Exception as e:
                with self._chan_lock:
                    self.link_up = False
                    self.link_err = i18n.tr("link.write_link_err", e)
                raise LinkError(i18n.tr("link.write_failed", self.port, e)) from None
        self.n_tx += 1
        return len(frame)

    # ---- 每台臂的收包槽 ----

    def channel(self, arm_id):
        """拿（必要时建）某一台臂的收包槽。arm_id 越界直接报错 —— 越界的帧板子会静默丢弃。"""
        if not 0 <= arm_id < self.board_defs.ARM_NUM:
            raise ValueError(i18n.tr("link.arm_id_range",
                                     self.board_defs.ARM_NUM - 1, self.board_defs.ARM_NUM, arm_id))
        with self._chan_lock:
            ch = self._channels.get(arm_id)
            if ch is None:
                ch = self._channels[arm_id] = ArmChannel(arm_id)
            return ch

    def arm(self, arm_id):
        """拿（必要时建）一台 `Arm`。**同一个 arm_id 只会有一个 `Arm` 对象。**

        注意别把构造放在锁里：`Arm.__init__` 会调 `self.link.channel(arm_id)`，而那个方法
        要拿的正是这把锁 —— `threading.Lock` 不可重入，那样会当场死锁。
        """
        from .arm import Arm   # 延迟 import：arm.py 要用 Link，模块级 import 会成环
        with self._chan_lock:
            a = self._arms.get(arm_id)
        if a is None:
            a = Arm(arm_id, self)
            with self._chan_lock:
                a = self._arms.setdefault(arm_id, a)   # 万一有两个线程同时建，留先来的那个
        return a

    def arms(self):
        """所有已经建过的 `Arm`。"""
        with self._chan_lock:
            return list(self._arms.values())

    # ---- 收包线程 ----

    def _reader(self):
        """收包线程：读串口 → 交给 `inject_bytes`；`link_up` 是假时先重连。

        读写出错**不退出**（退了就再没人把链路接回来）：置成断开，下一圈走 `_reconnect`。
        真正退出的只有三种情况：`close()`、自动重连关着、以及这一轮试满了上限。
        """
        while self._alive:
            if not self.link_up:
                if not self._reconnect():
                    return
                continue
            try:
                chunk = self.ser.read(4096)
            except Exception as e:
                with self._chan_lock:
                    self.link_up = False
                    self.link_err = i18n.tr("link.read_link_err", e)
                continue
            if not chunk:
                continue
            try:
                self.inject_bytes(chunk)
            except Exception as e:
                with self._chan_lock:
                    self.link_up = False
                    self.link_err = i18n.tr("link.parse_link_err", e)
                continue

    def inject_bytes(self, chunk):
        """把**已经到手的字节**喂进解析器并按 `arm_id` 分流。

        真链路上这是收包线程调用的（`_reader` 就剩「读串口 + 交给这里」两件事）；
        假链路（`sdk.sim.SimLink`、自测里的回环替身）也走这一条 —— 切帧、分流、统计
        的口径因此完全一致，不会出现"模拟板子上好、真板子上坏"。
        """
        try:
            frames = self.parser.feed(chunk)
        except Exception as e:
            with self._chan_lock:
                self.link_up = False
                self.link_err = i18n.tr("link.parse_link_err", e)
            return
        now = time.monotonic()
        for tid, arm_id, payload in frames:
            self.n_rx += 1
            if arm_id >= self.board_defs.ARM_NUM:
                # 越界 arm_id：和板子同一个口径 —— 丢弃、计数、不报错
                self.n_bad_arm += 1
                continue
            if tid == protocol.TID_ARM_FEEDBACK and len(payload) == protocol.FB_LEN:
                try:
                    self.channel(arm_id).put_feedback(
                        protocol.decode_feedback(payload), now)
                except ValueError:
                    pass
            elif tid == protocol.TID_MOTOR_SET_ZERO_ACK and \
                    len(payload) == protocol.SET_ZERO_ACK_LEN:
                try:
                    self.channel(arm_id).put_ack(
                        protocol.decode_setzero_ack(payload), now)
                except ValueError:
                    pass
            elif tid == protocol.TID_VERSION and len(payload) == 4:
                self.board_version = protocol.decode_version(payload)
                self.version_seen = True
            elif tid == protocol.TID_LINK_STATUS and len(payload) == protocol.LINK_STATUS_LEN:
                # 链路级：**不上 channel**。板子每 100 ms 只发一帧，不带"是哪台臂"的语义
                # （帧头那个 arm_id 只是"哪台先握的手"），所以存 Link 上，和板子版本码同款。
                self.link_status = protocol.decode_link_status(payload)
                self.link_status_t = now
            else:
                # 认不出来的 tid：计数，不报错、不丢弃连接 —— 板子以后加包不该让老上位机炸
                self.channel(arm_id).note_unknown(tid)

    # ---- 等待 ----

    def wait_feedback(self, arm_id, after_seq=0, timeout=3.0, poll=0.005):
        """等到**序号大于 `after_seq`** 的一帧反馈，返回它；超时返回 None。

        按序号等（而不是"时间戳变了就算新帧"），所以没有竞态：调用前先取
        `ch.snapshot_feedback()[2]` 当 `after_seq`，就一定是"这一帧之后的"。
        """
        ch = self.channel(arm_id)
        deadline = time.monotonic() + timeout
        while True:
            fb, _, seq = ch.snapshot_feedback()
            if fb is not None and seq > after_seq:
                return fb
            if time.monotonic() >= deadline:
                return None
            time.sleep(poll)

    # ---- 诊断 ----

    @property
    def stats(self):
        return {
            "port": self.port,
            "kind": self.kind,
            "is_open": self.is_open,
            "link_up": self.link_up,
            "link_err": self.link_err,
            "n_tx": self.n_tx,
            "n_rx": self.n_rx,
            "n_bad_arm": self.n_bad_arm,
            "n_err_crc8": self.parser.n_err_crc8,
            "n_err_crc16": self.parser.n_err_crc16,
            "n_drop": self.parser.n_drop,
            "board_version": self.board_version,
            "link_status": self.link_status,
            "auto_reconnect": self.auto_reconnect,
            "reconnect_tries": self.reconnect_tries,
            "reconnect_max_tries": self.reconnect_max_tries,
            "reconnect_err": self.reconnect_err,
            "reconnect_gave_up": self.reconnect_gave_up,
            "n_reconnect": self.n_reconnect,
        }

    def describe(self):
        p = self.port or i18n.tr("link.auto_port")
        return i18n.tr("link.describe_line", p, kind_name(self.kind))

    def __repr__(self):
        return "<Link %s %s>" % ("open" if self.is_open else "closed", self.port)
