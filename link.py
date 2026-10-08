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
（`Arm.keepalive()`），SDK 不起自动保活线程：板子的保活判据是"500 ms 内收到过**任意**
有效帧"，你的周期控制帧本身就是保活，再自动加一条只会和它抢时序。

`import sdk` 不会 import pyserial（`serial` 只在 `open()` 里 import），所以没装 pyserial
也能用 SDK 的协议层和离线自测。
"""

import glob
import os
import threading
import time

from . import board_defs, protocol
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

# 链路类型 → 人话。板子那侧对应 `App/inc/robot_state.h` 的 ROBOT_LINK。
# 两边选反了的现象是"一切都对但一个字节都没有"，所以选中了哪种要写出来。
KIND_NAMES = {"USB": "板子 USB CDC", "RS485": "USB 转 RS485 适配器",
              "FAKE": "假板子（sdk.sim，不接硬件）"}


def _read_sys(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def usb_identity(path):
    """这个 tty 是哪个 USB 设备：往上找到有 idVendor 的那一层。查不到返回 None。"""
    name = os.path.basename(os.path.realpath(path))
    d = os.path.realpath("/sys/class/tty/%s/device" % name)
    for _ in range(8):
        vid = _read_sys(os.path.join(d, "idVendor"))
        if vid:
            return {
                "idVendor": vid,
                "idProduct": _read_sys(os.path.join(d, "idProduct")) or "?",
                "product": _read_sys(os.path.join(d, "product")) or "",
                "manufacturer": _read_sys(os.path.join(d, "manufacturer")) or "",
            }
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
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
        return "%s（查不到 USB 身份）" % path
    kind = classify_port(ident)
    tag = "，%s" % KIND_NAMES[kind] if kind else "，**不是链路设备**"
    return "%s（USB %s:%s %s %s%s）" % (path, ident["idVendor"], ident["idProduct"],
                                       ident["product"], ident["manufacturer"], tag)


def link_hint():
    """没找到合适口子时给的提示：两条链路各要配什么，一起说清楚。"""
    lines = ["跑 UART 那条(ROBOT_LINK_UART)要接 USB 转 RS485 适配器；",
             "跑 USB 那条(ROBOT_LINK_USB)要把板子自己的 USB 口插上（%s:%s）。"
             % (BOARD_VID, BOARD_PID)]
    cands = list_links()
    if cands:
        lines.append("现在机器上看到这些候选：")
        for path, kind, _ in cands:
            lines.append("    %s%s" % (path, "  ← %s" % KIND_NAMES[kind] if kind else ""))
    else:
        lines.append("现在一个 ttyUSB*/ttyACM* 都没看到 —— 线插了吗？")
    lines.append("用 port= 可以指定口子。")
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
            raise LinkError(
                "拒绝打开 %s：查出来是 J-Link 探针的 VCOM 口。\n"
                "  探针要留给 JLink / 烧录用（占着它就连不上）。\n  %s"
                % (path, describe_port(path)))
        return
    # 查不到身份（没有 /sys、设备还没枚举出来）：名字像 SEGGER 的直接拒；
    # 名字是裸 ttyACM* 的也拒（宁可不连，也不赌它是探针）
    real = os.path.realpath(path)
    blob = (path + " " + real).lower()
    if any(h in blob for h in FORBIDDEN_HINTS) or \
            os.path.basename(real).lower().startswith("ttyacm"):
        raise LinkError(
            "拒绝打开 %s：查不到它的 USB 身份，而名字像 J-Link 的探针口。\n"
            "  别赌（探针那条多半也在 ttyACM* 上）。\n  %s" % (path, link_hint()))


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
    """

    def __init__(self, port=None, baud=921600, *, timeout=0.1, verify_on_open=True):
        self.port = port                  # None = open() 时自动挑
        self.baud = baud
        self.ser_timeout = timeout        # 读超时，决定收包线程多久醒一次
        self.verify_on_open = verify_on_open

        self.ser = None
        self.kind = None
        self.link_up = False
        self.link_err = None
        self.board_version = None         # 板子回过来的 0x01 里的版本码
        self.version_seen = False

        self.parser = protocol.FrameParser()
        self._channels = {}
        self._chan_lock = threading.Lock()
        self._arms = {}
        self._write_lock = threading.Lock()
        self._alive = False
        self._thr = None

        self.n_tx = 0
        self.n_rx = 0
        self.n_bad_arm = 0

    # ---- 生命周期 ----

    def open(self, port=None):
        """打开串口并起收包线程。已经开着就直接返回（幂等）。"""
        if self.is_open:
            return self
        if port is not None:
            self.port = port
        if self.port is None:
            self.port = pick_default_port()
        assert_port_allowed(self.port)

        import serial  # 只有真要连的时候才需要 pyserial

        try:
            # 板子 USB(CDC) 那条上波特率是**没有意义的** —— CDC 是包传输，tcsetattr 会把
            # SET_LINE_CODING 交给设备，STM32 那边 CDC_Control_HS 对它是 no-op 直接返回 OK。
            # 所以照设不误，只是别据此以为它有用。串口那条(RS485)上它才是真的。
            self.ser = serial.Serial(self.port, self.baud, timeout=self.ser_timeout)
        except Exception as e:
            raise LinkError("打不开 %s：%s\n%s" % (self.port, e, link_hint())) from None

        self.kind = classify_port(usb_identity(self.port))
        self._alive = True
        self.link_up = True
        self.link_err = None
        self._thr = threading.Thread(target=self._reader, name="sdk-rx", daemon=True)
        self._thr.start()
        return self

    def close(self):
        """停收包线程、关串口。幂等。"""
        self._alive = False
        if self._thr is not None:
            self._thr.join(timeout=1.0)
            self._thr = None
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.link_up = False

    def reopen(self, port=None):
        """关掉再开。读写出错（拔线、适配器重枚举）之后用这个复位。"""
        self.close()
        return self.open(port)

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
        交错在一起，板子两帧都解不出来（而且是静默的）。
        """
        if self.ser is None:
            raise NotConnected("链路还没 open（先 link.open() 或 arm.connect()）")
        frame = protocol.encode(tid, payload, arm_id)
        try:
            with self._write_lock:
                self.ser.write(frame)
        except Exception as e:
            with self._chan_lock:
                self.link_up = False
                self.link_err = "写串口失败: %s" % e
            raise LinkError("写 %s 失败：%s" % (self.port, e)) from None
        self.n_tx += 1
        return len(frame)

    # ---- 每台臂的收包槽 ----

    def channel(self, arm_id):
        """拿（必要时建）某一台臂的收包槽。arm_id 越界直接报错 —— 越界的帧板子会静默丢弃。"""
        if not 0 <= arm_id < board_defs.ARM_NUM:
            raise ValueError("arm_id 只能是 0..%d（板上 ARM_NUM=%d），收到 %r"
                             % (board_defs.ARM_NUM - 1, board_defs.ARM_NUM, arm_id))
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
        while self._alive:
            try:
                chunk = self.ser.read(4096)
            except Exception as e:
                with self._chan_lock:
                    self.link_up = False
                    self.link_err = "读串口失败: %s" % e
                return
            if not chunk:
                continue
            try:
                self.inject_bytes(chunk)
            except Exception as e:
                with self._chan_lock:
                    self.link_up = False
                    self.link_err = "解析失败: %s" % e
                return

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
                self.link_err = "解析失败: %s" % e
            return
        now = time.monotonic()
        for tid, arm_id, payload in frames:
            self.n_rx += 1
            if arm_id >= board_defs.ARM_NUM:
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
        }

    def describe(self):
        p = self.port or "(自动)"
        kind = KIND_NAMES.get(self.kind, "未知链路")
        return "%s，%s" % (p, kind)

    def __repr__(self):
        return "<Link %s %s>" % ("open" if self.is_open else "closed", self.port)
