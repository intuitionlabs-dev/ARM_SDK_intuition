"""H7 串口链路：端口选择、后台接收、协议解析和按 arm_id 分流。

同一串口只由一个进程和一个 Link 使用，双臂共享该链路。
每台臂须独立保活；命令预检由 Arm 执行。
读写失败后按配置退避重连，并通过 USB 身份查找重新枚举的设备。
pyserial 在打开串口时加载，协议操作和离线测试不依赖它。
"""

import glob
import os
import threading
import time

from . import i18n, protocol
from .board_defs import arx_x5 as board_defs, get_profile
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
    """返回当前语言的链路类型名称。"""
    return i18n.tr(KIND_NAMES.get(kind) or "link.kind_unknown")


def _read_sys(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def usb_identity(path):
    """沿 sysfs 查找 USB 身份；查不到返回 None，缺失序列号记为空串。"""
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
    """比较 VID:PID；两侧均有序列号时还须匹配序列号。不访问设备。"""
    if not a or not b:
        return False
    if a["idVendor"].lower() != b["idVendor"].lower() or \
            a["idProduct"].lower() != b["idProduct"].lower():
        return False
    sa, sb = a.get("serial") or "", b.get("serial") or ""
    return not (sa and sb) or sa == sb


def match_ident(ident, candidates):
    """从候选中返回 USB 身份匹配的首条路径，按字典序选择；无匹配返回 None。"""
    for path, other in sorted(candidates, key=lambda c: c[0]):
        if same_usb_ident(ident, other):
            return path
    return None


def classify_port(ident):
    """按 USB 身份返回 USB、RS485 或 None，不访问设备。"""
    if ident is None:
        return None
    vid = ident["idVendor"].lower()
    if vid == BOARD_VID and ident["idProduct"].lower() == BOARD_PID:
        return "USB"
    if vid == ADAPTER_VID:
        return "RS485"
    return None


def _pick_from(candidates):
    """优先选择板载 USB，其次 RS485；同类设备按路径字典序选择。"""
    for want in ("USB", "RS485"):
        for path, ident in sorted(candidates, key=lambda c: c[0]):
            if classify_port(ident) == want:
                return path
    return None


def _port_candidates():
    """枚举串口及 USB 身份，按真实路径去重。"""
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
    """返回候选链路列表：(路径, 类型, USB 身份)。"""
    out = []
    for path, ident in _port_candidates():
        out.append((path, classify_port(ident), ident))
    return out


def pick_default_port():
    """优先选择板载 USB，再选 RS485，最后使用回退路径；排除 SEGGER 探针。"""
    got = _pick_from(_port_candidates())
    return got if got else DEFAULT_PORT


def describe_port(path):
    """返回包含链路类型的单行端口描述。"""
    ident = usb_identity(path)
    if ident is None:
        return i18n.tr("link.desc_no_ident", path)
    kind = classify_port(ident)
    tag = (i18n.tr("link.desc_kind_tag", kind_name(kind)) if kind
           else i18n.tr("link.not_link_dev"))
    return i18n.tr("link.desc_port", path, ident["idVendor"], ident["idProduct"],
                   ident["product"], ident["manufacturer"], tag)


def link_hint():
    """返回 USB CDC 和 RS485 的接线及固件配置提示。"""
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
    """拒绝调试探针端口；优先核对 USB 身份，身份缺失时检查路径。"""
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
    """单臂收包状态，写入受锁保护；反馈和 ACK 使用不可变对象。"""

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
        """返回 (反馈, 接收时刻, 序号)；尚无反馈时第一项为 None。"""
        with self.lock:
            return self.feedback, self.feedback_t, self.feedback_seq

    def feedback_age(self):
        """返回反馈年龄，单位 s；尚无反馈时返回 None。"""
        with self.lock:
            t = self.feedback_t
        return None if t == 0.0 else time.monotonic() - t

    def snapshot_ack(self):
        with self.lock:
            return self.ack, self.ack_t, self.ack_seq


# ==================== 链路 ====================

class Link:
    """串口链路，后台接收，发送时加锁。

    robot_type 选择整条链路的上位机参数，默认 ARX，不修改固件。
    读写失败后按配置退避重连；达到重试上限后等待 reopen()。
    发送计数仅表示写入串口，命令是否执行应通过反馈判断。
    """

    board_defs = board_defs

    def __init__(self, port=None, baud=921600, *, timeout=0.1, verify_on_open=True,
                 auto_reconnect=True, reconnect_wait=RECONNECT_WAIT_S,
                 reconnect_max_wait=RECONNECT_MAX_WAIT_S,
                 reconnect_max_tries=RECONNECT_MAX_TRIES, robot_type=None):
        self.board_defs = type(self).board_defs if robot_type is None else get_profile(robot_type)
        self.gripper_calib_store = self.board_defs.default_gripper_store()
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
        """打开串口并启动接收线程；已打开或正在重连时直接返回。首次打开失败抛 LinkError。"""
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
        """停止接收和重连，关闭串口；可重复调用。"""
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
        """关闭后重新打开；失败抛 LinkError，启用自动重连时继续后台重试。"""
        self.close()
        try:
            return self.open(port)
        except LinkError:
            self._restart_reader()
            raise

    # ---- 串口句柄（自动重连要用，单独拆出来）----

    def _open_serial(self, path):
        """替换串口句柄，不改变线程或 link_up；失败抛 LinkError。"""
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
        """清除并关闭当前串口句柄。"""
        with self._write_lock:
            ser, self.ser = self.ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _after_open(self):
        """初始化连接状态并启动接收线程。"""
        self._on_serial_open()
        self._stop_ev.clear()
        with self._chan_lock:
            self._alive = True
            self.reconnect_tries = 0
            self.reconnect_gave_up = False
        self._thr = threading.Thread(target=self._reader, name="sdk-rx", daemon=True)
        self._thr.start()

    def _on_serial_open(self):
        """新连接建立后丢弃残留半帧和旧板级状态，保留累计统计。"""
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
        """在原接收线程已退出时启动重连线程。"""
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
        """优先复用原路径；路径失效时按 USB 身份查找，未找到则重试原路径。"""
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
        """退避重连，直到成功、关闭或达到上限。成功返回 True，否则返回 False。"""
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
        """加锁发送完整帧，返回写入字节数。

        未连接抛 NotConnected；写入失败标记断开，交由接收线程处理重连。
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
        """获取或创建指定臂的收包槽；编号越界抛 ValueError。"""
        if not 0 <= arm_id < self.board_defs.ARM_NUM:
            raise ValueError(i18n.tr("link.arm_id_range",
                                     self.board_defs.ARM_NUM - 1, self.board_defs.ARM_NUM, arm_id))
        with self._chan_lock:
            ch = self._channels.get(arm_id)
            if ch is None:
                ch = self._channels[arm_id] = ArmChannel(arm_id)
            return ch

    def arm(self, arm_id):
        """获取或创建指定臂，按 arm_id 缓存。构造在锁外进行，避免 channel() 重入死锁。"""
        from .arm import Arm   # 延迟 import：arm.py 要用 Link，模块级 import 会成环
        with self._chan_lock:
            a = self._arms.get(arm_id)
        if a is None:
            a = Arm(arm_id, self)
            with self._chan_lock:
                a = self._arms.setdefault(arm_id, a)   # 万一有两个线程同时建，留先来的那个
        return a

    def arms(self):
        """返回已创建的 Arm 对象。"""
        with self._chan_lock:
            return list(self._arms.values())

    # ---- 收包线程 ----

    def _reader(self):
        """读取串口并解析；断开后按配置重连，关闭或重试结束时退出。"""
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
        """解析输入字节并按 arm_id 分流，供实机、模拟器和测试共用。"""
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
        """等待序号大于 after_seq 的反馈；超时返回 None。"""
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
