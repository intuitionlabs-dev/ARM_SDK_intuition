"""单片机串口链路：端口选择、后台接收、协议解析和按 arm_id 分流。

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

# 端口选择：按 USB 身份匹配，避免依赖设备路径。

BY_ID_DIR = "/dev/serial/by-id"
DEFAULT_PORT = BY_ID_DIR + "/usb-1a86_USB_Serial-if00-port0"

SEGGER_VID = "1366"     # 端口校验使用的厂商标识
ADAPTER_VID = "1a86"    # RS485 适配器识别标识
BOARD_VID = "0483"      # USB CDC 识别标识
BOARD_PID = "5740"

FORBIDDEN_HINTS = ("segger", "j-link", "jlink")

# 断开后按配置退避重试，达到上限后等待人工重开。

RECONNECT_WAIT_S = 0.5        # 首次重试等待时间
RECONNECT_MAX_WAIT_S = 2.0    # 最大退避时间
RECONNECT_MAX_TRIES = 10      # 一轮掉线最多试几次（<=0 = 不限）

# 保存翻译键，在显示时取文案，以支持运行时切换语言。
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
    """沿 sysfs 查找 USB 身份；查不到返回 None。"""
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
    """根据 USB 身份字段判断是否为同一设备。"""
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
    """优先选择受支持的 USB CDC，其次 RS485；同类设备按路径字典序选择。"""
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
    """优先选择受支持的 USB CDC，再选 RS485，最后使用回退路径。"""
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
    """返回链路类型和候选端口说明。"""
    lines = [i18n.tr("link.hint_uart"),
             i18n.tr("link.hint_usb")]
    cands = list_links()
    if cands:
        lines.append(i18n.tr("link.hint_cands"))
        for path, kind, _ in cands:
            # 固定排版，不参与翻译。
            lines.append("    %s%s" % (path, ("  ← %s" % kind_name(kind)) if kind else ""))
    else:
        lines.append(i18n.tr("link.hint_no_tty"))
    lines.append(i18n.tr("link.hint_port_arg"))
    return "\n".join(lines)


def assert_port_allowed(path):
    """校验端口是否可用于控制链路。"""
    if not path:
        return
    ident = usb_identity(path)
    if ident is not None:
        blob = (ident["manufacturer"] + " " + ident["product"]).lower()
        if ident["idVendor"].lower() == SEGGER_VID or any(h in blob for h in FORBIDDEN_HINTS):
            raise LinkError(i18n.tr("link.reject_probe_ident", path, describe_port(path)))
        return
    # USB 身份缺失时按路径执行端口校验。
    real = os.path.realpath(path)
    blob = (path + " " + real).lower()
    if any(h in blob for h in FORBIDDEN_HINTS) or \
            os.path.basename(real).lower().startswith("ttyacm"):
        raise LinkError(i18n.tr("link.reject_probe_name", path, link_hint()))


# 每台臂的收包槽

class ArmChannel:
    """单臂收包状态，写入受锁保护；反馈和 ACK 使用不可变对象。"""

    __slots__ = ("arm_id", "lock", "feedback", "feedback_t", "feedback_seq",
                 "n_feedback", "ack", "ack_t", "ack_seq", "ack_event", "unknown_tids",
                 "parameters", "parameters_t", "parameters_seq")

    def __init__(self, arm_id):
        self.arm_id = arm_id
        self.lock = threading.Lock()
        self.feedback = None       # protocol.Feedback
        self.feedback_t = 0.0      # 单调时钟；0 表示尚无反馈。
        self.feedback_seq = 0      # 反馈序号，用于等待新帧
        self.n_feedback = 0
        self.ack = None            # protocol.SetZeroAck
        self.ack_t = 0.0
        self.ack_seq = 0
        self.ack_event = threading.Event()
        self.unknown_tids = {}     # tid -> 计数
        self.parameters = None
        self.parameters_t = 0.0
        self.parameters_seq = 0

    # 写侧（只有 RX 线程调）

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

    def put_parameters(self, parameters, ts):
        with self.lock:
            self.parameters = parameters
            self.parameters_t = ts
            self.parameters_seq += 1

    def snapshot_parameters(self):
        """返回参数反馈、接收时刻和序号。"""
        with self.lock:
            return self.parameters, self.parameters_t, self.parameters_seq

    # 读侧

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


# 链路

class Link:
    """连接单片机的串口链路，后台接收，发送时加锁。

    robot_type 选择整条链路的参数，默认 ARX。
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
        self.ser_timeout = timeout        # 串口读取超时
        self.verify_on_open = verify_on_open
        self.auto_reconnect = auto_reconnect
        self.reconnect_wait = reconnect_wait
        self.reconnect_max_wait = reconnect_max_wait
        self.reconnect_max_tries = reconnect_max_tries

        self.ser = None
        self.kind = None
        self.link_up = False
        self.link_err = None
        self.board_version = None         # 最近收到的单片机协议版本码
        self.version_seen = False
        self.link_status = None           # 最近的链路级 0x81 状态
        self.link_status_t = None         # 接收时刻，使用单调时钟

        self.open_gen = 0                 # 每次连接成功递增
        self.n_reconnect = 0              # 累计重连成功次数
        self.reconnect_tries = 0          # 本轮重试次数
        self.reconnect_err = None         # 最近的重连错误
        self.reconnect_gave_up = False    # 是否已达到重试上限

        self.parser = protocol.FrameParser()
        self._channels = {}
        self._chan_lock = threading.Lock()   # 保护收包槽、臂对象及链路状态
        self._arms = {}
        self._write_lock = threading.Lock()
        self._alive = False
        self._thr = None
        self._port_ident = None           # 重连时用于匹配设备
        self._stop_ev = threading.Event() # 关闭时中断重连等待

        self.n_tx = 0
        self.n_rx = 0
        self.n_bad_arm = 0

    # 生命周期

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

    # 串口句柄管理

    def _open_serial(self, path):
        """替换串口句柄，不改变线程或 link_up；失败抛 LinkError。"""
        assert_port_allowed(path)
        import serial  # 打开串口时才加载 pyserial。

        try:
            # USB CDC 不使用波特率控制传输速度；RS485 使用该设置。
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
            for channel in self._channels.values():
                with channel.lock:
                    channel.parameters = None
                    channel.parameters_t = 0.0
            self.link_up = True
            self.link_err = None

    def _restart_reader(self):
        """在原接收线程已退出时启动重连线程。"""
        if self._thr is not None and self._thr.is_alive():
            return          # 接收线程已运行，避免重复启动。
        self._stop_ev.clear()
        with self._chan_lock:
            self._alive = True
            self.reconnect_tries = 0
            self.reconnect_gave_up = False
        self._thr = threading.Thread(target=self._reader, name="sdk-rx", daemon=True)
        self._thr.start()

    # 自动重连

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
        self._drop_serial()          # 清除失效句柄，使后续发送报告未连接。
        delay = self.reconnect_wait
        while self._alive:
            if self._stop_ev.wait(delay):    # 关闭时立即结束等待。
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
            if not self._alive:              # 连接成功后再次检查是否已请求关闭。
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

    # 发送

    def send(self, tid, payload=b"", arm_id=protocol.DEFAULT_ARM_ID):
        """加锁发送完整帧，返回写入字节数。

        未连接抛 NotConnected；写入失败标记断开，交由接收线程处理重连。
        """
        frame = protocol.encode(tid, payload, arm_id)
        with self._write_lock:
            ser = self.ser
            if ser is None:
                # 区分未打开与重连中的链路。
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

    # 每台臂的收包槽

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
        from .arm import Arm   # 延迟导入，避免 arm 与 link 循环依赖。
        with self._chan_lock:
            a = self._arms.get(arm_id)
        if a is None:
            a = Arm(arm_id, self)
            with self._chan_lock:
                a = self._arms.setdefault(arm_id, a)   # 并发创建时保留先缓存的对象。
        return a

    def arms(self):
        """返回已创建的 Arm 对象。"""
        with self._chan_lock:
            return list(self._arms.values())

    # 收包线程

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
                # 丢弃并统计越界臂编号。
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
            elif tid == protocol.TID_PARAMETER_FEEDBACK:
                try:
                    self.channel(arm_id).put_parameters(
                        protocol.decode_parameter_feedback(payload), now)
                except ValueError:
                    pass
            elif tid == protocol.TID_LINK_STATUS and len(payload) == protocol.LINK_STATUS_LEN:
                # 0x81 属于整条链路，存入 Link，不按臂分流。
                self.link_status = protocol.decode_link_status(payload)
                self.link_status_t = now
            else:
                # 未知帧仅计数，保持连接以兼容协议扩展。
                self.channel(arm_id).note_unknown(tid)

    # 等待

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

    # 链路状态

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
