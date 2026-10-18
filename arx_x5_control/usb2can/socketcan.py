"""通过 Linux SocketCAN 收发标准数据帧。"""

import os
import shutil
import socket
import struct
import subprocess
import time

from .frame import CANFrame


class SocketCANTransport:
    """使用内核接收时间戳的标准 CAN 原始套接字。"""

    _frame = struct.Struct("=IB3x8s")
    _timespec = struct.Struct("@ll")
    _timestampns = getattr(socket, "SO_TIMESTAMPNS", 35)

    def __init__(self, channel, *, bitrate=1000000):
        if bitrate != 1000000:
            raise ValueError("SocketCAN transport requires 1000000 bit/s")
        self._channel = channel
        self._configured = False
        self._index = socket.if_nametoindex(channel)
        self._socket = self._open_socket()

    def _open_socket(self):
        channel = self._channel
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            if self._timespec.size != 16:
                raise OSError("SocketCAN timestamps require a 64-bit Linux ABI")
            sock.setsockopt(socket.SOL_SOCKET, self._timestampns, 1)
            sock.setsockopt(socket.SOL_CAN_RAW, getattr(socket, "CAN_RAW_ERR_FILTER", 2),
                                    struct.pack("=I", 0x1FFFFFFF))
            sock.setblocking(False)
            sock.bind((channel,))
            return sock
        except BaseException:
            sock.close()
            raise

    def configure(self):
        """固定配置 1 Mbps、正常收发、16 帧队列，并启用接口。"""
        if self._configured:
            return
        if socket.if_nametoindex(self._channel) != self._index:
            raise OSError("SocketCAN interface was recreated")
        ip = shutil.which("ip")
        if ip is None:
            raise FileNotFoundError("SocketCAN configuration requires iproute2")
        deadline = time.monotonic()+2
        commands = (("down",), ("type", "can", "bitrate", "1000000", "listen-only", "off"),
                    ("txqueuelen", "16"), ("up",))
        try:
            for options in commands:
                self._run_ip(ip, options, deadline)
            if socket.if_nametoindex(self._channel) != self._index:
                raise OSError("SocketCAN interface was recreated")
            previous = self._socket
            self._socket = self._open_socket()
            previous.close()
        except BaseException:
            try:
                self._run_ip(ip, ("down",), deadline)
            except Exception:
                pass
            raise
        self._configured = True

    def _run_ip(self, ip, options, deadline):
        command = [ip, "link", "set", "dev", self._channel, *options]
        environment = dict(os.environ, LC_ALL="C")
        for privileged in (False, True):
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError("SocketCAN configuration exceeded 2 seconds")
            try:
                result = subprocess.run((["sudo", "-n"] if privileged else [])+command,
                                        capture_output=True, text=True, env=environment,
                                        timeout=remaining, check=False)
            except subprocess.TimeoutExpired as error:
                raise TimeoutError("SocketCAN configuration exceeded 2 seconds") from error
            if result.returncode == 0:
                return
            message = (result.stderr or result.stdout).strip()
            if (not privileged and os.geteuid() != 0
                    and any(text in message for text in ("Operation not permitted", "Permission denied"))):
                continue
            raise OSError(f"cannot configure SocketCAN {self._channel}: {message or result.returncode}")

    def send(self, frame):
        data = self._frame.pack(frame.arbitration_id, len(frame.data), frame.data.ljust(8, b"\0"))
        if self._socket.send(data) != len(data):
            raise OSError("incomplete SocketCAN write")

    def receive(self):
        if socket.if_nametoindex(self._channel) != self._index:
            raise OSError("SocketCAN interface was recreated")
        output = []
        for _ in range(128):
            try:
                data, ancillary, flags, _ = self._socket.recvmsg(16, socket.CMSG_SPACE(16))
            except BlockingIOError:
                break
            if flags & socket.MSG_DONTROUTE:
                continue
            if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC) or len(data) != 16:
                raise OSError("truncated SocketCAN frame")
            can_id, size, payload = self._frame.unpack(data)
            if can_id & 0x20000000:
                raise OSError(f"CAN bus error frame: {can_id:#x}")
            if can_id & 0xC0000000:
                continue
            if size > 8:
                raise OSError("invalid classic CAN length")
            stamps = [self._timespec.unpack(value) for level, kind, value in ancillary
                      if level == socket.SOL_SOCKET and kind == self._timestampns and len(value) == 16]
            if len(stamps) != 1:
                raise OSError("SocketCAN receive timestamp missing")
            seconds, nanos = stamps[0]
            age = time.time_ns() - (seconds*1_000_000_000 + nanos)
            if age < 0:
                raise OSError("SocketCAN receive clock discontinuity")
            output.append((CANFrame(can_id, payload[:size]), time.monotonic_ns()-age))
        return output

    def close(self):
        self._socket.close()
