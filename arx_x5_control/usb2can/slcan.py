"""通过串口收发 SLCAN 标准数据帧。"""

import os
import select
import time

from .frame import CANFrame


class CANableTransport:
    """由 CANMotorIO 后台线程独占的串口连接。"""

    def __init__(self, channel, *, bitrate=1000000, serial_factory=None):
        if bitrate != 1000000:
            raise ValueError("CANable transport requires 1000000 bit/s")
        if serial_factory is None:
            try:
                from serial import Serial
            except ImportError as error:
                raise ImportError("install arx-x5-control[can] for CANable support") from error
            serial_factory = Serial
        self._serial = serial_factory(channel, 115200, timeout=0, write_timeout=.02, exclusive=True)
        self._broken = False
        self._fd = self._serial.fileno() if os.name == "posix" and hasattr(self._serial, "fileno") else None
        self._poll = None
        if self._fd is not None:
            self._poll = select.poll()
            self._poll.register(self._fd, select.POLLIN | select.POLLERR | select.POLLHUP)
        self._send_count = 0
        self._buffer = bytearray()
        self._partial_at = None
        self._clock = time.monotonic_ns
        self._configured = False
        try:
            self.configure()
        except BaseException:
            self._serial.close()
            raise

    def configure(self):
        """固定使用 1 Mbps、正常模式和适配器重传配置。"""
        if self._configured:
            return
        for command in (b"C\r", b"S8\r", b"M0\r", b"A0\r"):
            self._write(command)
            time.sleep(.02)
        self._serial.reset_input_buffer()
        self._configured = True

    def start(self):
        self._serial.reset_input_buffer()
        self._write(b"O\r")

    def _write(self, data):
        if self._broken:
            raise OSError("SLCAN serial stream is incomplete")
        try:
            written = os.write(self._fd, data) if self._fd is not None else self._serial.write(data)
        except Exception:
            self._broken = True
            raise
        if written != len(data):
            self._broken = True
            raise OSError("incomplete SLCAN serial write")

    def send(self, frame):
        if self._send_count % 8 == 0 and getattr(self._serial, "out_waiting", 0) > 512:
            raise OSError("SLCAN transmit backlog exceeded")
        self._write(f"t{frame.arbitration_id:03X}{len(frame.data):X}{frame.data.hex().upper()}\r".encode("ascii"))
        self._send_count += 1

    def receive(self):
        if self._fd is None:
            data = self._serial.read(4096)
        else:
            events = self._poll.poll(0)
            if any(flags & (select.POLLERR | select.POLLHUP | select.POLLNVAL) for _, flags in events):
                raise OSError("SLCAN serial device disconnected")
            data = b""
            if events:
                try:
                    data = os.read(self._fd, 4096)
                except BlockingIOError:
                    pass
        if len(data) == 4096 and self._serial.in_waiting > 16384:
            raise OSError("SLCAN receive backlog exceeded")
        now = self._clock()
        if data and not self._buffer:
            self._partial_at = now
        self._buffer.extend(data)
        output = []
        while b"\r" in self._buffer:
            line, _, tail = self._buffer.partition(b"\r")
            received_at = self._partial_at or now
            self._buffer = bytearray(tail)
            self._partial_at = now if tail else None
            if not line:
                continue
            if line.startswith(b"\x07"):
                raise OSError("SLCAN adapter rejected a command")
            if line[:1] in (b"r", b"R", b"T"):
                continue
            if line[:1] != b"t":
                raise OSError("unexpected SLCAN adapter response")
            try:
                can_id, dlc = int(line[1:4], 16), int(line[4:5], 16)
                if len(line) != 5 + dlc*2 or dlc > 8:
                    raise ValueError("invalid payload length")
                frame = CANFrame(can_id, bytes.fromhex(line[5:].decode("ascii")))
            except (ValueError, UnicodeError) as error:
                raise OSError("invalid SLCAN data frame") from error
            output.append((frame, received_at))
        if len(self._buffer) > 128 or (self._partial_at is not None and now-self._partial_at > 20_000_000):
            raise OSError("incomplete SLCAN frame")
        return output

    def close(self):
        try:
            self._write(b"C\r")
        finally:
            self._serial.close()
