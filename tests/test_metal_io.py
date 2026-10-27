"""Actual CAN worker/process paths over a fake transport; no devices opened."""
from collections import deque
import time
import unittest

from metal_arm_control import CANChannel, CANMotorIO, ProcessCANMotorIO, MotorRequest, MotorAction
from metal_arm_control.usb2can import CANFrame


class EchoTransport:
    """Minimal Damiao emulator at the raw frame boundary, not MotorIO."""
    def __init__(self, config):
        self.frames = []
        self.received = deque()
        self.enabled = [False]*7
        self.positions = [32767]*7
        self.closed = False

    def open(self):
        return self

    def send(self, frame):
        self.frames.append(frame)
        if frame.arbitration_id == 0x7ff:
            motor = frame.data[0]
        else:
            motor = frame.arbitration_id
            if frame.data[:7] == b'\xff'*7:
                if frame.data[7] == 0xfc: self.enabled[motor-1] = True
                if frame.data[7] == 0xfd: self.enabled[motor-1] = False
                if frame.data[7] == 0xfe: self.positions[motor-1] = 32767
            elif self.enabled[motor-1]:
                self.positions[motor-1] = int.from_bytes(frame.data[:2], 'big')
        p = self.positions[motor-1]
        status = 1 if self.enabled[motor-1] else 0
        self.received.append((CANFrame(motor+0x10, bytes(((status<<4)|motor, p>>8, p&255,
                                0x7f,0xf7,0xff,25,26))), time.monotonic_ns()))

    def receive(self, timeout=0):
        rows = tuple(self.received)
        self.received.clear()
        return rows

    def close(self):
        self.closed = True


def factory(config):
    return EchoTransport(config)


def req(arm, i, action, **kw):
    return MotorRequest(arm,i,i+1,'dm',action,0,**kw)


class IOTests(unittest.TestCase):
    def test_readonly_query_does_not_enable(self):
        transport = EchoTransport(None)
        with CANMotorIO([CANChannel(0,'fake')],read_only=True,transport_factory=lambda c:transport) as io:
            origin=time.monotonic()
            io.poll_feedback(0)
            io.query_feedback()
            io.flush()
            deadline=time.monotonic()+1
            seen=set()
            while time.monotonic()<deadline and len(seen)<7:
                seen.update(f.motor_index for f in io.poll_feedback(int((time.monotonic()-origin)*1000)))
                time.sleep(0.001)
            self.assertEqual(seen,set(range(7)))
            self.assertTrue(all(f.arbitration_id==0x7ff for f in transport.frames))
            self.assertFalse(any(transport.enabled))
            with self.assertRaises(PermissionError): io.write_motor_requests([req(0,0,MotorAction.ENABLE)])

    def test_worker_transmits_dual_arm_and_disables_every_motor_on_exit(self):
        transports={}
        def create(c):
            transports[c.arm_id]=EchoTransport(c)
            return transports[c.arm_id]
        with CANMotorIO([CANChannel(0,'left'),CANChannel(1,'right')],transport_factory=create) as io:
            origin=time.monotonic()
            io.poll_feedback(0)
            commands=[req(a,i,MotorAction.ENABLE) for a in range(2) for i in range(7)]
            commands += [req(a,i,MotorAction.CONTROL,position=-0.2 if i==1 else 0.2,kp=10,kd=1)
                         for a in range(2) for i in range(7)]
            io.write_motor_requests(commands)
            io.flush()
            deadline=time.monotonic()+1
            seen=set()
            while time.monotonic()<deadline and len(seen)<14:
                seen.update((f.arm_id,f.motor_index) for f in io.poll_feedback(int((time.monotonic()-origin)*1000)))
                time.sleep(0.001)
            self.assertEqual(len(seen),14)
            self.assertEqual(io.stats().get('query_tx_frames',0),0)
        for t in transports.values():
            self.assertTrue(t.closed)
            self.assertFalse(any(t.enabled))
            disabled={f.arbitration_id for f in t.frames if f.data==b'\xff'*7+b'\xfd'}
            self.assertEqual(disabled,set(range(1,8)))

    def test_fault_exit_delivers_disable(self):
        t=EchoTransport(None)
        io=CANMotorIO([CANChannel(0,'fake')],transport_factory=lambda c:t,command_timeout_ms=20).open()
        try:
            io.write_motor_requests([req(0,0,MotorAction.ENABLE), req(0,0,MotorAction.CONTROL,kp=10)])
            io.flush()
            deadline=time.monotonic()+1
            while not t.closed and time.monotonic()<deadline: time.sleep(.005)
            self.assertTrue(t.closed)
            self.assertFalse(any(t.enabled))
            with self.assertRaises(OSError): io.check_health()
        finally:
            try: io.close()
            except OSError: pass

    def test_spawned_process_routes_feedback(self):
        with ProcessCANMotorIO([CANChannel(0,'fake')],transport_factory=factory,read_only=True) as io:
            origin=time.monotonic()
            io.poll_feedback(0)
            deadline=time.monotonic()+3
            next_query=origin
            seen=set()
            while time.monotonic()<deadline and len(seen)<7:
                # A single reply can expire during CI process scheduling. Query as a
                # live read-only client would, while keeping the production age gate.
                now=time.monotonic()
                if now>=next_query:
                    io.query_feedback()
                    next_query=now+.05
                seen.update(f.motor_index for f in io.poll_feedback(int((time.monotonic()-origin)*1000)))
                time.sleep(.005)
            self.assertEqual(seen,set(range(7)))
            io.check_health()


if __name__=='__main__': unittest.main()
