"""Hardware-free protocol, control, dynamics and public SDK contract tests."""
import importlib
import inspect
import math
from pathlib import Path
import unittest
from dataclasses import replace

import sdk
from metal_arm_control import (ArmConfig, ArmMode, Command, CommandKind as K,
    ControlSystem, MotorAction as A, MotorFeedback, MotorRequest, RobotState as S)
from metal_arm_control.can_protocol import encode_request, decode_feedback, feedback_queries
from metal_arm_control.config import (MOTOR_IDS, MOTOR_RX_IDS, MOTOR_PMAX, MOTOR_VMAX,
    MOTOR_TMAX, JOINT_MIN, JOINT_MAX, GRAVITY_SCALES)
from metal_arm_control.motor import Motor
from metal_arm_control.simulation import SimulatedMotors
from metal_arm_control.usb2can import CANFrame, TransportRegistry
from metal_arm_control.gravity_compensation import DynamicsCompensator
from metal_arm_control.sdk_adapter.link import LocalLink


def request(i=0, action=A.CONTROL, **values):
    return MotorRequest(0, i, MOTOR_IDS[i], 'dm', action, 0, **values)


def feedback_frame(i, p=0, v=0, t=0, status=1):
    # Independent integer bit packing of the vendor feedback wire layout.
    return CANFrame(0x11+i, bytes(((status << 4) | (i+1), p >> 8, p & 255,
        v >> 4, ((v & 15) << 4) | (t >> 8), t & 255, 31, 37)))


class ProtocolTests(unittest.TestCase):
    def test_special_commands_all_seven_motors(self):
        for i in range(7):
            for action, tail in ((A.ENABLE, 0xfc), (A.DISABLE, 0xfd), (A.SAVE_ZERO, 0xfe)):
                self.assertEqual(encode_request(request(i, action)), CANFrame(i+1, bytes([255]*7+[tail])))
            self.assertEqual(feedback_queries(i), (CANFrame(0x7ff, bytes((i+1, 0, 0xcc, 0))),))

    def test_zero_and_full_scale_commands(self):
        for i in range(7):
            self.assertEqual(encode_request(request(i)).data, bytes.fromhex('7fff7ff0000007ff'))
            self.assertEqual(encode_request(request(i, position=MOTOR_PMAX[i], velocity=MOTOR_VMAX[i],
                torque=MOTOR_TMAX[i], kp=500, kd=5)).data, b'\xff'*8)
            self.assertEqual(encode_request(request(i, position=-MOTOR_PMAX[i], velocity=-MOTOR_VMAX[i],
                torque=-MOTOR_TMAX[i])).data, b'\x00'*8)
            self.assertEqual(encode_request(request(i, A.SOFT_STOP, position=1, torque=12, kp=300)).data,
                             bytes.fromhex('7fff7ff0000007ff'))

    def test_per_joint_feedback_scales_and_status(self):
        for i in range(7):
            for maximum in (False, True):
                f = decode_feedback(1, feedback_frame(i, 65535 if maximum else 0,
                    4095 if maximum else 0, 4095 if maximum else 0, status=0xc), 42)
                sign = 1 if maximum else -1
                self.assertAlmostEqual(f.position, sign*MOTOR_PMAX[i])
                self.assertAlmostEqual(f.velocity, sign*MOTOR_VMAX[i])
                self.assertAlmostEqual(f.torque, sign*MOTOR_TMAX[i])
                self.assertEqual((f.arm_id, f.motor_index, f.timestamp_ms, f.error_code, f.temperature),
                                 (1, i, 42, 0xc, 37))

    def test_reject_x5_feedback_and_invalid_commands(self):
        self.assertIsNone(decode_feedback(0, CANFrame(0, b'\x11'+b'\x00'*7), 0))
        self.assertIsNone(decode_feedback(0, CANFrame(0x11, b'\x12'+b'\x00'*7), 0))
        self.assertIsNone(decode_feedback(0, CANFrame(0x11, b'\x11'), 0))
        for bad in (replace(request(), family='encos'), replace(request(), motor_id=8),
                    request(torque=float('nan')), request(kd=-1)):
            with self.assertRaises(ValueError):
                encode_request(bad)

    def test_transport_registry_uses_metal_namespace(self):
        registry = TransportRegistry()
        self.assertEqual(registry.names, ('slcan', 'socketcan'))
        for name in registry.names:
            self.assertTrue(registry.specification(name).class_path.startswith('metal_arm_control.'))


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.system = ControlSystem((ArmConfig(), ArmConfig()))
        self.plant = SimulatedMotors()
        self.now = 0
        self.sent = []
        self.auto_kinds = set()

    def step(self, commands=(), missing=(), fault=None):
        fb = self.plant.feedback(self.now)
        fb = [replace(f, error_code=0xc) if fault == (f.arm_id, f.motor_index) else f
              for f in fb if (f.arm_id, f.motor_index) not in missing]
        result = self.system.step(self.now, commands=commands, motor_feedback=fb)
        self.auto_kinds.update(a.command.auto for a in self.system.arms if a.command.auto is not None)
        self.plant.apply(result.motor_requests)
        self.sent.extend(result.motor_requests)
        self.now += 1
        return result

    def initialize(self):
        self.step([Command(i, K.CONNECT) for i in range(2)])
        self.step()
        self.step([Command(i, K.INITIALIZE) for i in range(2)])
        for _ in range(2200):
            self.step()
        self.assertEqual([a.state for a in self.system.arms], [S.NORMAL]*2)

    def test_initialization_preserves_gripper_zero(self):
        self.plant.positions[0][6] = 0.7
        self.initialize()
        self.assertTrue(all(a.initialized[6] for a in self.system.arms))
        self.assertFalse(any(r.action == A.SAVE_ZERO for r in self.sent))
        self.assertAlmostEqual(self.plant.positions[0][6], 0.7, places=5)
        self.assertTrue(all(r.family == 'dm' for r in self.sent))
        self.assertEqual(self.auto_kinds, {'reset'})

    def test_reinitialization_keeps_gripper_position_and_factory_zero(self):
        self.initialize()
        self.plant.positions[0][6] = 1.2
        self.sent.clear()
        self.step([Command(0, K.INITIALIZE)])
        for _ in range(2200): self.step()
        self.assertEqual(self.system.arms[0].state, S.NORMAL)
        self.assertEqual(self.system.arms[0].actual_mode, ArmMode.DAMPING)
        self.assertAlmostEqual(self.plant.positions[0][6], 1.2, places=5)
        self.assertFalse(any(r.action == A.SAVE_ZERO for r in self.sent))
        self.assertEqual(self.auto_kinds, {'reset'})

    def test_stop_keeps_enabling_all_motors_without_position_or_torque(self):
        self.initialize()
        self.step([Command(0, K.STOP)])
        for _ in range(4): self.step()
        self.sent.clear()
        for _ in range(250): self.step()
        requests = [r for r in self.sent if r.arm_id == 0]
        for i in range(7):
            enables = [r.timestamp_ms for r in requests if r.motor_index == i and r.action == A.ENABLE]
            self.assertGreaterEqual(len(enables), 3)
            self.assertTrue(all(75 <= b-a <= 78 for a,b in zip(enables,enables[1:])))
        self.assertFalse(any(r.action in (A.CONTROL, A.DISABLE, A.SAVE_ZERO) for r in requests))
        self.assertTrue(all((r.position,r.velocity,r.kp,r.kd,r.torque) == (0,0,0,0,0)
                            for r in requests if r.action == A.SOFT_STOP))
        self.assertEqual(self.system.arms[0].actual_mode, ArmMode.STOP)

    def test_periodic_enables_skip_calibration_participants_and_faults(self):
        self.step([Command(0, K.CONNECT)])
        for _ in range(4): self.step()
        self.step([Command(0, K.CALIBRATE)])
        for _ in range(400):
            # During calibration only the calibrator may temporarily wake the
            # current joint. Other participating joints must remain disabled.
            zero = self.system.arms[0].zero
            was_calibrating = zero.busy or zero.pending
            result = self.step()
            for r in result.motor_requests:
                if (was_calibrating or zero.busy) and r.arm_id == 0 and r.motor_index < 6 and r.action == A.ENABLE:
                    self.assertTrue(zero.wake_mask & (1 << r.motor_index))
        self.sent.clear()
        for _ in range(250): self.step(fault=(0,6))
        self.assertFalse(any(r.arm_id == 0 and r.motor_index == 6 and r.action == A.ENABLE
                             for r in self.sent))

    def test_motion_limits_and_watchdog(self):
        self.initialize()
        self.step([Command(0, K.START), Command(0, K.POSITION, (20,-30,30,0,0,0), 30)])
        for _ in range(500): self.step()
        a = self.system.arms[0]
        self.assertEqual(a.actual_mode, ArmMode.RUN)
        self.assertAlmostEqual(a.joint_angle[1], -30, delta=1)
        self.assertEqual(self.system.arms[1].actual_mode, ArmMode.DAMPING)
        self.step([Command(0, K.POSITION, (-999,999,999,-999,999,999), -20)])
        for _ in range(2): self.step()
        self.assertEqual(tuple(a.command.source), (JOINT_MIN[0],JOINT_MAX[1],JOINT_MAX[2],
                         JOINT_MIN[3],JOINT_MAX[4],JOINT_MAX[5]))
        self.assertEqual(a.command.gripper_angle, 0)
        for _ in range(3010): self.step()
        self.assertEqual(a.actual_mode, ArmMode.DAMPING)

    def test_motor_loss_and_fault_stop_whole_arm(self):
        self.initialize()
        self.step([Command(0, K.START)])
        for _ in range(4): self.step(fault=(0,1))
        self.assertEqual(self.system.arms[0].actual_mode, ArmMode.STOP)
        for _ in range(110): self.step(missing=((0,0),))
        self.assertEqual(self.system.arms[0].actual_mode, ArmMode.STOP)
        self.assertEqual(self.system.arms[1].actual_mode, ArmMode.DAMPING)

    def test_shutdown_disables_all_seven(self):
        self.initialize()
        result = self.system.shutdown(self.now)
        for arm in range(2):
            self.assertEqual({r.motor_id for r in result.motor_requests if r.arm_id == arm and r.action == A.DISABLE},
                             set(range(1,8)))

    def test_zero_calibration_targets_all_six_not_gripper(self):
        self.step([Command(0, K.CONNECT)])
        for _ in range(2): self.step()
        self.step([Command(0, K.CALIBRATE)])
        for _ in range(1500): self.step()
        zeroed = {r.motor_id for r in self.sent if r.action == A.SAVE_ZERO}
        self.assertEqual(zeroed, set(range(1,7)))
        self.assertEqual(self.system.arms[0].state, S.WAIT_INIT)

    def test_passive_modes_and_native_pd(self):
        m = Motor(0, 1)
        m.accept(MotorFeedback(0, 1, 0, position=-0.1, velocity=0, error_code=1))
        m.reference_deg = -10
        m.mode = ArmMode.RUN
        r = m.control(0)
        self.assertEqual((r.kp, r.kd, r.torque), (500,5,0))
        self.assertAlmostEqual(r.position, math.radians(-10))
        m.mode = ArmMode.GRAVITY
        m.output_comp = 3.2
        self.assertEqual((m.control(0).kp, m.control(0).kd, m.control(0).torque), (0,0,3.2))
        m.mode = ArmMode.DAMPING
        self.assertEqual((m.control(0).kp, m.control(0).kd), (0,5))


class DynamicsTests(unittest.TestCase):
    def test_gravity_matches_potential_energy_gradient(self):
        for name in ('metal_with_gripper','metal_no_gripper'):
            d = DynamicsCompensator.from_model(name)
            self.assertEqual(len(d.joint_names), 6)
            q = (0.2,-0.7,1.1,-0.3,0.4,0.1)
            gravity = d.raw_torques(q)
            for i in range(6):
                plus, minus = list(q), list(q)
                plus[i] += 1e-6; minus[i] -= 1e-6
                numerical = (d.potential_energy(plus)-d.potential_energy(minus))/2e-6
                self.assertAlmostEqual(gravity[i], numerical, places=6)
            self.assertEqual(d.sdk_torques(q), tuple(t*s for t,s in zip(gravity,GRAVITY_SCALES)))
            for actual, expected in zip(d.torques(q,(0,)*6,(0,)*6), d.sdk_torques(q)):
                self.assertAlmostEqual(actual, expected, places=10)


class SDKTests(unittest.TestCase):
    def test_module_layout_and_public_signatures(self):
        root=Path(__file__).resolve().parents[1]
        modules = {p.relative_to(root/'arx_x5_control') for p in (root/'arx_x5_control').rglob('*.py')}
        self.assertEqual(modules, {p.relative_to(root/'metal_arm_control') for p in (root/'metal_arm_control').rglob('*.py')})
        import arx_x5_control as arx
        import metal_arm_control as metal
        self.assertEqual(arx.__all__, metal.__all__)
        for module in ('arm','system','motor','can_io','can_process','sdk_adapter.link'):
            x=importlib.import_module('arx_x5_control.'+module)
            m=importlib.import_module('metal_arm_control.'+module)
            for name, cls in vars(x).items():
                if inspect.isclass(cls) and cls.__module__==x.__name__ and not name.startswith('_'):
                    other=getattr(m,name)
                    for method, fn in vars(cls).items():
                        if inspect.isfunction(fn) and (not method.startswith('_') or method=='__init__'):
                            # Parameter names/defaults, excluding package-specific annotations.
                            a=inspect.signature(fn); b=inspect.signature(getattr(other,method))
                            self.assertEqual([(p.name,p.default) for p in a.parameters.values()],
                                             [(p.name,p.default) for p in b.parameters.values()], (module,name,method))

    def test_public_arm_commands_and_units(self):
        link=LocalLink.simulated(background=False, configurations=[ArmConfig()])
        with link:
            arm=sdk.Arm(0,link)
            arm.send_version()
            for now in range(4): link.step(now)
            self.assertEqual(arm.robot_state,link.board_defs.STATE_WAIT_INIT)
            arm.init_arm()
            for now in range(4,2210): link.step(now)
            self.assertEqual(arm.robot_state,link.board_defs.STATE_NORMAL)
            arm.start()
            for now in range(2210,2220): link.step(now)
            arm.send_ctrl_pos((0,-0.2,0.2,0,0,0),gripper_m=0.05)
            for now in range(2220,2600): link.step(now)
            self.assertAlmostEqual(arm.joint_angles[1],-0.2,delta=0.02)
            self.assertAlmostEqual(arm.gripper_opening,0.05,delta=0.002)
            self.assertIs(arm.board_defs,link.board_defs)
            self.assertNotEqual(arm.board_defs.JOINT_ANGLE_MIN_RAD,sdk.board_defs.arx_x5.JOINT_ANGLE_MIN_RAD)
            arm.stop()
            for now in range(2600,2610): link.step(now)
            self.assertEqual(arm.mode,link.board_defs.MODE_STOP)

    def test_factory_gripper_table(self):
        arm=sdk.Arm(0,LocalLink.simulated(background=False))
        calibration=arm.calibration
        self.assertAlmostEqual(calibration.raw_deg_from_opening(0.05),math.degrees(1.10383))
        self.assertEqual(calibration.opening_from_raw_deg(math.degrees(1.10383)),0.05)
        self.assertAlmostEqual(calibration.raw_deg_from_opening(0),math.degrees(0.002))
        for bad in (-0.001,0.101,float('nan')):
            with self.assertRaises(sdk.CalibrationError): calibration.raw_deg_from_opening(bad)


class UITests(unittest.TestCase):
    def test_metal_session_limits_and_rendering(self):
        import time
        from unittest.mock import patch
        from metal_arm_control.sdk_adapter.tui import parse_options, LocalConsoleSession
        from metal_arm_control.sdk_adapter.tui_app import LocalUi
        class Screen:
            def __init__(self): self.rows=[]
            def getmaxyx(self): return (80,180)
            def erase(self): pass
            def addstr(self,*args): self.rows.append(args)
            def noutrefresh(self): pass
        options=parse_options(['--simulate'])
        with LocalLink.simulated(background=False) as link:
            session=LocalConsoleSession(link,options)
            for arm in session.arms.values(): arm.send_version()
            for now in range(4): link.step(now)
            session.tick(time.monotonic())
            session.seed_from_feedback(session.fb(),'test')
            session.target[1]=0.0
            session.jog(1,-1)
            self.assertLess(session.target[1],0)
            self.assertEqual(session.gripper_row(session.fb())['hi'],0.1)
            self.assertEqual(session.gripper_row(session.fb())['lo'],0.0)
            screen=Screen()
            with patch('curses.color_pair',return_value=0), patch('curses.doupdate'):
                ui=LocalUi(screen,session,options)
                ui.draw()
            self.assertTrue(screen.rows)


if __name__=='__main__': unittest.main()
