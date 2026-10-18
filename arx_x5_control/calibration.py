"""Nonblocking motor zero calibration, ported from App/arm_zero.c."""

from .types import MotorAction, ZeroResult, ZeroState


class ZeroCalibration:
    def __init__(self, bank, include_encos=True):
        self.bank = bank
        self.include_encos = include_encos
        self.state = ZeroState.IDLE
        self.results = [ZeroResult.NONE] * 7
        self.retries = [0] * 7
        self.request_mask = None
        self.mask = 0
        self.index = 0
        self.started_ms = self.phase_ms = self.sent_ms = self.retry_ms = 0
        self.elapsed_ms = 0
        self.before_position = 0.0
        self.woken = False
        self.disable_confirmed = False
        self.wake_mask = self.ack_missing_mask = 0

    @property
    def busy(self):
        return self.state in (ZeroState.DISABLE, ZeroState.WAIT_DISABLE, ZeroState.SEND, ZeroState.CHECK)

    @property
    def pending(self):
        return self.request_mask is not None

    def request(self, mask=0):
        if self.busy or self.pending:
            return False
        if type(mask) is not int or not 0 <= mask <= 0x7f:
            raise ValueError("participation mask must be 0..127")
        self.request_mask = mask
        return True

    def _participants(self):
        return [i for i in range(7) if self.mask & (1 << i)]

    def _operation(self, i, action, now, output):
        request = self.bank.operation(i, action, now)
        if request is not None:
            output.append(request)
            return True
        return False

    def _disable(self, now, output):
        for i in self._participants():
            self._operation(i, MotorAction.DISABLE, now, output)

    def _finish(self, now, output, forced=False):
        if self.woken:
            self._operation(self.index, MotorAction.DISABLE, now, output)
            self.woken = False
        bad = forced or any(self.results[i] in (ZeroResult.NONE, ZeroResult.FAIL, ZeroResult.TIMEOUT)
                            for i in self._participants())
        if forced:
            for i in self._participants():
                if self.results[i] == ZeroResult.NONE:
                    self.results[i] = ZeroResult.TIMEOUT
        self.elapsed_ms = now - self.started_ms
        self.state = ZeroState.TIMEOUT if bad else ZeroState.DONE

    def cancel(self, now):
        output = []
        self.request_mask = None
        if self.busy:
            self._disable(now, output)
            self._finish(now, output, forced=True)
        return output

    def _advance(self, now, output):
        # Always undo a temporary DM wake, including a timeout path.
        if self.woken:
            self._operation(self.index, MotorAction.DISABLE, now, output)
            self.woken = False
        remaining = [i for i in self._participants() if i > self.index]
        self.phase_ms = now
        if remaining:
            self.index = remaining[0]
            self.state = ZeroState.SEND
        else:
            self._finish(now, output)

    def _resend_dm(self, now, output):
        if self.woken:
            self._operation(self.index, MotorAction.DISABLE, now, output)
            self.woken = False
        self._operation(self.index, MotorAction.SAVE_ZERO, now, output)
        self.retries[self.index] += 1
        self.sent_ms = now

    def _evaluate_dm(self, now, output):
        motor = self.bank.motors[self.index]
        if abs(motor.position) <= 0.01:
            self.results[self.index] = ZeroResult.UNCONFIRMED if abs(self.before_position) <= .01 else ZeroResult.OK
            if self.results[self.index] == ZeroResult.OK:
                motor.rounds = 0
            self._advance(now, output)
        else:
            self._resend_dm(now, output)

    def tick(self, now):
        output = []
        if not self.busy:
            if not self.pending:
                return output
            self.mask = self.request_mask or (0x3f if self.include_encos else 0x38)
            self.request_mask = None
            self.results = [ZeroResult.NONE if self.mask & (1 << i) else ZeroResult.SKIP for i in range(7)]
            self.retries = [0] * 7
            self.started_ms = self.phase_ms = self.retry_ms = now
            self.index = self._participants()[0]
            self.woken = False
            self.before_position = 0.0
            self.elapsed_ms = 0
            self.disable_confirmed = False
            self.wake_mask = self.ack_missing_mask = 0
            self._disable(now, output)
            self.state = ZeroState.WAIT_DISABLE
            return output
        for index in self._participants():
            self.bank.motors[index].keep_alive(now)
        self.elapsed_ms = now - self.started_ms
        if self.elapsed_ms >= 15000:
            self._finish(now, output, forced=True)
            return output
        if self.state == ZeroState.WAIT_DISABLE:
            disabled = all(self.bank.motors[i].error_code == 0
                           or now - self.bank.motors[i].last_rx_ms >= 300
                           for i in self._participants() if i >= 3)
            if disabled:
                self.disable_confirmed = True
                self.phase_ms = now
                self.state = ZeroState.SEND
            elif now - self.phase_ms >= 1500:
                self._finish(now, output, forced=True)
            elif now - self.retry_ms >= 100:
                self.retry_ms = now
                self._disable(now, output)
        elif self.state == ZeroState.SEND:
            motor = self.bank.motors[self.index]
            self.before_position = motor.position
            if self._operation(self.index, MotorAction.SAVE_ZERO, now, output):
                self.sent_ms = now
                self.state = ZeroState.CHECK
        elif self.state == ZeroState.CHECK:
            self._check(now, output)
        return output

    def _check(self, now, output):
        motor = self.bank.motors[self.index]
        limit = 2500 if motor.encos else 1000
        if now - self.phase_ms >= limit:
            if self.results[self.index] != ZeroResult.FAIL:
                self.results[self.index] = (ZeroResult.UNCONFIRMED if motor.encos and motor.zero_ack
                                            else ZeroResult.TIMEOUT)
            self._advance(now, output)
            return
        if motor.encos:
            if motor.zero_ack is None:
                if now - self.sent_ms < 1500:
                    return
            elif not motor.zero_ack:
                self.results[self.index] = ZeroResult.FAIL
                if self._operation(self.index, MotorAction.SAVE_ZERO, now, output):
                    self.retries[self.index] += 1
                    self.sent_ms = now
                return
            if motor.last_position_ms < self.sent_ms:
                if now - self.retry_ms >= 120:
                    self.retry_ms = now
                    self._operation(self.index, MotorAction.DISABLE, now, output)
                return
            if motor.zero_ack is None:
                self.ack_missing_mask |= 1 << self.index
            if abs(motor.position) <= .01:
                self.results[self.index] = (ZeroResult.UNCONFIRMED if abs(self.before_position) <= .01
                                            else ZeroResult.OK)
                self._advance(now, output)
            else:
                self.results[self.index] = ZeroResult.FAIL
                if self._operation(self.index, MotorAction.SAVE_ZERO, now, output):
                    self.retries[self.index] += 1
                    self.sent_ms = now
            return
        if not self.woken:
            if now - self.sent_ms < 20:
                return
            if motor.last_position_ms >= self.sent_ms:
                self._evaluate_dm(now, output)
            else:
                self._operation(self.index, MotorAction.ENABLE, now, output)
                self.woken = True
                self.wake_mask |= 1 << self.index
                self.sent_ms = now
        elif now - self.sent_ms >= 100:
            if motor.last_position_ms < self.sent_ms:
                self._resend_dm(now, output)
            else:
                self._evaluate_dm(now, output)
