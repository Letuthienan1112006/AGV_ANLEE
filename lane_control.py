"""Lane-control state with no model, socket, or hardware dependencies."""
import math
import time
from collections import deque


def clamp(value, low, high):
    return max(low, min(high, value))


class LanePID:
    def __init__(self, kp, ki, kd, max_output, integral_limit, step_limit,
                 clock=None, reset_integral_on_sign_cross=True,
                 integral_leak_tau=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_output = max_output
        self.integral_limit = integral_limit
        self.step_limit = step_limit
        # Two runs on this car produced OPPOSITE failures, which is why the
        # on/off flag is the wrong question:
        #   reset=True,  run 20260916_005537 frame 20: raw crossed +15.2 ->
        #     -12.3, the wipe took integral 68.6 -> 0 and the same-sign gate
        #     blocked rebuilding it, so the command was 0 at the exact frame
        #     the car began to swing; by frame 23 the error was -202px.
        #   reset=False, run 20260916_021246 frame 34: raw was +19.7 while
        #     the retained integral still commanded steer=-4 - output
        #     opposite to the error - and the next frame read -53.8/-13.
        # Both are real. The integral has no way to FORGET: it either keeps
        # history forever or loses it instantly. integral_leak_tau gives it
        # a half-life instead (seconds; 0 disables). Decay is applied over
        # elapsed dt, not per frame, because this loop runs 300-650ms and a
        # per-frame factor would silently change meaning with frame rate.
        self.reset_integral_on_sign_cross = reset_integral_on_sign_cross
        self.integral_leak_tau = integral_leak_tau
        self.clock = clock or time.monotonic
        self.error_history = deque(maxlen=5)
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.last_error = None
        self.last_measurement = None
        self.last_output = 0.0
        self.last_time = None
        self.error_history.clear()

    def compute(self, error, hold_integral=False, measured_error=None):
        """P/D use filtered error; raw error detects crossing before EMA does.

        Missing observations belong in LaneTrackingGuard. The first sample
        after reset has no derivative or elapsed-time integral.
        """
        measured_error = error if measured_error is None else measured_error
        if not math.isfinite(error) or not math.isfinite(measured_error):
            self.reset()
            raise ValueError("lane error must be finite")
        now = self.clock()
        dt = 0.0 if self.last_time is None else max(now - self.last_time, 0.001)
        derivative = 0.0 if self.last_error is None else (error - self.last_error) / dt

        # Forget old history smoothly. A standing error still builds the
        # integral up to about (error * tau), so steady-state bias is still
        # cancelled; what it can no longer do is carry a stale bias across a
        # crossing indefinitely (run 021246) or wind to the clamp on a
        # phantom offset (run 012523 reached the 100 cap in 3s on a
        # miscalibrated STEER_CENTER_X). Time-based, so a 300ms frame and a
        # 650ms frame decay by the right amount rather than the same amount.
        if self.integral_leak_tau > 0 and dt > 0:
            self.integral *= math.exp(-dt / self.integral_leak_tau)

        # Run 015830, frames 35/51: raw error crossed zero, while retained
        # integral kept steering toward the previous side. Clear that bias.
        if (self.reset_integral_on_sign_cross
                and self.last_measurement is not None
                and self.last_measurement * measured_error <= 0):
            self.integral = 0.0

        tentative = clamp(self.integral + error * dt,
                          -self.integral_limit, self.integral_limit)
        unsat = self.kp * error + self.ki * tentative + self.kd * derivative
        winding_up = abs(unsat) > self.max_output and unsat * error > 0
        # The same-sign gate is also EMA-lag-only: it stops the integral
        # rebuilding on the side the filtered error still points at. Without
        # smoothing error IS measured_error, so it is always satisfied - and
        # with the reset off it must not be what blocks accumulation either.
        same_side = (error * measured_error > 0
                     if self.reset_integral_on_sign_cross else True)
        if not hold_integral and not winding_up and same_side:
            self.integral = tentative

        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        out = clamp(out, self.last_output - self.step_limit,
                    self.last_output + self.step_limit)
        out = clamp(out, -self.max_output, self.max_output)
        self.last_output = out
        self.last_time = now
        self.last_error = error
        self.last_measurement = measured_error
        self.error_history.append(error)
        return int(out)


class LaneTrackingGuard:
    """Require consecutive line observations before starting or restarting."""
    def __init__(self, required_frames=3, lost_frames=1):
        if required_frames < 1 or lost_frames < 1:
            raise ValueError("confirmation/loss frame counts must be positive")
        self.required_frames = required_frames
        self.lost_frames = lost_frames
        self.good = 0
        self.lost = 0
        self.ready = False
        self.state = "VERIFY_LINE"

    def update(self, line_valid, blocked=False):
        if blocked:
            self.good = self.lost = 0
            self.ready = False
            self.state = "BLOCKED"
        elif not line_valid:
            self.good = 0
            self.lost += 1
            if self.lost >= self.lost_frames:
                self.ready = False
            self.state = "TRACK" if self.ready else "LINE_LOST"
        else:
            self.lost = 0
            self.good = min(self.good + 1, self.required_frames)
            self.ready = self.ready or self.good >= self.required_frames
            self.state = "TRACK" if self.ready else "VERIFY_LINE"
        return self.ready

    def command(self, steer, speed):
        return (steer, speed) if self.ready else (0, 0)
