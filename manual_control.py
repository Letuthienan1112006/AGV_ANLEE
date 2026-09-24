"""Keyboard driving for `run control`, with no hardware or model imports.

The automatic pipeline is untouched: camera, YOLO26 inference, person/vehicle
alerts, telemetry, video and the bridge's LiDAR safety layer all keep running
exactly as in `run road`/`run lane`. Only the final steer/speed pair comes
from the keys instead of the lane PID.

Keys (a terminal sends a repeated character while a key is held down, which
is what makes hold-to-drive work over SSH):
    w  di tien - giu de chay, nha ra la tu dung
    s  phanh - ve speed 0 ngay
    a  lai trai (giu de re lien tuc)
    d  lai phai
    space  dung khan cap (speed 0, steer 0)
    q / ESC  thoat

Two safety behaviours are deliberate, not incidental:

*Dead-man stop.* Speed falls to 0 when no key has arrived for
``idle_stop_sec``. Holding a key streams characters via the terminal's
auto-repeat, so "hold to go, release to stop" works; if SSH freezes, the
operator walks away, or the terminal dies, the car stops on its own instead
of latching the last command forever.

*Steering self-centres.* Steer decays to 0 after ``steer_return_sec`` with no
a/d, like a spring-return wheel - otherwise every turn has to be manually
undone and a forgotten steer becomes a slow circle.

REVERSE IS NOT AVAILABLE. The live firmware (motor_test_bts7960) clamps it
away in three places even though the H-bridge driver layer would accept a
negative PWM: startVelocityCommand does `speed = constrain(speed, 0,
SPEED_COMMAND_MAX)`, both wheel targets are constrained to >= 0, and
updateOneWheelPI returns 0 for `targetTicks <= 0` and clamps its output to
>= 0. So `s` brakes; it cannot back up. Reverse needs a firmware change plus
a reflash, and the LiDAR cone only looks forward, so there is no rear
sensing to make it safe - see the module notes in run.sh.
"""
import time

QUIT_KEYS = ("q", "\x1b", "\x03")  # q, ESC, Ctrl-C
STOP_KEYS = (" ",)
FORWARD_KEY = "w"
BRAKE_KEY = "s"
LEFT_KEY = "a"
RIGHT_KEY = "d"
CENTER_KEY = "x"


def clamp(value, low, high):
    return max(low, min(high, value))


class ManualDriver:
    """Turns a stream of keypresses into the (steer, speed) pair the loop
    already sends. Pure state - no terminal, no socket, so it is testable
    without hardware."""

    def __init__(self, speed_step=4, steer_step=4, max_speed=16, max_steer=18,
                 idle_stop_sec=1.2, steer_return_sec=0.5, clock=None):
        if idle_stop_sec <= 0 or steer_return_sec <= 0:
            raise ValueError("timeouts must be positive")
        self.speed_step = speed_step
        self.steer_step = steer_step
        self.max_speed = max_speed
        self.max_steer = max_steer
        self.idle_stop_sec = idle_stop_sec
        self.steer_return_sec = steer_return_sec
        self.clock = clock or time.monotonic
        self.speed = 0
        self.steer = 0
        self.quit = False
        self.last_key_time = None
        self.last_steer_key_time = None

    def feed(self, key):
        """Apply one keypress. Unknown keys count as activity (they refresh
        the dead-man timer) but change nothing else."""
        now = self.clock()
        self.last_key_time = now

        if key in QUIT_KEYS:
            self.quit = True
            self.speed = 0
            self.steer = 0
            return
        if key in STOP_KEYS:
            self.speed = 0
            self.steer = 0
            return
        if key == FORWARD_KEY:
            self.speed = clamp(self.speed + self.speed_step, 0, self.max_speed)
        elif key == BRAKE_KEY:
            # Not reverse - see the module docstring. Firmware cannot.
            self.speed = 0
        elif key == LEFT_KEY:
            self.steer = clamp(self.steer - self.steer_step,
                               -self.max_steer, self.max_steer)
            self.last_steer_key_time = now
        elif key == RIGHT_KEY:
            self.steer = clamp(self.steer + self.steer_step,
                               -self.max_steer, self.max_steer)
            self.last_steer_key_time = now
        elif key == CENTER_KEY:
            self.steer = 0

    def command(self):
        """The (steer, speed) to send this frame, after applying the
        dead-man stop and the steering self-centre."""
        now = self.clock()
        if self.last_key_time is None:
            return 0, 0
        if now - self.last_key_time >= self.idle_stop_sec:
            self.speed = 0
            self.steer = 0
        elif (self.last_steer_key_time is not None
                and now - self.last_steer_key_time >= self.steer_return_sec):
            self.steer = 0
        return int(self.steer), int(self.speed)

    @property
    def status(self):
        return "steer={:+3d} speed={:2d}".format(int(self.steer),
                                                 int(self.speed))


class RawKeyboard:
    """Reads single keys without Enter and without blocking the control
    loop. Context manager because a terminal left in raw mode is unusable
    afterwards - the restore has to happen even if the loop raises."""

    def __init__(self, stream=None):
        self.stream = stream
        self.fd = None
        self.saved = None

    def __enter__(self):
        import sys
        import termios
        import tty
        self.stream = self.stream or sys.stdin
        if not self.stream.isatty():
            # Piped stdin (a test harness, nohup) - nothing to put in raw
            # mode, and no keys will arrive either.
            return self
        self.fd = self.stream.fileno()
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        if self.saved is not None:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
        return False

    def pending_keys(self):
        """Every key buffered since the last call, oldest first. Returns []
        rather than blocking, so a frame with no input still gets sent."""
        import select
        if self.fd is None:
            return []
        keys = []
        while select.select([self.stream], [], [], 0)[0]:
            ch = self.stream.read(1)
            if not ch:
                break
            keys.append(ch)
        return keys
