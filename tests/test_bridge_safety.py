"""Offline regression tests: hardware imports and UART writes are all faked."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]


def load_modules():
    config = types.ModuleType("Confg")
    for key, value in {
        "BENCH_MODE": False, "LIDAR_STOP_DIST_M": 1.2,
        "LIDAR_RESUME_DIST_M": 1.6, "LIDAR_MAX_VALID_DIST_M": 12.0,
        "LIDAR_STOP_CONFIRM_FRAMES": 3, "LIDAR_CLEAR_CONFIRM_FRAMES": 8,
    }.items():
        setattr(config, key, value)
    lidar_driver = types.ModuleType("rplidar")
    lidar_driver.RPLidar = mock.Mock(side_effect=AssertionError("Hardware forbidden"))
    lidar_driver.RPLidarException = RuntimeError

    def load(name, filename):
        spec = importlib.util.spec_from_file_location(name, ROOT / filename)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    with mock.patch.dict(sys.modules, {
        "Confg": config, "cv2": types.ModuleType("cv2"),
        "rplidar": lidar_driver, "serial": None,
    }), mock.patch.object(sys, "argv", ["offline-tests"]):
        lidar = load("offline_lidar", "lidar_safety.py")
        with mock.patch.dict(sys.modules, {"lidar_safety": lidar}):
            bridge = load("offline_bridge", "real_car_socket.py")
    return lidar, bridge


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class SenderSafetyTests(unittest.TestCase):
    def setUp(self):
        self.lidar, self.bridge = load_modules()
        self.clock = FakeClock()
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(mock.patch.object(self.lidar, "time", self.clock))
        self.stack.enter_context(mock.patch.object(self.bridge, "time", self.clock))
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.safety = self.lidar.LidarSafety()
        # Valid front/side returns. A real obstacle allows avoidance to start.
        self.scan = [(10, 130.0, 500.0)] * 20 + [
            (10, 190.0, 800.0), (10, 70.0, 600.0)]
        self.safety.update(self.scan)

    def run_sender(self, after_first=None, speed=15, alive=True, bench=False,
                   manual=False):
        command = self.bridge.SharedCommand()
        if alive:
            command.set(0, speed)
        clock = self.clock
        writes = []

        class StopEvent:
            count = 0
            stopped = False

            def is_set(inner):
                return inner.stopped

            def set(inner):
                inner.stopped = True

            def wait(inner, seconds):
                clock.sleep(seconds)
                inner.count += 1
                if inner.count == 1 and after_first:
                    after_first(command)
                # Keep AI alive so LiDAR tests don't pass due to AI timeout.
                if alive:
                    st, sp, _ = command.get()
                    command.set(st, sp)
                inner.stopped = inner.count >= 3

        with mock.patch.object(self.bridge, "BENCH_MODE", bench), \
                mock.patch.object(self.bridge, "MANUAL_CONTROL", manual), \
                mock.patch.object(self.bridge, "AVOIDANCE_TRIGGER_SEC", 0.0), \
                mock.patch.object(self.bridge, "write_stm32",
                                  side_effect=lambda _port, st, sp: writes.append((st, sp)) or True):
            self.bridge.sender_thread_fn(None, command, self.safety, StopEvent())
        # Final STOP writes must also remain in place on every exit path.
        self.assertEqual(writes[-5:], [(0, 0)] * 5)
        return writes[:3]

    def test_valid_obstacle_can_be_avoided(self):
        self.assertEqual(self.run_sender(), [(-18, 12)] * 3)

    def test_manual_driver_stops_at_obstacle_without_autonomous_avoidance(self):
        self.assertEqual(self.run_sender(manual=True), [(0, 0)] * 3)

    def test_unknown_side_clearance_does_not_start_avoidance(self):
        self.safety.update([(10, 130.0, 500.0)] * 20)
        self.assertEqual(self.run_sender(), [(0, 0)] * 3)

    def test_unknown_side_is_not_preferred_to_measured_clearance(self):
        self.safety.update([(10, 130.0, 500.0)] * 20 + [(10, 70.0, 800.0)])
        self.assertEqual(self.run_sender(), [(18, 12)] * 3)

    def test_losing_chosen_side_range_cancels_active_avoidance(self):
        writes = self.run_sender(lambda _: self.safety.update([(10, 130.0, 500.0)] * 20))
        self.assertEqual(writes, [(-18, 12), (0, 0), (0, 0)])

    def test_obstacle_on_chosen_side_cancels_active_avoidance(self):
        writes = self.run_sender(lambda _: self.safety.update(
            [(10, 130.0, 500.0)] * 20 + [(10, 190.0, 300.0)]))
        self.assertEqual(writes, [(-18, 12), (0, 0), (0, 0)])

    def test_reconnect_cancels_active_avoidance(self):
        writes = self.run_sender(lambda _: self.safety.force_stop("lidar reconnect"))
        self.assertEqual(writes, [(-18, 12), (0, 0), (0, 0)])

    def test_invalid_scan_cancels_active_avoidance(self):
        writes = self.run_sender(lambda _: self.safety.update([]))
        self.assertEqual(writes, [(-18, 12), (0, 0), (0, 0)])

    def test_stale_scan_cancels_active_avoidance_with_fresh_ai(self):
        writes = self.run_sender(lambda _: self.clock.sleep(2.0))
        self.assertEqual(writes, [(-18, 12), (0, 0), (0, 0)])

    def test_missing_front_range_cancels_active_avoidance(self):
        writes = self.run_sender(lambda _: self.safety.update([(10, 190.0, 800.0)] * 20))
        self.assertEqual(writes, [(-18, 12), (0, 0), (0, 0)])

    def test_ai_stop_cancels_active_avoidance(self):
        writes = self.run_sender(lambda cmd: cmd.set(0, 0))
        self.assertEqual(writes, [(-18, 12), (0, 0), (0, 0)])

    def test_zero_speed_never_starts_avoidance(self):
        self.assertEqual(self.run_sender(speed=0), [(0, 0)] * 3)

    def test_no_ai_never_starts_avoidance(self):
        self.assertEqual(self.run_sender(alive=False), [(0, 0)] * 3)

    def test_bench_does_not_avoid_obstacle(self):
        self.assertEqual(self.run_sender(bench=True), [(0, 0)] * 3)

    def test_explicit_bench_bypass_still_allows_command(self):
        self.safety.bypass("offline stand fixture")
        self.assertEqual(self.run_sender(bench=True), [(0, 15)] * 3)

    def test_force_stop_invalidates_cached_ranges(self):
        self.safety.force_stop("lidar reconnect")
        snapshot = self.safety.snapshot()
        self.assertFalse(snapshot["scan_valid"])
        self.assertFalse(snapshot["scan_fresh"])
        self.assertIsNone(snapshot["dist_mm"])
        self.assertIsNone(snapshot["left_clear_mm"])


if __name__ == "__main__":
    unittest.main()


class ImuTelemetryParsingTests(unittest.TestCase):
    """The IMU field grew gyro axes on 2026-09-12. Old and new firmware
    have to both parse, because the flashed firmware and the repo have
    disagreed before."""

    NEW = ("ENC,3249,3076,DELTA,0,0,TARGET,0,0,PWM,0,0,STEER,0,SPEED,0,"
           "MODE,S,IMU,-1.2,1.2,0.21,83.8,0.06,-0.13,4.81")
    OLD = ("ENC,3249,3076,DELTA,0,0,TARGET,0,0,PWM,0,0,STEER,0,SPEED,0,"
           "MODE,S,IMU,-1.2,1.2,0.21,83.8")

    def setUp(self):
        _, self.bridge = load_modules()

    def test_new_format_yields_all_three_gyro_axes(self):
        self.assertEqual(
            self.bridge.parse_imu_gyro(self.NEW), (0.06, -0.13, 4.81)
        )

    def test_gyro_fields_do_not_disturb_the_lift_detection_fields(self):
        # The lift detector reads roll/pitch/accel/yaw and must be blind
        # to anything appended after them.
        self.assertEqual(
            self.bridge.parse_imu_telemetry(self.NEW),
            self.bridge.parse_imu_telemetry(self.OLD),
        )
        self.assertEqual(
            self.bridge.parse_imu_telemetry(self.NEW), (-1.2, 1.2, 0.21, 83.8)
        )

    def test_old_firmware_reports_absence_not_zero(self):
        # Zero is a legitimate yaw rate. A missing measurement must not
        # be recorded as a still car.
        self.assertEqual(
            self.bridge.parse_imu_gyro(self.OLD), (None, None, None)
        )

    def test_sensor_not_ready_is_absence_too(self):
        line = "ENC,0,0,DELTA,0,0,TARGET,0,0,PWM,0,0,STEER,0,SPEED,0," \
               "MODE,S,IMU,NA,NA,NA,NA,NA,NA,NA"
        self.assertEqual(
            self.bridge.parse_imu_gyro(line), (None, None, None)
        )
        self.assertEqual(
            self.bridge.parse_imu_telemetry(line), (None, None, None, None)
        )

    def test_truncated_and_absent_fields_do_not_raise(self):
        for line in ("ENC,1,2,MODE,S", "", "IMU,", "IMU,1.0,2.0",
                     "IMU,1.0,2.0,3.0,4.0,5.0"):
            self.assertEqual(
                self.bridge.parse_imu_gyro(line), (None, None, None), line
            )


class LidarReasonLabelTests(unittest.TestCase):
    """A valid scan with no return in the front cone is not the hysteresis
    band. Reporting it as such sent me looking for an obstacle that was
    not there, in a run whose LiDAR was measurably healthy."""

    def setUp(self):
        self.lidar, _ = load_modules()
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.safety = self.lidar.LidarSafety()

    @staticmethod
    def ring(skip_front=False, dist=3000.0):
        """A full revolution, optionally with the front cone unsampled.

        The cone is 100..160 deg raw: FRONT_CENTER_DEG 130 +- FRONT_CONE_DEG
        30, matching analyze_scan.
        """
        points = [(10, i * 0.9, dist) for i in range(400)]
        if skip_front:
            points = [p for p in points if not (100.0 <= p[1] <= 160.0)]
        return points

    def test_empty_front_cone_is_not_called_hysteresis(self):
        scan = self.ring(skip_front=True)
        self.safety.update(scan)
        snap = self.safety.snapshot()

        self.assertTrue(snap["scan_valid"])
        self.assertIsNone(snap["dist_mm"])
        self.assertNotIn("hysteresis", snap["reason"])
        # 2026-09-15: front_raw == 0 (no raw hit at all in the cone) now
        # counts toward clear_confirm - see the reason for the fix in
        # update(). A single scan is not enough frames to reach GO, but
        # the reason must already say it is being counted, not stuck.
        self.assertEqual(snap["front_raw"], 0)
        self.assertEqual(snap["reason"], "dang xac nhan clear")

    def test_object_in_the_dead_band_still_reads_as_hysteresis(self):
        scan = self.ring()
        scan += [(10, 130.0 + d, 1430.0) for d in (-2, 0, 2)]
        self.safety.update(scan)
        snap = self.safety.snapshot()

        self.assertEqual(snap["reason"], "vung hysteresis")
        self.assertAlmostEqual(snap["dist_mm"], 1430.0, places=1)

    def test_a_truly_empty_front_cone_now_reaches_go(self):
        """2026-09-15: reversed from the earlier deliberate no-fix. Measured
        on the car (2026-09-15, open ground, 8m ahead): 69/69 scans had
        front_raw == 0 while total valid_points stayed 53-83 - the LiDAR was
        healthy, just returning nothing in that specific 60-degree arc at
        that range. The state machine could never leave STOP. front_raw == 0
        is the strongest possible evidence nothing is intercepting the beam:
        a real surface, however dark or however far within MAX_VALID_DIST_MM,
        returns at least a few weak hits - literally zero raw returns is not
        that. So it now counts as a clear frame."""
        scan = self.ring(skip_front=True)
        state = None
        for _ in range(self.lidar.LIDAR_CLEAR_CONFIRM_FRAMES):
            state = self.safety.update(scan)
        self.assertEqual(state, "GO")

    def test_a_real_close_obstacle_still_overrides_immediately(self):
        """The fix must never weaken the STOP side. An obstacle appearing
        after the car has accumulated clear-confirm frames from an empty
        cone must still force STOP on the very next scan."""
        empty = self.ring(skip_front=True)
        for _ in range(self.lidar.LIDAR_CLEAR_CONFIRM_FRAMES):
            self.safety.update(empty)
        self.assertEqual(self.safety.snapshot()["state"], "GO")

        obstacle = self.ring()
        obstacle += [(10, 130.0, 500.0)]
        for _ in range(self.lidar.LIDAR_STOP_CONFIRM_FRAMES):
            state = self.safety.update(obstacle)
        self.assertEqual(state, "STOP")
        self.assertEqual(self.safety.snapshot()["reason"], "obstacle")

    def test_a_filtered_return_in_the_cone_still_refuses_to_clear(self):
        """The narrow, deliberate case that must stay stuck: something DID
        reflect in the cone (front_raw > 0) but every hit was filtered -
        low quality (a dark or angled surface) or outside the valid
        distance band. That is real ambiguity, unlike front_raw == 0, and
        the fix must not touch it."""
        scan = self.ring(skip_front=True)
        # One point lands in the cone but fails the quality gate.
        scan += [(1, 130.0, 500.0)]
        state = None
        for _ in range(self.lidar.LIDAR_CLEAR_CONFIRM_FRAMES + 5):
            state = self.safety.update(scan)

        self.assertEqual(state, "STOP")
        snap = self.safety.snapshot()
        self.assertEqual(snap["front_raw"], 1)
        self.assertIn("khong co diem phan xa", snap["reason"])


class ImuLiftDetectionTimingTests(unittest.TestCase):
    """2026-09-15: PRINT_INTERVAL_MS halved from 200ms to 100ms on
    2026-09-14 (motor_test_bts7960/src/main.cpp), and imu_status.update()
    fires once per telemetry line - so every IMU_* reading-count constant
    silently represented half its documented real-world duration until
    this fix. Measured on the car: a normal startup jolt (wheels breaking
    static friction, ~0.2-0.3s) tripped the lift detector 3/3 times with
    the stale, halved window. These tests plant a synthetic jolt of that
    same shape and duration."""

    def setUp(self):
        _, self.bridge = load_modules()

    def steady(self, status, n, roll=0.0, pitch=0.0, accel=0.0):
        for _ in range(n):
            status.update(roll, pitch, accel)

    def jolt(self, status, n, roll=3.0, pitch=2.0, accel=0.6):
        """A brief burst of elevated-but-below-'sudden'-threshold noise,
        alternating sign so pstdev is genuinely high - the shape of a
        vibration, not a single spike."""
        for i in range(n):
            sign = 1 if i % 2 == 0 else -1
            status.update(sign * roll, sign * pitch, accel)

    def test_a_brief_startup_jolt_no_longer_trips_lift(self):
        status = self.bridge.ImuStatus()
        self.steady(status, self.bridge.IMU_WINDOW_SIZE)  # fill the window, settled
        # ~0.3s at the real 100ms telemetry rate = 3 samples.
        self.jolt(status, 3)
        self.assertFalse(status.snapshot()["lifted"])

    def test_a_sustained_lift_is_still_detected(self):
        status = self.bridge.ImuStatus()
        self.steady(status, self.bridge.IMU_WINDOW_SIZE)
        # A real pick-up keeps shaking for seconds, not one brief burst -
        # enough jolt samples to both fill the std window and satisfy the
        # confirm-readings count.
        self.jolt(status, self.bridge.IMU_WINDOW_SIZE + self.bridge.IMU_LIFT_CONFIRM_READINGS)
        self.assertTrue(status.snapshot()["lifted"])

    def test_sudden_path_is_untouched_by_the_timing_fix(self):
        """A real hard tilt/jerk must still trip immediately regardless of
        window size - this path does not depend on IMU_WINDOW_SIZE."""
        status = self.bridge.ImuStatus()
        self.steady(status, self.bridge.IMU_WINDOW_SIZE)
        for _ in range(self.bridge.IMU_LIFT_CONFIRM_READINGS):
            status.update(30.0, 0.0, 0.0)  # roll past IMU_LIFT_TILT_DEG=25
        self.assertTrue(status.snapshot()["lifted"])

    def test_window_and_confirm_readings_now_match_their_documented_durations(self):
        # 100ms is the real, current telemetry interval
        # (motor_test_bts7960/src/main.cpp PRINT_INTERVAL_MS).
        sample_interval_s = 0.1
        self.assertAlmostEqual(
            self.bridge.IMU_WINDOW_SIZE * sample_interval_s, 2.0, places=1
        )
        self.assertAlmostEqual(
            self.bridge.IMU_LIFT_CONFIRM_READINGS * sample_interval_s, 0.4, places=1
        )
        self.assertAlmostEqual(
            self.bridge.IMU_LIFT_CLEAR_READINGS * sample_interval_s, 1.2, places=1
        )
