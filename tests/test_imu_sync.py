"""
Tests for tools/sync_imu.py against synthetic data with a KNOWN offset,
latency and steering authority.

The point of this file is that the tool recovers numbers we planted, and
refuses when the recording protocol was not followed. Three bugs were found
this way and none of them were visible by reading the code:

  - the sync window must stop where the wheels start: once the car drives,
    steering turns yaw into a long ramp, and two ramps correlate at almost
    any lag, so the peak disappears
  - a long lag leaves a sliver of overlap that correlates well by accident,
    which swallowed the peak margin
  - the offset sign was inverted in both measurement functions, invisible
    while the test data happened to have a zero offset
  - averaging a whole constant-steer segment includes the previous
    command's response, usually of the opposite sign, which biases the
    steering-authority slope toward zero
"""
import math
import os
from pathlib import Path
import random
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import sync_imu


TRUE_OFFSET = 7.350      # recorder started this long before the run
TRUE_LATENCY = 0.420     # s, command to yaw-rate response
TRUE_AUTHORITY = 1.80    # deg/s per unit of steer command
WHEEL_HZ = 5.0           # BNO055 via the STM32 telemetry line
IMU_HZ = 95.0            # MTi-630R
DURATION = 70.0
DT = 0.002


def steer_cmd(t):
    if t < 25.0:
        return 0.0       # the car is stationary, being rotated by hand
    steps = [0, 6, -6, 8, -8, 4, -4, 7, -7, 5, 0, -5]
    return float(steps[int((t - 25.0) / 3.5) % len(steps)])


def hand_rotation(t, shape="asymmetric"):
    """The sync mark, in degrees of body yaw."""
    if shape == "none":
        return 0.0
    if shape == "rhythmic":
        # What not to do: equal correlation peaks every 4 s.
        return 25.0 * math.sin(2 * math.pi * (t - 6.0) / 4.0) \
            if 6.0 <= t <= 18.0 else 0.0

    keys = [(6.0, 0.0), (8.0, 40.0), (11.0, 40.0),
            (12.5, -15.0), (15.0, -15.0), (17.0, 0.0)]
    if t <= keys[0][0] or t >= keys[-1][0]:
        return 0.0
    for (t0, v0), (t1, v1) in zip(keys, keys[1:]):
        if t0 <= t <= t1:
            u = (t - t0) / (t1 - t0)
            return v0 + (v1 - v0) * u * u * (3 - 2 * u)
    return 0.0


def build(shape="asymmetric"):
    """Integrate the true yaw rate once, so both files describe one reality."""
    times, yaws, rates = [], [], []
    yaw = 137.0
    t = 0.0
    while t <= DURATION:
        rate = (
            (hand_rotation(t + 0.01, shape) - hand_rotation(t - 0.01, shape))
            / 0.02
            + TRUE_AUTHORITY * steer_cmd(t - TRUE_LATENCY)
        )
        times.append(t)
        yaws.append(yaw)
        rates.append(rate)
        yaw += rate * DT
        t += DT
    return yaws, rates


def write_pair(directory, shape="asymmetric"):
    yaws, rates = build(shape)
    random.seed(7)

    def at(t):
        i = min(int(t / DT), len(yaws) - 1)
        return yaws[i], rates[i]

    wheel_path = os.path.join(directory, "wheel_telem.csv")
    with open(wheel_path, "w") as handle:
        handle.write("t,cmd_steer,cmd_speed,enc_l,enc_r,tgt_l,tgt_r,"
                     "pwm_l,pwm_r,fw_steer,fw_speed,mode,yaw\n")
        for k in range(int(DURATION * WHEEL_HZ)):
            t = k / WHEEL_HZ
            yaw = round(at(t)[0], 1) % 360.0      # 0.1 deg resolution
            steer = steer_cmd(t)
            enc = 0 if t < 22.0 else 30           # wheels roll from 22 s
            handle.write(
                "%.3f,%.1f,16.0,%d,%d,24,24,90,90,%.1f,16,D,%.1f\n"
                % (t, steer, enc, enc, steer, yaw)
            )

    imu_path = os.path.join(directory, "imu_log.csv")
    with open(imu_path, "w") as handle:
        handle.write("t_wall,t_msg,wx,wy,wz,ax,ay,az,roll,pitch,yaw\n")
        base = 1789000000.0
        for k in range(int((DURATION + TRUE_OFFSET) * IMU_HZ)):
            t_car = k / IMU_HZ - TRUE_OFFSET
            if not (-TRUE_OFFSET <= t_car <= DURATION):
                continue
            yaw, rate = at(max(t_car, 0.0))
            if t_car < 0.0:
                yaw, rate = yaws[0], 0.0
            yaw += random.gauss(0, 0.005)         # measured MTi noise
            rate += random.gauss(0, 0.02)
            stamp = base + k / IMU_HZ
            handle.write(
                "%.6f,%.6f,+0.0,+0.0,%+.6f,+0.1,+0.1,+9.8,+0.1,+0.1,%.4f\n"
                % (stamp, stamp, math.radians(rate), yaw % 360.0)
            )

    return imu_path, wheel_path


class SyncTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)

    def load(self, shape="asymmetric"):
        imu_path, wheel_path = write_pair(self.temp.name, shape)
        imu = sync_imu.read_imu(imu_path)
        return imu, sync_imu.read_wheel(wheel_path)

    def test_recovers_the_planted_offset(self):
        (imu_t, imu_yaw, _), wheel = self.load()
        sync = sync_imu.find_offset(imu_t, imu_yaw, wheel)

        self.assertTrue(sync["ok"], sync.get("why"))
        # Convention: imu_t + offset == wheel_t, so the recorder's head
        # start appears as a negative offset.
        self.assertAlmostEqual(sync["offset"], -TRUE_OFFSET, delta=0.10)
        self.assertGreater(abs(sync["r"]), 0.99)

    def test_sync_window_stops_where_the_wheels_start(self):
        _, wheel = self.load()
        # Past this point steering turns yaw into a long ramp, and two
        # ramps correlate at almost any lag.
        self.assertAlmostEqual(sync_imu.stationary_end(wheel), 22.4, delta=0.3)

    def test_latency_brackets_the_planted_value(self):
        (imu_t, imu_yaw, imu_wz), wheel = self.load()
        sync = sync_imu.find_offset(imu_t, imu_yaw, wheel)
        events = sync_imu.measure_latency(
            wheel, imu_t, imu_wz, sync["offset"]
        )

        self.assertGreaterEqual(len(events), sync_imu.LATENCY_MIN_EVENTS)
        measured = sync_imu.median(events)
        # cmd_steer is only sampled at 5 Hz, so the recorded command time
        # is late by up to one interval and the measurement reads short.
        # The truth must sit between the raw median and the corrected one.
        corrected = measured + 0.5 / WHEEL_HZ
        self.assertLess(measured, TRUE_LATENCY)
        self.assertGreater(corrected, TRUE_LATENCY)

    def test_recovers_the_planted_steering_authority(self):
        (imu_t, imu_yaw, imu_wz), wheel = self.load()
        sync = sync_imu.find_offset(imu_t, imu_yaw, wheel)
        points, slope, r = sync_imu.measure_authority(
            wheel, imu_t, imu_wz, sync["offset"]
        )

        self.assertGreaterEqual(len(points), sync_imu.AUTHORITY_MIN_SEGMENTS)
        self.assertAlmostEqual(slope, TRUE_AUTHORITY, delta=0.05)
        self.assertGreater(r, 0.99)

    def test_settling_time_is_what_makes_authority_correct(self):
        """Averaging a whole segment includes the previous command's tail,
        which is usually the opposite sign, and biases the slope toward
        zero. This is why AUTHORITY_SETTLE_SEC exists."""
        (imu_t, imu_yaw, imu_wz), wheel = self.load()
        sync = sync_imu.find_offset(imu_t, imu_yaw, wheel)

        original = sync_imu.AUTHORITY_SETTLE_SEC
        sync_imu.AUTHORITY_SETTLE_SEC = 0.0
        try:
            _, biased, _ = sync_imu.measure_authority(
                wheel, imu_t, imu_wz, sync["offset"]
            )
        finally:
            sync_imu.AUTHORITY_SETTLE_SEC = original

        self.assertLess(biased, 0.9 * TRUE_AUTHORITY)

    def test_refuses_when_nobody_rotated_the_car(self):
        (imu_t, imu_yaw, _), wheel = self.load("none")
        sync = sync_imu.find_offset(imu_t, imu_yaw, wheel)

        self.assertFalse(sync["ok"])
        self.assertIn("xoay", sync["why"])

    def test_a_rhythmic_mark_works_but_with_less_headroom(self):
        """A rhythmic wiggle is not fatal: its start and end break the
        periodicity, so one peak still wins. But it leaves less room above
        the threshold, which is why the instructions ask for one distinct
        rotation instead."""
        (imu_t, imu_yaw, _), wheel = self.load("rhythmic")
        rhythmic = sync_imu.find_offset(imu_t, imu_yaw, wheel)

        (imu_t, imu_yaw, _), wheel = self.load("asymmetric")
        asymmetric = sync_imu.find_offset(imu_t, imu_yaw, wheel)

        self.assertTrue(rhythmic["ok"], rhythmic.get("why"))
        self.assertAlmostEqual(rhythmic["offset"], -TRUE_OFFSET, delta=0.10)
        self.assertLess(rhythmic["margin"], asymmetric["margin"])

    def test_interp_rejects_mismatched_series(self):
        with self.assertRaises(ValueError):
            sync_imu.interp([0.0, 1.0, 2.0], [0.0, 1.0], [0.5])

    def test_real_runs_have_no_sync_mark_and_are_refused(self):
        """The three runs already on disk were recorded before this
        protocol existed. The tool must say so rather than invent an
        offset from 4-7 degrees of incidental yaw."""
        root = Path(__file__).resolve().parent.parent / "runs_pulled"
        checked = 0
        for name in ("20260911_080230_road", "20260911_080510_road"):
            path = root / name / "wheel_telem.csv"
            if not path.is_file():
                continue
            checked += 1
            wheel = sync_imu.read_wheel(str(path))
            end = sync_imu.stationary_end(wheel)
            step = 1.0 / sync_imu.RESAMPLE_HZ
            grid = [i * step for i in range(int(end / step) + 1)]
            observed = sync_imu.span(
                sync_imu.interp(wheel["t"], wheel["yaw"], grid)
            )
            self.assertLess(observed, sync_imu.MIN_YAW_SPAN_DEG)

        if checked == 0:
            self.skipTest("runs_pulled fixtures not present")


if __name__ == "__main__":
    unittest.main()


class FrontConeRejectTests(unittest.TestCase):
    """An empty front cone has two causes that front_bins alone cannot
    separate: nothing is there, or something is there whose return gets
    filtered out. Treating the second as clear would drive into it."""

    def setUp(self):
        import types
        self._temporary_modules = []
        if "Confg" not in sys.modules:
            cfg = types.ModuleType("Confg")
            for key, value in {
                "BENCH_MODE": False, "LIDAR_STOP_DIST_M": 1.2,
                "LIDAR_RESUME_DIST_M": 1.6, "LIDAR_MAX_VALID_DIST_M": 12.0,
                "LIDAR_STOP_CONFIRM_FRAMES": 3,
                "LIDAR_CLEAR_CONFIRM_FRAMES": 8,
            }.items():
                setattr(cfg, key, value)
            sys.modules["Confg"] = cfg
            self._temporary_modules.append("Confg")
        for name in ("cv2", "serial"):
            if name not in sys.modules:
                sys.modules[name] = types.ModuleType(name)
                self._temporary_modules.append(name)
        if "rplidar" not in sys.modules:
            rpl = types.ModuleType("rplidar")
            rpl.RPLidar = object
            rpl.RPLidarException = RuntimeError
            sys.modules["rplidar"] = rpl
            self._temporary_modules.append("rplidar")

        import lidar_recorder
        self.describe = lidar_recorder.describe_scan

    def tearDown(self):
        # Do not leak the minimal fake Confg into later tests that need the
        # project's real configuration (for example YOLO class mapping).
        for name in self._temporary_modules:
            sys.modules.pop(name, None)

    @staticmethod
    def ring(dist=3000.0, quality=10):
        return [(quality, i * 0.9, dist) for i in range(400)]

    def test_nothing_in_front_leaves_every_reject_count_at_zero(self):
        scan = [p for p in self.ring() if not (100.0 <= p[1] <= 160.0)]
        info = self.describe(scan)

        self.assertEqual(info["front_points"], 0)
        self.assertEqual(info["front_raw"], 0)
        self.assertEqual(info["front_low_quality"], 0)
        self.assertGreater(info["valid_points"], 100)

    def test_a_dark_object_in_front_is_counted_not_silently_dropped(self):
        # Same empty cone by front_points, but the returns were there.
        scan = [p for p in self.ring() if not (100.0 <= p[1] <= 160.0)]
        scan += [(1, 130.0 + d, 520.0) for d in (-4, -2, 0, 2, 4)]
        info = self.describe(scan)

        self.assertEqual(info["front_points"], 0)
        self.assertEqual(info["front_bins"], 0)
        self.assertEqual(info["front_raw"], 5)
        self.assertEqual(info["front_low_quality"], 5)

    def test_something_closer_than_the_minimum_is_also_counted(self):
        scan = [p for p in self.ring() if not (100.0 <= p[1] <= 160.0)]
        scan += [(10, 130.0, 40.0)]
        info = self.describe(scan)

        self.assertEqual(info["front_points"], 0)
        self.assertEqual(info["front_too_near"], 1)

    def test_a_healthy_cone_reports_full_coverage(self):
        info = self.describe(self.ring())
        self.assertGreaterEqual(info["front_bins"], 11)
        self.assertEqual(info["front_low_quality"], 0)

    def test_reject_counts_record_only_the_first_reason(self):
        """The filters run quality -> near -> far, as analyze_scan does, so
        a point that is BOTH low quality AND under the minimum distance
        lands only in front_low_quality. These are first-reason counts,
        not independent statistics, and the docstring says so."""
        scan = [p for p in self.ring() if not (100.0 <= p[1] <= 160.0)]
        scan += [(1, 130.0, 40.0)]          # low quality AND too near
        info = self.describe(scan)

        self.assertEqual(info["front_low_quality"], 1)
        self.assertEqual(info["front_too_near"], 0)

    def test_weak_returns_inside_the_stop_band_are_counted_separately(self):
        """A cluster of weak returns reporting a distance inside the STOP
        band is the safety signal that must not be lost to the quality
        filter, even though a weak point's own distance is unreliable."""
        scan = [p for p in self.ring() if not (100.0 <= p[1] <= 160.0)]
        scan += [(1, 130.0 + d, 600.0) for d in (-2, 0, 2)]
        info = self.describe(scan)

        self.assertEqual(info["front_points"], 0)
        self.assertEqual(info["front_low_quality"], 3)
        self.assertEqual(info["front_weak_near"], 3)

    def test_a_weak_return_beyond_the_stop_band_is_not_flagged_near(self):
        scan = [p for p in self.ring() if not (100.0 <= p[1] <= 160.0)]
        scan += [(1, 130.0, 5000.0)]
        info = self.describe(scan)

        self.assertEqual(info["front_low_quality"], 1)
        self.assertEqual(info["front_weak_near"], 0)

    def test_zero_raw_points_still_does_not_prove_the_road_is_clear(self):
        """An absorbing surface returns nothing at all - not a weak
        return. front_raw == 0 narrows the possibilities; it does not
        settle them, and nothing in this module should claim otherwise."""
        scan = [p for p in self.ring() if not (100.0 <= p[1] <= 160.0)]
        info = self.describe(scan)

        self.assertEqual(info["front_raw"], 0)
        self.assertEqual(info["front_low_quality"], 0)
        self.assertEqual(info["front_weak_near"], 0)
        # Indistinguishable from the same scan taken facing a black wall.
