"""Offline command regressions; no camera, network, UART, or model imports."""
import ast
from pathlib import Path
import unittest

from lane_control import LanePID, LaneTrackingGuard


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


class PIDTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.pid = LanePID(.05, .03, .02, 14, 175, 11, clock=self.clock)

    def step(self, filtered, raw=None, **kwargs):
        self.clock.now += .27
        return self.pid.compute(filtered, measured_error=raw, **kwargs)

    def test_startup_and_reset_have_no_derivative_kick(self):
        self.assertEqual(self.step(-39.4), -1)
        self.assertEqual(self.pid.integral, 0)
        self.step(-90)
        self.pid.reset()
        self.clock.now += 100
        self.assertEqual(self.step(39.4), 1)
        self.assertEqual(self.pid.integral, 0)

    def test_raw_crossing_clears_bias_before_filtered_error_crosses(self):
        for _ in range(20):
            self.step(-80)
        self.assertLess(self.pid.integral, 0)
        self.step(-25.4, raw=4.1)  # run 015830 frame 35
        self.assertEqual(self.pid.integral, 0)
        self.assertGreaterEqual(self.step(6.8, raw=38.9), 0)

    def test_opposite_crossing_does_not_rebuild_old_side_bias(self):
        for _ in range(20):
            self.step(80)
        self.step(15.2, raw=-34.6)  # frame 51: EMA still positive
        self.assertEqual(self.pid.integral, 0)
        self.step(8, raw=-60)
        self.assertEqual(self.pid.integral, 0)

    def test_sign_cross_reset_can_be_switched_off(self):
        # Run 20260916_005537 frames 13-20 on the car: the integral built to
        # 68.6 over six frames of steady ~27px error, then the raw error
        # crossed (+15.2 -> -12.3) and the reset wiped it, so the command
        # went to 0 at the frame the car began to swing. With the reset off
        # the accumulated authority has to survive that crossing.
        kept = LanePID(.05, .03, .02, 14, 175, 11, clock=self.clock,
                       reset_integral_on_sign_cross=False)
        for _ in range(6):
            self.clock.now += .42
            kept.compute(27.0, measured_error=27.0)
        built = kept.integral
        self.assertGreater(built, 20)
        self.clock.now += .42
        kept.compute(4.4, measured_error=-12.3)
        self.assertNotEqual(kept.integral, 0.0)

        wiped = LanePID(.05, .03, .02, 14, 175, 11, clock=self.clock,
                        reset_integral_on_sign_cross=True)
        for _ in range(6):
            self.clock.now += .42
            wiped.compute(27.0, measured_error=27.0)
        self.clock.now += .42
        wiped.compute(4.4, measured_error=-12.3)
        self.assertEqual(wiped.integral, 0.0)

    def test_reset_off_still_accumulates_when_signs_differ(self):
        # The same-sign gate is EMA-lag-only too: with the reset off it must
        # not be the thing that silently blocks the integral instead.
        pid = LanePID(.05, .03, .02, 14, 175, 11, clock=self.clock,
                      reset_integral_on_sign_cross=False)
        self.clock.now += .42
        pid.compute(4.4, measured_error=-12.3)
        self.clock.now += .42
        pid.compute(4.4, measured_error=-12.3)
        self.assertNotEqual(pid.integral, 0.0)

    def test_leak_survives_a_crossing_where_the_hard_reset_zeroed_authority(self):
        # Run 005537 frames 13-20: six frames of steady ~27px built the
        # integral to 68.6, then the raw error crossed and reset=True wiped
        # it, leaving steer 0 at the frame the car began to swing. A leak
        # must still be carrying authority through that same crossing.
        leaky = LanePID(.05, .03, .02, 14, 175, 11, clock=self.clock,
                        reset_integral_on_sign_cross=False,
                        integral_leak_tau=1.5)
        for _ in range(6):
            self.clock.now += .42
            leaky.compute(27.0, measured_error=27.0)
        self.assertGreater(leaky.integral, 15)
        self.clock.now += .42
        leaky.compute(4.4, measured_error=-12.3)
        self.assertGreater(abs(leaky.integral), 0.0)

    def test_leak_drops_a_stale_bias_the_no_reset_run_kept_pointing_backwards(self):
        # Run 021246 frame 34: with no reset and no leak, integral built on
        # the old side still commanded steer opposite to a +19.7 error. The
        # leak has to bleed that away within about a second of driving.
        stale = LanePID(.05, .03, .02, 14, 175, 11, clock=self.clock,
                        reset_integral_on_sign_cross=False,
                        integral_leak_tau=1.5)
        for _ in range(12):
            self.clock.now += .42
            stale.compute(-60.0, measured_error=-60.0)
        carried = stale.integral
        self.assertLess(carried, -20)
        # Now the car crosses and sits at the new side for ~1s.
        for _ in range(3):
            self.clock.now += .42
            out = stale.compute(19.7, measured_error=19.7)
        self.assertGreater(stale.integral, carried)
        self.assertGreaterEqual(out, 0)

    def test_leak_bounds_a_standing_error_below_the_clamp(self):
        # Run 012523: a phantom +36px offset (miscalibrated centre) wound
        # the integral to the 100 cap in ~3s, and that stored authority is
        # what eventually threw the car into the turn. With a leak the
        # steady state settles near error*tau instead of the clamp.
        bounded = LanePID(.05, .03, .02, 18, 100, 6, clock=self.clock,
                          reset_integral_on_sign_cross=False,
                          integral_leak_tau=1.5)
        for _ in range(40):
            self.clock.now += .42
            bounded.compute(36.0, measured_error=36.0)
        self.assertLess(bounded.integral, 100)
        self.assertAlmostEqual(bounded.integral, 36.0 * 1.5, delta=12)

    def test_leak_defaults_off_so_existing_tuning_is_unchanged(self):
        without = LanePID(.05, .03, .02, 14, 175, 11, clock=self.clock,
                          reset_integral_on_sign_cross=False)
        for _ in range(5):
            self.clock.now += .42
            without.compute(30.0, measured_error=30.0)
        # Four accumulating steps, not five: the first compute() after a
        # reset has dt=0 by design, so it contributes nothing.
        expected = 30.0 * .42 * 4
        self.assertAlmostEqual(without.integral, expected, delta=0.001)

    def test_zero_clears_integral(self):
        for _ in range(10):
            self.step(40)
        self.step(20, raw=0)
        self.assertEqual(self.pid.integral, 0)

    def test_blocked_integral_and_output_limits(self):
        self.step(20)
        saved = self.pid.integral
        for _ in range(10):
            self.step(20, hold_integral=True)
        self.assertEqual(self.pid.integral, saved)
        previous = self.pid.last_output
        for error in (10000, 10000, -10000, -10000, 0):
            out = self.step(error)
            self.assertLessEqual(abs(out), 14)
            self.assertLessEqual(abs(self.pid.last_output - previous), 11)
            self.assertLessEqual(abs(self.pid.integral), 175)
            previous = self.pid.last_output

    def test_nonfinite_measurement_is_rejected_and_clears_state(self):
        self.step(50)
        with self.assertRaises(ValueError):
            self.step(50, raw=float('nan'))
        self.assertIsNone(self.pid.last_time)
        self.assertEqual(self.pid.last_output, 0)


class TrackingTests(unittest.TestCase):
    def setUp(self):
        self.guard = LaneTrackingGuard(3, 1)

    def test_startup_requires_three_consecutive_observations(self):
        for valid, expected in ((True, False), (False, False),
                                (True, False), (True, False), (True, True)):
            self.assertEqual(self.guard.update(valid), expected)
            self.assertEqual(self.guard.command(-14, 15),
                             (-14, 15) if expected else (0, 0))

    def test_latest_run_single_fragment_cannot_restart_motion(self):
        for _ in range(3):
            self.guard.update(True)
        # Frames 56..61: HOLD, one LINE fragment, then four HOLD frames.
        for valid in (False, True, False, False, False, False):
            self.guard.update(valid)
            self.assertEqual(self.guard.command(-14, 15), (0, 0))
        for _ in range(2):
            self.assertFalse(self.guard.update(True))
        self.assertTrue(self.guard.update(True))

    def test_bridge_stop_requires_fresh_confirmation(self):
        for _ in range(3):
            self.guard.update(True)
        self.assertFalse(self.guard.update(True, blocked=True))
        self.assertEqual(self.guard.state, 'BLOCKED')
        self.assertFalse(self.guard.update(True))
        self.assertFalse(self.guard.update(True))
        self.assertTrue(self.guard.update(True))


class PatrolIntegrationTests(unittest.TestCase):
    def test_recovery_does_not_reduce_measured_turn_authority(self):
        import Confg
        self.assertGreaterEqual(Confg.RECOVERY_SPEED, Confg.MIN_SPEED)

    def test_production_defaults_to_unified_yolo26(self):
        source = (Path(__file__).resolve().parents[1]
                  / 'patrol_robot.py').read_text()
        self.assertIn('os.environ.get("AGV_USE_YOLO26_UNIFIED", "1")', source)

    def test_intersection_speed_uses_real_command_scale(self):
        path = Path(__file__).resolve().parents[1] / 'patrol_robot.py'
        tree = ast.parse(path.read_text())
        assignments = [n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                       and any(isinstance(t, ast.Name) and t.id == 'car_speed'
                               for t in n.targets)]
        self.assertFalse(any(isinstance(n.value, ast.Constant)
                             and n.value.value == 80 for n in assignments))

    def test_actual_final_gate_resets_pid_and_smoother_before_telemetry(self):
        # Execute the production gate itself without importing GPU/model code.
        path = Path(__file__).resolve().parents[1] / 'patrol_robot.py'
        tree = ast.parse(path.read_text())
        gate = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                    and isinstance(n.value, ast.Call)
                    and isinstance(n.value.func, ast.Attribute)
                    and isinstance(n.value.func.value, ast.Name)
                    and n.value.func.value.id == 'lane_guard'
                    and n.value.func.attr == 'command')
        reset = next(n for n in ast.walk(tree) if isinstance(n, ast.If)
                     and n.lineno == gate.end_lineno + 1)
        smoother_cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                            and n.name == 'ErrorSmoother')
        namespace = {'LANE_ERROR_SMOOTHING_ALPHA': .5}
        exec(compile(ast.Module(body=[smoother_cls], type_ignores=[]), str(path), 'exec'), namespace)
        clock = Clock()
        pid = LanePID(.05, .03, .02, 14, 175, 11, clock=clock)
        pid.compute(-50)
        clock.now = .5
        pid.compute(-50)
        smoother = namespace['ErrorSmoother']()
        smoother.update(-200)
        guard = LaneTrackingGuard()
        guard.update(False)
        namespace.update(lane_pid=pid, error_smoother=smoother,
                         lane_guard=guard, steer_angle=-14, car_speed=15)
        exec(compile(ast.Module(body=[gate, reset], type_ignores=[]), str(path), 'exec'), namespace)
        self.assertEqual((namespace['steer_angle'], namespace['car_speed']), (0, 0))
        self.assertEqual(pid.integral, 0)
        self.assertIsNone(pid.last_error)
        self.assertIsNone(smoother.value)


if __name__ == '__main__':
    unittest.main()
