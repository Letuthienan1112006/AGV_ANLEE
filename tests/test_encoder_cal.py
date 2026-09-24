"""Tests for encoder_cal.py analysis. No serial port is opened."""
import contextlib
import io
from pathlib import Path
import sys
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.modules.setdefault("serial", types.ModuleType("serial"))

import encoder_cal


def trace(left_steps, right_steps):
    """Build (t, cum_l, cum_r) samples from per-sample deltas."""
    samples, left, right = [], 1000, 5000
    for i, (dl, dr) in enumerate(zip(left_steps, right_steps)):
        left += dl
        right += dr
        samples.append((i * 0.1, left, right))
    return samples


def quiet(fn, *args):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        result = fn(*args)
    return result, out.getvalue()


class EncoderCalTests(unittest.TestCase):
    def test_one_direction_gives_pulses_per_revolution(self):
        # 3000 pulses over 10 revolutions = 300 per revolution
        steps = [0] * 5 + [100] * 30 + [0] * 5
        samples = trace(steps, [0] * 40)

        left, _ = quiet(encoder_cal.analyse, samples, 1, "TRAI", 10)
        self.assertAlmostEqual(left, 300.0, places=1)

    def test_a_wheel_that_never_moved_reports_absence_not_zero(self):
        samples = trace([0] * 20, [50] * 20)

        left, text = quiet(encoder_cal.analyse, samples, 1, "TRAI", 10)
        self.assertIsNone(left)
        self.assertIn("KHONG phat hien chuyen dong", text)
        self.assertIn("Vang mat la vang mat", text)

    def test_turning_back_and_forth_is_refused_not_averaged(self):
        """The counter is signed, so a back-and-forth cancels. Returning a
        smaller number would look like a real measurement."""
        steps = [0] * 3 + [100] * 10 + [-100] * 5 + [0] * 3
        samples = trace(steps, [0] * 21)

        left, text = quiet(encoder_cal.analyse, samples, 1, "TRAI", 10)
        self.assertIsNone(left)
        self.assertIn("DOI CHIEU", text)

    def test_reverse_direction_is_accepted_as_a_magnitude(self):
        """Turning the agreed direction may decrease the counter depending
        on mounting; the magnitude is what matters."""
        steps = [0] * 3 + [-100] * 30 + [0] * 3
        samples = trace(steps, [0] * 36)

        left, _ = quiet(encoder_cal.analyse, samples, 1, "TRAI", 10)
        self.assertAlmostEqual(left, 300.0, places=1)

    def test_noise_below_the_threshold_is_not_movement(self):
        steps = [1, -1, 1, -1, 0, 1] * 4
        samples = trace(steps, [0] * 24)

        left, text = quiet(encoder_cal.analyse, samples, 1, "TRAI", 10)
        self.assertIsNone(left)
        self.assertIn("KHONG phat hien", text)

    def test_each_side_is_measured_independently(self):
        """Never assume both sides share a pulses-per-revolution: different
        encoders, gear ratios or counting modes all break that."""
        samples = trace([0] * 3 + [100] * 30 + [0] * 3,
                        [0] * 3 + [50] * 30 + [0] * 3)

        left, _ = quiet(encoder_cal.analyse, samples, 1, "TRAI", 10)
        right, _ = quiet(encoder_cal.analyse, samples, 2, "PHAI", 10)
        self.assertAlmostEqual(left, 300.0, places=1)
        self.assertAlmostEqual(right, 150.0, places=1)


if __name__ == "__main__":
    unittest.main()
