"""The mechanical steering trim lives in exactly ONE place.

It used to be added to the steer command in Python (STEER_TRIM). It now
lives in the firmware as a per-wheel tick offset (STEER_TRIM_TICKS). Both
at once is double compensation, and nothing at runtime would report it -
the car would simply pull to one side and every later tuning session would
chase it as a control problem. So the invariant is checked here, against
the two files themselves.
"""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "motor_test_bts7960" / "src" / "main.cpp"


def firmware_int(name):
    text = FIRMWARE.read_text()
    m = re.search(r"^const int %s\s*=\s*(-?\d+)\s*;" % name, text, re.M)
    if m is None:
        raise AssertionError("%s not found in %s" % (name, FIRMWARE))
    return int(m.group(1))


def confg_int(name):
    text = (ROOT / "Confg.py").read_text()
    m = re.search(r"^%s\s*=\s*(-?\d+)" % name, text, re.M)
    if m is None:
        raise AssertionError("%s not found in Confg.py" % name)
    return int(m.group(1))


class TrimOwnershipTests(unittest.TestCase):
    def test_trim_is_applied_in_exactly_one_layer(self):
        fw = firmware_int("STEER_TRIM_TICKS")
        py = confg_int("STEER_TRIM")
        self.assertFalse(
            fw != 0 and py != 0,
            "trim applied twice: firmware STEER_TRIM_TICKS=%d and "
            "Confg STEER_TRIM=%d. Zero one of them." % (fw, py),
        )

    def test_firmware_owns_the_trim_in_the_current_build(self):
        self.assertNotEqual(firmware_int("STEER_TRIM_TICKS"), 0)
        self.assertEqual(confg_int("STEER_TRIM"), 0)


class SteeringResolutionTests(unittest.TestCase):
    """The rounding fix is what makes small corrections distinguishable;
    a revert to truncation would silently restore the dead band that made
    steer +1/+2/+3 identical."""

    def setUp(self):
        self.base_ticks = firmware_int("TARGET_TICKS_AT_REFERENCE")
        self.speed_ref = firmware_int("SPEED_REFERENCE")
        self.gain = firmware_int("STEER_GAIN_PERCENT")
        self.steer_max = firmware_int("STEER_COMMAND_MAX")
        self.trim = firmware_int("STEER_TRIM_TICKS")

    def turn(self, base, steer):
        num = base * steer * self.gain
        den = self.steer_max * 100
        return ((num + den // 2) // den if num >= 0
                else -((-num + den // 2) // den))

    def differential(self, base, steer):
        t = self.turn(base, steer)
        return (base + t + self.trim) - (base - t - self.trim)

    def test_small_corrections_are_distinguishable_at_patrol_speed(self):
        base = 16 * self.base_ticks // self.speed_ref
        seen = [self.differential(base, s) for s in range(-3, 4)]
        self.assertEqual(len(set(seen)), len(seen),
                         "steer values collapse to the same differential: %s"
                         % seen)

    def test_steering_is_symmetric_about_straight(self):
        base = 16 * self.base_ticks // self.speed_ref
        straight = self.differential(base, 0)
        for s in range(1, 4):
            right = self.differential(base, s) - straight
            left = straight - self.differential(base, -s)
            self.assertEqual(right, left,
                             "steer %d: right=%d left=%d" % (s, right, left))

    def test_firmware_rounds_rather_than_truncates(self):
        # BOTH branches, checked separately: an earlier version of this test
        # searched for "denominator / 2" anywhere, which still matched after
        # the positive branch was reverted to truncation, because the
        # negative branch kept the string. A guard that survives the change
        # it guards against is worse than none.
        source = FIRMWARE.read_text()
        for branch, expected in (
            ("positive", "(numerator + denominator / 2) / denominator"),
            ("negative", "-((-numerator + denominator / 2) / denominator)"),
        ):
            self.assertIn(
                expected, source,
                "%s branch of turnTarget no longer rounds; truncation brings "
                "back the dead band where steer +1/+2/+3 were identical"
                % branch)


if __name__ == "__main__":
    unittest.main()
