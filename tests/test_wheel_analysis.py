"""Offline report tests. Fixtures are synthetic; no hardware modules imported."""
import contextlib
import csv
import io
import json
from pathlib import Path
import tempfile
import unittest

import verdict
import wheel_check


FIELDS = ("t", "cmd_steer", "cmd_speed", "enc_l", "enc_r", "tgt_l", "tgt_r",
          "pwm_l", "pwm_r", "fw_steer", "fw_speed", "mode", "yaw")


def sample(i=0, **overrides):
    row = dict(t=i * 0.2, cmd_steer=0, cmd_speed=15, enc_l=29, enc_r=31,
               tgt_l=24, tgt_r=24, pwm_l=43, pwm_r=43, fw_steer=0,
               fw_speed=15, mode="V", yaw=1.7)
    row.update(overrides)
    return row


class WheelReportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name) / "unlabelled"
        self.run.mkdir()

    def write_wheels(self, rows):
        with (self.run / "wheel_telem.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(rows)

    def write_mode(self, mode="road", on_stand=False, dry=False,
                   vision_backend="yolo26_unified"):
        (self.run / "run_mode.json").write_text(json.dumps(dict(
            schema_version=1, mode=mode, declared_on_stand=on_stand,
            dry_run=dry, lidar_bypassed=False, git_head=None,
            vision_backend=vision_backend)))

    def output(self, fn):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            fn(str(self.run))
        return out.getvalue()

    def test_unknown_fixture_does_not_infer_stand_from_constant_yaw(self):
        self.write_wheels([sample(i) for i in range(12)])
        text = self.output(wheel_check.main)
        self.assertIn("Chua biet xe tren gia hay tren dat", text)
        self.assertNotIn("XE TREN GIA theo khai bao", text)
        self.assertIn("Yaw khong doi/bi thieu KHONG chung minh", text)

    def test_missing_yaw_does_not_mean_stand(self):
        self.write_wheels([sample(i, yaw="") for i in range(12)])
        text = self.output(wheel_check.main)
        self.assertIn("Chua biet xe tren gia hay tren dat", text)
        self.assertNotIn("XE TREN GIA theo khai bao", text)

    def test_explicit_metadata_overrides_legacy_banner_and_name(self):
        dest = self.run.with_name("old_sweep")
        self.run.rename(dest)
        self.run = dest
        (self.run / "bridge.log").write_text("CHE DO BENCH\nDRY RUN: old text\n")
        self.write_mode()
        info = wheel_check.read_run_mode(str(self.run))
        self.assertEqual(info["mode"], "road")
        self.assertIs(info["declared_on_stand"], False)
        self.assertIs(info["dry_run"], False)
        self.assertEqual(info["source"], "run_mode.json")
        self.assertEqual(info["vision_backend"], "yolo26_unified")

    def test_lane_and_straight_metadata_are_not_dropped_as_unknown(self):
        for mode in ("lane", "straight", "wcal", "lidar"):
            with self.subTest(mode=mode):
                self.write_mode(mode=mode)
                info = wheel_check.read_run_mode(str(self.run))
                self.assertEqual(info["mode"], mode)
                self.assertEqual(info["source"], "run_mode.json")

    def test_legacy_bench_and_sweep_are_operator_declarations(self):
        (self.run / "bridge.log").write_text("CHE DO BENCH (AGV_BENCH=1)\n")
        self.assertTrue(wheel_check.read_run_mode(str(self.run))["declared_on_stand"])
        (self.run / "bridge.log").unlink()
        dest = self.run.with_name("legacy_sweep")
        self.run.rename(dest)
        self.run = dest
        self.assertTrue(wheel_check.read_run_mode(str(self.run))["declared_on_stand"])

    def test_legacy_dry_is_not_automatically_a_stand(self):
        (self.run / "bridge.log").write_text("[STM32] DRY RUN: khong mo UART\n")
        info = wheel_check.read_run_mode(str(self.run))
        self.assertTrue(info["dry_run"])
        self.assertIsNone(info["declared_on_stand"])
        self.assertFalse(verdict.is_bench(str(self.run)))

    def test_invalid_metadata_does_not_turn_strings_into_booleans(self):
        (self.run / "run_mode.json").write_text(json.dumps(dict(
            schema_version=1, mode="road", declared_on_stand="false", dry_run="false")))
        info = wheel_check.read_run_mode(str(self.run))
        self.assertIsNone(info["declared_on_stand"])
        self.assertIsNone(info["dry_run"])

    def test_malformed_metadata_is_safe_and_unknown(self):
        (self.run / "run_mode.json").write_text("not json")
        self.assertEqual(wheel_check.read_run_mode(str(self.run))["mode"], "unknown")

    def test_stand_target_error_is_retained(self):
        self.write_mode(mode="sweep", on_stand=True)
        self.write_wheels([sample(i) for i in range(12)])
        text = self.output(wheel_check.main)
        self.assertIn("XE TREN GIA theo khai bao", text)
        self.assertIn("sai so bam toc do VAN co y nghia", text)
        self.assertIn("+5.0 tick (median)", text)
        self.assertIn("+7.0 tick (median)", text)
        self.assertNotIn("chi phan LECH", text)

    def test_commanded_but_stalled_wheels_are_analyzed(self):
        self.write_wheels([sample(i, enc_l=0, enc_r=0) for i in range(12)])
        text = self.output(wheel_check.main)
        self.assertIn("CO LENH CHAY NHUNG KHONG CO HOAT DONG ENCODER", text)
        self.assertIn("-24.0 tick (median)", text)
        self.assertNotIn("chua danh gia duoc dap ung lenh", text)

    def test_signed_encoder_magnitudes_match_firmware_speed_error(self):
        self.write_wheels([sample(i, enc_l=-29, enc_r=-31) for i in range(12)])
        text = self.output(wheel_check.main)
        self.assertIn("encoder hoat dong: 12 mau", text)
        self.assertIn("+5.0 tick (median)", text)

    def test_same_row_command_difference_is_not_a_delivery_verdict(self):
        self.write_wheels([sample(i, fw_steer=8, cmd_steer=0) for i in range(12)])
        text = self.output(wheel_check.main)
        self.assertIn("12/12 mau", text)
        self.assertIn("Khong ket luan mat lenh hay truyen nguyen ven", text)
        self.assertNotIn("0 = den nguyen ven", text)

    def test_signed_slope_has_expected_negative_convention(self):
        self.write_wheels([sample(i, fw_steer=st, cmd_steer=st,
                                  enc_l=24 + st, enc_r=24 - st)
                           for i, st in enumerate([-8, -4, 0, 4, 8] * 3)])
        text = self.output(wheel_check.main)
        self.assertIn("/ steer: -2.00 tick", text)
        self.assertIn("do doc AM", text)
        self.assertNotIn("0.85", text)

    def test_single_sample_and_empty_yaw_rate_do_not_crash(self):
        self.assertEqual(wheel_check.yaw_rate([]), [])
        self.write_wheels([sample()])
        self.assertIn("mau khong phu hop", self.output(wheel_check.main))

    def test_nan_and_missing_fields_are_not_treated_as_valid_measurements(self):
        self.write_wheels([sample(0, enc_l="nan"), sample(1, enc_l=""),
                           sample(2, yaw="nan", cmd_steer="nan")])
        rows = wheel_check.load(str(self.run))
        self.assertEqual(len(rows), 1)
        self.assertIsNone(rows[0]["yaw"])
        self.assertIsNone(rows[0]["cmd_steer"])

    def test_yaw_wrap_and_negative_correlation_are_descriptive(self):
        rates = wheel_check.yaw_rate([sample(0, yaw=359), sample(1, yaw=1)])
        self.assertAlmostEqual(rates[0], 10)
        y = 100
        rows = []
        for i, st in enumerate([-8, -4, 0, 4, 8] * 4):
            rows.append(sample(i, fw_steer=st, yaw=y))
            y -= st * 0.2
        self.write_mode()
        self.write_wheels(rows)
        text = self.output(wheel_check.main)
        self.assertIn("tuong quan steer <-> toc do yaw : -1.00", text)
        self.assertNotIn("DAO CHIEU: xe quay NGUOC", text)
        self.assertIn("KHONG phai do tre da xac nhan", text)

    def test_verdict_prefers_timestamped_wheel_csv(self):
        self.write_wheels([sample()])
        (self.run / "bridge.log").write_text(
            "ENC,0,0,DELTA,0,0,TARGET,0,0,PWM,0,0,STEER,0,SPEED,0,MODE,S\n")
        rows = verdict.load_enc(str(self.run))
        self.assertEqual(rows[0]["dl"], 29)
        self.assertEqual(rows[0]["sp"], 15)

    def test_verdict_does_not_mistake_drive_command_for_motion(self):
        self.write_wheels([sample(i, enc_l=0, enc_r=0) for i in range(12)])
        text = self.output(verdict.main)
        self.assertIn("encoder hoat dong: 0/12 mau", text)
        self.assertIn("CO LENH CHAY: can kiem tra", text)
        self.assertIn("trai -24.0; phai -24.0", text)
        self.assertNotIn("tat ca MODE=S", text)

    def test_verdict_reports_stop_samples_without_claiming_continuous_obstacle(self):
        self.write_wheels([sample(i, mode="S", fw_speed=0, enc_l=0, enc_r=0,
                                  tgt_l=0, tgt_r=0) for i in range(12)])
        (self.run / "bridge.log").write_text("[NET] LIDAR=STOP reason=vung hysteresis\n")
        text = self.output(verdict.main)
        self.assertIn("ly do pho bien: vung hysteresis", text)
        self.assertIn("khong chung minh vat can ton tai suot", text)
        self.assertNotIn("Khong mot vong banh nao quay", text)

    def test_verdict_distinguishes_stand_from_road_and_avoids_pass_fail(self):
        self.write_mode(mode="sweep", on_stand=True)
        self.write_wheels([sample(i, fw_steer=8, enc_l=39, enc_r=21)
                           for i in range(12)])
        # Pin the ceiling this fixture assumes (fw_steer=8 must equal it,
        # exactly the branch under test) rather than reading whatever
        # PID_MAX_OUTPUT happens to be live in the repo's Confg.py right
        # now - that coupling is what steer_ceiling()'s own docstring
        # warns against, and it broke silently when PID_MAX_OUTPUT moved
        # 8 -> 14 on 2026-09-15.
        (self.run / "Confg_snapshot.py").write_text("PID_MAX_OUTPUT = 8\n")
        text = self.output(verdict.main)
        self.assertIn("KHAI BAO TREN GIA", text)
        self.assertNotIn("banh xe that, co tai", text)
        self.assertNotIn("lenh lai thanh chuyen dong that", text)
        self.assertIn("khong phai do doc hoi quy hay nguong dat/rot", text)

    def test_verdict_dry_run_and_saturated_endpoints_are_not_tracking_failure(self):
        self.write_mode(mode="dry", dry=True)
        with (self.run / "patrol_telem_test.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=("t", "src", "raw_err", "steer"))
            writer.writeheader()
            for i in range(4):
                writer.writerow(dict(t=(i + 1) * .2, src="LINE", raw_err=100 + i,
                                     steer=-8))
        (self.run / "Confg_snapshot.py").write_text("PID_MAX_OUTPUT = 8\n")
        text = self.output(verdict.main)
        self.assertIn("DRY RUN", text)
        self.assertIn("Hai mau co the thuoc cac doan khac nhau", text)
        self.assertNotIn("KHONG KEO LAI DUOC", text)
        self.assertNotIn("SAI CHIEU", text)
        self.assertNotIn("OK - dung chieu", text)


if __name__ == "__main__":
    unittest.main()


class PiCeilingTests(unittest.TestCase):
    """The reachable PWM is feedforward + KP*err + KI*INTEGRAL_LIMIT, not
    PWM_MAX. I read PWM 83 next to PWM_MAX=140 and concluded the loop had
    40% headroom; the integral clamp is worth 9 PWM, so at target 28 the
    ceiling is 84.6 and the wheel was saturated."""

    GAINS = {"BASE_PWM": 90.0, "PWM_MAX": 140.0, "KP": 0.45, "KI": 0.015,
             "INTEGRAL_LIMIT": 600.0, "TREF": 40.0}

    def test_ceiling_is_far_below_pwm_max_at_realistic_targets(self):
        # target 28, wheel delivering 8 ticks -> error 20
        ceiling = wheel_check.pi_ceiling(self.GAINS, 28, 20)
        self.assertAlmostEqual(ceiling, 63.0 + 9.0 + 9.0, places=3)
        self.assertLess(ceiling, self.GAINS["PWM_MAX"])

    def test_pwm_max_still_caps_the_result(self):
        self.assertEqual(
            wheel_check.pi_ceiling(self.GAINS, 70, 70), self.GAINS["PWM_MAX"]
        )

    def test_no_target_means_no_drive(self):
        self.assertEqual(wheel_check.pi_ceiling(self.GAINS, 0, 0), 0.0)

    def test_gains_are_read_from_the_firmware_not_duplicated_here(self):
        gains = wheel_check.read_firmware_gains()
        if gains is None:
            self.skipTest("firmware source not present")
        # If these ever drift, the analysis follows the firmware rather
        # than a stale copy in the analyser.
        self.assertGreater(gains["TREF"], 0)
        self.assertGreater(gains["PWM_MAX"], gains["BASE_PWM"])
        self.assertIn(wheel_check.FIRMWARE_SRC.split("/")[0], gains["path"])

    def test_missing_firmware_reports_absence_not_a_guess(self):
        self.assertIsNone(wheel_check.read_firmware_gains(root="/nonexistent"))


class PulseAuditTests(unittest.TestCase):
    """Three faults the user found by simulating data, all of which could
    have produced a wrong verdict about the encoders."""

    COLS = ("t", "seq", "cmd_steer", "cmd_speed", "count_l", "count_r",
            "enc_l", "enc_r", "tgt_l", "tgt_r", "pwm_l", "pwm_r",
            "fw_steer", "fw_speed", "mode", "yaw")

    def rows(self, counts, deltas=None, seqs=None):
        if deltas is None:
            deltas = [(0, 0)] + [(b[0] - a[0], b[1] - a[1])
                                 for a, b in zip(counts, counts[1:])]
        out = []
        for i, ((cl, cr), (dl, dr)) in enumerate(zip(counts, deltas)):
            out.append(dict(t=i * 0.1,
                            seq=None if seqs is None else seqs[i],
                            cmd_steer=0, cmd_speed=15,
                            count_l=cl, count_r=cr, enc_l=dl, enc_r=dr,
                            tgt_l=24, tgt_r=24, pwm_l=50, pwm_r=50,
                            fw_steer=0, fw_speed=15, mode="V", yaw=1.0))
        return out

    def test_lost_messages_are_counted_per_segment_across_a_reset(self):
        """SEQ 10, 13, 0, 1 used to give expected = -8 and lost = 0, while
        11 and 12 were missing before the restart. Counting across the
        reset is meaningless; each segment has to be counted on its own."""
        audit = wheel_check.pulse_audit(
            self.rows([(10, 10), (20, 20), (30, 30), (40, 40)],
                      seqs=[10, 13, 0, 1]))

        self.assertEqual(audit["restarts"], 1)
        self.assertEqual(audit["lost"], 2)
        self.assertEqual(audit["duplicates"], 0)

    def test_a_repeated_sequence_number_is_not_a_reset(self):
        """4, 4, 5 was called a reset because the test was b <= a. A
        duplicate may be a repeated line; it does not show the MCU
        restarted."""
        audit = wheel_check.pulse_audit(
            self.rows([(10, 10), (20, 20), (30, 30)], seqs=[4, 4, 5]))

        self.assertEqual(audit["restarts"], 0)
        self.assertEqual(audit["duplicates"], 1)
        self.assertEqual(audit["lost"], 0)

    def test_returning_to_the_start_counts_as_a_reversal(self):
        """counts 0, 10, 0 reverse once, but comparing each step against
        the NET change missed it: the net is zero, so no step is ever
        'against' it. Signs of consecutive non-zero steps, per wheel."""
        audit = wheel_check.pulse_audit(
            self.rows([(0, 0), (10, 10), (0, 0)]))

        self.assertEqual(audit["segments"][0]["net"], (0.0, 0.0))
        # one reversal on each wheel
        self.assertEqual(audit["reversals"], 2)

    def test_the_step_across_a_reset_is_not_added_to_the_path(self):
        """1030 -> 0 is the counter restarting, not 1030 ticks of travel."""
        audit = wheel_check.pulse_audit(
            self.rows([(1000, 1000), (1030, 1030), (0, 0), (30, 30)],
                      seqs=[0, 1, 0, 1]))

        self.assertEqual(audit["restarts"], 1)
        self.assertEqual(audit["path"], (60.0, 60.0))

    def test_audit_runs_with_no_drive_command_at_all(self):
        """The hand-turn calibration has motor power disconnected, so
        there is no drive command. wheel_check used to return before
        [2c] - the only section that test needs."""
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        run = Path(temp.name) / "unlabelled"
        run.mkdir()
        rows = self.rows([(0, 0), (300, 300), (600, 600)],
                         deltas=[(0, 0), (300, 300), (300, 300)],
                         seqs=[0, 1, 2])
        for r in rows:                      # nothing commanded, nothing driving
            r.update(cmd_speed=0, fw_speed=0, tgt_l=0, tgt_r=0,
                     pwm_l=0, pwm_r=0, mode="S")
        with (run / "wheel_telem.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.COLS)
            writer.writeheader()
            writer.writerows(rows)

        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            wheel_check.main(str(run))
        text = out.getvalue()

        self.assertIn("[2c]", text)
        self.assertIn("Khong mat ban tin, khong reset", text)
        self.assertIn("MOT CHIEU", text)          # the direction caveat
        self.assertIn("KHONG gia dinh", text)

    def test_reverse_rotation_is_not_reported_as_a_reset(self):
        """The ISR does ++ and --, so the counter is signed and hand-turning
        a wheel backwards decreases it. The old check called that an MCU
        reset - and it would have fired on the very pulses-per-revolution
        test the plan asks for."""
        audit = wheel_check.pulse_audit(
            self.rows([(-10, -10), (-20, -20), (-30, -30)]))

        self.assertTrue(audit["ok"])
        self.assertEqual(audit["restarts"], 0)
        self.assertEqual(audit["segments"][0]["net"], (-20.0, -20.0))
        self.assertEqual(audit["jumps"], [])

    def test_boundary_is_handled_so_the_two_methods_agree(self):
        """counts 10 -> 20 -> 30 with deltas 10/10/10 loses no messages.
        Comparing against ALL the deltas gave 30 against 20 and shouted
        50%; the first delta describes the interval BEFORE the subtraction
        starts."""
        rows = self.rows([(10, 10), (20, 20), (30, 30)],
                         deltas=[(10, 10), (10, 10), (10, 10)])
        audit = wheel_check.pulse_audit(rows)
        summed = sum(abs(r["enc_l"]) for r in rows[1:])

        self.assertEqual(audit["path"][0], 20.0)
        self.assertEqual(summed, 20)

    def test_a_drop_in_the_middle_is_visible_in_the_step_spread(self):
        """100 -> 120 -> 0 -> 140 used to pass because only first and last
        were compared. It is still not catchable by magnitude - 120 is a
        plausible step - so what the report owes the reader is the step
        distribution, and it must not claim to have detected a reset."""
        audit = wheel_check.pulse_audit(
            self.rows([(100, 100), (120, 120), (0, 0), (140, 140)]))

        steps = audit["segments"][0]["steps"][0]
        self.assertIn(-120.0, steps)
        self.assertGreater(audit["reversals"], 0)
        self.assertEqual(audit["jumps"], [])      # honest: too small to flag

    def test_a_gross_jump_is_flagged(self):
        audit = wheel_check.pulse_audit(
            self.rows([(0, 0), (50, 50), (90000, 90000)]), max_target=70)
        self.assertEqual(len(audit["jumps"]), 1)

    def test_sequence_numbers_confirm_lost_messages(self):
        audit = wheel_check.pulse_audit(
            self.rows([(10, 10), (20, 20), (30, 30)], seqs=[4, 7, 8]))

        self.assertEqual(audit["segments"][0]["seq"]["expected"], 5)
        self.assertEqual(audit["lost"], 2)
        self.assertEqual(audit["restarts"], 0)

    def test_sequence_numbers_confirm_a_reset(self):
        audit = wheel_check.pulse_audit(
            self.rows([(10, 10), (20, 20), (30, 30)], seqs=[500, 501, 0]))
        self.assertEqual(audit["restarts"], 1)

    def test_without_sequence_numbers_nothing_is_confirmed(self):
        audit = wheel_check.pulse_audit(
            self.rows([(10, 10), (20, 20), (30, 30)]))
        self.assertFalse(audit["have_seq"])
        self.assertIsNone(audit["lost"])

    def test_missing_counter_columns_report_absence(self):
        audit = wheel_check.pulse_audit(
            [dict(t=0.0, enc_l=30, enc_r=30), dict(t=0.1, enc_l=30, enc_r=30)])
        self.assertFalse(audit["ok"])
        self.assertIn("count_l", audit["why"])

    def test_report_says_two_methods_disagree_without_blaming_lost_lines(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        run = Path(temp.name) / "unlabelled"
        run.mkdir()
        counts = [(60 * i, 60 * i) for i in range(6)]
        with (run / "wheel_telem.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.COLS)
            writer.writeheader()
            writer.writerows(self.rows(counts, deltas=[(30, 30)] * 6,
                                       seqs=list(range(6))))

        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            wheel_check.main(str(run))
        text = out.getvalue()

        self.assertIn("HAI PHEP TINH KHONG KHOP", text)
        self.assertIn("KHONG chung minh", text)
        self.assertIn("Khong mat ban tin, khong reset", text)
        self.assertIn("KHONG gia dinh", text)
