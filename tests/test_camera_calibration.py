"""Offline tests: synthetic masks/records, no NumPy, model, socket or hardware."""
import contextlib
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest import mock

from camera_calibration import (
    HISTORY_VERSION, measure_line, scan_line_mid, summarize_placement,
    session_placements, propose_center,
)
import snapshot_check
import test_static_camera_calib


def samples(mid=344, heading=0.0, count=3):
    return [{"mid": mid, "heading_deg": heading, "heading_px": heading * 2}
            for _ in range(count)]


def record(identifier="placement-1", session="mount-a", identity=None, measured=None):
    return json.dumps({
        "schema": HISTORY_VERSION, "session": session,
        "placement_id": identifier, "confirmed": True,
        "identity": {"width": 640} if identity is None else identity,
        "samples": samples() if measured is None else measured,
    })


class MeasurementTests(unittest.TestCase):
    def mask(self):
        return [[1] * 640 for _ in range(360)]

    def test_road_is_not_a_line_measurement(self):
        measured = measure_line(self.mask(), 270, 180, 12, 20, 2)
        self.assertIsNone(measured["mid"])
        self.assertIsNone(measured["heading_deg"])

    def test_noise_below_controller_pixel_threshold_is_rejected(self):
        mask = self.mask()
        mask[270][330:349] = [2] * 19
        self.assertIsNone(scan_line_mid(mask, 270, 12, 20))

    def test_nearest_band_row_and_upper_tie_match_controller(self):
        mask = self.mask()
        mask[269][330:350] = [2] * 20
        mask[271][400:420] = [2] * 20
        self.assertEqual(scan_line_mid(mask, 270, 12, 20), 339)

    def test_far_line_is_required_for_calibration_heading(self):
        mask = self.mask()
        mask[270][335:355] = [2] * 20
        measured = measure_line(mask, 270, 180, 12, 20, 2)
        self.assertEqual(measured["mid"], 344)
        self.assertIsNone(measured["heading_deg"])

    def test_heading_uses_configured_near_far_and_weight(self):
        mask = self.mask()
        mask[270][335:355] = [2] * 20
        mask[180][339:359] = [2] * 20
        measured = measure_line(mask, 270, 180, 12, 20, 2)
        expected = math.degrees(math.atan2(4, 90))
        self.assertEqual(measured["mid"], 344)
        self.assertAlmostEqual(measured["heading_deg"], expected)
        self.assertAlmostEqual(measured["heading_px"], expected * 2)

    def test_heading_ignores_detached_far_false_positive(self):
        mask = self.mask()
        mask[270][335:375] = [2] * 40
        mask[180][340:380] = [2] * 40
        mask[180][500:580] = [2] * 80
        measured = measure_line(mask, 270, 180, 12, 20, 2)
        # Far association follows the component near the measured lane,
        # rather than spanning both components or choosing the wider ghost.
        expected = math.degrees(math.atan2(5, 90))
        self.assertAlmostEqual(measured["heading_deg"], expected)

    def test_center_candidate_matches_runtime_error_equation(self):
        summary = summarize_placement(samples(mid=403, heading=-3.0))
        self.assertTrue(summary["valid"])
        self.assertEqual(summary["center"], 397.0)

    def test_old_85_percent_scan_is_not_used(self):
        mask = self.mask()
        mask[306][100:120] = [2] * 20
        self.assertIsNone(scan_line_mid(mask, 270, 12, 20))


class PlacementTests(unittest.TestCase):
    def test_requires_three_valid_samples(self):
        self.assertFalse(summarize_placement(samples(count=2))["valid"])
        self.assertTrue(summarize_placement(samples())["valid"])

    def test_missing_heading_rejects_placement(self):
        measured = samples()
        measured[-1]["heading_deg"] = None
        self.assertFalse(summarize_placement(measured)["valid"])

    def test_one_crooked_sample_is_not_hidden_by_good_median(self):
        measured = samples()
        measured[-1]["heading_deg"] = 10
        self.assertFalse(summarize_placement(measured)["valid"])

    def test_nan_heading_is_rejected(self):
        self.assertFalse(summarize_placement(samples(heading=float("nan")))["valid"])

    def test_center_spread_rejects_placement(self):
        measured = samples()
        measured[-1]["mid"] += 26
        self.assertFalse(summarize_placement(measured)["valid"])

    def test_even_sample_median_is_not_upper_middle(self):
        measured = samples(count=4)
        measured[-2]["mid"] = 346
        measured[-1]["mid"] = 346
        self.assertEqual(summarize_placement(measured)["mid"], 345)

    def test_heading_contributes_to_proposed_center(self):
        placements = [summarize_placement(samples(mid=403, heading=-3.0))
                      for _ in range(3)]
        self.assertEqual(propose_center(placements)["center"], 397)


class HistoryTests(unittest.TestCase):
    def test_three_independent_confirmed_placements_propose_center(self):
        placements = session_placements([record(str(i)) for i in range(3)],
                                        "mount-a", {"width": 640})
        self.assertEqual(propose_center(placements)["center"], 344)

    def test_single_placement_does_not_propose_center(self):
        placements = session_placements([record()], "mount-a", {"width": 640})
        self.assertFalse(propose_center(placements)["valid"])

    def test_different_mount_session_is_not_pooled(self):
        placements = session_placements([record(session="old-mount"), record("new")],
                                        "mount-a", {"width": 640})
        self.assertEqual(len(placements), 1)

    def test_changed_model_or_geometry_in_same_session_rejected(self):
        for identity in ({"width": 800}, {"width": 640, "model_sha256": "changed"}):
            with self.subTest(identity=identity), self.assertRaises(ValueError):
                session_placements([record()], "mount-a", identity)

    def test_duplicate_placement_ids_cannot_count_twice(self):
        with self.assertRaises(ValueError):
            session_placements([record(), record()], "mount-a", {"width": 640})

    def test_old_unversioned_csv_cannot_be_pooled(self):
        with self.assertRaises(ValueError):
            session_placements(["20260911_010000,344,0,0"], "mount-a", {"width": 640})

    def test_crooked_history_cannot_be_promoted(self):
        with self.assertRaises(ValueError):
            session_placements([record(measured=samples(heading=10))],
                               "mount-a", {"width": 640})

    def test_unconfirmed_history_is_rejected(self):
        entry = json.loads(record())
        entry["confirmed"] = False
        with self.assertRaises(ValueError):
            session_placements([json.dumps(entry)], "mount-a", {"width": 640})

    def test_inconsistent_placements_do_not_propose_center(self):
        placements = [summarize_placement(samples(mid=mid)) for mid in (320, 344, 380)]
        self.assertFalse(propose_center(placements)["valid"])


class EntrypointTests(unittest.TestCase):
    def test_inspector_defaults_to_same_yolo26_backend_as_patrol(self):
        source = Path(snapshot_check.__file__).read_text(encoding="utf-8")
        self.assertIn('os.environ.get("AGV_USE_YOLO26_UNIFIED", "1")', source)

    def test_default_is_inspection_only(self):
        args = snapshot_check.parse_args([])
        self.assertEqual(args.count, 3)
        self.assertFalse(args.confirm_placement)
        self.assertIsNone(args.session)

    def test_confirm_requires_session_and_enough_samples(self):
        for args in (["--confirm-placement"],
                     ["2", "--session", "mount-a", "--confirm-placement"]):
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    snapshot_check.parse_args(args)

    def test_static_helper_delegates_exact_measurement_and_arguments(self):
        args = ["3", "--session", "mount-a", "--confirm-placement"]
        with mock.patch.object(snapshot_check, "main", return_value=0) as inspect:
            self.assertEqual(test_static_camera_calib.main(args), 0)
            inspect.assert_called_once_with(args)

    def test_inspection_does_not_write_calibration_history(self):
        with mock.patch.object(snapshot_check, "SnapshotInspector") as inspector_type, \
                mock.patch.object(snapshot_check.socket, "create_connection"), \
                mock.patch.object(snapshot_check.time, "sleep"), \
                mock.patch.object(snapshot_check, "save_placement") as save, \
                contextlib.redirect_stdout(io.StringIO()):
            inspector = inspector_type.return_value
            inspector.config = SimpleNamespace(SOCKET_IP="unused", SOCKET_PORT=0,
                                                CAMERA_WARMUP_FRAMES=0)
            inspector.one_shot.side_effect = samples()
            self.assertEqual(snapshot_check.main([]), 0)
            save.assert_not_called()

    def test_invalid_confirmed_placement_does_not_write_history(self):
        with mock.patch.object(snapshot_check, "SnapshotInspector") as inspector_type, \
                mock.patch.object(snapshot_check.socket, "create_connection"), \
                mock.patch.object(snapshot_check.time, "sleep"), \
                mock.patch.object(snapshot_check, "save_placement") as save, \
                contextlib.redirect_stdout(io.StringIO()):
            inspector = inspector_type.return_value
            inspector.config = SimpleNamespace(SOCKET_IP="unused", SOCKET_PORT=0,
                                                CAMERA_WARMUP_FRAMES=0)
            inspector.one_shot.side_effect = samples(heading=10)
            self.assertEqual(snapshot_check.main(["--session", "mount-a", "--confirm-placement"]), 1)
            save.assert_not_called()

    def test_incompatible_history_is_not_appended(self):
        storage = mock.mock_open(read_data=record() + "\n")
        with mock.patch("builtins.open", storage), self.assertRaises(ValueError):
            snapshot_check.save_placement(samples(), "mount-a", {"width": 800})
        self.assertEqual(storage.call_count, 1)


if __name__ == "__main__":
    unittest.main()
