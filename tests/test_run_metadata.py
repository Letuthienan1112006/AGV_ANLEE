"""Exercise only the metadata heredoc, never execute run.sh or hardware."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]


class RunMetadataTests(unittest.TestCase):
    def test_all_declared_modes(self):
        source = (ROOT / "run.sh").read_text(encoding="utf-8")
        marker = 'python3 - "$MODE" "$RUN_DIR" <<\'PY\'\n'
        code = source.split(marker, 1)[1].split("\nPY\n", 1)[0]
        for mode, stand, dry, bypass in (
            ("road", False, False, False), ("trim", False, False, False),
            ("lane", False, False, True),
            # control drives by hand but keeps LiDAR: it must never be
            # classified as bypassed or as on-a-stand.
            ("control", False, False, False),
            ("bench", True, False, False), ("sweep", True, False, True),
            ("wcal", True, False, True),
            ("dry", None, True, False), ("snap", None, True, False),
        ):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory, \
                    mock.patch.object(sys, "argv", ["metadata", mode, directory]), \
                    mock.patch.object(subprocess, "check_output", return_value=b"test-commit\n"), \
                    mock.patch.object(subprocess, "call", return_value=1), \
                    contextlib.redirect_stdout(io.StringIO()):
                exec(compile(code, "run.sh:metadata-only", "exec"), {})
                result = json.loads((Path(directory) / "run_mode.json").read_text())
                self.assertEqual(result["mode"], mode)
                self.assertIs(result["declared_on_stand"], stand)
                self.assertIs(result["dry_run"], dry)
                self.assertIs(result["lidar_bypassed"], bypass)
                self.assertEqual(result["vision_backend"], "yolo26_unified")
                self.assertEqual(result["git_head"], "test-commit")
                self.assertTrue(result["git_dirty"])

    def test_legacy_backend_must_be_explicit(self):
        source = (ROOT / "run.sh").read_text(encoding="utf-8")
        marker = 'python3 - "$MODE" "$RUN_DIR" <<\'PY\'\n'
        code = source.split(marker, 1)[1].split("\nPY\n", 1)[0]
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(sys, "argv", ["metadata", "lane", directory]), \
                mock.patch.dict("os.environ", {"AGV_USE_YOLO26_UNIFIED": "0"}), \
                mock.patch.object(subprocess, "check_output", return_value=b"test\n"), \
                mock.patch.object(subprocess, "call", return_value=0):
            exec(compile(code, "run.sh:metadata-only", "exec"), {})
            result = json.loads((Path(directory) / "run_mode.json").read_text())
            self.assertEqual(result["vision_backend"], "legacy_two_model")

    def test_snap_forwards_calibration_options(self):
        source = (ROOT / "run.sh").read_text(encoding="utf-8")
        self.assertIn('snapshot_check.py 3 "${@:2}"', source)


if __name__ == "__main__":
    unittest.main()
