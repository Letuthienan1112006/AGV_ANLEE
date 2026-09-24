"""Pure regressions for selecting one physical stripe from mask fragments."""
import unittest

import numpy as np

from lane_geometry import contiguous_line_segments, select_line_segment


class LaneSegmentTests(unittest.TestCase):
    def test_detached_blobs_are_never_spanned_as_one_lane(self):
        row = [0] * 640
        row[300:360] = [2] * 60
        row[500:540] = [2] * 40
        self.assertEqual(
            contiguous_line_segments(row, min_pixels=20),
            [(300, 359, 329), (500, 539, 519)],
        )
        self.assertEqual(select_line_segment(row, 20), (300, 359, 329))

    def test_far_scan_tracks_component_nearest_near_lane(self):
        row = [0] * 640
        row[280:320] = [2] * 40
        row[470:560] = [2] * 90  # wider false-positive
        self.assertEqual(
            select_line_segment(row, 20, reference_x=330),
            (280, 319, 299),
        )

    def test_each_component_must_meet_minimum_width_itself(self):
        row = [0] * 100
        row[5:16] = [2] * 11
        row[50:61] = [2] * 11
        # The old total-pixel test accepted 22 pixels then spanned 5..60.
        self.assertIsNone(select_line_segment(row, 20))

    def test_numpy_runtime_path_matches_list_calibration_path(self):
        row = [0] * 100
        row[10:35] = [2] * 25
        row[60:90] = [2] * 30
        self.assertEqual(
            contiguous_line_segments(np.asarray(row), min_pixels=20),
            contiguous_line_segments(row, min_pixels=20),
        )


if __name__ == "__main__":
    unittest.main()
