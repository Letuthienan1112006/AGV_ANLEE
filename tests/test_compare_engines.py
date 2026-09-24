"""Test phan numpy thuan cua tools/compare_engines.py: chi mask_iou() va
road_line_iou(). KHONG can TensorRT/GPU - tools/compare_engines.py de
import TensorRT lazy trong main() dung de cac test nay chay tren may dev."""
import unittest

import numpy as np

from tools.compare_engines import mask_iou, road_line_iou


class MaskIoUTests(unittest.TestCase):
    def test_identical_masks_give_one(self):
        a = np.array([[True, False], [True, True]])
        self.assertEqual(mask_iou(a, a.copy()), 1.0)

    def test_completely_disjoint_masks_give_zero(self):
        a = np.array([True, True, False, False])
        b = np.array([False, False, True, True])
        self.assertEqual(mask_iou(a, b), 0.0)

    def test_half_overlap_matches_the_formula(self):
        # a: {0,1}  b: {1,2}  intersection={1} -> 1  union={0,1,2} -> 3
        a = np.array([True, True, False, False])
        b = np.array([False, True, True, False])
        self.assertAlmostEqual(mask_iou(a, b), 1.0 / 3.0, places=6)

    def test_both_empty_is_one_not_nan(self):
        a = np.zeros((4, 4), dtype=bool)
        b = np.zeros((4, 4), dtype=bool)
        result = mask_iou(a, b)
        self.assertEqual(result, 1.0)
        self.assertFalse(np.isnan(result))

    def test_exactly_one_side_empty_is_zero_not_nan(self):
        # This is the dangerous case: a candidate engine going blind on one
        # class while the baseline still sees it. Must be a hard 0.0, not
        # NaN (NaN >= 0.90 is silently False and easy to miss in a log).
        a = np.zeros((4, 4), dtype=bool)
        b = np.zeros((4, 4), dtype=bool)
        b[1, 1] = True
        result = mask_iou(a, b)
        self.assertEqual(result, 0.0)
        self.assertFalse(np.isnan(result))

    def test_accepts_int_masks_not_just_bool(self):
        # pred_mask == ROAD_SEG_ID style comparisons already return bool,
        # but guard against callers passing 0/1 int arrays too.
        a = np.array([1, 1, 0, 0])
        b = np.array([1, 0, 0, 0])
        self.assertAlmostEqual(mask_iou(a, b), 1.0 / 2.0, places=6)


class RoadLineIoUTests(unittest.TestCase):
    def test_road_and_line_are_scored_separately_not_merged(self):
        # pred_mask ids: 0=bg, 1=road, 2=line. A candidate that gets road
        # perfectly right but line completely wrong must NOT be hidden by
        # averaging the two classes together.
        pred_a = np.array([
            [1, 1, 2, 2],
            [1, 1, 2, 2],
        ], dtype=np.uint8)
        pred_b = np.array([
            [1, 1, 0, 0],   # road identical to A
            [1, 1, 0, 0],   # line completely missing in B
        ], dtype=np.uint8)
        iou_road, iou_line = road_line_iou(pred_a, pred_b)
        self.assertEqual(iou_road, 1.0)
        self.assertEqual(iou_line, 0.0)

    def test_identical_full_masks_give_one_for_both_classes(self):
        pred = np.array([[0, 1, 2], [1, 2, 0]], dtype=np.uint8)
        iou_road, iou_line = road_line_iou(pred, pred.copy())
        self.assertEqual(iou_road, 1.0)
        self.assertEqual(iou_line, 1.0)


if __name__ == "__main__":
    unittest.main()
