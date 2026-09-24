"""Offline checks on the raw TensorRT/ONNX decode path - deterministic
pieces only (box math, NMS, rescale, mask/detection assembly). The
end-to-end numeric match against ultralytics' own decode (confirmed
within ~0.01 confidence and >92% mask pixel agreement across 8 real
images, 2026-09-15) needs the trained .onnx file and isn't reproduced
here as an automated test - see the session's verification notes."""
import unittest

try:
    import numpy as np
    from yolo26_raw_decode import (
        build_detections_from_boxes,
        build_pred_mask_from_masks,
        decode_masks,
        infer_from_raw_outputs,
        letterbox_image,
        letterbox_params,
        nms,
        rescale_boxes,
        xywh_to_xyxy,
    )
    DEPS = None
except ImportError as exc:                     # pragma: no cover
    DEPS = str(exc)


@unittest.skipIf(DEPS, "needs numpy: {}".format(DEPS))
class BoxMathTests(unittest.TestCase):
    def test_xywh_to_xyxy(self):
        boxes = np.array([[100, 100, 40, 20]], dtype=np.float32)
        out = xywh_to_xyxy(boxes)
        np.testing.assert_allclose(out[0], [80, 90, 120, 110])

    def test_letterbox_params_fits_the_longer_side(self):
        # 256x448 (h,w) into 640: width is the binding constraint.
        ratio, pad_x, pad_y = letterbox_params(256, 448, 640)
        self.assertAlmostEqual(ratio, 640 / 448, places=5)
        self.assertAlmostEqual(pad_x, 0, places=1)
        self.assertGreater(pad_y, 0)

    def test_rescale_boxes_undoes_letterbox(self):
        ratio, pad_x, pad_y = letterbox_params(256, 448, 640)
        # A box spanning the full content band (padding excluded on both
        # edges) should map back to the full original frame.
        boxes640 = np.array([[0, pad_y, 640, 640 - pad_y]], dtype=np.float32)
        back = rescale_boxes(boxes640, ratio, pad_x, pad_y, 256, 448)
        np.testing.assert_allclose(back[0], [0, 0, 448, 256], atol=1.0)

    def test_lane_threshold_admits_a_faint_line_the_shared_one_discards(self):
        # Night run 131525: the line was present but scored below 0.40, so
        # 78% of frames reported no lane at all. A lower LANE threshold has
        # to let it through while the person/car threshold stays put.
        from yolo26_raw_decode import decode_detections
        out = np.zeros((1, 4 + 5 + 32, 2), dtype=np.float32)
        out[0, :4, :] = np.array([[100, 100], [50, 50], [20, 20], [20, 20]])
        out[0, 4 + 1, 0] = 0.25   # line, faint
        out[0, 4 + 3, 1] = 0.25   # person, equally faint

        _, cls_shared, _, _ = decode_detections(out, conf_thres=0.40)
        self.assertEqual(len(cls_shared), 0)

        _, cls_split, _, _ = decode_detections(
            out, conf_thres=0.40, lane_conf_thres=0.15)
        self.assertEqual(list(cls_split), [1])   # line in, person still out

    def test_lane_threshold_defaults_to_the_shared_one(self):
        from yolo26_raw_decode import decode_detections
        out = np.zeros((1, 4 + 5 + 32, 1), dtype=np.float32)
        out[0, :4, 0] = [100, 50, 20, 20]
        out[0, 4 + 1, 0] = 0.25
        for kwargs in ({}, {"lane_conf_thres": None}):
            _, cls, _, _ = decode_detections(out, conf_thres=0.20, **kwargs)
            self.assertEqual(list(cls), [1])

    def test_rectangular_target_pads_only_the_axis_that_needs_it(self):
        # 640x360 camera into a 512x288 model: both are 16:9, so a correct
        # implementation adds essentially NO padding. The whole point of the
        # rectangular export is that a square 640x640 spends 44% of its
        # compute on grey pixels for this camera.
        ratio, pad_x, pad_y = letterbox_params(360, 640, 288, 512)
        self.assertAlmostEqual(ratio, 512 / 640, places=6)
        self.assertAlmostEqual(pad_x, 0, places=6)
        self.assertAlmostEqual(pad_y, 0, places=6)

    def test_rectangular_letterbox_fills_the_whole_destination_buffer(self):
        frame = np.full((360, 640, 3), 200, dtype=np.uint8)
        dest = np.empty((1, 3, 288, 512), dtype=np.float32)
        out, ratio, left, top = letterbox_image(
            frame, new_h=288, new_w=512, destination=dest)
        self.assertIs(out, dest)
        self.assertEqual((left, top), (0, 0))
        # No grey band anywhere: every pixel came from the image.
        self.assertGreater(out.min(), 114.0 / 255.0)

    def test_rectangular_target_still_pads_a_mismatched_aspect(self):
        # A 4:3 frame (640x480) into a 16:9 model is relatively TALLER than
        # the target, so height binds and the padding lands on the sides -
        # 384 of 512 columns used, 64 grey each side.
        ratio, pad_x, pad_y = letterbox_params(480, 640, 288, 512)
        self.assertAlmostEqual(ratio, 288 / 480, places=6)
        self.assertAlmostEqual(pad_y, 0, places=6)
        self.assertAlmostEqual(pad_x, 64, places=6)

    def test_square_callers_are_unchanged_by_the_rectangular_signature(self):
        # Existing call sites pass one size positionally; that must keep
        # meaning "square", or every square deployment silently changes.
        self.assertEqual(letterbox_params(256, 448, 640),
                         letterbox_params(256, 448, 640, 640))

    def test_rescale_boxes_clips_to_frame(self):
        ratio, pad_x, pad_y = letterbox_params(256, 448, 640)
        boxes640 = np.array([[-50, -50, 700, 700]], dtype=np.float32)
        back = rescale_boxes(boxes640, ratio, pad_x, pad_y, 256, 448)
        self.assertTrue((back >= 0).all())
        self.assertLessEqual(back[0, 2], 448)
        self.assertLessEqual(back[0, 3], 256)

    def test_direct_letterbox_destination_matches_allocating_path(self):
        # Runtime fills its persistent TensorRT input buffer. It must remain
        # numerically identical to the simple allocation path used as the
        # reference by calibration/tools.
        frame = np.arange(360 * 640 * 3, dtype=np.uint8).reshape(360, 640, 3)
        expected, ratio_a, left_a, top_a = letterbox_image(frame)
        destination = np.empty_like(expected)
        actual, ratio_b, left_b, top_b = letterbox_image(
            frame, destination=destination)
        self.assertIs(actual, destination)
        self.assertEqual((ratio_a, left_a, top_a), (ratio_b, left_b, top_b))
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-7)


@unittest.skipIf(DEPS, "needs numpy: {}".format(DEPS))
class NMSTests(unittest.TestCase):
    def test_duplicate_box_is_suppressed(self):
        boxes = np.array([[0, 0, 100, 100], [2, 2, 98, 98]], dtype=np.float32)
        scores = np.array([0.9, 0.8])
        classes = np.array([3, 3])
        keep = nms(boxes, scores, classes, iou_thres=0.5)
        self.assertEqual(list(keep), [0])

    def test_different_classes_both_survive_even_if_overlapping(self):
        # Per-class NMS, matching agnostic_nms=False used to train this
        # model: a person standing in front of a car must not delete one.
        boxes = np.array([[0, 0, 100, 100], [10, 10, 90, 90]], dtype=np.float32)
        scores = np.array([0.9, 0.8])
        classes = np.array([3, 0])
        keep = nms(boxes, scores, classes, iou_thres=0.5)
        self.assertEqual(sorted(keep), [0, 1])

    def test_far_apart_boxes_both_survive(self):
        boxes = np.array([[0, 0, 50, 50], [500, 500, 600, 600]], dtype=np.float32)
        scores = np.array([0.9, 0.85])
        classes = np.array([4, 4])
        keep = nms(boxes, scores, classes, iou_thres=0.7)
        self.assertEqual(sorted(keep), [0, 1])

    def test_keeps_the_higher_scoring_box_of_a_duplicate_pair(self):
        boxes = np.array([[0, 0, 100, 100], [1, 1, 99, 99]], dtype=np.float32)
        scores = np.array([0.6, 0.95])
        classes = np.array([1, 1])
        keep = nms(boxes, scores, classes, iou_thres=0.5)
        self.assertEqual(list(keep), [1])


@unittest.skipIf(DEPS, "needs numpy: {}".format(DEPS))
class MaskAndDetectionAssemblyTests(unittest.TestCase):
    def test_instance_mask_is_cropped_to_its_detection_box(self):
        coeffs = np.zeros((1, 32), dtype=np.float32)
        coeffs[0, 0] = 1.0
        proto = np.zeros((1, 32, 160, 160), dtype=np.float32)
        proto[0, 0, :, :] = 10.0  # would fill the whole image without crop
        boxes = np.array([[160, 160, 320, 320]], dtype=np.float32)
        decoded = decode_masks(coeffs, proto, boxes, 1.0, 0, 0, 640, 640)
        self.assertEqual(len(decoded), 1)
        mask = decoded[0]
        self.assertTrue(mask[240, 240])
        self.assertFalse(mask[80, 80])
        self.assertFalse(mask[500, 500])

    def test_road_then_line_priority_matches_the_polygon_path(self):
        # Same rule as yolo26_unified.build_pred_mask, just from full-res
        # boolean masks instead of polygons - the two paths must agree.
        h = w = 20
        road_mask = np.zeros((h, w), dtype=bool)
        road_mask[:, :] = True
        line_mask = np.zeros((h, w), dtype=bool)
        line_mask[8:12, :] = True
        out = build_pred_mask_from_masks([4, 1], [road_mask, line_mask], h, w)
        self.assertEqual(out[0, 0], 1)   # road only
        self.assertEqual(out[10, 10], 2)  # line wins the overlap

    def test_person_and_vehicle_masks_never_enter_the_lane_mask(self):
        h = w = 10
        full = np.ones((h, w), dtype=bool)
        out = build_pred_mask_from_masks([3, 0, 2], [full, full, full], h, w)
        self.assertEqual(out.max(), 0)

    def test_detection_filter_matches_the_polygon_path_min_area_rule(self):
        boxes = np.array([[0, 0, 10, 10], [0, 0, 100, 100]], dtype=np.float32)
        found = build_detections_from_boxes([3, 3], boxes, [0.9, 0.9],
                                            min_area=1500)
        self.assertEqual(len(found), 1)
        self.assertEqual(found[0]["area"], 10000)

    def test_road_and_line_class_ids_are_not_reported_as_detections(self):
        boxes = np.array([[0, 0, 200, 200], [0, 0, 200, 200]], dtype=np.float32)
        found = build_detections_from_boxes([4, 1], boxes, [0.99, 0.99],
                                            min_area=1)
        self.assertEqual(found, [])

    def test_full_decode_only_rasterises_lane_classes(self):
        # Regression for Jetson CPU time: detections still include person/
        # vehicle boxes, but only road/line coefficients may reach mask
        # decoding. A person-only output therefore produces no lane mask.
        output0 = np.zeros((1, 41, 1), dtype=np.float32)
        output0[0, 0:4, 0] = [320, 320, 100, 100]
        output0[0, 4 + 3, 0] = 0.9  # train id 3 = person
        output1 = np.zeros((1, 32, 160, 160), dtype=np.float32)
        mask, detections = infer_from_raw_outputs(
            output0, output1, 360, 640, 1.0, 0, 140,
            conf_thres=0.4, min_area=1,
        )
        self.assertEqual(mask.max(), 0)
        self.assertEqual(len(detections), 1)


if __name__ == "__main__":
    unittest.main()
