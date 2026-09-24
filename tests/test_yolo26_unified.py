"""Offline checks on the unified-model output contracts; no model loaded.

Unlike the rest of the suite these need numpy and cv2, because the mask
they check IS an array. They skip rather than error where those are
absent, so `python3 -m unittest discover -s tests` still runs everywhere -
run them under .venv/bin/python to actually exercise them.
"""
import unittest

try:
    import cv2  # noqa: F401 - build_pred_mask imports it itself
    import numpy as np
    from yolo26_unified import (
        TRAIN_ID_TO_SEG_ID,
        build_detections,
        build_pred_mask,
    )
    DEPS = None
except ImportError as exc:                     # pragma: no cover
    DEPS = str(exc)


def box(x0, y0, x1, y1):
    return np.array([x0, y0, x1, y1], dtype=np.float32)


def square(x0, y0, x1, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=np.float32)


@unittest.skipIf(DEPS, "needs numpy and cv2: {}".format(DEPS))
class MaskTests(unittest.TestCase):
    def test_road_and_line_land_on_the_ids_scan_lane_reads(self):
        # Train ids: 4=road, 1=line -> Confg ids: 1=road, 2=line.
        mask = build_pred_mask([4, 1], [square(0, 0, 40, 40),
                                       square(10, 10, 20, 20)], 50, 50)
        self.assertEqual(mask[35, 35], 1)
        self.assertEqual(mask[15, 15], 2)
        self.assertEqual(mask[45, 45], 0)

    def test_line_wins_where_it_overlaps_road(self):
        # Emitted road-last, so a naive draw order would bury the line.
        mask = build_pred_mask([1, 4], [square(5, 5, 15, 15),
                                        square(0, 0, 40, 40)], 50, 50)
        self.assertEqual(mask[10, 10], 2)

    def test_person_and_vehicle_masks_never_enter_the_lane_mask(self):
        # Train ids 3=person, 0=car, 2=motobike cover most of the frame here;
        # scan_lane() must not read them as road or line.
        mask = build_pred_mask([3, 0, 2], [square(0, 0, 49, 49)] * 3, 50, 50)
        self.assertEqual(mask.max(), 0)

    def test_degenerate_polygon_is_skipped_not_raised(self):
        mask = build_pred_mask([4, 4], [np.array([[1, 1], [2, 2]]),
                                        square(0, 0, 30, 30)], 40, 40)
        self.assertEqual(mask[10, 10], 1)

    def test_no_instances_gives_an_all_background_mask(self):
        mask = build_pred_mask([], [], 12, 34)
        self.assertEqual(mask.shape, (12, 34))
        self.assertEqual(mask.dtype, np.uint8)
        self.assertEqual(mask.max(), 0)


@unittest.skipIf(DEPS, "needs numpy and cv2: {}".format(DEPS))
class DetectionTests(unittest.TestCase):
    def test_labels_follow_the_trained_class_order(self):
        found = build_detections([3, 0, 2], [box(0, 0, 100, 100)] * 3,
                                 [0.9, 0.8, 0.7], min_area=1)
        self.assertEqual([d["label"] for d in found], ["Nguoi", "Xe", "Xe"])
        self.assertEqual([d["class_id"] for d in found], [3, 0, 2])

    def test_road_and_line_are_not_reported_as_intruders(self):
        self.assertEqual(
            build_detections([4, 1], [box(0, 0, 200, 200)] * 2,
                             [0.99, 0.99], min_area=1),
            [],
        )

    def test_small_boxes_are_dropped_like_the_old_min_area_filter(self):
        found = build_detections([3, 3], [box(0, 0, 10, 10),
                                          box(0, 0, 100, 100)],
                                 [0.9, 0.9], min_area=1500)
        self.assertEqual(len(found), 1)
        self.assertEqual(found[0]["area"], 10000)

    def test_confidence_and_area_are_plain_floats_for_the_alert_path(self):
        found = build_detections([3], [box(0, 0, 100, 50)], [np.float32(0.75)],
                                 min_area=1)
        self.assertIsInstance(found[0]["confidence"], float)
        self.assertIsInstance(found[0]["area"], float)
        self.assertAlmostEqual(found[0]["confidence"], 0.75, places=5)


@unittest.skipIf(DEPS, "needs numpy and cv2: {}".format(DEPS))
class MappingTests(unittest.TestCase):
    def test_every_trained_class_maps_to_a_distinct_confg_id(self):
        # Guards against the off-by-one that convert_coco introduced once:
        # a shifted table silently turns road into person.
        self.assertEqual(sorted(TRAIN_ID_TO_SEG_ID), [0, 1, 2, 3, 4])
        self.assertEqual(sorted(TRAIN_ID_TO_SEG_ID.values()), [1, 2, 3, 4, 5])

    def test_mapping_matches_confg_class_names(self):
        import Confg
        expected = {0: "car", 1: "line", 2: "motobike", 3: "person", 4: "road"}
        for train_id, seg_id in TRAIN_ID_TO_SEG_ID.items():
            self.assertEqual(Confg.SEG_CLASS_NAMES[seg_id], expected[train_id])


class PatrolBranchParityTests(unittest.TestCase):
    """The unified branch must set every name the frame's [STAGE] print and
    the lane code read, or the loop dies on frame 1 - on the car, in the
    lab. Checked by parsing patrol_robot.py, so no model import is needed."""

    NEEDED = {"pred_mask", "perf_seg_ms", "perf_pre_ms", "perf_gpu_ms"}

    def setUp(self):
        import ast
        from pathlib import Path
        source = (Path(__file__).resolve().parents[1]
                  / "patrol_robot.py").read_text()
        tree = ast.parse(source)
        self.ast = ast
        # Several `if USE_UNIFIED_YOLO26:` blocks exist (model loading, the
        # frame's inference, the alert path). The inference one is the only
        # one that calls unified_model.infer().
        def calls_infer(node):
            return any(isinstance(sub, ast.Attribute) and sub.attr == "infer"
                       and isinstance(sub.value, ast.Name)
                       and sub.value.id == "unified_model"
                       for sub in ast.walk(node))

        self.branch = next(
            n for n in ast.walk(tree)
            if isinstance(n, ast.If)
            and isinstance(n.test, ast.Name)
            and n.test.id == "USE_UNIFIED_YOLO26"
            and calls_infer(n)
        )

    def assigned(self, body):
        names = set()
        for node in body:
            for sub in self.ast.walk(node):
                if isinstance(sub, self.ast.Assign):
                    for target in sub.targets:
                        if isinstance(target, self.ast.Name):
                            names.add(target.id)
                        elif isinstance(target, self.ast.Tuple):
                            names.update(e.id for e in target.elts
                                         if isinstance(e, self.ast.Name))
        return names

    def test_unified_branch_sets_what_the_frame_print_reads(self):
        missing = self.NEEDED - self.assigned(self.branch.body)
        self.assertEqual(missing, set())

    def test_legacy_branches_still_set_the_same_names(self):
        # orelse holds `elif seg_model is not None:` and its own else.
        for body in (self.branch.orelse,):
            missing = self.NEEDED - self.assigned(body)
            self.assertEqual(missing, set())

    def test_unified_branch_publishes_the_detections_the_alert_path_reads(self):
        self.assertIn("unified_persons", self.assigned(self.branch.body))


class BackendParityTests(unittest.TestCase):
    """patrol_robot.py picks UnifiedYOLO26 (ultralytics, laptop) or
    TensorRTUnifiedYOLO26 (Jetson, no ultralytics) via a bare try/except
    ImportError - see the AGV_USE_YOLO26_UNIFIED block. That fallback is
    only safe if both classes expose the same call surface; checked via
    ast so this file needs neither ultralytics nor tensorrt installed."""

    def test_both_backends_expose_the_same_infer_and_warmup_signature(self):
        import ast
        import inspect
        from pathlib import Path

        root = Path(__file__).resolve().parents[1]

        def methods_of(path, class_name):
            tree = ast.parse((root / path).read_text())
            cls = next(n for n in ast.walk(tree)
                      if isinstance(n, ast.ClassDef) and n.name == class_name)
            out = {}
            for n in cls.body:
                if isinstance(n, ast.FunctionDef) and n.name in ("infer", "warmup"):
                    out[n.name] = [a.arg for a in n.args.args]
            return out

        laptop = methods_of("yolo26_unified.py", "UnifiedYOLO26")
        jetson = methods_of("yolo26_tensorrt_runtime.py", "TensorRTUnifiedYOLO26")
        self.assertEqual(set(laptop), {"infer", "warmup"})
        self.assertEqual(set(laptop), set(jetson))
        for name in laptop:
            self.assertEqual(laptop[name], jetson[name],
                             "{}() argument names diverged between backends".format(name))


if __name__ == "__main__":
    unittest.main()
