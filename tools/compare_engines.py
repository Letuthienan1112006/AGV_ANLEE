"""So sanh hai engine TensorRT tren cung mot bo anh: IoU mask road/line +
phan ra thoi gian suy luan. Dung de tra loi "engine moi co nhanh hon MA
KHONG mu di khong" - hien khong co cong cu nao khac lam viec nay trong
repo, nen con so IoU tu bao cao truoc do khong tai lap duoc.

Chi dung tren xe (can TensorRT that su tren engine .engine cua Jetson).
Import TensorRT chi o trong main(), de module nay VAN import duoc tren
may dev khong co TensorRT/pycuda (vd de chay tests/test_compare_engines.py
mot minh mask_iou()/road_line_iou() thuan numpy).

Quy uoc bien cua mask_iou(): ca hai mask rong -> 1.0 (khong co gi de bat
dong); dung mot ben rong -> 0.0 (mot ben mu, khong phai NaN).

Vi mot vach bi bo sot lam XE DUNG IM (xem Confg.py:118-129), cong dung
IoU nay lay MIN qua toan bo anh, khong lay mean - mot anh mu la du de xe
dung.
"""
import argparse
import statistics

import numpy as np

from yolo26_unified import LINE_SEG_ID, ROAD_SEG_ID


def mask_iou(a, b):
    """IoU thuan numpy cua hai mask boolean cung shape.

    Ca hai mask rong (khong pixel nao True) -> 1.0: khong co gi de bat
    dong, va tranh chia 0/0 -> NaN. Dung mot ben rong -> 0.0 tu nhien vi
    intersection = 0 nhung union > 0 (khong can nhanh dac biet)."""
    a = np.asarray(a, dtype=bool)
    b = np.asarray(b, dtype=bool)
    union = np.count_nonzero(a | b)
    if union == 0:
        return 1.0
    intersection = np.count_nonzero(a & b)
    return float(intersection) / float(union)


def road_line_iou(pred_mask_a, pred_mask_b):
    """IoU rieng cho road (id 1) va line (id 2) tu hai pred_mask day du
    (0=bg,1=road,2=line,...). Tach ro rang khoi mask_iou() de road va
    line khong bao gio bi gop chung khi so sanh - moi ben duoc loc ve
    dung id cua no truoc khi dua vao mask_iou()."""
    iou_road = mask_iou(pred_mask_a == ROAD_SEG_ID, pred_mask_b == ROAD_SEG_ID)
    iou_line = mask_iou(pred_mask_a == LINE_SEG_ID, pred_mask_b == LINE_SEG_ID)
    return iou_road, iou_line


def _measure(model_a, model_b, frame, warmup, iterations):
    """Chay warmup roi do xen ke A,B,A,B (khong phai 12 lan A roi 12 lan
    B), de nhiet do/throttle cua Tegra X1 khong don het vao engine do
    sau. Tra ve ket qua infer() CUOI CUNG cua moi ben (dung cho so sanh
    mask/detection - infer() la deterministic voi cung anh dau vao) va
    danh sach total_ms cua tung lan do (dung tinh median)."""
    for _ in range(warmup):
        model_a.infer(frame)
        model_b.infer(frame)
    times_a, times_b = [], []
    result_a = result_b = None
    for _ in range(iterations):
        result_a = model_a.infer(frame)
        times_a.append(model_a.last_timing_ms["total"])
        result_b = model_b.infer(frame)
        times_b.append(model_b.last_timing_ms["total"])
    return result_a, result_b, times_a, times_b


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True,
                        help="engine A, dang production hien tai")
    parser.add_argument("--candidate", required=True,
                        help="engine B, ung vien moi")
    parser.add_argument("images", nargs="+",
                        help="anh de so sanh, cung bo anh cho ca hai engine")
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--iterations", type=int, default=12)
    args = parser.parse_args()
    if args.warmup < 0 or args.iterations < 1:
        parser.error("warmup >= 0 va iterations >= 1")

    import cv2

    import Confg
    from yolo26_tensorrt_runtime import TensorRTUnifiedYOLO26

    kwargs = dict(
        conf_thres=Confg.UNIFIED_CONF_THRESHOLD,
        lane_conf_thres=Confg.UNIFIED_LANE_CONF_THRESHOLD,
        min_area=Confg.PERSON_MIN_AREA,
    )
    print("[compare_engines] baseline=%s" % args.baseline)
    print("[compare_engines] candidate=%s" % args.candidate)
    model_a = TensorRTUnifiedYOLO26(args.baseline, **kwargs)
    model_b = TensorRTUnifiedYOLO26(args.candidate, **kwargs)
    try:
        min_iou_road = 1.0
        min_iou_line = 1.0
        for path in args.images:
            frame = cv2.imread(path)
            if frame is None:
                parser.error("khong doc duoc anh: " + path)
            result_a, result_b, times_a, times_b = _measure(
                model_a, model_b, frame, args.warmup, args.iterations)
            mask_a, persons_a = result_a
            mask_b, persons_b = result_b
            iou_road, iou_line = road_line_iou(mask_a, mask_b)
            min_iou_road = min(min_iou_road, iou_road)
            min_iou_line = min(min_iou_line, iou_line)

            road_px_a = int((mask_a == ROAD_SEG_ID).sum())
            road_px_b = int((mask_b == ROAD_SEG_ID).sum())
            line_px_a = int((mask_a == LINE_SEG_ID).sum())
            line_px_b = int((mask_b == LINE_SEG_ID).sum())

            print("image=%s iou_road=%.4f iou_line=%.4f" % (
                path, iou_road, iou_line))
            print("  road_px A=%d B=%d  line_px A=%d B=%d" % (
                road_px_a, road_px_b, line_px_a, line_px_b))
            print("  detections(person/xe) A=%d B=%d" % (
                len(persons_a), len(persons_b)))
            print("  total_ms median A=%.1f B=%.1f  (warmup=%d iterations=%d, xen ke A,B)" % (
                statistics.median(times_a), statistics.median(times_b),
                args.warmup, args.iterations))
        print("=" * 60)
        print("min_iou_road=%.4f min_iou_line=%.4f qua %d anh" % (
            min_iou_road, min_iou_line, len(args.images)))
    finally:
        model_a.close()
        model_b.close()


if __name__ == "__main__":
    main()
