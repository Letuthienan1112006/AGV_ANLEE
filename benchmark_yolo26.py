"""Benchmark the Jetson TensorRT runtime on one image; no bridge or motors."""
import argparse
import statistics

import cv2

import Confg
from yolo26_tensorrt_runtime import TensorRTUnifiedYOLO26


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image")
    parser.add_argument("--engine", default=Confg.UNIFIED_ENGINE_PATH,
                        help="engine TensorRT can benchmark")
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--iterations", type=int, default=12)
    args = parser.parse_args()
    if args.warmup < 0 or args.iterations < 1:
        parser.error("warmup >= 0 va iterations >= 1")
    frame = cv2.imread(args.image)
    if frame is None:
        parser.error("khong doc duoc anh: " + args.image)

    model = TensorRTUnifiedYOLO26(
        args.engine,
        conf_thres=Confg.UNIFIED_CONF_THRESHOLD,
        min_area=Confg.PERSON_MIN_AREA,
    )
    try:
        for _ in range(args.warmup):
            model.infer(frame)
        samples = []
        for _ in range(args.iterations):
            mask, _ = model.infer(frame)
            timing = dict(model.last_timing_ms)
            timing["road_px"] = int((mask == 1).sum())
            timing["line_px"] = int((mask == 2).sum())
            samples.append(timing)
        keys = ("pre", "h2d", "execute", "d2h", "post", "total")
        print("iterations=%d road_px=%d line_px=%d" % (
            len(samples), samples[-1]["road_px"], samples[-1]["line_px"],
        ))
        for key in keys:
            values = [sample[key] for sample in samples]
            print("%s_ms mean=%.1f median=%.1f min=%.1f max=%.1f" % (
                key, statistics.mean(values), statistics.median(values),
                min(values), max(values),
            ))
        median_total = statistics.median(
            sample["total"] for sample in samples)
        print("continuous_fps=%.2f" % (1000.0 / median_total))
    finally:
        model.close()


if __name__ == "__main__":
    main()
