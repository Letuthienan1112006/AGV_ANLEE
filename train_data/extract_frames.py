#!/usr/bin/env python3
"""Rut anh (frame) tu video patrol de dua len tool label.

Vi du dung:
    # Rut het video trong agv_debug_videos, moi 3 frame lay 1 tam
    python train_data/extract_frames.py ../agv_debug_videos --stride 3

    # Chi 1 video, cach nhau 0.5 giay, bo frame mo va frame trung nhau
    python train_data/extract_frames.py ../agv_debug_videos/patrol_record_20260908_011906.avi \
        --every-sec 0.5 --dedup --drop-blur

    # Gioi han tong so anh xuat ra (chia deu tren toan video)
    python train_data/extract_frames.py ../agv_debug_videos --max-frames 300

Anh ra nam trong  train_data/frames_YYYYmmdd_HHMMSS/  (dat ten <ten_video>_fNNNNN.jpg),
zip lai roi keo len Roboflow la label duoc ngay.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2

VIDEO_EXTS = {".avi", ".mp4", ".mov", ".mkv", ".m4v"}


def find_videos(inputs):
    vids = []
    for item in inputs:
        p = Path(item)
        if p.is_dir():
            vids += sorted(q for q in p.rglob("*") if q.suffix.lower() in VIDEO_EXTS)
        elif p.is_file() and p.suffix.lower() in VIDEO_EXTS:
            vids.append(p)
        else:
            print(f"[bo qua] khong phai video/thu muc: {p}", file=sys.stderr)
    return vids


def blur_score(gray):
    """Cang thap cang mo (variance cua Laplacian)."""
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def frame_diff(prev_gray, gray):
    """Ty le pixel thay doi dang ke giua 2 frame (0..1)."""
    d = cv2.absdiff(prev_gray, gray)
    return float((d > 20).mean())


def pick_indices(total, stride, every_sec, fps, max_frames):
    if every_sec is not None:
        step = max(1, round(every_sec * fps))
    else:
        step = max(1, stride)
    idxs = list(range(0, total, step))
    if max_frames is not None and len(idxs) > max_frames:
        # chia deu lai cho du max_frames
        keep = [idxs[round(i * (len(idxs) - 1) / (max_frames - 1))] for i in range(max_frames)]
        idxs = sorted(set(keep))
    return idxs


def process_video(path, out_dir, args):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        print(f"[loi] mo khong duoc: {path}", file=sys.stderr)
        return 0
    fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    want = set(pick_indices(total, args.stride, args.every_sec, fps, args.max_frames))

    stem = path.stem
    saved = 0
    prev_gray = None
    idx = -1
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        idx += 1
        if idx not in want:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if args.drop_blur and blur_score(gray) < args.blur_thresh:
            continue
        if args.dedup and prev_gray is not None and frame_diff(prev_gray, gray) < args.min_diff:
            continue
        prev_gray = gray

        if args.resize_w and frame.shape[1] > args.resize_w:
            h = round(frame.shape[0] * args.resize_w / frame.shape[1])
            frame = cv2.resize(frame, (args.resize_w, h), interpolation=cv2.INTER_AREA)

        name = f"{stem}_f{idx:05d}.jpg"
        cv2.imwrite(str(out_dir / name), frame, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
        saved += 1

    cap.release()
    print(f"  {path.name}: {saved} anh (tong {total} frame, fps {fps:.1f})")
    return saved


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", help="File video hoac thu muc chua video")
    ap.add_argument("--stride", type=int, default=3, help="Cach bao nhieu frame lay 1 tam (mac dinh 3)")
    ap.add_argument("--every-sec", type=float, default=None, help="Lay 1 tam moi N giay (ghi de --stride)")
    ap.add_argument("--max-frames", type=int, default=None, help="Gioi han tong so anh moi video, chia deu")
    ap.add_argument("--dedup", action="store_true", help="Bo frame gan giong frame truoc (robot dung yen)")
    ap.add_argument("--min-diff", type=float, default=0.02, help="Nguong khac biet toi thieu cho --dedup (0..1)")
    ap.add_argument("--drop-blur", action="store_true", help="Bo frame bi mo")
    ap.add_argument("--blur-thresh", type=float, default=60.0, help="Nguong do net cho --drop-blur")
    ap.add_argument("--resize-w", type=int, default=0, help="Resize ve chieu rong nay (0 = giu nguyen)")
    ap.add_argument("--quality", type=int, default=95, help="Chat luong JPEG (mac dinh 95)")
    ap.add_argument("-o", "--out", default=None, help="Thu muc xuat (mac dinh train_data/frames_<timestamp>)")
    args = ap.parse_args()

    videos = find_videos(args.inputs)
    if not videos:
        print("Khong tim thay video nao.", file=sys.stderr)
        sys.exit(1)

    script_dir = Path(__file__).resolve().parent
    if args.out:
        out_dir = Path(args.out)
    else:
        out_dir = script_dir / f"frames_{time.strftime('%Y%m%d_%H%M%S')}"
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Xuat vao: {out_dir}")
    total_saved = 0
    for v in videos:
        total_saved += process_video(v, out_dir, args)

    print(f"\nXong: {total_saved} anh tu {len(videos)} video -> {out_dir}")
    print(f"Zip nhanh:  cd {out_dir.parent} && zip -r {out_dir.name}.zip {out_dir.name}")


if __name__ == "__main__":
    main()
