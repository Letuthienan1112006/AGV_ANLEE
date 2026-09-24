#!/usr/bin/env python3
"""Chup anh LIEN TUC tu camera de gom data train lai.

Mo thang camera USB (C270), luu frame ra dia lien tuc cho toi khi nhan Ctrl+C
(hoac het --max / --duration). Day robot di quanh cung/hanh lang, script tu luu.

Vi du:
    # Luu 2 tam / giay, bam Ctrl+C de dung
    python train_data/collect_frames.py --fps 2

    # Luu 3 tam/giay, bo frame gan giong nhau (dung 1 cho thi khong luu thua)
    python train_data/collect_frames.py --fps 3 --dedup

    # Chup toi da 500 tam roi tu dung
    python train_data/collect_frames.py --fps 2 --max 500

    # Chup trong 3 phut
    python train_data/collect_frames.py --fps 2 --duration 180

    # Xem truoc (may co man hinh)
    python train_data/collect_frames.py --fps 2 --show

Anh ra:  train_data/collect_YYYYmmdd_HHMMSS/cam_000001.jpg
Xong thi zip lai keo len Roboflow label -> train lai.

LUU Y: camera chi mot tien trinh xai duoc. Tren xe phai TAT bridge/patrol truoc
khi chay script nay, khong la tranh camera.
"""
import argparse
import signal
import sys
import time
from pathlib import Path

import cv2

# Lay default tu Confg.py neu import duoc (chay o thu muc goc repo)
try:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    import Confg
    DEF_INDEX = getattr(Confg, "CAMERA_INDEX", 0)
    DEF_W = getattr(Confg, "CAMERA_WIDTH", 640)
    DEF_H = getattr(Confg, "CAMERA_HEIGHT", 360)
except Exception:
    DEF_INDEX, DEF_W, DEF_H = 0, 640, 360

_stop = False


def _on_sigint(signum, frame):
    global _stop
    _stop = True
    print("\n[Ctrl+C] dang dung, ghi not anh cuoi...")


def frame_diff(prev_gray, gray):
    d = cv2.absdiff(prev_gray, gray)
    return float((d > 20).mean())


def open_camera(index, w, h):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    # MJPG cho webcam USB chay nhanh hon YUYV
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--index", type=int, default=DEF_INDEX, help=f"Camera index (mac dinh {DEF_INDEX})")
    ap.add_argument("--width", type=int, default=DEF_W, help=f"Chieu rong (mac dinh {DEF_W})")
    ap.add_argument("--height", type=int, default=DEF_H, help=f"Chieu cao (mac dinh {DEF_H})")
    ap.add_argument("--fps", type=float, default=2.0, help="So tam LUU moi giay (mac dinh 2)")
    ap.add_argument("--every-sec", type=float, default=None, help="Luu 1 tam moi N giay (ghi de --fps)")
    ap.add_argument("--dedup", action="store_true", help="Bo frame gan giong frame vua luu")
    ap.add_argument("--min-diff", type=float, default=0.02, help="Nguong khac biet cho --dedup (0..1)")
    ap.add_argument("--max", type=int, default=None, help="Dung sau khi luu du bay nhieu tam")
    ap.add_argument("--duration", type=float, default=None, help="Dung sau bay nhieu giay")
    ap.add_argument("--quality", type=int, default=95, help="Chat luong JPEG (mac dinh 95)")
    ap.add_argument("--show", action="store_true", help="Hien cua so xem truoc (can man hinh)")
    ap.add_argument("-o", "--out", default=None, help="Thu muc xuat (mac dinh train_data/collect_<timestamp>)")
    args = ap.parse_args()

    interval = args.every_sec if args.every_sec is not None else (1.0 / args.fps if args.fps > 0 else 0.0)

    cap = open_camera(args.index, args.width, args.height)
    if cap is None:
        print(f"[loi] mo camera index {args.index} khong duoc. "
              f"Kiem tra camera co bi tien trinh khac (bridge/patrol) giu khong.", file=sys.stderr)
        sys.exit(1)

    script_dir = Path(__file__).resolve().parent
    out_dir = Path(args.out) if args.out else script_dir / f"collect_{time.strftime('%Y%m%d_%H%M%S')}"
    out_dir.mkdir(parents=True, exist_ok=True)

    signal.signal(signal.SIGINT, _on_sigint)

    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera {args.index} @ {aw}x{ah} | luu moi {interval:.2f}s | -> {out_dir}")
    print("Bam Ctrl+C de dung.\n")

    saved = 0
    prev_gray = None
    t_start = time.time()
    t_next = t_start

    while not _stop:
        ok, frame = cap.read()
        if not ok:
            print("[canh bao] doc frame loi, thu lai...", file=sys.stderr)
            time.sleep(0.1)
            continue

        now = time.time()
        if args.show:
            cv2.imshow("collect (q = thoat)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        if now < t_next:
            continue
        t_next = now + interval

        if args.dedup:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if prev_gray is not None and frame_diff(prev_gray, gray) < args.min_diff:
                continue
            prev_gray = gray

        saved += 1
        cv2.imwrite(str(out_dir / f"cam_{saved:06d}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
        if saved % 20 == 0:
            rate = saved / (now - t_start) if now > t_start else 0
            print(f"  da luu {saved} tam ({rate:.1f}/s)")

        if args.max is not None and saved >= args.max:
            print(f"\nDu {args.max} tam, dung.")
            break
        if args.duration is not None and (now - t_start) >= args.duration:
            print(f"\nHet {args.duration:.0f}s, dung.")
            break

    cap.release()
    if args.show:
        cv2.destroyAllWindows()

    dt = time.time() - t_start
    print(f"\nXong: {saved} tam trong {dt:.0f}s -> {out_dir}")
    print(f"Zip:  cd {out_dir.parent} && zip -r {out_dir.name}.zip {out_dir.name}")


if __name__ == "__main__":
    main()
