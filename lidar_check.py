"""
lidar_check.py - xem LiDAR dang thay gi, theo tung huong.

Dung khi bridge bao "LIDAR STOP" ma mat thuong khong thay vat can nao,
hoac de kiem tra FRONT_CENTER_DEG co dung la huong di toi hay khong.

    python3 lidar_check.py          # quet 5 giay
    python3 lidar_check.py 10       # quet 10 giay

In ra khoang cach GAN NHAT theo tung cung 15 do, danh dau nón truoc dang
dung de quyet dinh dung/di. Neu cung gan nhat KHONG nam trong nón truoc
ma xe van bi chan, thi nón dat sai huong.
"""
import sys
import time
from collections import defaultdict

from rplidar import RPLidar

from Confg import LIDAR_STOP_DIST_M, LIDAR_RESUME_DIST_M

# 2026-09-12: truoc day file nay tu khai lai PORT, FRONT_CENTER_DEG va
# FRONT_CONE_DEG kem chu thich "khop lidar_safety.py". Chung da lech:
# non truoc duoc tinh la CENTER +- CONE/2 (115..145 do) trong khi
# analyze_scan trong lidar_safety dung CENTER +- CONE (100..160 do). Tuc
# bai chan doan ve mot non RONG MOT NUA cai ma lop an toan thuc su dung,
# nen mot vat o 105 do se chan xe ma khong hien trong "non truoc" - dung
# cai ket luan sai "non dat sai huong" ma file nay sinh ra de tranh.
# Gio import truc tiep, khong con hai ban de lech.
from lidar_safety import (
    FRONT_CENTER_DEG,
    FRONT_CONE_DEG,
    MAX_VALID_DIST_MM,
    MIN_QUALITY,
    MIN_VALID_DIST_MM,
    PORT,
)

BUCKET = 15


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    stop_mm = LIDAR_STOP_DIST_M * 1000
    resume_mm = LIDAR_RESUME_DIST_M * 1000
    lo = FRONT_CENTER_DEG - FRONT_CONE_DEG
    hi = FRONT_CENTER_DEG + FRONT_CONE_DEG

    print("Nguong: STOP <= %.0fmm | RESUME >= %.0fmm" % (stop_mm, resume_mm))
    print("Non truoc: %.0f..%.0f do (tam %.0f)\n" % (lo, hi, FRONT_CENTER_DEG))

    lidar = RPLidar(PORT)
    nearest = defaultdict(lambda: 1e9)
    counts = defaultdict(int)
    front = []
    t0 = time.time()
    try:
        for scan in lidar.iter_scans(max_buf_meas=2500):
            for quality, angle, dist in scan:
                if quality < MIN_QUALITY:
                    continue
                if not (MIN_VALID_DIST_MM <= dist <= MAX_VALID_DIST_MM):
                    continue
                b = int(angle // BUCKET) * BUCKET
                nearest[b] = min(nearest[b], dist)
                counts[b] += 1
                if lo <= angle <= hi:
                    front.append(dist)
            if time.time() - t0 > secs:
                break
    finally:
        lidar.stop()
        lidar.disconnect()

    print("%-12s %10s %8s  %s" % ("cung (do)", "gan nhat", "so diem", ""))
    overall = None
    for b in sorted(nearest):
        d = nearest[b]
        mark = ""
        if lo <= b + BUCKET / 2 <= hi or lo <= b <= hi:
            mark += " <== NON TRUOC"
        if d <= stop_mm:
            mark += "  [duoi nguong STOP]"
        if overall is None or d < overall[1]:
            overall = (b, d)
        print("%4d-%-7d %8.0fmm %8d %s" % (b, b + BUCKET, d, counts[b], mark))

    print()
    if front:
        front.sort()
        print("NON TRUOC: %d diem, gan nhat %.0fmm, trung vi %.0fmm" % (
            len(front), front[0], front[len(front) // 2]))
        print("  -> %s" % ("BI CHAN (<= %.0fmm)" % stop_mm if front[0] <= stop_mm
                           else ("vung hysteresis, chua du de di" if front[0] < resume_mm
                                 else "du thoang de di")))
    else:
        print("NON TRUOC: KHONG CO DIEM NAO.")
        print("  Hai kha nang, va cot 'so diem' o tren phan biet duoc:")
        print("  - ca vong cung it diem  -> cam bien/cap/nguon USB co van de")
        print("  - ca vong nhieu diem    -> phia truoc that su khong co gi")
        print("  Luu y: o trang thai nay lidar_safety KHONG BAO GIO chuyen")
        print("  sang GO, vi distance=None khong tinh vao path_clear.")
    if overall:
        print("Vat gan nhat toan vong: cung %d do, %.0fmm" % overall)


if __name__ == "__main__":
    main()
