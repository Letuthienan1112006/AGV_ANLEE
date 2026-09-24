"""
trim_test.py - do STEER_TRIM dung bang cach lai HO VONG va doc goc yaw IMU.

Van de: xe cu moi lan chay la quẹo phai mot doan roi bo dieu khien moi keo
lai. Do duoc: khi lenh di thang, xe troi phai 4-12 px/giay. Nhung do KHONG
phai do banh quay lech nhau - gop 370 mau tu moi lan chay, chenh lech toc
do hai banh khi di thang chi -0.06 tick, tuc can bang.

Bo PI trong STM32 can bang SO VONG QUAY, khong phai QUANG DUONG. Nen lech
duong kinh banh, lech giong banh hay lech trong luong deu vo hinh voi no.
Chi co the bu bang mot do lech co dinh: STEER_TRIM.

Khong tinh duoc tri so nay tu du lieu chay binh thuong, vi do la vong kin -
steer chinh la phan ung cua bo dieu khien voi loi, khong tach duoc nguyen
nhan voi ket qua. Phai lai HO VONG: giu steer co dinh roi do xe quay bao
nhieu do moi giay.

  CAN MOT DOAN THANG TRONG, dai khoang 10m. Xe se chay va KHONG TU BE LAI.
  Chay qua bridge nen LiDAR van dung khi co vat can. `off` van dung duoc.

    run trim
"""
import math
import re
import socket
import sys
import time

TRIMS = [-4, -2, 0, 2]
SPEED = 16
DRIVE_SEC = 4.0
PAUSE_SEC = 2.5

HOST, PORT = "127.0.0.1", 54321
# 2026-09-12: firmware chen ",SEQ,<n>" giua MODE va IMU. Day la parser duy
# nhat trong repo doi MODE lien ngay IMU (cac file khac dung o "MODE,(\w)"),
# nen no la cho duy nhat bi vo. Nhan CA HAI dinh dang: dong cu khong co SEQ
# van doc duoc, va so nhom khong doi.
ENC_RE = re.compile(r"STEER,(-?\d+),SPEED,(-?\d+),MODE,(\w)"
                    r"(?:,SEQ,\d+)?,IMU,"
                    r"(-?[\d.]+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)")


def recv_frame(sock):
    data = bytearray()
    while len(data) < 2_000_000:
        chunk = sock.recv(65536)
        if not chunk:
            raise ConnectionError("bridge dong ket noi")
        data.extend(chunk)
        if data.rstrip().endswith(b"}"):
            return


def drive(sock, steer, speed, secs):
    end = time.time() + secs
    while time.time() < end:
        sock.sendall(("%d %d" % (steer, speed)).encode("ascii"))
        recv_frame(sock)


def run_phases():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    sock.connect((HOST, PORT))
    try:
        print("[TRIM] Cho bridge on dinh...")
        drive(sock, 0, 0, 2.0)
        for trim in TRIMS:
            print("[TRIM] steer=%+d, speed=%d, %.1fs" % (trim, SPEED, DRIVE_SEC))
            drive(sock, trim, SPEED, DRIVE_SEC)
            print("[TRIM]   dung, dat lai xe neu can")
            drive(sock, 0, 0, PAUSE_SEC)
    finally:
        try:
            drive(sock, 0, 0, 1.0)
        except Exception:
            pass
        sock.close()
    print("[TRIM] Xong phan lai.")


def unwrap(seq):
    """Goc yaw chay 0..360, noi lai cho lien tuc de tinh do quay that."""
    out = [seq[0]]
    for v in seq[1:]:
        d = v - out[-1]
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        out.append(out[-1] + d)
    return out


def analyse(path):
    """Gom theo gia tri STEER trong log - moi pha co mot gia tri rieng, nen
    khong can dau thoi gian de tach pha."""
    groups = {}
    for line in open(path, errors="ignore"):
        m = ENC_RE.search(line)
        if not m:
            continue
        steer, speed, mode = int(m.group(1)), int(m.group(2)), m.group(3)
        yaw = float(m.group(7))
        if mode != "V" or speed <= 0:
            continue
        groups.setdefault(steer, []).append(yaw)

    print("\n" + "=" * 58)
    print(" TOC DO QUAY THAT CUA XE (tu yaw IMU)")
    print("=" * 58)
    if not groups:
        print(" Khong co mau nao - xe co chay khong? LiDAR co chan khong?")
        return

    rows = []
    print(" %6s %6s | %12s | %s" % ("steer", "n", "quay tong", "toc do quay"))
    for steer in sorted(groups):
        ys = groups[steer]
        if len(ys) < 6:
            continue
        u = unwrap(ys)
        total = u[-1] - u[0]
        # moi mau ~0.5s (STM32_TELEMETRY_LOG_INTERVAL)
        rate = total / (len(ys) * 0.5)
        rows.append((steer, rate))
        print(" %+6d %6d | %+9.1f do | %+7.2f do/giay  %s" % (
            steer, len(ys), total, rate,
            "-> quay PHAI" if rate < 0 else "-> quay TRAI"))

    if len(rows) >= 2:
        n = len(rows)
        sx = sum(r[0] for r in rows); sy = sum(r[1] for r in rows)
        sxx = sum(r[0] ** 2 for r in rows); sxy = sum(r[0] * r[1] for r in rows)
        den = n * sxx - sx * sx
        if den:
            a = (n * sxy - sx * sy) / den
            b = (sy - a * sx) / n
            print("\n toc do quay = %+.3f x steer %+.3f" % (a, b))
            if a:
                print(" => STEER_TRIM nen dat = %+.1f (lam tron tu %.2f)" % (
                    round(-b / a), -b / a))
                print("    (hien tai dang la -1)")


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--analyse":
        analyse(sys.argv[2])
    else:
        run_phases()
        if len(sys.argv) > 1:
            analyse(sys.argv[1])
