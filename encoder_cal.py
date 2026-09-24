"""
encoder_cal.py - do CHUAN XUNG/VONG cua tung banh, bang cach quay tay.

CHI DOC UART. Khong bao gio ghi mot byte nao ra /dev/ttyACM0, nen khong
the lam banh quay. Day la ly do khong dung wheel_calibration.py hay
steer_authority.py cho viec nay: ca hai deu LAI dong co.

    python3 encoder_cal.py [giay] [so_vong]        # mac dinh 60 giay, 10 vong

CACH CAP NGUON - khac voi bai "quay tay cam nhan do nang":
    MCU + encoder   PHAI CO DIEN   (khong co dien thi khong dem duoc)
    Nguon cong suat motor   NGAT RIENG
Neu hai duong khong tach duoc thi DUNG lam bai nay bang tay: banh co the
bi driver keo bat ngo.

CACH LAM trong cua so thoi gian:
  1. Quay banh TRAI dung <so_vong> vong, MOT CHIEU da quy uoc, roi dung.
  2. Nghi vai giay.
  3. Quay banh PHAI dung <so_vong> vong, CUNG MOT CHIEU, roi dung.

Dung quay qua lai: bo dem co dau (++/-- trong ISR firmware), nen qua lai
se triet tieu va so xung thu duoc nho hon thuc te. Script bao so lan doi
chieu de biet dieu do da xay ra.

KHONG gia dinh hai ben cung so xung moi vong. Dieu do chi dung neu
encoder, ti so truyen va cach dem (x1/x2/x4) giong nhau ca hai ben. Muc
dich cua bai nay chinh la do rieng tung ben.
"""

import re
import sys
import time

try:
    import serial
except ImportError:
    sys.stderr.write("Thieu pyserial: pip3 install pyserial\n")
    raise SystemExit(1)


PORT = "/dev/ttyACM0"
BAUD = 115200

# Hai so ngay sau "ENC," la bo dem TICH LUY, khong phai DELTA.
ENC_RE = re.compile(r"ENC,(-?\d+),(-?\d+),")

# Mot buoc coi la "dang quay" khi vuot nguong nay, de nhieu 1 xung khong
# bi tinh la chuyen dong.
MOVE_THRESHOLD = 2


def collect(seconds):
    """Doc UART trong khoang thoi gian cho truoc. Tra [(t, cum_l, cum_r)]."""
    samples = []
    with serial.Serial(PORT, BAUD, timeout=1.0) as ser:
        print("[CAL] Doc %s (CHI DOC, khong gui gi). %0.0f giay."
              % (PORT, seconds), flush=True)
        t0 = time.monotonic()
        last_print = 0.0
        while time.monotonic() - t0 < seconds:
            raw = ser.readline()
            if not raw:
                continue
            match = ENC_RE.search(raw.decode("ascii", errors="ignore"))
            if not match:
                continue
            t = time.monotonic() - t0
            left, right = int(match.group(1)), int(match.group(2))
            samples.append((t, left, right))
            if t - last_print >= 1.0:
                last_print = t
                print("  %5.1fs  trai %8d   phai %8d" % (t, left, right),
                      flush=True)
    return samples


def analyse(samples, side, name, revolutions):
    """Tim doan banh nay quay, va bao lai nhung gi do duoc."""
    counts = [s[side] for s in samples]
    times = [s[0] for s in samples]

    steps = [(b - a) for a, b in zip(counts, counts[1:])]
    moving = [i for i, d in enumerate(steps) if abs(d) >= MOVE_THRESHOLD]

    print("\n--- banh %s ---" % name)
    if not moving:
        print("  KHONG phat hien chuyen dong (nguong %d xung/mau)."
              % MOVE_THRESHOLD)
        print("  Vang mat la vang mat: khong ket luan gi ve ben nay.")
        return None

    first, last = moving[0], moving[-1] + 1
    total = counts[last] - counts[first]
    path = sum(abs(d) for d in steps[first:last])

    signs = [1 if d > 0 else -1 for d in steps[first:last]
             if abs(d) >= MOVE_THRESHOLD]
    reversals = sum(1 for a, b in zip(signs, signs[1:]) if a != b)

    print("  quay tu %.1fs den %.1fs" % (times[first], times[last]))
    print("  bo dem: %d -> %d" % (counts[first], counts[last]))
    print("  thay doi RONG (co dau): %+d xung" % total)
    print("  duong di (tong tri tuyet doi): %d xung" % path)

    if reversals:
        print("  !! %d lan DOI CHIEU - bai nay doi quay MOT CHIEU." % reversals)
        print("     Bo dem co dau nen qua lai triet tieu nhau. Lam lai.")
        return None

    if path != abs(total):
        print("  !! duong di (%d) khac tri tuyet doi cua rong (%d)"
              % (path, abs(total)))
        print("     nghia la co chuyen dong nguoc khong bi bat - lam lai.")
        return None

    per_rev = abs(total) / float(revolutions)
    print("  => %.1f xung moi vong  (%d xung / %d vong)"
          % (per_rev, abs(total), revolutions))
    return per_rev


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    revolutions = int(sys.argv[2]) if len(sys.argv) > 2 else 10

    print(__doc__.split("CACH LAM")[0].strip())
    print("\n=== BAT DAU: quay banh TRAI %d vong, nghi, roi banh PHAI %d vong"
          % (revolutions, revolutions))

    samples = collect(seconds)
    print("\n[CAL] Thu duoc %d mau." % len(samples))
    if len(samples) < 5:
        print("[CAL] Qua it mau. Firmware co dang in telemetry khong?")
        print("      Kiem: co tien trinh nao khac dang giu %s?" % PORT)
        return 1

    left = analyse(samples, 1, "TRAI", revolutions)
    right = analyse(samples, 2, "PHAI", revolutions)

    print("\n=== KET QUA ===")
    if left is None or right is None:
        print("Chua du de ket luan - xem canh bao o tren.")
        return 1

    print("  trai %.1f xung/vong   phai %.1f xung/vong" % (left, right))
    ratio = max(left, right) / min(left, right) if min(left, right) > 0 else 0
    print("  ty le %.3f" % ratio)
    if ratio > 1.05:
        print("\n  >> HAI BEN LECH %.1f%%." % (100 * (ratio - 1)))
        print("     Neu encoder, ti so truyen va cach dem GIONG nhau hai ben")
        print("     thi day la mot phat hien: mot ben dem sai. Neu phan cung")
        print("     hai ben KHAC nhau thi day chi la hang so cua tung ben -")
        print("     va gio da do duoc, dung cho phep doi chieu video.")
    else:
        print("\n  >> Hai ben khop trong %.1f%%." % (100 * (ratio - 1)))
        print("     Encoder dem nhat quan. Dung hai con so nay lam chuan cho")
        print("     phep doi chieu so vong tu video.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
