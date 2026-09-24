"""
steer_authority.py - do LUC LAI THUC TE cua xe o tung muc SPEED.

Cau hoi can tra loi: vi sao be lai het co (steer=-20) ma xe khong quay?
Gia thuyet (2026-09-10): firmware quy doi speed -> vong banh theo
SPEED_REFERENCE=90, nhung bridge kep SPEED_MAX=25, nen baseTarget chi
dat 9/70 ticks. Chenh lech PWM giua hai banh khi be het co chi con ~5%,
ngang co voi do lech co khi cua xe -> lenh lai bi lech co khi lan at.

Script nay noi THANG voi STM32 qua /dev/ttyACM0, KHONG qua bridge, vi
bridge kep SPEED_MAX=25 nen khong the test tren muc do.

  XE PHAI DUOC KE LEN GIA. Banh se quay.

Chay tren Jetson:
    python3 steer_authority.py            # quet mac dinh
    python3 steer_authority.py 21 40 60   # chi test cac muc nay

Doc ket qua: cot "chenh DELTA" la chenh lech toc do that giua hai banh.
Neu no tang manh theo SPEED thi gia thuyet dung, va con so do cho biet
can chay o toc do nao thi lai moi du luc.
"""
import re
import sys
import time

import serial

PORT = "/dev/ttyACM0"
BAUD = 115200

SEND_INTERVAL = 0.1     # khop CONTROL_INTERVAL_MS, va < TIMEOUT_MS=500
SETTLE_SEC = 1.5        # bo qua luc banh dang tang toc
MEASURE_SEC = 2.0       # thoi gian lay mau

DEFAULT_SPEEDS = [21, 30, 40, 60]
STEERS = [0, -20, 20]

ENC_RE = re.compile(
    r"ENC,(-?\d+),(-?\d+),DELTA,(-?\d+),(-?\d+),"
    r"TARGET,(-?\d+),(-?\d+),PWM,(-?\d+),(-?\d+),"
    r"STEER,(-?\d+),SPEED,(-?\d+)"
)


def drive(ser, steer, speed, seconds, collect=False):
    """Gui lenh lien tuc trong `seconds`. Tra ve cac mau ENC neu collect."""
    samples = []
    deadline = time.time() + seconds
    next_send = 0.0
    while time.time() < deadline:
        now = time.time()
        if now >= next_send:
            ser.write(("%d %d\n" % (steer, speed)).encode("ascii"))
            next_send = now + SEND_INTERVAL
        line = ser.readline().decode("ascii", "replace").strip()
        if collect and line:
            m = ENC_RE.search(line)
            if m:
                g = [int(x) for x in m.groups()]
                samples.append({
                    "dl": g[2], "dr": g[3],
                    "tl": g[4], "tr": g[5],
                    "pl": g[6], "pr": g[7],
                })
    return samples


def mean(xs):
    return sum(xs) / float(len(xs)) if xs else 0.0


def main():
    speeds = [int(x) for x in sys.argv[1:]] or DEFAULT_SPEEDS

    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    time.sleep(2.0)          # STM32 reset khi mo cong serial
    ser.reset_input_buffer()

    print("XE PHAI DANG KE TREN GIA - banh se quay.\n")
    print("%5s %6s | %13s | %13s | %13s | %s" % (
        "speed", "steer", "TARGET t/p", "PWM t/p", "DELTA t/p", "chenh DELTA"))
    print("-" * 82)

    results = {}
    try:
        for speed in speeds:
            for steer in STEERS:
                drive(ser, steer, speed, SETTLE_SEC)
                s = drive(ser, steer, speed, MEASURE_SEC, collect=True)
                drive(ser, 0, 0, 0.6)      # dung giua cac buoc
                if not s:
                    print("%5d %6d | (khong doc duoc ENC nao)" % (speed, steer))
                    continue
                dl, dr = mean([x["dl"] for x in s]), mean([x["dr"] for x in s])
                pl, pr = mean([x["pl"] for x in s]), mean([x["pr"] for x in s])
                tl, tr = mean([x["tl"] for x in s]), mean([x["tr"] for x in s])
                print("%5d %6d | %6.0f/%-6.0f | %6.0f/%-6.0f | %6.1f/%-6.1f | %+.1f" % (
                    speed, steer, tl, tr, pl, pr, dl, dr, dr - dl))
                results[(speed, steer)] = (dr - dl, pr - pl)
            print("-" * 82)
    finally:
        for _ in range(6):
            ser.write(b"0 0\n")
            time.sleep(0.05)
        ser.close()

    print("\nTOM TAT - luc lai (chenh lech toc do banh khi be het co):")
    print("%5s | %12s | %12s | %s" % (
        "speed", "steer=-20", "steer=+20", "so voi speed thap nhat"))
    base = None
    for speed in speeds:
        l = results.get((speed, -20), (0, 0))[0]
        r = results.get((speed, 20), (0, 0))[0]
        mag = (abs(l) + abs(r)) / 2.0
        if base is None:
            base = mag if mag else 1.0
        print("%5d | %12.1f | %12.1f | %.2fx" % (speed, l, r, mag / base))


if __name__ == "__main__":
    main()
