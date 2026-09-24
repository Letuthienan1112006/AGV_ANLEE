#!/usr/bin/env python3
"""
sync_imu.py - Can chinh dong du lieu MTi-630R voi telemetry banh xe cua
xe, roi do hai con so hien dang bi nhiem: do tre lenh->chuyen dong, va do
manh lai khi co tai.

DOC TRUOC KHI DUNG (2026-09-12): con MTi da bot can thiet di nhieu, va
cho phep do DO TRE thi no khong sua duoc gi.

  - DO MANH LAI can DO PHAN GIAI toc do quay. Firmware gio da phat gyro
    tho o 0.0625 do/s (VECTOR_GYROSCOPE qua getVector), tot hon 16 lan so
    voi lay vi phan yaw Euler. Dieu nay du cho toc do quay vai do/s. Khong
    can MTi nua.
  - DO TRE la phep do THOI GIAN, va nut co that nam o phia LENH:
    cmd_steer chi ton tai trong log o moi dong telemetry. Mot IMU 95 Hz
    khong lam thoi diem lenh chinh xac hon. PRINT_INTERVAL_MS da ha
    200 -> 100 ms, cho +-50 ms, VA giu lenh voi chuyen dong tren cung mot
    dong ho - dieu ma hai thiet bi rieng bien khong bao gio co, ke ca khi
    can chinh hoan hao.

Dung file nay khi muon MOT NGUON DOC LAP de doi chieu (vi du nghi gyro
BNO055 hay encoder dang noi doi), chu khong phai vi no chinh xac hon cho
hai con so tren.

    python3 tools/sync_imu.py imu_log.csv runs_pulled/<run>/wheel_telem.csv

Hai dong du lieu khong cung goc thoi gian (mot la dong ho laptop, mot la
giay tinh tu luc run bat dau) nen khong the tru thang. Cach can chinh la
tuong quan cheo hai duong yaw: neu than xe bi xoay bang tay luc dau run,
ca hai cam bien deu thay cung mot chuyen dong, va do lech cho tuong quan
cao nhat chinh la do lech thoi gian.

Nguyen tac cua file nay: **tu choi ket luan khi du lieu khong du**. Moi
phep do deu in ra n, do phan tan va he so tuong quan. Mot con so do tre
lay tu 3 mau thi khong dung duoc - da mac loi do mot lan roi.
"""

import csv
import math
import os
import sys


RESAMPLE_HZ = 20.0
MAX_LAG_SEC = 90.0
MIN_SYNC_WINDOW_SEC = 6.0   # doan dung yen phai du dai de chua cu xoay
COARSE_STEP_SEC = 0.25

# Nguong tin cay cua buoc can chinh.
MIN_YAW_SPAN_DEG = 10.0     # phai co chuyen dong thuc de tuong quan
MIN_SYNC_R = 0.80
MIN_PEAK_MARGIN = 0.15      # dinh chinh phai hon dinh phu bao nhieu
PEAK_EXCLUDE_SEC = 1.0      # ban kinh loai quanh dinh chinh khi tim dinh phu
MIN_OVERLAP_FRAC = 0.70     # do lech lam hai doan chong nhau it hon thi loai

# Nguong tin cay cua hai phep do.
STEER_STEP_MIN = 3.0        # coi la mot buoc lenh lai
LATENCY_MIN_EVENTS = 5
LATENCY_WINDOW_SEC = 2.0
LATENCY_ONSET_FRAC = 0.25
AUTHORITY_MIN_SEGMENTS = 4
AUTHORITY_SETTLE_SEC = 1.0   # bo doan qua do dau moi doan lenh lai
AUTHORITY_USEFUL_SEC = 0.6   # phan con lai phai du dai
AUTHORITY_MIN_R = 0.50


def unwrap_deg(values):
    """Noi lien duong yaw qua cho nhay 0/360."""
    if not values:
        return []
    out = [values[0]]
    offset = 0.0
    for previous, current in zip(values, values[1:]):
        delta = current - previous
        if delta > 180.0:
            offset -= 360.0
        elif delta < -180.0:
            offset += 360.0
        out.append(current + offset)
    return out


def read_imu(path):
    """-> (t, yaw_unwrapped, yaw_rate_dps) voi t tinh tu mau dau."""
    t, yaw, wz = [], [], []
    with open(path) as handle:
        for row in csv.DictReader(handle):
            try:
                t.append(float(row["t_wall"]))
                yaw.append(float(row["yaw"]))
                wz.append(math.degrees(float(row["wz"])))
            except (KeyError, ValueError, TypeError):
                continue

    if not t:
        raise ValueError("{}: khong doc duoc mau nao".format(path))

    t0 = t[0]
    return [x - t0 for x in t], unwrap_deg(yaw), wz


def read_wheel(path):
    """-> dict cac cot can dung, t tinh tu mau dau."""
    t, yaw, steer, speed, enc = [], [], [], [], []
    with open(path) as handle:
        for row in csv.DictReader(handle):
            try:
                yaw_value = float(row["yaw"])
                t.append(float(row["t"]))
            except (KeyError, ValueError, TypeError):
                continue
            yaw.append(yaw_value)

            def get(name):
                try:
                    return float(row.get(name, "") or 0.0)
                except ValueError:
                    return 0.0

            steer.append(get("cmd_steer"))
            speed.append(get("cmd_speed"))
            enc.append(abs(get("enc_l")) + abs(get("enc_r")))

    if not t:
        raise ValueError("{}: khong doc duoc mau nao".format(path))

    t0 = t[0]
    return {
        "t": [x - t0 for x in t],
        "yaw": unwrap_deg(yaw),
        "steer": steer,
        "speed": speed,
        "enc": enc,
    }


def interp(t_src, y_src, t_query):
    """Noi suy tuyen tinh. Ngoai mien tra None thay vi keo phang - mot
    gia tri bia ra o ria se lam lech tuong quan."""
    if len(t_src) != len(y_src):
        raise ValueError(
            "interp: t_src {} mau nhung y_src {} mau".format(
                len(t_src), len(y_src)
            )
        )

    out = []
    index = 0
    last = len(t_src) - 1
    for query in t_query:
        if query < t_src[0] or query > t_src[last]:
            out.append(None)
            continue
        while index < last and t_src[index + 1] < query:
            index += 1
        t0, t1 = t_src[index], t_src[min(index + 1, last)]
        y0, y1 = y_src[index], y_src[min(index + 1, last)]
        if t1 == t0:
            out.append(y0)
        else:
            out.append(y0 + (y1 - y0) * (query - t0) / (t1 - t0))
    return out


def pearson(xs, ys):
    """Tuong quan tren cac cap ca hai ben deu co gia tri. -> (r, n)."""
    pairs = [(x, y) for x, y in zip(xs, ys) if x is not None and y is not None]
    n = len(pairs)
    if n < 10:
        return None, n

    mean_x = sum(p[0] for p in pairs) / n
    mean_y = sum(p[1] for p in pairs) / n
    sxy = sxx = syy = 0.0
    for x, y in pairs:
        dx, dy = x - mean_x, y - mean_y
        sxy += dx * dy
        sxx += dx * dx
        syy += dy * dy

    if sxx <= 0.0 or syy <= 0.0:
        return None, n
    return sxy / math.sqrt(sxx * syy), n


def span(values):
    real = [v for v in values if v is not None]
    if not real:
        return 0.0
    return max(real) - min(real)


def median(values):
    if not values:
        return None
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def quartiles(values):
    if len(values) < 4:
        return None, None
    ordered = sorted(values)
    half = len(ordered) // 2
    return median(ordered[:half]), median(ordered[-half:])


def stationary_end(wheel):
    """
    Thoi diem xe bat dau lan banh, tinh theo dong ho cua xe.

    Cua so dong bo phai nam trong doan xe con dung yen. Khi xe da chay,
    lenh lai lam yaw tang thanh mot duong doc dai, va tuong quan cua hai
    duong doc cao gan nhu nhau o moi do lech - dinh bien mat. Cu xoay tay
    dien ra luc xe dung, nen day cung la doan dung.
    """
    run = 0
    for t, enc in zip(wheel["t"], wheel["enc"]):
        if enc > 0:
            run += 1
            if run >= 3:        # 3 mau lien tiep, khong phai mot nhieu don
                return t
        else:
            run = 0
    return wheel["t"][-1]


def find_offset(imu_t, imu_yaw, wheel):
    """
    Tim do lech thoi gian: imu_t + offset == wheel_t.

    Tra ve dict co 'offset', 'r', 'margin', va 'ok' cung 'why' neu khong
    du tin cay.
    """
    wheel_t, wheel_yaw = wheel["t"], wheel["yaw"]

    step = 1.0 / RESAMPLE_HZ
    end = stationary_end(wheel)
    grid = [i * step for i in range(int(end / step) + 1)]

    if end < MIN_SYNC_WINDOW_SEC:
        return {
            "ok": False,
            "window": end,
            "why": (
                "Xe lan banh tu giay {:.1f} - doan dung yen qua ngan "
                "(can >= {:.0f}s). Cu xoay tay phai lam TRUOC khi cho xe "
                "chay, va phai ghi du lieu truoc do.".format(
                    end, MIN_SYNC_WINDOW_SEC
                )
            ),
        }

    wheel_grid = interp(wheel_t, wheel_yaw, grid)
    wheel_span = span(wheel_grid)

    if wheel_span < MIN_YAW_SPAN_DEG:
        return {
            "ok": False,
            "window": end,
            "why": (
                "Trong {:.1f}s xe dung yen, yaw chi bien thien {:.1f} do "
                "(can >= {:.0f}). Khong ai xoay than xe, nen khong co moc "
                "dong bo. Day khong phai loi du lieu - chi la quy trinh do "
                "chua duoc thuc hien.".format(
                    end, wheel_span, MIN_YAW_SPAN_DEG
                )
            ),
        }

    def correlate(offset):
        # Lay mau IMU tai thoi diem tuong ung voi moc thoi gian cua xe.
        shifted = interp(imu_t, imu_yaw, [g - offset for g in grid])
        return pearson(wheel_grid, shifted)

    # Quet tho roi tinh chinh, thay vi quet min tren ca +-90s.
    lags = []
    lag = -MAX_LAG_SEC
    while lag <= MAX_LAG_SEC:
        lags.append(lag)
        lag += COARSE_STEP_SEC

    min_overlap = max(10, int(MIN_OVERLAP_FRAC * len(grid)))

    scored = []
    for lag in lags:
        r, n = correlate(lag)
        if r is not None and n >= min_overlap:
            scored.append((abs(r), r, lag, n))

    if not scored:
        return {
            "ok": False,
            "window": end,
            "why": (
                "Khong co do lech nao giu duoc {:.0f}% cua so chong nhau "
                "({} diem). Hai lan ghi du lieu khong trung thoi gian - "
                "bat ca hai ben truoc khi bat dau.".format(
                    100.0 * MIN_OVERLAP_FRAC, min_overlap
                )
            ),
        }

    scored.sort(reverse=True)
    _, best_r, best_lag, best_n = scored[0]

    refined = (abs(best_r), best_r, best_lag, best_n)
    fine = best_lag - COARSE_STEP_SEC
    while fine <= best_lag + COARSE_STEP_SEC:
        r, n = correlate(fine)
        if r is not None and n >= min_overlap and abs(r) > refined[0]:
            refined = (abs(r), r, fine, n)
        fine += step
    _, best_r, best_lag, best_n = refined

    # Dinh phu: dinh cao nhat nam ngoai ban kinh loai.
    others = [s for s in scored if abs(s[2] - best_lag) > PEAK_EXCLUDE_SEC]
    second = others[0][0] if others else 0.0
    margin = abs(best_r) - second

    result = {
        "window": end,
        "offset": best_lag,
        "r": best_r,
        "margin": margin,
        "n": best_n,
        "wheel_span": wheel_span,
        "imu_span": span(
            interp(imu_t, imu_yaw, [g - best_lag for g in grid])
        ),
        "ok": True,
        "why": "",
    }

    problems = []
    if abs(best_r) < MIN_SYNC_R:
        problems.append(
            "tuong quan dinh chi {:+.2f} (can |r| >= {:.2f})."
            .format(best_r, MIN_SYNC_R)
        )
    if margin < MIN_PEAK_MARGIN:
        problems.append(
            "dinh chinh chi hon dinh phu {:.2f} (can >= {:.2f}) - "
            "co nhieu do lech khop gan nhu nhau, khong the chon."
            .format(margin, MIN_PEAK_MARGIN)
        )

    if problems:
        result["ok"] = False
        result["why"] = " ".join(problems)

    return result


def measure_latency(wheel, imu_t, imu_wz, offset):
    """Do tre tu buoc lenh lai den luc toc do quay bat dau doi."""
    events = []
    steer = wheel["steer"]
    times = wheel["t"]

    for i in range(1, len(steer)):
        if abs(steer[i] - steer[i - 1]) < STEER_STEP_MIN:
            continue
        if wheel["enc"][i] <= 0:
            continue        # xe dung, khong the do phan ung

        t_cmd = times[i]
        window = [
            (t + offset, rate)
            for t, rate in zip(imu_t, imu_wz)
            if 0.0 <= (t + offset) - t_cmd <= LATENCY_WINDOW_SEC
        ]
        before = [
            rate
            for t, rate in zip(imu_t, imu_wz)
            if -0.4 <= (t + offset) - t_cmd < 0.0
        ]
        if len(window) < 10 or len(before) < 3:
            continue

        baseline = sum(before) / len(before)
        settled = window[-1][1]
        change = settled - baseline
        if abs(change) < 2.0:
            continue        # phan ung qua nho, khong doc duoc thoi diem

        threshold = baseline + LATENCY_ONSET_FRAC * change
        onset = None
        for t, rate in window:
            crossed = (rate >= threshold) if change > 0 else (rate <= threshold)
            if crossed:
                onset = t - t_cmd
                break

        if onset is not None:
            events.append(onset)

    return events


def measure_authority(wheel, imu_t, imu_wz, offset):
    """Do manh lai: toc do quay (do/s) tren mot don vi lenh lai."""
    points = []
    steer = wheel["steer"]
    times = wheel["t"]

    start = 0
    for i in range(1, len(steer) + 1):
        if i < len(steer) and steer[i] == steer[start]:
            continue

        t_to = times[i - 1]
        # Bo AUTHORITY_SETTLE_SEC dau doan: trong khoang do toc do quay
        # van dang la phan ung cua lenh TRUOC, thuong trai dau.
        t_from = times[start] + AUTHORITY_SETTLE_SEC
        moving = all(
            wheel["enc"][k] > 0 for k in range(start, i)
        )
        if t_to - t_from >= AUTHORITY_USEFUL_SEC and moving:
            rates = [
                rate
                for t, rate in zip(imu_t, imu_wz)
                if t_from <= (t + offset) <= t_to
            ]
            if len(rates) >= 10:
                points.append((steer[start], sum(rates) / len(rates)))
        start = i

    if len(points) < AUTHORITY_MIN_SEGMENTS:
        return points, None, None

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    r, _ = pearson(xs, ys)

    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    sxx = sum((x - mean_x) ** 2 for x in xs)
    if sxx <= 0.0:
        return points, None, r
    slope = sum(
        (x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)
    ) / sxx

    return points, slope, r


def main():
    if len(sys.argv) < 3:
        print(__doc__.strip())
        return 2

    imu_path, wheel_path = sys.argv[1], sys.argv[2]
    for path in (imu_path, wheel_path):
        if not os.path.isfile(path):
            print("Khong thay file: {}".format(path))
            return 2

    imu_t, imu_yaw, imu_wz = read_imu(imu_path)
    wheel = read_wheel(wheel_path)

    print("=" * 68)
    print("DU LIEU VAO")
    print("=" * 68)
    print(
        "MTi   {:<34s} {:5d} mau  {:6.1f}s  {:5.1f} Hz".format(
            os.path.basename(imu_path), len(imu_t), imu_t[-1],
            len(imu_t) / imu_t[-1] if imu_t[-1] > 0 else 0.0,
        )
    )
    print(
        "Xe    {:<34s} {:5d} mau  {:6.1f}s  {:5.1f} Hz".format(
            os.path.basename(wheel_path), len(wheel["t"]), wheel["t"][-1],
            len(wheel["t"]) / wheel["t"][-1] if wheel["t"][-1] > 0 else 0.0,
        )
    )

    print()
    print("=" * 68)
    print("BUOC 1 - CAN CHINH THOI GIAN")
    print("=" * 68)

    sync = find_offset(imu_t, imu_yaw, wheel)

    if not sync.get("ok"):
        print("KHONG CAN CHINH DUOC.")
        print(sync["why"])
        print()
        print("Khong the do do tre hay do manh lai ma khong co moc dong bo.")
        print(
            "Lan chay sau, TRUOC khi cho xe chay: xoay than xe mot cu ro\n"
            "rang - vi du quay +40 do, giu 3 giay, ve -15 do, roi ve 0.\n"
            "Cu xoay khong doi xung cho do noi dinh rong hon mot cu lac\n"
            "theo nhip (0.24 so voi 0.18, nguong 0.15), nen dung cu xoay."
        )
        return 1

    print(
        "cua so      0 - {:.1f}s (doan xe dung yen truoc khi lan banh)".format(
            sync["window"]
        )
    )
    print(
        "offset      {:+.3f}s   (thoi gian MTi + offset = thoi gian xe)".format(
            sync["offset"]
        )
    )
    print("tuong quan  r = {:+.3f} tren {} diem".format(sync["r"], sync["n"]))
    print(
        "do noi dinh {:.3f} so voi dinh phu gan nhat".format(sync["margin"])
    )
    print(
        "bien thien  yaw xe {:.1f} do | yaw MTi {:.1f} do".format(
            sync["wheel_span"], sync["imu_span"]
        )
    )
    if sync["r"] < 0:
        print()
        print(
            "CANH BAO: tuong quan AM. Truc yaw cua MTi nguoc chieu voi cua\n"
            "xe - chac la gan lat. Can chinh van dung, nhung moi dau trong\n"
            "hai phep do duoi day deu bi dao. Sua bang cach gan lai cam bien\n"
            "cho dung chieu, dung dao dau trong code."
        )

    offset = sync["offset"]

    print()
    print("=" * 68)
    print("BUOC 2 - DO TRE LENH -> CHUYEN DONG")
    print("=" * 68)

    intervals = [
        b - a for a, b in zip(wheel["t"], wheel["t"][1:]) if b > a
    ]
    wheel_dt = median(intervals) or 0.0

    events = measure_latency(wheel, imu_t, imu_wz, offset)
    if len(events) < LATENCY_MIN_EVENTS:
        print(
            "Chi tim duoc {} su kien dung duoc (can >= {}). "
            "KHONG KET LUAN.".format(len(events), LATENCY_MIN_EVENTS)
        )
        print(
            "Can mot run co lenh lai doi buoc >= {:.0f} don vi nhieu lan\n"
            "trong luc banh dang quay.".format(STEER_STEP_MIN)
        )
    else:
        q1, q3 = quartiles(events)
        print("n = {} su kien".format(len(events)))
        print("trung vi  {:.0f} ms".format(1000.0 * median(events)))
        if q1 is not None:
            print(
                "khoang tu phan vi  {:.0f} - {:.0f} ms".format(
                    1000.0 * q1, 1000.0 * q3
                )
            )
        print(
            "min / max {:.0f} / {:.0f} ms".format(
                1000.0 * min(events), 1000.0 * max(events)
            )
        )
        print()
        print(
            "Lech he thong: cmd_steer chi duoc lay mau moi {:.0f} ms, nen\n"
            "thoi diem lenh doi bi ghi tre toi {:.0f} ms va con so tren bi\n"
            "THAP hon thuc te khoang {:.0f} ms. Do tre thuc nam gan\n"
            "{:.0f} ms.".format(
                1000.0 * wheel_dt, 1000.0 * wheel_dt, 500.0 * wheel_dt,
                1000.0 * (median(events) + 0.5 * wheel_dt),
            )
        )
        print("So sanh: con so ~600ms dang dung lay tu 3 mau o 200ms.")

    print()
    print("=" * 68)
    print("BUOC 3 - DO MANH LAI KHI CO TAI")
    print("=" * 68)

    points, slope, r = measure_authority(wheel, imu_t, imu_wz, offset)
    if slope is None:
        print(
            "Chi co {} doan lenh lai on dinh (can >= {}). "
            "KHONG KET LUAN.".format(len(points), AUTHORITY_MIN_SEGMENTS)
        )
    else:
        print("n = {} doan".format(len(points)))
        for steer_value, rate in sorted(points):
            print("  lenh lai {:+6.1f}  ->  {:+7.2f} do/s".format(
                steer_value, rate
            ))
        print()
        print("do doc  {:+.3f} do/s tren mot don vi lenh lai".format(slope))
        print("tuong quan r = {:+.3f}".format(r))
        if abs(r) < AUTHORITY_MIN_R:
            print(
                "r qua thap - do doc nay khong dung de chinh PID_MAX_OUTPUT."
            )
        else:
            print(
                "Day la con so thay cho 0.30 ticks/don vi lay tu run\n"
                "banh chi dat 25% target - con so do da bi nhiem."
            )

    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
