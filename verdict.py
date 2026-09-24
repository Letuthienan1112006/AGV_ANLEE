"""
verdict.py - cham diem mot lan chay: xe co bam vach duoc khong, va neu
khong thi hong o TANG NAO.

    python3 verdict.py runs_pulled/20260910_014718_road

Doc CA HAI nguon, vi moi nguon tra loi mot cau hoi khac nhau:
  - patrol_telem_*.csv : AI NHIN thay gi va RA LENH gi
  - wheel_telem.csv / bridge.log ENC : encoder banh xe ghi nhan gi

Cau hoi then chot, theo thu tu tang:
  1. NHAN DIEN  - co thay vach khong?            (ty le LINE/HOLD)
  2. DIEU KHIEN - co ra lenh dung chieu khong?   (dau cua steer vs raw_err)
  3. CO KHI     - banh co quay khac nhau khong?  (chenh lech DELTA)

Tang 3 la tang da am tham hong suot nhieu tuan: 2026-09-10 do duoc tren
duong, be lai het co ma hai banh quay chenh nhau -0.1 tick, tuc xe di
thang bang trong khi bo dieu khien dang be het suc.
"""
import csv
import glob
import io
import os
import re
import statistics as st
import sys

from wheel_check import load as load_wheel_telem, read_run_mode, slope

def steer_ceiling(d=None):
    """Doc PID_MAX_OUTPUT thang tu Confg.py.

    2026-09-11: khong import Confg duoc vi no keo theo cv2, ma laptop khong
    co. Quan trong hon, truoc day file nay HARD-CODE nguong 20/19 - khi tran
    lai ha xuong 8 thi steer khong bao gio cham 19 nua, nen tang 3 lang le
    ngung do dung cai no sinh ra de do, va bao "khong co mau nao be lai het
    co" nhu the do la binh thuong. Dung ho loi "config chet" da gap hai lan
    truoc, lan nay nam trong chinh cong cu cham diem.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    cands = []
    if d:
        # Ban config CUA CHINH LAN CHAY DO truoc tien - config hien tai co
        # the da doi nhieu lan ke tu luc chay.
        cands.append(os.path.join(d, "Confg_snapshot.py"))
    cands.append(os.path.join(here, "Confg.py"))
    for c in cands:
        try:
            with io.open(c, encoding="utf-8") as f:
                src = f.read()
        except IOError:
            continue
        m = re.search(r"^PID_MAX_OUTPUT\s*=\s*(\d+)", src, re.M)
        if m:
            return int(m.group(1))
    return 20


ENC_RE = re.compile(
    r"ENC,(-?\d+),(-?\d+),DELTA,(-?\d+),(-?\d+),TARGET,(-?\d+),(-?\d+),"
    r"PWM,(-?\d+),(-?\d+),STEER,(-?\d+),SPEED,(-?\d+),MODE,(\w)"
)


def load_telem(d):
    fs = glob.glob(os.path.join(d, "patrol_telem_*.csv"))
    if not fs:
        return []
    with open(fs[0]) as f:
        return list(csv.DictReader(f))


def is_bench(d):
    """Compatibility helper: declared stand mode, not physical detection."""
    return read_run_mode(d)["declared_on_stand"] is True


def block_reason(d):
    """Xe co bi bridge chan khong, va vi sao.

    2026-09-11: mot lan chay 398 khung / 54.5s trong do LiDAR chan tu dau
    den cuoi. Banh KHONG QUAY mot vong nao, nhung file nay van cham tang 2
    va in "KHONG KEO LAI DUOC" - doc len tuong nhu bo dieu khien hong, trong
    khi that ra no chua bao gio duoc thu. Xe khong di chuyen thi loi khong
    the giam: phai noi thang ra thay vi de nguoi doc tu suy.
    """
    p = os.path.join(d, "bridge.log")
    if not os.path.exists(p):
        return None
    reasons = {}
    try:
        with io.open(p, encoding="utf-8", errors="replace") as f:
            for ln in f:
                m = re.search(r"LIDAR=STOP reason=(.+)", ln)
                if m:
                    reason = m.group(1).strip()
                    reasons[reason] = reasons.get(reason, 0) + 1
    except IOError:
        return None
    if not reasons:
        return None
    return max(reasons.items(), key=lambda kv: kv[1])[0]


def load_enc(d):
    # Prefer every timestamped firmware sample over rate-limited log output.
    wheel = load_wheel_telem(d)
    if wheel:
        return [dict(dl=abs(r["enc_l"]), dr=abs(r["enc_r"]),
                     tl=r["tgt_l"], tr=r["tgt_r"], pl=r["pwm_l"], pr=r["pwm_r"],
                     st=r["fw_steer"], sp=r["fw_speed"], mode=r["mode"])
                for r in wheel]
    p = os.path.join(d, "bridge.log")
    if not os.path.exists(p):
        return []
    out = []
    with open(p, errors="ignore") as f:
        for ln in f:
            m = ENC_RE.search(ln)
            if m:
                g = m.groups()
                out.append(dict(
                    dl=abs(int(g[2])), dr=abs(int(g[3])), tl=int(g[4]), tr=int(g[5]),
                    pl=int(g[6]), pr=int(g[7]), st=int(g[8]), sp=int(g[9]),
                    mode=g[10],
                ))
    return out


def main(d=None):
    d = d if d is not None else (sys.argv[1] if len(sys.argv) > 1 else ".")
    rows, enc = load_telem(d), load_enc(d)
    context = read_run_mode(d)
    on_stand = context["declared_on_stand"] is True
    dry = context["dry_run"] is True
    print("=" * 66)
    print(" CHAM DIEM: %s" % os.path.basename(os.path.abspath(d)))
    print("=" * 66)
    print(" Khai bao: %s (%s), khong phai xac nhan tu the xe."
          % (context["mode"], context["source"]))
    if context["vision_backend"] is not None:
        print(" Vision backend: %s" % context["vision_backend"])
    if context["declared_on_stand"] is None:
        print(" Chua ro xe tren gia hay tren dat; khong tu suy ra tu encoder/yaw.")
    if dry:
        print(" DRY RUN: khong dung ket qua nay de xac nhan xe chay that.")

    # ---- Tang 1: nhan dien ----
    print("\n[1] NHAN DIEN")
    if not rows:
        print("    khong co telemetry CSV")
    else:
        n = len(rows)
        t = float(rows[-1]["t"])
        line = sum(1 for r in rows if r["src"] == "LINE")
        # fps trung vi, khong phai n/t: khung dau tien co the mat hang chuc
        # giay vi cuDNN khoi tao (do duoc 25.4s ngay 2026-09-10), va mot
        # khung nhu vay du keo con so trung binh tu 3.5 xuong 1.70 fps.
        ts = [float(r["t"]) for r in rows]
        gaps = sorted(ts[i] - ts[i - 1] for i in range(1, len(ts)))
        med = gaps[len(gaps) // 2] if gaps else 0
        print("    %d khung / %.1fs" % (n, t))
        if med > 0:
            print("    nhip trung vi: %.0f ms = %.2f fps   (trung binh tho %.2f fps)"
                  % (med * 1000, 1.0 / med, n / t if t > 0 else 0))
            if max(gaps) > 3 * med:
                print("    khoang khung cham nhat: %.1fs (can log de xac dinh nguyen nhan)"
                      % max(gaps))
        print("    LINE %.0f%%   KHAC %.0f%%" % (
            100.0 * line / n, 100.0 * (n - line) / n))
        print("    LINE la nhan do thuat toan tu bao, khong phai do chinh xac nhan dien.")

    # Commands, wheel activity and body displacement are different evidence.
    drive = [r for r in enc if r["mode"] == "V" and r["sp"] != 0]
    activity = [r for r in enc if r["dl"] + r["dr"] > 4]
    print("\n    Firmware co lenh chay: %d mau; encoder hoat dong: %d/%d mau."
          % (len(drive), len(activity), len(enc)))
    print("    Nguong hoat dong encoder: |DELTA_L| + |DELTA_R| > 4 ticks/mau.")
    print("    Encoder quay KHONG tu chung minh than xe di chuyen hay bam vach.")
    if enc and not activity:
        why = block_reason(d)
        print("\n" + "!" * 66)
        print("  KHONG CO HOAT DONG ENCODER DANG KE TRONG CAC MAU GHI LAI.")
        if drive:
            print("  CO LENH CHAY: can kiem tra dap ung motor/encoder; khong coi la chua thu.")
        else:
            print("  Khong co mau firmware nhan lenh chay; chua danh gia duoc dap ung lenh.")
        if why:
            print("  Bridge co ghi LiDAR STOP, ly do pho bien: %s" % why)
            print("  Dong log nay khong chung minh vat can ton tai suot ca lan chay.")
        print("!" * 66)

    # ---- Tang 2: dieu khien ----
    no_road_evidence = on_stand or dry or not activity
    label = "   (KHAI BAO TREN GIA)" if on_stand else (
        "   (DRY RUN)" if dry else "")
    print("\n[2] DIEU KHIEN%s" % label)
    if rows:
        # PID uses filtered error plus derivative/integral: a sign difference
        # against raw error at one instant is not proof of reversed steering.
        pairs = [(float(r["raw_err"]), int(r["steer"]))
                 for r in rows if r["src"] == "LINE" and abs(float(r["raw_err"])) > 20]
        if pairs:
            dung = sum(1 for e, s in pairs if e * s > 0)
            print("    steer cung dau voi raw_err tren %d/%d khung (%.0f%%)" % (
                dung, len(pairs), 100.0 * dung / len(pairs)))
            print("    Chi mo ta dau; can xet loc loi, PID va do tre truoc khi ket luan dao chieu.")
        cap = steer_ceiling(d)
        sat = [r for r in rows if abs(int(r["steer"])) >= cap]
        print("    steer bao hoa +-%d tren %d/%d khung (%.0f%%)" % (
            cap, len(sat), len(rows), 100.0 * len(sat) / len(rows)))
        # loi co giam khi steer bao hoa khong - cau hoi quan trong nhat
        if len(sat) >= 4:
            a, b = abs(float(sat[0]["raw_err"])), abs(float(sat[-1]["raw_err"]))
            print("    |loi| o mau bao hoa dau/cuoi: %.0f -> %.0f" % (a, b))
            print("    Hai mau co the thuoc cac doan khac nhau: khong tu ket luan keo lai duoc/khong.")
            if no_road_evidence:
                print("    Che do thu/encoder chua cung cap bang chung bam vach tren duong.")

    # ---- Tang 3: co khi ----
    print("\n[3] DAP UNG BANH XE (khong tu xac nhan tai hay chuyen dong than xe)")
    if not enc:
        print("    khong co telemetry encoder hop le")
    else:
        cap = steer_ceiling(d)
        run = drive
        sat = [r for r in run if abs(r["st"]) >= cap - 1]
        straight = [r for r in run if r["st"] == 0]
        print("    %d mau firmware co lenh chay (bao gom banh dung neu co)" % len(run))
        if run:
            print("    Sai so encoder - target, median: trai %+.1f; phai %+.1f ticks"
                  % (st.median(r["dl"] - r["tl"] for r in run),
                     st.median(r["dr"] - r["tr"] for r in run)))
            print("    Sai so nay co y nghia o dieu kien thu; gom ca qua do, khong chi on dinh.")
        if straight:
            print("    steer=0    : chenh lech DELTA %+.1f ticks (gom ca qua do)"
                  % st.mean([r["dr"] - r["dl"] for r in straight]))
        if sat:
            diff = st.mean([abs(r["dr"] - r["dl"]) for r in sat])
            pwm = st.mean([abs(r["pr"] - r["pl"]) for r in sat])
            spd = st.mean([(r["dl"] + r["dr"]) / 2.0 for r in sat])
            lock = st.mean([abs(r["st"]) for r in sat])
            per = diff / lock if lock else 0.0
            print("    gan moc lai AI +-%d (|steer| TB %.0f): chenh lech DELTA %.1f ticks"
                  " (PWM chenh %.0f nac, banh quay %.0f ticks)"
                  % (cap, lock, diff, pwm, spd))
            print("    ty so do lon |DELTA_R-DELTA_L| / |steer|: %.2f" % per)
            print("    Day khong phai do doc hoi quy hay nguong dat/rot.")
            outer = max(max(r["pl"], r["pr"]) for r in sat)
            print("    PWM lon nhat trong nhom mau: %d" % outer)
            print("    PWM nay chua xac dinh duoc gioi han dien/co khi hay tran firmware da nap.")
        else:
            print("    khong co mau gan moc lai AI da luu")
        k = slope([r["st"] for r in run], [r["dr"] - r["dl"] for r in run])
        if k is not None:
            print("    do doc (|enc_phai|-|enc_trai|) / steer: %+.2f ticks" % k)
            print("    Dau ky vong AM voi quy uoc steer duong -> trai nhanh hon.")
        print("    Khong so sanh ket luan co tai voi tren gia; can cung toc do va kich ban lenh.")
    print()


if __name__ == "__main__":
    main()
