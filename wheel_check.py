"""wheel_check.py - banh xe co lam dung lenh khong, va neu khong thi vi sao.

    python3 wheel_check.py runs_pulled/<lan_chay>

Doc wheel_telem.csv (bridge ghi, mot dong moi khung telemetry STM32) va doi
chieu BON muc theo cung moc thoi gian:

    lenh AI gui  ->  toc do muc tieu hai banh  ->  encoder thuc te  ->  yaw

Ton tai vi ba cau hoi khac nhau tung bi gop thanh mot cau "xe khong be lai":
  1. AI va firmware ghi nhan lenh gi?          (khong dong bo theo lenh)
  2. Firmware co dat duoc muc tieu no tu dat?  (enc vs tgt, tung banh)
  3. Chenh lech banh co thanh chuyen dong quay? (delta vs toc do yaw)
Moi cau tra loi khac nhau dan tot mot huong sua khac nhau; tra loi sai cau
la sua sai cho. 2026-09-11: mot lan chay cho thay lenh be trai da den
firmware, muc tieu hai banh da lech dung chieu, nhung encoder banh phai hut
muc tieu 9 tick - do la cau 2, khong phai cau 1 hay 3.
"""
import csv
import glob
import io
import json
import math
import os
import re
import statistics as st
import sys


def read_run_mode(d):
    """Read operator-declared test conditions; never infer a stand from yaw.

    None means unknown, not false. Legacy names/banners are declarations too,
    not proof that an operator actually lifted the wheels.
    """
    result = dict(mode="unknown", declared_on_stand=None, dry_run=None,
                  lidar_bypassed=None, vision_backend=None, source="unknown")
    path = os.path.join(d, "run_mode.json")
    try:
        with open(path, encoding="utf-8") as f:
            info = json.load(f)
        if (isinstance(info, dict) and info.get("schema_version") == 1
                and info.get("mode") in ("road", "lane", "bench", "dry",
                                         "snap", "trim", "sweep", "wcal",
                                         "straight", "lidar")):
            result.update(mode=info["mode"], source="run_mode.json")
            for key in ("declared_on_stand", "dry_run", "lidar_bypassed"):
                value = info.get(key)
                result[key] = value if isinstance(value, bool) else None
            backend = info.get("vision_backend")
            if backend in ("yolo26_unified", "legacy_two_model"):
                result["vision_backend"] = backend
            return result
    except (OSError, ValueError):
        pass

    try:
        with open(os.path.join(d, "bridge.log"), encoding="utf-8",
                  errors="replace") as f:
            head = f.read(12000)
    except OSError:
        head = ""
    suffix = os.path.basename(os.path.abspath(d)).rsplit("_", 1)[-1]
    if suffix in ("road", "bench", "dry", "snap", "trim", "sweep"):
        result.update(mode=suffix, source="legacy directory name")
        if suffix in ("bench", "sweep"):
            result["declared_on_stand"] = True
        if suffix == "dry":
            result["dry_run"] = True
    if "CHE DO BENCH" in head:
        result.update(declared_on_stand=True, source="legacy bridge banner")
        if result["mode"] == "unknown":
            result["mode"] = "bench"
    if "DRY RUN:" in head:
        result.update(mode="dry", dry_run=True, source="legacy bridge banner")
    return result


FIRMWARE_SRC = "motor_test_bts7960/src/main.cpp"

FIRMWARE_CONSTANTS = (
    ("BASE_PWM", r"const int BASE_PWM\s*=\s*(\d+)"),
    ("PWM_MAX", r"const int PWM_MAX\s*=\s*(\d+)"),
    ("KP", r"const float KP\s*=\s*([0-9.]+)f"),
    ("KI", r"const float KI\s*=\s*([0-9.]+)f"),
    ("INTEGRAL_LIMIT", r"const float INTEGRAL_LIMIT\s*=\s*([0-9.]+)f"),
    ("TREF", r"TARGET_TICKS_AT_REFERENCE\s*=\s*(\d+)"),
    ("RIGHT_TRIM", r"const float RIGHT_WHEEL_PWM_TRIM\s*=\s*([0-9.]+)f"),
    ("MAX_TARGET", r"MAX_WHEEL_TARGET_TICKS\s*=\s*(\d+)"),
)


def read_firmware_gains(root=None):
    """
    Doc hang so vong PI tu ma nguon firmware, thay vi ghi lai o day.

    Ghi lai la cach dead-config sinh ra: se co hai ban roi chung lech nhau
    ma khong ai biet. Tra None khi khong doc duoc - vang mat phai la vang
    mat, khong duoc thay bang gia tri doan.

    CANH BAO: day la ban trong repo. No KHONG phai bang chung ve firmware
    dang nap tren xe. Trong repo co hai cay firmware va cay dang chay la
    cay co ten gay nham.
    """
    if root is None:
        root = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(root, FIRMWARE_SRC)
    try:
        with io.open(path, encoding="utf-8", errors="replace") as handle:
            src = handle.read()
    except (IOError, OSError):
        return None

    gains = {}
    for name, pattern in FIRMWARE_CONSTANTS:
        match = re.search(pattern, src)
        if not match:
            return None
        gains[name] = float(match.group(1))

    if gains["TREF"] <= 0:
        return None
    gains["path"] = path
    return gains


def pi_ceiling(gains, target, error):
    """
    PWM cao nhat vong PI co the phat ra cho mot muc tieu va sai so.

    Khong phai PWM_MAX. Khau tich phan bi chan o KI*INTEGRAL_LIMIT, nen
    voi target nho thi tran thuc te thap hon PWM_MAX rat nhieu:
    doc PWM_MAX=140 roi ket luan "con du dia" la sai.
    """
    if target <= 0:
        return 0.0
    feed_forward = gains["BASE_PWM"] * target / gains["TREF"]
    integral = gains["KI"] * gains["INTEGRAL_LIMIT"]
    return min(
        gains["PWM_MAX"],
        feed_forward + gains["KP"] * error + integral,
    )


def finite(value):
    return value is not None and math.isfinite(value)


# Mot buoc nhay phai lon hon nhieu lan muc tieu toi da moi duoc coi la bat
# thuong: PI co the vuot, va quay banh bang tay cung tao delta lon.
JUMP_FACTOR = 10.0


def pulse_audit(rows, max_target=None):
    """
    Kiem toan bo dem encoder. KHONG phu thuoc vao viec firmware co nhan
    lenh chay hay khong: bai quay tay voi nguon cong suat da ngat thi
    khong co lenh nao, va do la dieu binh thuong.

    Cac sua, tat ca deu la loi that da tai hien duoc bang du lieu gia:

    1. Bien khoang: count_cuoi - count_dau phai so voi TONG DELTA TU DONG
       THU HAI. Delta dong dau mo ta khoang TRUOC moc bat dau phep tru.
       10 -> 20 -> 30 voi delta 10/10/10 khong mat gi, nhung cong ca ba ra
       30 so voi 20 va bao lech 50%.

    2. "Bo dem giam = MCU reset" LA SAI. Encoder co dau (++/-- trong ISR),
       quay banh nguoc chieu lam no giam binh thuong. Reset chi duoc
       khang dinh khi SEQ nhay LUI.

    3. Dem mat ban tin phai CHIA DOAN tai moi lan reset. SEQ 10, 13, 0, 1
       tung cho expected = -8 va lost = 0, trong khi da mat 11 va 12.

    4. SEQ trung (4, 4, 5) KHONG phai reset. Bao rieng: co the la dong
       lap, chua chung minh MCU khoi dong lai.

    5. Dem doi chieu phai so dau cac BUOC KHAC 0 LIEN TIEP, tung banh.
       So dau tung buoc voi thay doi RONG bo sot 0 -> 10 -> 0, vi rong
       bang 0 nen khong buoc nao "trai dau".

    6. Buoc vat qua moc reset khong duoc cong vao duong di - no la mot
       buoc nhay cua bo dem, khong phai chuyen dong.

    GIOI HAN khong the go: duong di la tong |count_sau - count_truoc|
    giua CAC MAU. Neu banh quay toi roi lui trong mot khoang khong duoc
    lay mau, chuyen dong do triet tieu va khong xuat hien o day. Vi vay
    bai do chuan xung/vong phai quay MOT CHIEU.
    """
    series = []
    for r in rows:
        left, right = r.get("count_l"), r.get("count_r")
        if left in (None, "") or right in (None, ""):
            continue
        try:
            seq = r.get("seq")
            series.append((float(left), float(right),
                           None if seq in (None, "") else int(float(seq))))
        except (TypeError, ValueError):
            continue

    if len(series) < 2:
        return {"ok": False,
                "why": "khong co cot count_l/count_r (run truoc 2026-09-12)"}

    limit = JUMP_FACTOR * (max_target if max_target else 70.0)
    have_seq = all(x[2] is not None for x in series)

    # Chia doan tai moi lan SEQ nhay LUI. Khong co SEQ thi mot doan duy
    # nhat, va bao rang khong khang dinh duoc gi.
    bounds = [0]
    duplicates = 0
    if have_seq:
        for i in range(1, len(series)):
            previous, current = series[i - 1][2], series[i][2]
            if current < previous:
                bounds.append(i)
            elif current == previous:
                duplicates += 1
    bounds.append(len(series))

    segments = []
    for start, end in zip(bounds, bounds[1:]):
        chunk = series[start:end]
        if len(chunk) < 2:
            # Mot mau khong lam nen mot khoang; van ghi nhan de dem doan.
            segments.append({"n": len(chunk), "net": (0.0, 0.0),
                             "path": (0.0, 0.0), "reversals": 0,
                             "steps": ([], []), "jumps": [],
                             "seq": None})
            continue

        net = (chunk[-1][0] - chunk[0][0], chunk[-1][1] - chunk[0][1])
        steps_l, steps_r, jumps = [], [], []
        for (l0, r0, _), (l1, r1, _) in zip(chunk, chunk[1:]):
            dl, dr = l1 - l0, r1 - r0
            steps_l.append(dl)
            steps_r.append(dr)
            if abs(dl) > limit or abs(dr) > limit:
                jumps.append((dl, dr))

        def reversals(steps):
            """So lan doi dau giua cac buoc KHAC 0 lien tiep."""
            signs = [1 if x > 0 else -1 for x in steps if x != 0]
            return sum(1 for a, b in zip(signs, signs[1:]) if a != b)

        seg_seq = None
        if have_seq:
            ids = [x[2] for x in chunk]
            unique = len(set(ids))
            expected = ids[-1] - ids[0] + 1
            seg_seq = {"first": ids[0], "last": ids[-1],
                       "have": unique, "expected": expected,
                       "lost": max(0, expected - unique)}

        segments.append({
            "n": len(chunk),
            "net": net,
            "path": (sum(abs(x) for x in steps_l),
                     sum(abs(x) for x in steps_r)),
            "reversals": reversals(steps_l) + reversals(steps_r),
            "steps": (steps_l, steps_r),
            "jumps": jumps,
            "seq": seg_seq,
        })

    usable = [g for g in segments if g["n"] >= 2]
    return {
        "ok": True,
        "n": len(series),
        "segments": segments,
        "restarts": len(bounds) - 2,
        "duplicates": duplicates,
        "have_seq": have_seq,
        "limit": limit,
        # Duong di cong don qua cac doan, KHONG gom buoc vat qua moc reset.
        "path": (sum(g["path"][0] for g in usable),
                 sum(g["path"][1] for g in usable)),
        "reversals": sum(g["reversals"] for g in usable),
        "lost": (sum(g["seq"]["lost"] for g in usable if g["seq"])
                 if have_seq else None),
        "jumps": [j for g in usable for j in g["jumps"]],
    }


def load(d):
    fs = glob.glob(os.path.join(d, "wheel_telem.csv"))
    if not fs:
        return []
    rows = []
    required = ("t", "enc_l", "enc_r", "tgt_l", "tgt_r", "pwm_l", "pwm_r",
                "fw_steer", "fw_speed")
    with open(fs[0]) as f:
        for r in csv.DictReader(f):
            try:
                row = {k: (float(r[k]) if r.get(k) not in ("", None) else None)
                       for k in required + ("cmd_steer", "cmd_speed", "yaw",
                                            "count_l", "count_r", "seq")}
            except (ValueError, KeyError):
                continue
            if not all(finite(row[k]) for k in required):
                continue
            # count_l/count_r chi co tu 2026-09-12. Cac run cu khong co,
            # va do phai la vang mat, khong phai 0.
            for key in ("cmd_steer", "cmd_speed", "yaw",
                        "count_l", "count_r", "seq"):
                if not finite(row[key]):
                    row[key] = None
            row["mode"] = r.get("mode", "")
            rows.append(row)
    return rows


def yaw_rate(rows):
    """do/s, da thao goi vong 0-360."""
    out = []
    for a, b in zip(rows, rows[1:]):
        if a["yaw"] is None or b["yaw"] is None:
            out.append(None)
            continue
        dt = b["t"] - a["t"]
        if dt <= 0:
            out.append(None)
            continue
        d = b["yaw"] - a["yaw"]
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        out.append(d / dt)
    if rows:
        out.append(None)
    return out


def corr(xs, ys):
    pairs = [(x, y) for x, y in zip(xs, ys) if finite(x) and finite(y)]
    if len(pairs) < 8:
        return None
    xa = [p[0] for p in pairs]
    ya = [p[1] for p in pairs]
    if len(set(xa)) < 2 or len(set(ya)) < 2:
        return None
    mx, my = st.mean(xa), st.mean(ya)
    num = sum((x - mx) * (y - my) for x, y in pairs)
    dx = math.sqrt(sum((x - mx) ** 2 for x in xa))
    dy = math.sqrt(sum((y - my) ** 2 for y in ya))
    return num / (dx * dy) if dx and dy else None


def slope(xs, ys):
    pairs = [(x, y) for x, y in zip(xs, ys) if finite(x) and finite(y)]
    if len(pairs) < 8:
        return None
    xa = [p[0] for p in pairs]
    mx, my = st.mean(xa), st.mean([p[1] for p in pairs])
    den = sum((x - mx) ** 2 for x in xa)
    if not den:
        return None
    return sum((x - mx) * (y - my) for x, y in pairs) / den


def report_pulses(rows, gains):
    """
    Muc [2c]. Chay DOC LAP voi viec firmware co nhan lenh chay hay khong:
    bai quay tay de do chuan xung/vong co nguon cong suat motor da ngat,
    nen khong co lenh nao va do la dieu binh thuong. Truoc day muc nay
    nam sau mot lenh `return` som va khong bao gio chay trong bai do.
    """
    print("\n[2c] BO DEM ENCODER (de doi chieu voi so vong dem tu video)")
    audit = pulse_audit(rows, gains.get("MAX_TARGET") if gains else None)

    if not audit["ok"]:
        print("     Khong dung duoc: %s" % audit["why"])
        print("     Cong don cot enc_l/enc_r KHONG thay the duoc: firmware tinh")
        print("     DELTA moi CONTROL_INTERVAL_MS va truoc 2026-09-12 chi in moi")
        print("     200ms, nen mot nua cac khoang dem chua bao gio duoc xuat ra.")
        return

    print("     %d mau co bo dem." % audit["n"])

    if not audit["have_seq"]:
        print("     Khong co cot seq (firmware truoc 2026-09-12): KHONG khang")
        print("     dinh duoc co mat ban tin hay MCU da reset. Ca hai chi la")
        print("     nghi van, va so lieu duoi day duoc coi la MOT doan lien tuc.")
    else:
        print("     SEQ: %d lan nhay LUI (MCU reset), %d so trung."
              % (audit["restarts"], audit["duplicates"]))
        if audit["duplicates"]:
            print("       So trung KHONG phai reset - co the la dong lap.")
        if audit["lost"]:
            print("       MAT %d ban tin, tinh RIENG tung doan giua cac lan"
                  % audit["lost"])
            print("       reset (cong ca khoang thi ra so vo nghia).")
        elif audit["restarts"] == 0:
            print("       Khong mat ban tin, khong reset.")

    usable = [g for g in audit["segments"] if g["n"] >= 2]
    if len(usable) > 1:
        print("     Chia thanh %d doan tai cac lan reset; thay doi RONG chi co"
              % len(usable))
        print("     nghia TRONG mot doan, khong vat qua moc reset:")
        for i, seg in enumerate(usable, 1):
            extra = ""
            if seg["seq"]:
                extra = "  SEQ %d..%d, mat %d" % (
                    seg["seq"]["first"], seg["seq"]["last"], seg["seq"]["lost"])
            print("       doan %d (%d mau): rong trai %+.0f phai %+.0f%s"
                  % (i, seg["n"], seg["net"][0], seg["net"][1], extra))
    elif usable:
        net_l, net_r = usable[0]["net"]
        print("     Thay doi RONG (co dau, am = quay nguoc):")
        print("       trai %+.0f   phai %+.0f" % (net_l, net_r))

    path_l, path_r = audit["path"]
    print("     Duong di (tong tri tuyet doi tung buoc, bo buoc vat qua reset):")
    print("       trai %.0f   phai %.0f" % (path_l, path_r))
    if audit["reversals"]:
        print("       %d lan doi dau giua cac buoc khac 0 - co doi chieu."
              % audit["reversals"])
        print("       Doi chieu voi video thi phai dung DUNG dai luong: rong")
        print("       neu dem vong co dau, duong di neu dem tong vong.")

    # Bo delta dong dau: no mo ta khoang TRUOC moc bat dau phep tru.
    sum_l = sum(abs(r["enc_l"]) for r in rows[1:])
    sum_r = sum(abs(r["enc_r"]) for r in rows[1:])
    print("     Cong don DELTA (bo dong dau, cho khop bien khoang):")
    print("       trai %.0f   phai %.0f" % (sum_l, sum_r))
    for name, path, ssum in (("trai", path_l, sum_l), ("phai", path_r, sum_r)):
        if path <= 0:
            continue
        gap = 100.0 * (ssum - path) / path
        if abs(gap) < 5.0:
            print("       %-5s khop trong %.1f%%" % (name, abs(gap)))
        else:
            print("       %-5s lech %+.1f%% - HAI PHEP TINH KHONG KHOP."
                  % (name, gap))
            print("             Can kiem tra khoang do, nhip lay mau va mat")
            print("             dong. Lech mot minh KHONG chung minh mat ban tin.")

    for name, seg in (("trai", 0), ("phai", 1)):
        steps = [x for g in usable for x in g["steps"][seg]]
        if steps:
            ordered = sorted(steps)
            print("     buoc %-5s: min %+.0f  trung vi %+.0f  max %+.0f"
                  % (name, ordered[0], ordered[len(ordered) // 2], ordered[-1]))

    if audit["jumps"]:
        print("     %d buoc vuot %.0f xung/mau - BAT KHA ve vat ly."
              % (len(audit["jumps"]), audit["limit"]))
    print("     Gioi han cua nguong tren: chi bat buoc nhay GROSS. Mot lan")
    print("     reset tu bo dem nho (120 -> 0) nam trong khoang binh thuong")
    print("     va khong bat duoc bang do lon - chi SEQ bat duoc.")

    print("     Duong di chi ghi nhan chuyen dong GIUA CAC MAU: banh quay toi")
    print("     roi lui trong mot khoang khong duoc lay mau thi triet tieu va")
    print("     khong xuat hien o day. Vi vay bai chuan xung/vong phai quay")
    print("     MOT CHIEU da quy uoc.")
    print("     Va KHONG gia dinh hai ben cung so xung moi vong: dieu do chi")
    print("     dung neu encoder, ti so truyen va cach dem (x1/x2/x4) giong")
    print("     nhau. Xac dinh chuan cua TUNG ben truoc.")


def main(d):
    rows = load(d)
    print("=" * 66)
    print(" DAP UNG BANH XE: %s" % os.path.basename(d.rstrip("/")))
    print("=" * 66)
    if not rows:
        print("\n Khong co wheel_telem.csv trong thu muc nay.")
        print(" File nay chi co tu 2026-09-11 tro di, va chi o che do co mo")
        print(" UART (run / run bench) - 'run dry' khong mo UART nen khong co.")
        return

    # Include stalled wheels: filtering on encoder motion would discard the
    # very command/response failures this report needs to expose.
    drive = [r for r in rows if r["mode"] == "V" and r["fw_speed"] != 0]
    activity = [r for r in rows if abs(r["enc_l"]) + abs(r["enc_r"]) > 4]
    context = read_run_mode(d)
    on_stand = context["declared_on_stand"] is True
    dry = context["dry_run"] is True
    print("\n %d mau, %.1fs. Firmware co lenh chay: %d mau; encoder hoat dong: %d mau."
          % (len(rows), rows[-1]["t"] - rows[0]["t"], len(drive), len(activity)))
    print(" Nguong hoat dong encoder: |DELTA_L| + |DELTA_R| > 4 ticks/mau.")
    print(" Khai bao: %s (%s); khong phai phat hien tu the xe."
          % (context["mode"], context["source"]))
    if context["vision_backend"] is not None:
        print(" Vision backend: %s" % context["vision_backend"])
    if on_stand:
        print(" XE TREN GIA theo khai bao: chi danh gia dap ung banh, khong cham bam duong.")
    elif context["declared_on_stand"] is None:
        print(" Chua biet xe tren gia hay tren dat; yaw it/khong doi KHONG xac dinh duoc.")
    if dry:
        print(" DRY RUN duoc khai bao: khong dung file nay de xac nhan chay xe that.")
    print(" Encoder hoat dong khong tu chung minh than xe da di chuyen.")
    if not drive:
        # Bai quay tay de do chuan xung/vong co nguon cong suat motor da
        # ngat, nen KHONG co lenh chay - va do la dieu binh thuong. Truoc
        # day lenh return o day chan luon muc [2c], la muc duy nhat bai do
        # can. [2c] khong phu thuoc vao lenh motor, nen chay no roi moi ve.
        print(" Khong co mau firmware nhan lenh chay -> chua danh gia duoc")
        print(" dap ung lenh. Muc [2c] duoi day khong phu thuoc vao dieu do.")
        report_pulses(rows, read_firmware_gains())
        return
    if not activity:
        print(" CO LENH CHAY NHUNG KHONG CO HOAT DONG ENCODER trong cac mau ghi lai.")

    # cmd_* is the latest AI snapshot at receipt, not a synchronized record of
    # the actual post-safety UART write. Differences are descriptive only.
    print("\n[1] LENH AI VA LENH FIRMWARE GHI NHAN")
    ds = [r["fw_steer"] - r["cmd_steer"] for r in drive
          if r["cmd_steer"] is not None]
    if ds:
        print("     fw_steer - cmd_steer cung dong: median %+.1f"
              % st.median(ds))
        bad = sum(1 for v in ds if abs(v) > 1.5)
        print("     lech qua 1.5 don vi  : %d/%d mau" % (bad, len(ds)))
    print("     Khong dong bo theo lenh: do tre/lenh safety co the tao sai khac.")
    print("     Khong ket luan mat lenh hay truyen nguyen ven tu phep so sanh nay.")

    # --- 2. Firmware co dat duoc muc tieu cua chinh no ---
    print("\n[2] BANH CO DAT DUOC MUC TIEU FIRMWARE TU DAT KHONG")
    print("     (am = banh quay CHAM hon muc tieu)")
    if on_stand:
        print("     Tren gia: sai so bam toc do VAN co y nghia o tai thu nay;")
        print("     khong suy ra sai so se giong nhu vay khi chay tren duong.")
    print("     Thong ke gom ca qua do va banh dung; khong chi la trang thai on dinh.")
    for name, ek, tk, pk in (("TRAI", "enc_l", "tgt_l", "pwm_l"),
                             ("PHAI", "enc_r", "tgt_r", "pwm_r")):
        err = [abs(r[ek]) - r[tk] for r in drive]
        print("     banh %-4s: %+.1f tick (median)   PWM trung binh %.0f"
              % (name, st.median(err), st.mean([r[pk] for r in drive])))
    # tach theo chieu lai: banh TRONG va banh NGOAI chiu tai khac nhau
    for tag, sel in (("be TRAI (fw_steer<-2)", lambda r: r["fw_steer"] < -2),
                     ("di THANG (|fw_steer|<=2)",
                      lambda r: abs(r["fw_steer"]) <= 2),
                     ("be PHAI (fw_steer>2)", lambda r: r["fw_steer"] > 2)):
        sub = [r for r in drive if sel(r)]
        if len(sub) < 4:
            continue
        el = st.median([abs(r["enc_l"]) - r["tgt_l"] for r in sub])
        er = st.median([abs(r["enc_r"]) - r["tgt_r"] for r in sub])
        print("       %-26s n=%3d  trai %+5.1f   phai %+5.1f"
              % (tag, len(sub), el, er))

    # --- 2b. Vong PI con du dia khong ---
    #
    # Hai lan doc sai truoc do, ghi lai de khong lap:
    #
    # 1. Doc PWM 83 canh PWM_MAX=140 roi ket luan "con 40% du dia". Sai:
    #    tran thuc te la feedforward + KP*err + KI*INTEGRAL_LIMIT, va
    #    KI*INTEGRAL_LIMIT chi dang 9 nac PWM.
    # 2. Roi tinh tran bang KP*target thay vi KP*error, va so PWM trong log
    #    voi tran do ma KHONG go RIGHT_WHEEL_PWM_TRIM ra. Firmware gan
    #    rightPWM = rightPWM * TRIM TRUOC khi printTelemetry() chay, nen
    #    PWM_R trong log la gia tri SAU trim. Dau hieu cua sai nay la du
    #    dia AM: PWM log 83 vuot tran 81, dieu bat kha voi mot gia tri
    #    tien-trim. Bay gio go trim ra truoc khi so.
    #
    # Va mot gioi han khong the go duoc: integral KHONG duoc ghi vao
    # telemetry. Du dia o day la SUY RA tu cho hai con so khop nhau, khong
    # phai do duoc. Ket luan "loi khong nam o dieu khien" duoi day dua vao
    # HIEU SUAT tick/PWM, thu chi can so lieu trong log.
    print("\n[2b] VONG PI CON DU DIA KHONG (tran cau truc, khong phai PWM_MAX)")
    gains = read_firmware_gains()
    if gains is None:
        print("     Khong doc duoc hang so tu %s - khong ket luan." % FIRMWARE_SRC)
    else:
        print("     Hang so tu %s (ban trong REPO, khong phai bang chung"
              % FIRMWARE_SRC)
        print("     ve firmware dang nap tren xe):")
        print("       BASE_PWM=%.0f TREF=%.0f KP=%.2f KI=%.3f ILIM=%.0f PWM_MAX=%.0f"
              % (gains["BASE_PWM"], gains["TREF"], gains["KP"], gains["KI"],
                 gains["INTEGRAL_LIMIT"], gains["PWM_MAX"]))
        print("       => khau tich phan gop toi da %.1f nac PWM"
              % (gains["KI"] * gains["INTEGRAL_LIMIT"]))
        trim = gains["RIGHT_TRIM"]
        print("       RIGHT_WHEEL_PWM_TRIM=%.2f - PWM_R trong log la SAU trim,"
              % trim)
        print("       nen duoc chia lai truoc khi so voi tran.")
        for name, ek, tk, pk, div in (("TRAI", "enc_l", "tgt_l", "pwm_l", 1.0),
                                      ("PHAI", "enc_r", "tgt_r", "pwm_r", trim)):
            pts = [r for r in drive if r[tk] > 0]
            if len(pts) < 4:
                print("     banh %-4s: chi %d mau co muc tieu > 0 - khong ket luan."
                      % (name, len(pts)))
                continue
            heads, ceils, pwms = [], [], []
            for r in pts:
                ceiling = pi_ceiling(gains, r[tk], r[tk] - abs(r[ek]))
                pi_out = r[pk] / div
                ceils.append(ceiling)
                pwms.append(pi_out)
                heads.append(ceiling - pi_out)
            head = st.median(heads)
            span = gains["KI"] * gains["INTEGRAL_LIMIT"]
            used = 100.0 * max(0.0, min(1.0, (span - head) / span)) if span else 0.0
            print("     banh %-4s: dau ra PI %.1f / tran %.1f  (du dia %+.1f "
                  "cua %.0f nac tich phan, ~%.0f%% da dung)"
                  % (name, st.median(pwms), st.median(ceils), head, span, used))
            if head < -1.0:
                print("       du dia AM - mot khau nao chua duoc tinh den. "
                      "Khong ket luan tu con so nay.")

        # TY SO DAP UNG encoder/PWM trong lan chay nay. KHONG goi la
        # "hieu suat" - day khong phai mot dai luong vat ly doc lap:
        #   - vong kin tu tang PWM VI encoder cham, nen hai dai luong co
        #     quan he phan hoi, khong phai nhan/qua
        #   - hai banh chiu target va tai khac nhau moi khi xe cua
        # Dieu no noi duoc, va noi chac: trong CUNG mot lan chay, banh nao
        # duoc yeu cau PWM cao hon ma encoder tra ve it hon.
        eff = {}
        for name, ek, pk in (("TRAI", "enc_l", "pwm_l"),
                             ("PHAI", "enc_r", "pwm_r")):
            pts = [r for r in drive if r[pk] > 0]
            total_pwm = sum(r[pk] for r in pts)
            if len(pts) >= 4 and total_pwm > 0:
                eff[name] = sum(abs(r[ek]) for r in pts) / total_pwm
        if len(eff) == 2:
            print("     Ty so dap ung encoder/PWM trong lan chay nay:")
            print("       trai %.3f / phai %.3f tick moi nac PWM"
                  % (eff["TRAI"], eff["PHAI"]))
            lo_name = min(eff, key=lambda k: eff[k])
            hi_name = max(eff, key=lambda k: eff[k])
            if eff[lo_name] > 0:
                print("       banh %s thap hon banh %s %.1f lan."
                      % (lo_name, hi_name, eff[hi_name] / eff[lo_name]))
            print("     Chi dung so lieu trong log; khong suy ra gi ve integral,")
            print("     va KHONG phai hieu suat vat ly (vong kin tu tang PWM vi")
            print("     encoder cham, nen hai dai luong co quan he phan hoi).")
            print("     Ket luan chac nhat: trong cung lan chay, banh %s duoc"
                  % lo_name)
            print("     yeu cau PWM cao hon nhung encoder tra ve it hon dang ke,")
            print("     nen loi nhieu kha nang nam SAU tang tao lenh: co khi,")
            print("     day/dau noi, BTS7960, nguon tai kenh do, hoac encoder")
            print("     khi co tai.")
            print("     CHUA loai tru: encoder do sai, hay PWM khong thuc su den")
            print("     motor. Phan biet re nhat: TAT NGUON, quay tay tung banh.")

            # Dau yaw lam gia thuyet "chi encoder dem thieu" KEM THUYET
            # PHUC hon, nhung khong loai tru duoc no. Toi da tung viet la
            # "loai tru"; do la mot buoc suy luan tu dap ung khong hop le:
            # PI day them PWM chi bao dam LENH tang, khong bao dam banh
            # quay nhanh hon - va ca cuoc dieu tra nay la ve viec PWM
            # KHONG chuyen thanh toc do o banh phai. Khong the vua noi
            # "PWM khong thanh toc do" vua noi "them PWM se nhanh hon".
            print("     Ve dau yaw: neu encoder mot ben dem THIEU ma banh van")
            print("     quay binh thuong, PI day them PWM - nhung do chi la")
            print("     LENH tang, chua bao dam banh quay nhanh hon. Muon suy")
            print("     ra chieu quay con can: muc tieu hai banh phu hop, bo")
            print("     dieu khien du kha nang bu, da qua giai doan khoi dong,")
            print("     va chieu yaw duoc xac nhan - trong khi run nay dang co")
            print("     sai so bam toc do lon. Ket luan dung muc:")
            print("       'Chieu quay quan sat duoc CHUA UNG HO gia thuyet")
            print("        encoder mot ben dem thieu la nguyen nhan duy nhat.'")
            print("     Phep kiem TRUC TIEP: dan bang dinh len banh, quay phim,")
            print("     dem so VONG thuc te va so voi tong xung encoder cung")
            print("     khoang thoi gian - xem muc [2c]. So sanh tung ben voi")
            print("     chuan xung/vong CUA CHINH BEN DO, dung gia dinh hai ben")
            print("     bang nhau: dieu do chi dung neu encoder, ti so truyen va")
            print("     cach dem (x1/x2/x4) giong nhau.")

    report_pulses(rows, gains)

    # --- 3. Chenh lech banh co thanh chuyen dong quay ---
    print("\n[3] CHENH LECH BANH CO THANH CHUYEN DONG QUAY KHONG")
    diff = [abs(r["enc_r"]) - abs(r["enc_l"]) for r in drive]
    stv = [r["fw_steer"] for r in drive]
    k = slope(stv, diff)
    if k is not None:
        print("     do doc (|enc_phai|-|enc_trai|) / steer: %+.2f tick" % k)
        print("     Quy uoc hien tai: steer duong -> trai nhanh hon -> do doc AM.")
        print("     Chi so sanh cac lan cung tai, toc do va kich ban lenh; khong phai diem dat/rot.")
    yr = yaw_rate(rows)
    idx = {id(r): i for i, r in enumerate(rows)}
    mv_yaw = [yr[idx[id(r)]] for r in drive]
    c = None if on_stand or dry else corr(stv, mv_yaw)
    if on_stand or dry:
        print("     yaw: khong danh gia dap ung quay tren duong voi che do da khai bao.")
    elif c is None:
        print("     yaw: thieu mau hop le hoac steer/yaw khong bien thien de tinh tuong quan.")
        print("     Yaw khong doi/bi thieu KHONG chung minh xe dang ke tren gia.")
    else:
        print("     tuong quan steer <-> toc do yaw : %+.2f" % c)
        ky = slope(stv, mv_yaw)
        if ky is not None:
            print("     toc do yaw / don vi steer      : %+.2f do/s" % ky)
        print("     Mo ta tuong quan, KHONG tu ket luan dung/sai chieu hay bam duong tot;")
        print("     can kiem chung chieu truc IMU, do tre va dieu kien dat xe.")

    # --- 4. Do tre ---
    print("\n[4] DO TRE TU LENH DEN CHUYEN DONG")
    gaps = [b["t"] - a["t"] for a, b in zip(rows, rows[1:])
            if b["t"] > a["t"]]
    if on_stand or dry or not gaps:
        print("     Khong uoc luong do tre quay tren duong: che do thu/mau khong phu hop.")
        print()
        return
    dt = st.median(gaps)
    best = None
    for lag in range(min(13, len(rows))):
        # Exclude transitions through STOP rather than correlating parked
        # samples or a command before STOP with a later independent restart.
        indices = [i for i in range(len(rows) - lag)
                   if all(r["mode"] == "V" and r["fw_speed"] != 0
                          for r in rows[i:i + lag + 2])]
        xs = [rows[i]["fw_steer"] for i in indices]
        ys = [yr[i + lag] for i in indices]
        c = corr(xs, ys)
        if c is not None and (best is None or abs(c) > abs(best[1])):
            best = (lag, c)
    if best is None or abs(best[1]) < 0.3:
        print("     Khong du bien thien/tuong quan de uoc luong do tre.")
    else:
        print("     tuong quan cao nhat o do tre %d mau = %.0f ms (r=%+.2f)"
              % (best[0], best[0] * dt * 1000, best[1]))
        print("     nhip mau telemetry: %.0f ms" % (dt * 1000))
        print("     Chi la do dich mau co tuong quan cao nhat, KHONG phai do tre da xac nhan.")
        print("     Sai phan yaw nam giua hai mau; nhip khong deu/chu ky lenh co the lam sai uoc luong.")
    print()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1])
