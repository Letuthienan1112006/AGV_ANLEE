"""
lidar_safety.py - Lop an toan LiDAR cho AGV.

- FRONT_CENTER_DEG=130.0 da duoc xac nhan bang log thuc te.
- Hysteresis STOP/GO dung cac nguong trong Confg.py.
- Scan loi, scan qua it diem hoac timeout deu ep STOP.
- Tat ca state deu duoc bao ve bang RLock de dung an toan giua cac thread.
"""

import math
import threading
import time

from rplidar import RPLidar

from Confg import (
    LIDAR_CLEAR_CONFIRM_FRAMES,
    LIDAR_MAX_VALID_DIST_M,
    LIDAR_RESUME_DIST_M,
    LIDAR_STOP_CONFIRM_FRAMES,
    LIDAR_STOP_DIST_M,
)


PORT = "/dev/ttyUSB0"

FRONT_CENTER_DEG = 130.0
FRONT_CONE_DEG = 30.0

# Cone 2 ben de danh gia huong ne vat (dung boi ne_avoidance trong
# real_car_socket.py, khong anh huong logic STOP/GO phia truoc).
SIDE_CONE_MIN_DEG = 40.0
SIDE_CONE_MAX_DEG = 100.0

MIN_QUALITY = 5
MIN_VALID_DIST_MM = 100.0
MAX_VALID_DIST_MM = LIDAR_MAX_VALID_DIST_M * 1000.0
MIN_VALID_SCAN_POINTS = 20
SCAN_TIMEOUT_SEC = 1.0

STOP_DIST_MM = LIDAR_STOP_DIST_M * 1000.0
RESUME_DIST_MM = LIDAR_RESUME_DIST_M * 1000.0


def normalize_angle(angle):
    """Dua goc ve mien [-180, 180]."""
    result = angle % 360.0
    if result > 180.0:
        result -= 360.0
    return result


def analyze_scan(scan):
    """
    Tra ve (scan_valid, closest_point, valid_point_count, front_raw_count).

    closest_point la (distance_mm, relative_angle_deg, quality), hoac None
    neu scan hop le nhung khong co diem phan xa trong cone phia truoc.

    front_raw_count dem SO TIA RAW roi vao cone truoc, TRUOC khi loc theo
    quality/khoang cach - khac han front_points (sau loc). 2026-09-15: day
    la du lieu can de phan biet "cone thuc su khong co gi" (front_raw=0,
    khong mot tia nao cham vao huong do) voi "co vat nhung phan hoi bi loc"
    (front_raw>0 nhung bi loai vi quality/khoang cach) - hai tinh huong
    truoc day gop chung thanh "cone rong". Xem cach dung trong update().
    """
    valid_point_count = 0
    front_points = []
    front_raw_count = 0

    for quality, raw_angle, distance in scan:
        relative_angle = normalize_angle(raw_angle - FRONT_CENTER_DEG)
        in_front = -FRONT_CONE_DEG <= relative_angle <= FRONT_CONE_DEG
        if in_front:
            front_raw_count += 1

        if quality < MIN_QUALITY:
            continue
        if not math.isfinite(distance):
            continue
        if not (MIN_VALID_DIST_MM <= distance <= MAX_VALID_DIST_MM):
            continue

        valid_point_count += 1

        if in_front:
            front_points.append((distance, relative_angle, quality))

    if valid_point_count < MIN_VALID_SCAN_POINTS:
        return False, None, valid_point_count, front_raw_count

    if not front_points:
        return True, None, valid_point_count, front_raw_count

    return (True, min(front_points, key=lambda point: point[0]),
            valid_point_count, front_raw_count)


def closest_front_point(scan):
    """Tra diem hop le gan nhat trong cone phia truoc, hoac None."""
    scan_valid, point, _, _ = analyze_scan(scan)
    if not scan_valid:
        return None
    return point


def analyze_side_clearance(scan):
    """Tra (left_min_mm, right_min_mm) - khoang cach gan nhat ben trai/
    phai (cone SIDE_CONE_MIN_DEG..SIDE_CONE_MAX_DEG tuong doi so voi
    FRONT_CENTER_DEG). None = khong co diem nao trong cone do: KHONG BIET,
    khong duoc coi la bang chung huong nay dang thoang.
    Dung cho tinh nang ne vat can, khong dung cho logic STOP/GO chinh."""
    left_dists = []
    right_dists = []

    for quality, raw_angle, distance in scan:
        if quality < MIN_QUALITY:
            continue
        if not math.isfinite(distance):
            continue
        if not (MIN_VALID_DIST_MM <= distance <= MAX_VALID_DIST_MM):
            continue

        relative_angle = normalize_angle(raw_angle - FRONT_CENTER_DEG)

        if SIDE_CONE_MIN_DEG <= relative_angle <= SIDE_CONE_MAX_DEG:
            left_dists.append(distance)
        elif -SIDE_CONE_MAX_DEG <= relative_angle <= -SIDE_CONE_MIN_DEG:
            right_dists.append(distance)

    left_min = min(left_dists) if left_dists else None
    right_min = min(right_dists) if right_dists else None
    return left_min, right_min


class LidarSafety:
    def __init__(self):
        self._lock = threading.RLock()

        self.state = "STOP"
        self.reason = "startup"
        self.stop_confirm = 0
        self.clear_confirm = 0

        self.last_valid_scan_time = None
        self.last_dist_mm = None
        self.last_angle = None
        self.last_valid_point_count = 0
        self.last_front_raw = 0
        self.last_scan_valid = False
        self.left_clear_mm = None
        self.right_clear_mm = None

        # Cong tac bo qua LiDAR. Dat o DAY, mot cho duy nhat, thay vi rai
        # dieu kien khap real_car_socket.py - lam vay chac chan se sot mot
        # duong nao do, va duong bi sot trong mot lop an toan la duong nguy
        # hiem nhat.
        #
        # 2026-09-11: can no vi lab qua chat. Nguong bench la STOP 0.20m /
        # RESUME 0.35m, ma do duoc non truoc cua xe dang ke tren gia chi con
        # 237mm - ket giua hai nguong, nen da dung la khong bao gio nha, va
        # khong test duoc dap ung banh du banh dang treo trong khong khi.
        #
        # Chi duoc bat khi xe KE TREN GIA: real_car_socket.py tu choi khoi
        # dong neu co --no-lidar ma khong co AGV_BENCH.
        self._bypassed = False

    def bypass(self, reason):
        """Ghim GO va lam moi duong khac vo hieu. CHI dung khi xe ke tren
        gia - banh khong cham dat thi LiDAR khong bao ve duoc gi."""
        with self._lock:
            self._bypassed = True
            self.state = "GO"
            self.reason = reason
            print(f"[LIDAR-SAFETY] BO QUA LIDAR ({reason})", flush=True)
            return self.state

    def clear_bypass(self):
        """Tra lai quyen dung cho lop an toan.

        Can khi shutdown: bridge goi force_stop("bridge shutdown"), ma dang
        bypass thi lenh do se bi lam vo hieu - dung y do cua bypass, nhung
        sai y do cua shutdown. Go bypass truoc roi moi dung."""
        with self._lock:
            self._bypassed = False
            return self.state

    def force_stop(self, reason):
        with self._lock:
            if self._bypassed:
                return "GO"

            changed = self.state != "STOP" or self.reason != reason

            self.state = "STOP"
            self.reason = reason
            self.stop_confirm = LIDAR_STOP_CONFIRM_FRAMES
            self.clear_confirm = 0
            # A forced STOP is a sensor/lifecycle fault, not a fresh obstacle
            # observation. Never let avoidance reuse ranges from before it.
            self.last_scan_valid = False
            self.last_dist_mm = None
            self.last_angle = None
            self.left_clear_mm = None
            self.right_clear_mm = None

            if changed:
                print(f"[LIDAR-SAFETY] FORCE STOP ({reason})", flush=True)

            return self.state

    def update(self, scan):
        if self._bypassed:
            return "GO"

        scan_valid, point, valid_count, front_raw = analyze_scan(scan)

        with self._lock:
            self.last_scan_valid = scan_valid
            self.last_valid_point_count = valid_count
            self.last_front_raw = front_raw

            if not scan_valid:
                self.last_dist_mm = None
                self.last_angle = None
                return self.force_stop(
                    f"scan khong hop le: {valid_count} diem"
                )

            self.last_valid_scan_time = time.monotonic()
            self.left_clear_mm, self.right_clear_mm = analyze_side_clearance(scan)

            if point is None:
                distance = None
                angle = None
            else:
                distance, angle, _quality = point

            self.last_dist_mm = distance
            self.last_angle = angle

            obstacle_close = (
                distance is not None and distance <= STOP_DIST_MM
            )

            # 2026-09-15: front_raw == 0 nghia la KHONG MOT TIA LASER RAW
            # nao cham vao cone truoc - manh hon "front_points rong", vi
            # front_points rong co the do loc quality/khoang cach tren mot
            # tia THUC SU da roi vao do (vat toi mau hoac o ria khoang
            # cach hop le). Khong mot tia raw nao la bang chung manh nhat
            # co the co rang khong co gi chan duong: mot be mat that, du
            # toi hay xa, van tra ve it nhat vai tia yeu.
            #
            # Do doi hoi scan_valid (>= MIN_VALID_SCAN_POINTS diem tren ca
            # 360 do) truoc khi toi day, nen day khong phai cam bien hong -
            # no dang thay du thu, chi rieng huong truoc khong co gi de
            # phan xa (thuong la khoang cach vuot MAX_VALID_DIST_MM hoac
            # huong ra khong gian mo).
            #
            # KHONG dung dieu kien nay khi front_raw > 0: du chi 1 tia bi
            # loc cung du de KHONG tu dong coi la trong - giu nguyen thai
            # do than trong voi truong hop con nghi ngo (vat toi mau/gan
            # ria khoang cach), dung nhu truoc gio.
            front_truly_empty = point is None and front_raw == 0

            path_clear = (
                distance is not None and distance >= RESUME_DIST_MM
            ) or front_truly_empty

            if obstacle_close:
                self.stop_confirm += 1
                self.clear_confirm = 0
            elif path_clear:
                self.clear_confirm += 1
                self.stop_confirm = 0
            else:
                # Dai hysteresis: giu nguyen trang thai hien tai.
                self.stop_confirm = 0
                self.clear_confirm = 0

            # Khi dang STOP, cap nhat ly do ngay ca khi trang thai khong doi.
            # Neu khong, log co the giu sai ly do startup "chua co scan hop le"
            # du LiDAR da co scan hop le va dang thay vat can that.
            if self.state == "STOP":
                if obstacle_close:
                    self.reason = "obstacle"
                elif path_clear:
                    self.reason = "dang xac nhan clear"
                elif distance is None:
                    # 2026-09-12: truoc day truong hop nay cung bao "vung
                    # hysteresis", tuc log noi co vat nam giua STOP va
                    # RESUME trong khi KHONG co diem phan xa nao trong cone
                    # truoc. Hai tinh huong khac han nhau, va nham lan nay
                    # dan toi di tim mot vat can khong ton tai.
                    #
                    # CHU Y - day chi doi NHAN, khong doi hanh vi. May
                    # trang thai van ket o STOP trong tinh huong nay, vi
                    # path_clear doi distance is not None nen clear_confirm
                    # khong bao gio dem len. Xem ghi chu trong main().
                    self.reason = "cone truoc khong co diem phan xa"
                else:
                    self.reason = "vung hysteresis"

            if self.state == "GO":
                if self.stop_confirm >= LIDAR_STOP_CONFIRM_FRAMES:
                    self.state = "STOP"
                    self.reason = "obstacle"
                    print(
                        f"[LIDAR-SAFETY] -> STOP (dist={distance:.0f}mm)",
                        flush=True,
                    )
            elif self.clear_confirm >= LIDAR_CLEAR_CONFIRM_FRAMES:
                self.state = "GO"
                self.reason = "clear"
                print("[LIDAR-SAFETY] -> GO", flush=True)

            return self.state

    def check_timeout(self):
        with self._lock:
            if self._bypassed:
                return "GO"

            if self.last_valid_scan_time is None:
                return self.force_stop("chua co scan hop le")

            elapsed = time.monotonic() - self.last_valid_scan_time
            if elapsed > SCAN_TIMEOUT_SEC:
                return self.force_stop(
                    f"mat scan qua {SCAN_TIMEOUT_SEC:.1f}s"
                )

            return self.state

    def is_stop(self):
        with self._lock:
            if self._bypassed:
                return False
            return self.state == "STOP"

    def snapshot(self):
        """Doc mot snapshot atomic de thread khac khong thay state nua cu nua moi."""
        with self._lock:
            age = (None if self.last_valid_scan_time is None else
                   time.monotonic() - self.last_valid_scan_time)
            return {
                "state": self.state,
                "reason": self.reason,
                "dist_mm": self.last_dist_mm,
                "angle": self.last_angle,
                "valid_points": self.last_valid_point_count,
                "front_raw": self.last_front_raw,
                "scan_valid": self.last_scan_valid,
                "scan_age_s": age,
                "scan_fresh": (self.last_scan_valid and age is not None
                               and 0.0 <= age <= SCAN_TIMEOUT_SEC),
                "bypassed": self._bypassed,
                "left_clear_mm": self.left_clear_mm,
                "right_clear_mm": self.right_clear_mm,
            }


def main():
    """
    Kiem tra LiDAR CHI DOC. Khong mo UART, khong gui lenh nao cho xe.

    Tra loi ba cau hoi, va in ra ly do chu khong chi in trang thai:
      1. LiDAR co quay va tra ve du diem khong?
      2. Co vat nam trong vung STOP..RESUME khong?
      3. Neu chua GO thi vi sao?

    Cau 1 can HAI so, khong phai mot. valid_points dem tren ca 360 do;
    cone la so o 5 do trong cone truoc co diem. Mot con so khong tach
    duoc "cam bien gan mu" (diem it, cone rong) khoi "duong trong that"
    (diem nhieu, cone rong vi phia truoc khong co gi phan xa).
    """
    # Import trong ham: lidar_recorder import tu module nay, nen import o
    # cap module se thanh vong tron.
    from lidar_recorder import FRONT_BIN_COUNT, describe_scan

    safety = LidarSafety()
    lidar = None

    scans = 0
    go_scans = 0
    points = []
    bins = []
    front_raw = []
    front_lowq = []
    front_near = []
    front_weak_near = []
    hyst_dists = []
    reasons = {}
    t_start = time.monotonic()
    t_last_summary = t_start

    def group(reason):
        """Bo phan so lieu thay doi de dem duoc.

        Ly do cua scan khong hop le mang luon so diem ("scan khong hop le:
        17 diem"), nen dem tho se tach thanh mot dong cho moi gia tri va
        tong ket doc thanh vo nghia.
        """
        return reason.split(":")[0].strip()

    def summarise(title):
        span = time.monotonic() - t_start
        print("")
        print("=" * 64)
        print(f"{title} - {scans} scan trong {span:.1f}s "
              f"({scans / span if span > 0 else 0:.1f} scan/s)")
        print("=" * 64)

        if not scans:
            print("Khong nhan duoc scan nao. LiDAR khong quay, hoac cap/"
                  "nguon USB khong len, hoac dang bi tien trinh khac giu "
                  "/dev/ttyUSB0 (bridge?).")
            return

        def stat(values):
            values = sorted(values)
            return values[0], values[len(values) // 2], values[-1]

        print("")
        print("1. LiDAR co quay va tra du diem khong?")
        lo, mid, hi = stat(points)
        note = ("BENH: duoi nguong MIN_VALID_SCAN_POINTS"
                if mid < MIN_VALID_SCAN_POINTS else "ok")
        print(f"   valid_points (ca 360 do): {lo} / {mid} / {hi}   <- {note}")
        lo_b, mid_b, hi_b = stat(bins)
        if mid_b >= FRONT_BIN_COUNT - 2:
            note_b = "cone truoc duoc lay mau day du"
        elif mid_b <= 2 and mid >= 100:
            # KHONG ket luan "duong trong that" o day. Cone rong trong khi
            # ca vong nhieu diem loai tru duoc "cam bien mu", nhung khong
            # loai tru duoc "co vat ma phan hoi bi loc bo". Ba cot ly do
            # bi loai ben duoi moi tra loi duoc.
            note_b = "cone truoc rong nhung ca vong nhieu diem - xem ly do bi loai"
        elif mid_b <= 2:
            note_b = "cone truoc gan nhu khong duoc lay mau - nghi CAM BIEN BENH"
        else:
            note_b = "cone truoc lay mau thua"
        print(f"   cone truoc (/{FRONT_BIN_COUNT} o 5 do): "
              f"{lo_b} / {mid_b} / {hi_b}   <- {note_b}")

        lo_r, mid_r, hi_r = stat(front_raw)
        lo_q, mid_q, hi_q = stat(front_lowq)
        lo_n, mid_n, hi_n = stat(front_near)
        print(f"   cone truoc, diem THO truoc loc: {lo_r} / {mid_r} / {hi_r}")
        print(f"   trong do bi loai vi quality < {MIN_QUALITY}: "
              f"{lo_q} / {mid_q} / {hi_q}")
        print(f"   bi loai vi gan hon {MIN_VALID_DIST_MM:.0f}mm: "
              f"{lo_n} / {mid_n} / {hi_n}")
        lo_w, mid_w, hi_w = stat(front_weak_near)
        print(f"   diem YEU bao khoang cach <={STOP_DIST_MM:.0f}mm (ke ca <100mm): "
              f"{lo_w} / {mid_w} / {hi_w}")
        print("   (day la ly do loai DAU TIEN, khong phai thong ke day du)")
        if mid_b <= 2:
            if mid_w >= 2:
                print("   -> NGHI CO VAT GAN: phan hoi yeu bao khoang cach")
                print("      trong vung STOP; tiep tuc giu STOP.")
                print("      Chua chung minh chac chan co vat - khoang cach")
                print("      cua mot diem quality thap tu no khong dang tin.")
            elif mid_q >= 3 or mid_n >= 3:
                print("   -> cone KHONG rong: co phan hoi nhung bi loc bo.")
                print("      Co the la vat toi mau hoac be mat cheo. KHONG")
                print("      tu dong coi la thoang - day van la STOP.")
            elif mid_r <= 2:
                # 2026-09-15: front_raw = 0 gio la dieu kien du de tinh vao
                # clear_confirm (xem update()). Khac voi ba nhanh tren, day
                # KHONG con la trang thai ket vinh vien.
                print("   -> cone khong co tia raw nao ca - bang chung manh")
                print("      nhat co the co rang khong gi chan duong (qua")
                print("      xa de phan xa, hoac huong ra khong gian mo).")
                print("      Tu 2026-09-15, truong hop nay DUOC TINH la mot")
                print(f"      lan clear; sau {LIDAR_CLEAR_CONFIRM_FRAMES} lan")
                print("      lien tiep se chuyen sang GO.")

        print("")
        print("2. Co vat trong vung STOP..RESUME khong?")
        if hyst_dists:
            lo_d, mid_d, hi_d = stat(hyst_dists)
            print(f"   {len(hyst_dists)} scan ket trong vung hysteresis, "
                  f"khoang cach {lo_d:.0f} / {mid_d:.0f} / {hi_d:.0f} mm")
            print(f"   Vung nay la {STOP_DIST_MM:.0f}-{RESUME_DIST_MM:.0f} mm. "
                  f"Co vat that o day thi xe se KHONG BAO GIO chay.")
        else:
            print("   Khong scan nao ket trong vung hysteresis.")

        print("")
        print("3. Tai sao chua GO?")
        print(f"   GO {go_scans}/{scans} scan ({100.0 * go_scans / scans:.0f}%)")
        for reason, count in sorted(reasons.items(), key=lambda kv: -kv[1]):
            print(f"   {count:5d}x  {reason}")

        # 2026-09-15: sau khi sua, nhan "cone truoc khong co diem phan xa"
        # CHI con xuat hien khi front_raw > 0 (co tia nhung bi loc) - truong
        # hop front_raw == 0 gio bao "dang xac nhan clear" va tien toi GO,
        # khong con dung o day. Nen wedged o day dung nghia la: van con
        # tin hieu ma khong ro nguon goc, KHONG phai "chua bao gio thoat
        # duoc STOP".
        wedged = reasons.get("cone truoc khong co diem phan xa", 0)
        if wedged and go_scans == 0:
            print("")
            print("   !!! Con scan bao 'cone khong co diem phan xa' MA VAN")
            print("   con tia raw trong cone (front_raw > 0, bi loc boi")
            print(f"   quality < {MIN_QUALITY} hoac ngoai "
                  f"{MIN_VALID_DIST_MM:.0f}..{MAX_VALID_DIST_MM:.0f}mm).")
            print("   Day la truong hop CO NGHI VAN THUC SU - mot vat toi")
            print("   mau/gan ria khoang cach co the tao ra dung dau hieu")
            print("   nay - nen may trang thai CO CHU DICH khong tu clear")
            print("   o day. Neu ban da kiem tra truc tiep va biet chac")
            print("   phia truoc trong, doc lai muc 1 de xem co phai front_raw")
            print("   luon > 0 (nghia thuc su co gi) hay chi thinh thoang.")

    try:
        lidar = RPLidar(PORT, timeout=SCAN_TIMEOUT_SEC)

        print("=== LIDAR INFO ===")
        print(lidar.get_info())

        print("=== HEALTH ===")
        health = lidar.get_health()
        print(health)
        # Khong dung o day nua. Muc dich cua bai nay la tim hieu VI SAO
        # scan yeu; health khong Good chinh la mot phan cua cau tra loi,
        # va cac scan sau do van dang xem.
        if not health or health[0] != "Good":
            print("")
            print("*** CANH BAO: health khong Good. Van chay tiep de xem "
                  "scan, nhung ket qua duoi day khong dai dien cho mot "
                  "cam bien khoe. ***")

        print("=== NGUONG ===")
        print(
            f"STOP <= {STOP_DIST_MM:.0f}mm | "
            f"RESUME >= {RESUME_DIST_MM:.0f}mm | "
            f"FRONT_CENTER_DEG={FRONT_CENTER_DEG:.1f} | "
            f"CONE +-{FRONT_CONE_DEG:.0f}deg"
        )
        print(
            f"MIN_VALID_SCAN_POINTS={MIN_VALID_SCAN_POINTS} | "
            f"SCAN_TIMEOUT={SCAN_TIMEOUT_SEC:.1f}s | "
            f"STOP_CONFIRM={LIDAR_STOP_CONFIRM_FRAMES} | "
            f"CLEAR_CONFIRM={LIDAR_CLEAR_CONFIRM_FRAMES}"
        )
        print("")
        print("Dat xe DUNG YEN va cho den khi thay GO on dinh.")
        print("Ctrl+C de dung va xem tong ket.")
        print("")

        for scan in lidar.iter_scans():
            state = safety.update(scan)
            snapshot = safety.snapshot()
            info = describe_scan(scan)

            scans += 1
            points.append(snapshot["valid_points"])
            bins.append(info["front_bins"])
            front_raw.append(info["front_raw"])
            front_lowq.append(info["front_low_quality"])
            front_near.append(info["front_too_near"])
            front_weak_near.append(info["front_weak_near"])
            reason = snapshot["reason"]
            key = group(reason)
            reasons[key] = reasons.get(key, 0) + 1
            if state == "GO":
                go_scans += 1
            if reason == "vung hysteresis" and snapshot["dist_mm"] is not None:
                hyst_dists.append(snapshot["dist_mm"])

            # Mot dinh dang duy nhat cho moi truong hop, de doc duoc khi
            # dong chu chay 5 dong moi giay.
            dist = snapshot["dist_mm"]
            angle = snapshot["angle"]
            print(
                f"[{scans:5d}] {state:4s} "
                f"reason={reason:<22s} "
                f"points={snapshot['valid_points']:4d} "
                f"cone={info['front_bins']:3d}/{FRONT_BIN_COUNT} "
                f"dist={'     -' if dist is None else f'{dist:6.0f}'}mm "
                f"angle={'     -' if angle is None else f'{angle:+6.1f}'}"
                + (
                    ""
                    if info["front_points"]
                    else f"  [cone tho={info['front_raw']} "
                         f"yeu={info['front_low_quality']} "
                         f"gan={info['front_too_near']}]"
                )
            )

            now = time.monotonic()
            if now - t_last_summary >= 5.0:
                t_last_summary = now
                recent = points[-30:]
                top = max(reasons.items(), key=lambda kv: kv[1])
                print(
                    f"        --- {now - t_start:5.1f}s | GO "
                    f"{100.0 * go_scans / scans:3.0f}% | diem gan day "
                    f"{min(recent)}-{max(recent)} | ly do nhieu nhat: "
                    f"{top[0]} x{top[1]} ---"
                )

            time.sleep(0.05)

    except KeyboardInterrupt:
        safety.force_stop("Ctrl+C")
        summarise("TONG KET")
    except Exception as error:
        safety.force_stop(f"{type(error).__name__}: {error}")
        print(f"\n[LIDAR-SAFETY] LOI: {type(error).__name__}: {error}")
        summarise("TONG KET (dung vi loi)")
    else:
        summarise("TONG KET (iter_scans ket thuc)")
    finally:
        if lidar is not None:
            try:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
            except Exception as error:
                print(f"[LIDAR-SAFETY] Loi khi dong ket noi: {error}")

        print("")
        print("DONE - safety state = STOP. Khong co lenh nao duoc gui cho xe.")


if __name__ == "__main__":
    main()
