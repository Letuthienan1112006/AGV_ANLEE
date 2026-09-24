"""
lidar_recorder.py - Ghi lai scan LiDAR de tra loi mot cau hoi duy nhat:
khi safety noi "scan khong hop le: 17 diem", la vi nguong 20 qua chat,
hay vi cam bien thuc su khong thay gi?

Module nay CHI QUAN SAT. No khong sua, khong doc, khong tham gia vao
quyet dinh STOP/GO cua lidar_safety.py. Hai file ra:

  lidar_scan_summary.csv  mot dong moi scan, luon bat, rat nhe
  lidar_scan_raw.csv      toan bo diem, chi cho cac scan dang quan tam

Ly do tach hai file: dong sub ghi het thi 400 diem x 5.5 Hz lam day the
nho va lam cham thread LiDAR. Nen ghi day du dung o cho dang tranh chap
(scan bi xu la khong hop le) cong vai scan khoe lam moc so sanh.
"""

import math
import os

from lidar_safety import (
    FRONT_CENTER_DEG,
    FRONT_CONE_DEG,
    MAX_VALID_DIST_MM,
    MIN_QUALITY,
    MIN_VALID_DIST_MM,
    STOP_DIST_MM,
    normalize_angle,
)


# Chia cone truoc thanh cac o 5 do. Dem so o CO diem la cach do "cone
# truoc co duoc lay mau deu khong", khac han voi dem tong so diem tren
# ca 360 do - chinh la cho MIN_VALID_SCAN_POINTS bi lan.
BIN_DEG = 5.0
FRONT_BIN_COUNT = int(round(2.0 * FRONT_CONE_DEG / BIN_DEG))

# Ngan sach ghi raw.
RAW_FIRST_SCANS = 5          # vai scan dau de biet luc moi khoi dong
RAW_INVALID_LIMIT = 40       # cac scan bi xu la khong hop le
RAW_PERIODIC_SEC = 10.0      # mot scan khoe moi 10s lam moc so sanh

SUMMARY_NAME = "lidar_scan_summary.csv"
RAW_NAME = "lidar_scan_raw.csv"

SUMMARY_HEADER = (
    "t,raw_points,valid_points,front_points,front_bins,front_span_deg,"
    "nearest_mm,nearest_deg,front_raw,front_low_quality,front_weak_near,"
    "front_too_near,front_too_far,state,scan_valid,reason\n"
)
RAW_HEADER = "t,scan_id,quality,angle_deg,rel_angle_deg,dist_mm,kept\n"


def describe_scan(scan):
    """
    Do mot scan ma khong phan xet. Tra ve dict cac con so tho.

    front_bins la so o 5 do trong cone truoc co it nhat mot diem hop le.
    front_span_deg la khoang cach goc giua diem trai nhat va phai nhat
    trong cone. Hai so nay noi cone truoc co duoc lay mau khong; con
    valid_points noi ca vong quay thu duoc bao nhieu diem.

    Cone truoc RONG nhung khong co diem hop le co the do nhieu nguyen nhan
    khac nhau, va front_bins mot minh KHONG tach duoc:
      - khong co gi o do
      - co vat nhung phan hoi bi loc bo: quality duoi MIN_QUALITY (vat
        toi mau, be mat cheo), gan hon MIN_VALID_DIST_MM, hay xa hon
        MAX_VALID_DIST_MM
    Nen dem luon so diem trong cone bi loai. Day la khac biet quan trong
    ve an toan: mot cai chan nguoi mac quan toi o 0.5m co the tra ve
    quality thap va bi loc, khien cone "rong". Coi cone rong la "duong
    thoang" trong tinh huong do la lai xe vao no.

    HAI GIOI HAN CUA CAC BO DEM NAY, doc truoc khi dung chung de ket luan:

    1. Day la LY DO LOAI DAU TIEN, khong phai thong ke day du tung dac
       tinh. Cac phep kiem chay theo thu tu quality -> gan -> xa, giong
       analyze_scan, nen mot diem VUA quality thap VUA duoi 100mm chi
       duoc tinh vao front_low_quality. Muon biet "co gi gan khong" thi
       doc front_weak_near, dem rieng cac diem quality thap ma khoang
       cach BAO VE lai nam trong vung STOP (ke ca duoi 100mm; chi loai
       distance == 0 vi RPLidar bao 0 khi khong co phep do) - luu y chinh
       khoang cach cua mot diem quality thap cung khong dang tin, nen day
       la NGHI VAN, khong phai bang chung co vat.

    2. front_raw = 0, tuc khong nhan duoc diem nao ke ca truoc khi loc,
       VAN khong phan biet duoc "duong trong" voi "cam bien khong nhin
       thay vat do". Mot be mat hap thu hoan toan khong tra ve gi ca,
       khong phai tra ve quality thap. Cac bo dem NARROW cac kha nang,
       chung khong giai quyet chung.
    """
    valid_points = 0
    front = []
    bins = set()
    front_raw = 0
    front_low_quality = 0
    front_too_near = 0
    front_too_far = 0
    front_weak_near = 0

    for quality, raw_angle, distance in scan:
        if not math.isfinite(raw_angle):
            continue

        rel = normalize_angle(raw_angle - FRONT_CENTER_DEG)
        in_front = -FRONT_CONE_DEG <= rel <= FRONT_CONE_DEG
        if in_front:
            front_raw += 1

        if quality < MIN_QUALITY:
            if in_front:
                front_low_quality += 1
                # Khoang cach cua mot diem quality thap khong dang tin,
                # nhung mot chum diem yeu bao khoang cach trong vung STOP
                # la tin hieu an toan khong duoc bo qua.
                #
                # KHONG dat san MIN_VALID_DIST_MM: mot diem vua yeu vua
                # duoi 100mm chinh la truong hop dang lo nhat (vat ap sat
                # cam bien), va gioi han duoi se lam no bien mat. Chi loai
                # distance == 0, vi giao thuc RPLidar bao 0 khi KHONG CO
                # phep do - do la "khong thay gi", khong phai "vat o 0mm".
                if (math.isfinite(distance)
                        and 0.0 < distance <= STOP_DIST_MM):
                    front_weak_near += 1
            continue
        if not math.isfinite(distance):
            continue
        if distance < MIN_VALID_DIST_MM:
            if in_front:
                front_too_near += 1
            continue
        if distance > MAX_VALID_DIST_MM:
            if in_front:
                front_too_far += 1
            continue

        valid_points += 1

        if in_front:
            front.append((distance, rel))
            index = int((rel + FRONT_CONE_DEG) / BIN_DEG)
            bins.add(min(index, FRONT_BIN_COUNT - 1))

    if front:
        nearest = min(front, key=lambda point: point[0])
        angles = [point[1] for point in front]
        span = max(angles) - min(angles)
    else:
        nearest = (None, None)
        span = 0.0

    return {
        "raw_points": len(scan),
        "valid_points": valid_points,
        "front_points": len(front),
        "front_bins": len(bins),
        "front_span_deg": span,
        "nearest_mm": nearest[0],
        "nearest_deg": nearest[1],
        "front_raw": front_raw,
        "front_low_quality": front_low_quality,
        "front_too_near": front_too_near,
        "front_too_far": front_too_far,
        "front_weak_near": front_weak_near,
    }


class LidarRecorder:
    """
    Ghi scan vao AGV_RUN_DIR. Neu khong co AGV_RUN_DIR thi tu vo hieu
    hoa, de chay tay ngoai run.sh khong sinh file rac.

    Moi loi ghi deu bi nuot. Bo ghi log khong bao gio duoc phep lam
    chet thread LiDAR - do la thread an toan.
    """

    def __init__(self, run_dir=None, clock=None):
        if run_dir is None:
            run_dir = os.environ.get("AGV_RUN_DIR", "")

        self.enabled = bool(run_dir) and os.path.isdir(run_dir)
        self.scan_id = 0
        self.raw_invalid_written = 0
        self.last_periodic_t = None
        self._clock = clock
        self._summary = None
        self._raw = None

        if not self.enabled:
            return

        try:
            self._summary = open(os.path.join(run_dir, SUMMARY_NAME), "w")
            self._summary.write(SUMMARY_HEADER)
            self._raw = open(os.path.join(run_dir, RAW_NAME), "w")
            self._raw.write(RAW_HEADER)
            print(
                "[LIDAR-REC] Ghi scan vao {}".format(
                    os.path.join(run_dir, SUMMARY_NAME)
                ),
                flush=True,
            )
        except (IOError, OSError) as error:
            print("[LIDAR-REC] Tat ghi scan: {}".format(error), flush=True)
            self.close()
            self.enabled = False

    def _now(self):
        if self._clock is not None:
            return self._clock()
        import time

        return time.time()

    def _want_raw(self, t, scan_valid):
        """Scan nay co dang ghi day du khong."""
        if self.scan_id <= RAW_FIRST_SCANS:
            return True

        if not scan_valid:
            if self.raw_invalid_written < RAW_INVALID_LIMIT:
                self.raw_invalid_written += 1
                return True
            return False

        if (self.last_periodic_t is None
                or t - self.last_periodic_t >= RAW_PERIODIC_SEC):
            self.last_periodic_t = t
            return True

        return False

    def record(self, scan, snapshot):
        """
        Goi ngay sau safety.update(scan), truyen vao safety.snapshot().
        snapshot chi de ghi lai quyet dinh cua lop an toan, khong duoc
        dung de tinh toan gi.
        """
        if not self.enabled:
            return

        self.scan_id += 1
        t = self._now()

        try:
            info = describe_scan(scan)
            scan_valid = bool(snapshot.get("scan_valid"))

            def fmt(value, spec):
                return "" if value is None else format(value, spec)

            self._summary.write(
                "{:.3f},{},{},{},{},{:.1f},{},{},{},{},{},{},{},{},{},{}\n".format(
                    t,
                    info["raw_points"],
                    info["valid_points"],
                    info["front_points"],
                    info["front_bins"],
                    info["front_span_deg"],
                    fmt(info["nearest_mm"], ".0f"),
                    fmt(info["nearest_deg"], "+.1f"),
                    info["front_raw"],
                    info["front_low_quality"],
                    info["front_weak_near"],
                    info["front_too_near"],
                    info["front_too_far"],
                    snapshot.get("state", ""),
                    int(scan_valid),
                    str(snapshot.get("reason", "")).replace(",", ";"),
                )
            )
            self._summary.flush()

            if self._want_raw(t, scan_valid):
                for quality, raw_angle, distance in scan:
                    kept = (
                        quality >= MIN_QUALITY
                        and math.isfinite(distance)
                        and MIN_VALID_DIST_MM <= distance <= MAX_VALID_DIST_MM
                    )
                    self._raw.write(
                        "{:.3f},{},{},{:.2f},{:+.2f},{:.1f},{}\n".format(
                            t,
                            self.scan_id,
                            quality,
                            raw_angle,
                            normalize_angle(raw_angle - FRONT_CENTER_DEG),
                            distance,
                            int(kept),
                        )
                    )
                self._raw.flush()

        except Exception as error:  # noqa: BLE001 - xem docstring class
            print("[LIDAR-REC] Loi ghi, tat: {}".format(error), flush=True)
            self.close()
            self.enabled = False

    def close(self):
        for handle in (self._summary, self._raw):
            if handle is None:
                continue
            try:
                handle.close()
            except (IOError, OSError):
                pass
        self._summary = None
        self._raw = None
