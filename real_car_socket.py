"""
real_car_socket.py - Bridge xe that v3 cho patrol_robot.py.

Kien truc:
  - Main: nhan mot command AI, sau do gui dung mot frame JSON.
  - Sender: gui STM32 moi 100 ms, doc lap voi toc do AI.
  - LiDAR: doc scan va cap nhat LidarSafety.
  - STM32 reader: doc telemetry.

Thu tu uu tien an toan:
  shutdown/serial fault > LiDAR STOP > AI timeout > command AI.

Port co dinh:
  STM32: /dev/ttyACM0
  LiDAR: /dev/ttyUSB0

Protocol voi patrol_robot.py hien tai khong co length-prefix. Vi vay bridge
giu nghiem ngat mot command hop le -> mot frame JSON, khong gui frame tu do.
"""

import base64
import collections
import json
import math
import os
import re
import socket
import statistics
import sys
import threading
import time

import cv2
from lidar_recorder import LidarRecorder
from rplidar import RPLidar, RPLidarException

try:
    import serial
except ImportError:
    serial = None

from Confg import BENCH_MODE, LIDAR_RESUME_DIST_M, LIDAR_STOP_DIST_M
from lidar_safety import LidarSafety, SCAN_TIMEOUT_SEC


HOST = "127.0.0.1"
PORT = 54321

CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 360
CAMERA_FPS = 30
JPEG_QUALITY = 70

STM32_PORT = "/dev/ttyACM0"
STM32_BAUDRATE = 115200
STM32_READ_TIMEOUT = 0.05
STM32_WRITE_TIMEOUT = 0.30
STM32_TELEMETRY_LOG_INTERVAL = 0.50
STM32_MAX_CONSECUTIVE_WRITE_ERRORS = 3
STM32_RECOVERY_STOP_WRITES = 2

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BUFFER_LIMIT = 2500
LIDAR_RECONNECT_BASE_SEC = 0.5
LIDAR_RECONNECT_MAX_SEC = 4.0
LIDAR_RESET_AFTER_ERRORS = 2

STEER_MIN = -20
STEER_MAX = 20    # Khop firmware STM32 Safe V2
SPEED_MIN = 0
SPEED_MAX = 25    # 2026-09-04: nang tu 20 -> 25, ban dau la 20 chi de an toan cho lan test dong co that dau tien, gio da qua nhieu buoc kiem chung khac

SEND_INTERVAL = 0.10
AI_TIMEOUT = 1.4    # Tang tu 1.0 -> 1.4, khop thoi gian inference thuc te 0.8-1.04s
SOCKET_POLL_TIMEOUT = 0.20

# --- Ne vat can khi bi chan lien tuc (khong co waypoint/ban do that,
# ne xong tra quyen lai cho camera lane-following binh thuong) ---
AVOIDANCE_TRIGGER_SEC = 10.0      # bi chan lien tuc bao lau thi bat dau ne
AVOIDANCE_MAX_DURATION_SEC = 15.0 # ne toi da bao lau, qua thi bo cuoc ve STOP
AVOIDANCE_MIN_CLEARANCE_MM = 500.0  # huong ne phai co it nhat tung nay
AVOIDANCE_EMERGENCY_MM = 150.0    # dang ne ma phia truoc gan hon nay -> dung khan cap
AVOIDANCE_STEER = 18              # gan max, re gap ve phia trong
AVOIDANCE_SPEED = 12              # cham, giong RECOVERY_SPEED ben AI

# --- IMU (BNO055 qua STM32) - chi de phat hien bi nhac len va BAO
# CAO, khong tham gia dieu khien dong co. Nguong CHUA duoc hieu
# chinh tren phan cung that - can tune lai sau khi co du lieu that.
IMU_LIFT_ACCEL_THRESHOLD = 3.0    # m/s^2 (gia toc tuyen tinh, da tru trong luc)
IMU_LIFT_TILT_DEG = 25.0          # do lech roll/pitch so voi mat phang
# Duong "giat manh" o tren rieng no khong du: test thuc te cho thay nhac xe len
# nhe nhang tay khong bao gio dat toi 25 do / 3.0 m/s^2. Tin hieu that su ro rang
# hon la DO DAO DONG (jitter) - xe nam yen tren gia/mat dat gan nhu dung im
# tuyet doi, con bi cam tay thi luon rung nhe theo tay du khong nghieng manh.
# 2026-09-15: ca ba hang so DOC (khong phai giay) duoi day duoc canh
# chinh luc firmware con in telemetry moi 200ms. Firmware da doi sang
# PRINT_INTERVAL_MS=100 (2026-09-14, xem motor_test_bts7960/src/main.cpp),
# va imu_status.update() duoc goi DUNG MOT LAN moi dong telemetry - nen ca
# ba khoang thoi gian thuc te duoi day bi CAT DOI ma khong ai chinh lai:
# cua so dao dong con ~1s thay vi ~2s, xac nhan nhac len con ~200ms thay vi
# ~400ms. Do dung la nguyen nhan cu giat khoi dong binh thuong (banh pha vo
# ma sat tinh, ~0.2-0.3s) bi bat nham la nhac len: no chiem mot phan lon
# hon han trong mot cua so da bi thu hep mot nua. Do lai bang so DOC de
# khong con phu thuoc nhip in - xem quang thoi gian that o cot ben.
IMU_WINDOW_SIZE = 20              # ~2.0s @100ms/mau (nhu cu tai 10 @200ms)
IMU_JITTER_ROLL_STD = 1.0         # do lech chuan roll (do) - vuot nguong = dang bi cam
IMU_JITTER_PITCH_STD = 1.0        # do lech chuan pitch (do)
IMU_JITTER_ACCEL_STD = 0.12       # do lech chuan accel (m/s^2)
IMU_LIFT_CONFIRM_READINGS = 4     # ~0.4s @100ms/mau (nhu cu tai 2 @200ms)
IMU_LIFT_CLEAR_READINGS = 12      # ~1.2s @100ms/mau (nhu cu tai 6 @200ms)
# Bat doi xung voi clear: neu chi can 1 mau "yen" la clear ngay, nguoi dang cam
# xe lien tuc van co the tinh co roi vao 1 mau it dao dong hon -> lifted nhap
# nhay tat/bat lien tuc, spam alert (da thay that trong test). Phai giu ON du
# lau moi thuc su tat.

DRY_RUN = "--dry-run" in sys.argv
LANE_TEST_NO_LIDAR = os.environ.get("AGV_LANE_TEST_NO_LIDAR") == "1"
MANUAL_CONTROL = os.environ.get("AGV_MANUAL_CONTROL") == "1"

# --no-lidar: bo qua hoan toan lop an toan LiDAR. Mac dinh CHI cho xe ke
# tren gia. Ngoai le AGV_LANE_TEST_NO_LIDAR chi do `run lane` dat de thu
# bo dieu khien lane tren mot doan duong da don trong; `run road` khong dat.
#
# 2026-09-11: lab qua chat de test tren gia - non truoc do duoc 237mm, ket
# giua nguong STOP 200mm va RESUME 350mm cua che do bench, nen bridge dung va
# khong bao gio nha, du banh dang treo trong khong khi va khong the gay hai
# cho ai.
#
# Chot vao BENCH_MODE, khong phai vao y tot cua nguoi go lenh: neu ai do go
# --no-lidar ma quen AGV_BENCH thi day la mot chiec xe chay tren dat khong
# con phanh. Trong truong hop do THOAT, khong phai canh bao roi chay tiep.
NO_LIDAR = "--no-lidar" in sys.argv
if NO_LIDAR and not (BENCH_MODE or LANE_TEST_NO_LIDAR):
    print("=" * 62)
    print("  TU CHOI KHOI DONG: --no-lidar ma khong co AGV_BENCH=1.")
    print("  Dung 'run sweep' khi xe ke tren gia, hoac 'run lane' de thu")
    print("  bam lane tren doan duong trong co nguoi san sang STOP.")
    print("=" * 62)
    sys.exit(1)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class SharedCommand:
    def __init__(self):
        self._lock = threading.Lock()
        self._steer = 0.0
        self._speed = 0.0
        self._last_update = None

    def set(self, steer, speed):
        if not math.isfinite(steer) or not math.isfinite(speed):
            raise ValueError("command khong huu han")

        with self._lock:
            self._steer = steer
            self._speed = speed
            self._last_update = time.monotonic()

    def get(self):
        with self._lock:
            return self._steer, self._speed, self._last_update

    def invalidate(self):
        with self._lock:
            self._steer = 0.0
            self._speed = 0.0
            self._last_update = None


class ImuStatus:
    """Trang thai IMU dung chung giua stm32_reader_fn (ghi) va
    encode_frame (doc, de nhung vao JSON gui cho patrol_robot.py).
    Chi de BAO CAO bi nhac len, khong lien quan an toan dong co."""

    def __init__(self):
        self._lock = threading.Lock()
        self._present = False
        self._roll = None
        self._pitch = None
        self._accel = None
        self._yaw = None
        self._window = collections.deque(maxlen=IMU_WINDOW_SIZE)
        self._confirm_count = 0
        self._clear_count = 0
        self._lifted = False
        self._lift_since = None

    def update(self, roll, pitch, accel, yaw=None):
        with self._lock:
            self._present = True
            self._roll = roll
            self._pitch = pitch
            self._accel = accel
            self._yaw = yaw

            if roll is not None and pitch is not None and accel is not None:
                self._window.append((roll, pitch, accel))

            # Duong 1: nghieng/giat manh tuc thoi (vd lat xe, giat manh)
            sudden = (
                (accel is not None and accel > IMU_LIFT_ACCEL_THRESHOLD)
                or (roll is not None and abs(roll) > IMU_LIFT_TILT_DEG)
                or (pitch is not None and abs(pitch) > IMU_LIFT_TILT_DEG)
            )

            # Duong 2: do dao dong (jitter) trong cua so gan nhat - dac trung
            # cho viec bi cam tren tay (run tay tu nhien), khac han xe nam yen
            jittery = False
            if len(self._window) == IMU_WINDOW_SIZE:
                rolls, pitches, accels = zip(*self._window)
                jittery = (
                    statistics.pstdev(rolls) > IMU_JITTER_ROLL_STD
                    or statistics.pstdev(pitches) > IMU_JITTER_PITCH_STD
                    or statistics.pstdev(accels) > IMU_JITTER_ACCEL_STD
                )

            suspicious = sudden or jittery

            if suspicious:
                self._confirm_count += 1
                self._clear_count = 0
            else:
                self._confirm_count = 0
                self._clear_count += 1
                if self._lifted and self._clear_count >= IMU_LIFT_CLEAR_READINGS:
                    self._lifted = False
                    self._lift_since = None
                    print("[IMU] Het bi nhac len (da dat lai mat dat)", flush=True)

            if (
                not self._lifted
                and self._confirm_count >= IMU_LIFT_CONFIRM_READINGS
            ):
                self._lifted = True
                self._lift_since = time.strftime("%Y-%m-%d %H:%M:%S")
                print(
                    f"[IMU] PHAT HIEN BI NHAC LEN luc {self._lift_since} "
                    f"(roll={roll} pitch={pitch} accel={accel} "
                    f"jittery={jittery} sudden={sudden})",
                    flush=True,
                )

    def snapshot(self):
        with self._lock:
            return {
                "present": self._present,
                "lifted": self._lifted,
                "lift_since": self._lift_since,
                "roll": self._roll,
                "pitch": self._pitch,
                "accel": self._accel,
                "yaw": self._yaw,
            }


def parse_imu_telemetry(message):
    """Tra (roll, pitch, accel, yaw) tu dong telemetry co doan
    'IMU,r,p,a,yaw'. Tra (None, None, None, None) neu khong tim thay hoac
    khong parse duoc (vi du firmware bao 'IMU,NA,NA,NA,NA' khi cam bien
    khong san sang). yaw co the la None neu firmware cu (chi 3 truong) -
    tuong thich nguoc, khong bat buoc phai co. Cac truong sau yaw (gyro,
    tu 2026-09-12) bi bo qua o day; xem parse_imu_gyro."""
    idx = message.find("IMU,")
    if idx == -1:
        return None, None, None, None
    parts = message[idx + 4:].split(",")
    if len(parts) < 3:
        return None, None, None, None
    try:
        roll, pitch, accel = float(parts[0]), float(parts[1]), float(parts[2])
    except ValueError:
        return None, None, None, None
    yaw = None
    if len(parts) >= 4:
        try:
            yaw = float(parts[3])
        except ValueError:
            yaw = None
    return roll, pitch, accel, yaw


SEQ_RE = re.compile(r",SEQ,(\d+)")


def parse_telemetry_seq(message):
    """
    So thu tu telemetry, hoac None voi firmware cu.

    Bat dau tu 0 moi lan MCU khoi dong, nen no tra loi ca "co mat ban tin
    khong" (lo hong) va "MCU co reset khong" (nhay ve gan 0). Truoc day
    phan tich phai suy "da reset" tu viec bo dem encoder giam - sai, vi
    bo dem co dau va quay banh nguoc chieu lam no giam binh thuong.
    """
    match = SEQ_RE.search(message)
    if not match:
        return None
    try:
        return int(match.group(1))
    except ValueError:
        return None


def parse_imu_gyro(message):
    """
    Tra (gx, gy, gz) do/giay tu doan 'IMU,r,p,a,yaw,gx,gy,gz', hoac
    (None, None, None) voi firmware cu chi co 4 truong.

    KHONG chon san truc nao la toc do quay thang dung: truc do phu thuoc
    cach gan cam bien tren xe, va doan sai thi khong phat hien duoc tu
    log. Ghi ca ba, de phan tich doi chieu voi vi phan yaw Euler ma chon.
    """
    idx = message.find("IMU,")
    if idx == -1:
        return None, None, None

    parts = message[idx + 4:].split(",")
    if len(parts) < 7:
        return None, None, None

    try:
        return (
            float(parts[4]),
            float(parts[5]),
            float(parts[6]),
        )
    except ValueError:
        return None, None, None


class LidarWorker:
    def __init__(self, safety, stop_event, recorder=None):
        self.safety = safety
        self.stop_event = stop_event
        self._lidar = None
        self._lidar_lock = threading.Lock()
        # Bo ghi scan chi quan sat. Tao mot lan cho ca run de file trai
        # qua nhieu lan reconnect thay vi bi ghi de moi lan.
        self.recorder = (
            LidarRecorder() if recorder is None else recorder
        )

    def run(self):
        consecutive_errors = 0
        recoverable_errors = (RPLidarException, ValueError, OSError)
        if serial is not None:
            recoverable_errors += (serial.SerialException,)

        while not self.stop_event.is_set():
            lidar = None

            try:
                self.safety.force_stop("dang ket noi LiDAR")

                lidar = RPLidar(
                    LIDAR_PORT,
                    timeout=SCAN_TIMEOUT_SEC,
                )
                with self._lidar_lock:
                    self._lidar = lidar

                print(f"[LIDAR] Connected: {LIDAR_PORT}", flush=True)

                # Dua sensor ve idle va xoa het byte scan cu truoc khi
                # gui GET_INFO/GET_HEALTH. clean_input() khong hoat dong
                # neu scanning van dang True.
                lidar.stop()
                time.sleep(0.15)
                lidar.clean_input()

                info = lidar.get_info()
                if not isinstance(info, dict):
                    raise RPLidarException(
                        f"get_info khong hop le: {info!r}"
                    )
                print(f"[LIDAR] Info: {info}", flush=True)

                health = lidar.get_health()
                if not isinstance(health, tuple) or len(health) != 2:
                    raise RPLidarException(
                        f"get_health khong hop le: {health!r}"
                    )
                print(f"[LIDAR] Health: {health}", flush=True)

                if health[0] != "Good":
                    raise RPLidarException(
                        f"health khong Good: {health}"
                    )

                # Tat auto-restart bi loi cua thu vien. Bridge tu phat
                # hien backlog va reconnect toan bo ket noi.
                for scan in lidar.iter_scans(max_buf_meas=False):
                    if self.stop_event.is_set():
                        break

                    buffered = lidar._serial.inWaiting()
                    if buffered > LIDAR_BUFFER_LIMIT:
                        raise RPLidarException(
                            f"input buffer backlog: "
                            f"{buffered}/{LIDAR_BUFFER_LIMIT}"
                        )

                    self.safety.update(scan)
                    self.recorder.record(scan, self.safety.snapshot())
                    consecutive_errors = 0

                if not self.stop_event.is_set():
                    raise RPLidarException(
                        "iter_scans ket thuc bat thuong"
                    )
                break

            except recoverable_errors as error:
                if self.stop_event.is_set():
                    break

                consecutive_errors += 1
                reason = (
                    f"{type(error).__name__}: {error}"
                )
                print(
                    f"[LIDAR] LOI {consecutive_errors}: "
                    f"{reason} - FORCE STOP va reconnect",
                    flush=True,
                )
                self.safety.force_stop(
                    f"lidar reconnect: {reason}"
                )

                if (
                    lidar is not None
                    and consecutive_errors >= LIDAR_RESET_AFTER_ERRORS
                ):
                    try:
                        lidar.stop()
                        lidar.reset()
                        print(
                            "[LIDAR] Da reset sensor",
                            flush=True,
                        )
                    except Exception as reset_error:
                        print(
                            f"[LIDAR] Reset that bai: {reset_error}",
                            flush=True,
                        )

            except Exception as error:
                if not self.stop_event.is_set():
                    print(
                        f"[LIDAR] LOI KHONG PHUC HOI: "
                        f"{type(error).__name__}: {error}",
                        flush=True,
                    )
                    self.safety.force_stop(
                        f"lidar loi khong phuc hoi: {error}"
                    )
                break

            finally:
                self._close_lidar(lidar)
                with self._lidar_lock:
                    if self._lidar is lidar:
                        self._lidar = None

            if not self.stop_event.is_set():
                delay = min(
                    LIDAR_RECONNECT_BASE_SEC
                    * (2 ** min(consecutive_errors - 1, 3)),
                    LIDAR_RECONNECT_MAX_SEC,
                )
                print(
                    f"[LIDAR] Thu ket noi lai sau {delay:.1f}s",
                    flush=True,
                )
                self.stop_event.wait(delay)

        self.recorder.close()
        print("[LIDAR] Thread da dung", flush=True)

    def shutdown(self):
        self.safety.force_stop("bridge shutdown")

        with self._lidar_lock:
            lidar = self._lidar

        # Dong ket noi tu main de danh thuc iter_scans() neu no dang block.
        self._close_lidar(lidar)

    @staticmethod
    def _close_lidar(lidar):
        if lidar is None:
            return

        try:
            lidar.stop()
        except Exception:
            pass
        try:
            lidar.stop_motor()
        except Exception:
            pass
        try:
            lidar.disconnect()
        except Exception:
            pass


def open_camera():
    print("[CAMERA] Dang mo camera...")
    camera = cv2.VideoCapture(CAMERA_INDEX)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    camera.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

    if not camera.isOpened():
        raise RuntimeError(f"khong mo duoc camera index={CAMERA_INDEX}")

    ok, frame = camera.read()
    if not ok:
        camera.release()
        raise RuntimeError("camera mo duoc nhung khong doc duoc frame")

    print(f"[CAMERA] OK: frame={frame.shape}")
    return camera


def open_stm32():
    if DRY_RUN:
        print("[STM32] DRY RUN: khong mo UART, khong the dieu khien motor")
        return None

    if serial is None:
        raise RuntimeError("chua cai pyserial")

    try:
        stm32 = serial.Serial(
            STM32_PORT,
            STM32_BAUDRATE,
            timeout=STM32_READ_TIMEOUT,
            write_timeout=STM32_WRITE_TIMEOUT,
        )
        time.sleep(2.0)
        stm32.reset_input_buffer()
        stm32.reset_output_buffer()
        print(f"[STM32] Connected: {STM32_PORT} @ {STM32_BAUDRATE}")
        return stm32
    except Exception:
        raise


def write_stm32(stm32, steer, speed):
    if stm32 is None:
        return True

    final_steer = clamp(int(float(steer)), STEER_MIN, STEER_MAX)
    final_speed = clamp(int(float(speed)), SPEED_MIN, SPEED_MAX)
    message = f"{final_steer} {final_speed}\n"

    try:
        stm32.write(message.encode("ascii"))
        return True
    except Exception as error:
        print(f"[STM32] LOI GHI: {type(error).__name__}: {error}", flush=True)
        return False


def sender_thread_fn(stm32, ai_command, safety, stop_event):
    last_status = None
    consecutive_write_errors = 0
    uart_recovering = False
    recovery_stop_writes = 0

    # State ne vat can (xem khoi quyet dinh final_steer/final_speed o duoi)
    blocked_since = None
    avoidance_active = False
    avoidance_dir = 0
    avoidance_start = None

    try:
        while not stop_event.is_set():
            loop_started = time.monotonic()

            steer, speed, last_update = ai_command.get()
            reasons = []

            if last_update is None:
                reasons.append("chua nhan lenh AI")
            elif time.monotonic() - last_update > AI_TIMEOUT:
                reasons.append("AI timeout")

            # Ne vat chi hop le khi AI dang that su lai xe va bi ket. Khong
            # co AI thi khong ai chiu trach nhiem cho chuyen dong, nen bridge
            # tuyet doi khong tu quay banh (bug phat hien 2026-09-10: bridge
            # chay mot minh trong lab chat van tu re trai o toc do 12).
            ai_alive = (
                last_update is not None
                and time.monotonic() - last_update <= AI_TIMEOUT
            )

            safety.check_timeout()
            snapshot = safety.snapshot()
            lidar_fault = not (snapshot["bypassed"] or snapshot["scan_fresh"])
            if snapshot["state"] == "STOP":
                reasons.append(
                    f"LIDAR STOP ({snapshot['reason']})"
                )
            elif lidar_fault:
                reasons.append("LIDAR scan khong hop le/da cu")

            if uart_recovering:
                reasons.append("STM32 dang phuc hoi")

            # --- Theo doi thoi gian bi chan lien tuc boi vat can that
            # (chi tinh reason=="obstacle", khong tinh mat scan/dang ket
            # noi lai - do la loi cam bien, khong phai vat can that). ---
            now_mono = time.monotonic()
            if snapshot["reason"] == "obstacle" and not lidar_fault and speed > 0:
                if blocked_since is None:
                    blocked_since = now_mono
            else:
                blocked_since = None
            blocked_duration = (
                (now_mono - blocked_since) if blocked_since else 0.0
            )

            # --- Bat dau ne vat neu du dieu kien (chua ne, dang bi
            # chan qua lau, UART on dinh, co huong du trong) ---
            if (
                not avoidance_active
                and not MANUAL_CONTROL
                and not BENCH_MODE
                and ai_alive
                and speed > 0
                and not uart_recovering
                and not lidar_fault
                and snapshot["state"] == "STOP"
                and snapshot["reason"] == "obstacle"
                and blocked_duration >= AVOIDANCE_TRIGGER_SEC
            ):
                left_mm = snapshot.get("left_clear_mm")
                right_mm = snapshot.get("right_clear_mm")
                # No return is UNKNOWN, not proof of infinite clearance.
                left_val = left_mm if left_mm is not None else 0.0
                right_val = right_mm if right_mm is not None else 0.0
                if max(left_val, right_val) >= AVOIDANCE_MIN_CLEARANCE_MM:
                    avoidance_active = True
                    avoidance_start = now_mono
                    # steer>0 -> re phai, steer<0 -> re trai (da xac nhan
                    # bang encoder that). Ben nao trong hon thi re ve ben do.
                    avoidance_dir = -1 if left_val > right_val else 1
                    print(
                        f"[AVOID] Bat dau ne vat, huong="
                        f"{'trai' if avoidance_dir < 0 else 'phai'} "
                        f"(left={left_mm} right={right_mm})",
                        flush=True,
                    )

            if avoidance_active:
                front_mm = snapshot.get("dist_mm")
                turn_side_mm = snapshot.get(
                    "left_clear_mm" if avoidance_dir < 0 else "right_clear_mm")
                too_close = (
                    front_mm is not None
                    and front_mm < AVOIDANCE_EMERGENCY_MM
                )
                timed_out = (
                    now_mono - avoidance_start
                ) > AVOIDANCE_MAX_DURATION_SEC
                cleared = snapshot["state"] == "GO"

                # Kiem tra truoc moi dieu kien khac: nhanh nay bo qua
                # `reasons`, nen loi LiDAR/AI/UART phai duoc chan tai day.
                # Khong duoc dung khoang cach cu hoac coi None la "thoang".
                # Dat truoc `cleared` cung khien
                # viec tra quyen ve `steer, speed` khong bao gio dung lenh
                # AI da cu.
                if lidar_fault or front_mm is None:
                    avoidance_active = False
                    blocked_since = None
                    final_steer, final_speed = 0, 0
                    status = "STOP: ne vat huy - LiDAR loi/cu hoac mat diem phia truoc"
                elif turn_side_mm is None or turn_side_mm < AVOIDANCE_MIN_CLEARANCE_MM:
                    avoidance_active = False
                    blocked_since = None
                    final_steer, final_speed = 0, 0
                    status = "STOP: ne vat huy - huong dang re khong du/khong ro khoang trong"
                elif not ai_alive or uart_recovering:
                    avoidance_active = False
                    blocked_since = now_mono
                    final_steer, final_speed = 0, 0
                    status = "STOP: ne vat huy - " + (
                        "mat AI" if not ai_alive else "STM32 dang phuc hoi"
                    )
                elif speed <= 0:
                    avoidance_active = False
                    blocked_since = None
                    final_steer, final_speed = 0, 0
                    status = "STOP: ne vat huy - AI yeu cau dung"
                elif too_close:
                    avoidance_active = False
                    blocked_since = now_mono
                    final_steer, final_speed = 0, 0
                    status = "STOP: ne vat huy - vat qua gan"
                elif timed_out:
                    avoidance_active = False
                    blocked_since = now_mono
                    final_steer, final_speed = 0, 0
                    status = "STOP: ne vat het gio, cho can thiep"
                elif cleared:
                    avoidance_active = False
                    blocked_since = None
                    final_steer, final_speed = steer, speed
                    status = "AI ALLOWED (vua ne vat xong)"
                else:
                    final_steer = AVOIDANCE_STEER * avoidance_dir
                    final_speed = AVOIDANCE_SPEED
                    status = (
                        "NE VAT dang re "
                        f"{'trai' if avoidance_dir < 0 else 'phai'}"
                    )
            elif reasons:
                final_steer, final_speed = 0, 0
                status = "STOP: " + "; ".join(reasons)
            else:
                final_steer, final_speed = steer, speed
                status = "AI ALLOWED"

            if status != last_status:
                print(f"[SENDER] {status}", flush=True)
                last_status = status

            write_ok = write_stm32(
                stm32,
                final_steer,
                final_speed,
            )

            if not write_ok:
                consecutive_write_errors += 1
                uart_recovering = True
                recovery_stop_writes = 0

                if stm32 is not None:
                    try:
                        stm32.reset_output_buffer()
                    except Exception as reset_error:
                        print(
                            f"[STM32] Khong reset duoc output "
                            f"buffer: {reset_error}",
                            flush=True,
                        )

                print(
                    f"[STM32] Loi ghi "
                    f"{consecutive_write_errors}/"
                    f"{STM32_MAX_CONSECUTIVE_WRITE_ERRORS} "
                    "- chi gui STOP",
                    flush=True,
                )

                if (
                    consecutive_write_errors
                    >= STM32_MAX_CONSECUTIVE_WRITE_ERRORS
                ):
                    print(
                        "[STM32] Loi ghi lien tiep - shutdown",
                        flush=True,
                    )
                    stop_event.set()
                    break

            else:
                consecutive_write_errors = 0

                if uart_recovering:
                    recovery_stop_writes += 1

                    if (
                        recovery_stop_writes
                        >= STM32_RECOVERY_STOP_WRITES
                    ):
                        uart_recovering = False
                        recovery_stop_writes = 0
                        print(
                            "[STM32] UART da phuc hoi sau "
                            "2 lenh STOP",
                            flush=True,
                        )

            elapsed = time.monotonic() - loop_started
            stop_event.wait(
                max(0.0, SEND_INTERVAL - elapsed)
            )

    except Exception as error:
        print(
            f"[SENDER] LOI: "
            f"{type(error).__name__}: {error}",
            flush=True,
        )
        stop_event.set()

    finally:
        # Sender la noi duy nhat ghi UART.
        for _ in range(5):
            write_stm32(stm32, 0, 0)
            time.sleep(0.02)
        print(
            "[SENDER] Da gui STOP cuoi cung",
            flush=True,
        )

# Moi truong telemetry ENC tu STM32, du de doi chieu THEO THOI GIAN:
#   lenh gui di  ->  toc do muc tieu hai banh  ->  encoder thuc te  ->  yaw
# 2026-09-11: bridge.log KHONG dung duoc cho viec nay. No khong co dau thoi
# gian, dong [CMD] chi in KHI LENH DOI (nen khong biet mot lenh duoc giu bao
# lau), va dong ENC bi gioi han tan suat in. Ba thu do cong lai lam khong the
# noi duoc "tai thoi diem lenh be trai, banh da lam gi".
ENC_LINE_RE = re.compile(
    r"ENC,(-?\d+),(-?\d+),DELTA,(-?\d+),(-?\d+),TARGET,(-?\d+),(-?\d+),"
    r"PWM,(-?\d+),(-?\d+),STEER,(-?\d+),SPEED,(-?\d+),MODE,(\w)"
)


def stm32_reader_fn(stm32, stop_event, imu_status=None, ai_command=None):
    if stm32 is None:
        return

    last_telemetry_print = 0.0

    wheel_csv = None
    run_dir = os.environ.get("AGV_RUN_DIR", "")
    if run_dir:
        try:
            wheel_csv = open(os.path.join(run_dir, "wheel_telem.csv"), "w")
            # count_l/count_r la BO DEM TICH LUY cua firmware; enc_l/enc_r
            # la DELTA tung chu ky. Cong don cot DELTA KHONG cho tong xung:
            # firmware tinh delta moi CONTROL_INTERVAL_MS va chi in moi
            # PRINT_INTERVAL_MS, nen bat ky dong nao bi mat (hay bat ky
            # nhip in nao khong khop nhip dieu khien) deu lam mat han mot
            # khoang dem. Muon doi chieu voi video thi lay
            # count_cuoi - count_dau, phep tru mien nhiem voi dong bi mat.
            wheel_csv.write(
                "t,seq,cmd_steer,cmd_speed,count_l,count_r,enc_l,enc_r,"
                "tgt_l,tgt_r,pwm_l,pwm_r,fw_steer,fw_speed,mode,yaw,"
                "gyro_x,gyro_y,gyro_z\n"
            )
            print("[WHEEL] Ghi telemetry banh xe vao %s/wheel_telem.csv"
                  % run_dir, flush=True)
        except IOError as err:
            print("[WHEEL] Khong mo duoc file telemetry: %s" % err, flush=True)
            wheel_csv = None
    t_start = time.monotonic()

    while not stop_event.is_set():
        try:
            raw_line = stm32.readline()
            if raw_line:
                message = raw_line.decode(
                    "ascii",
                    errors="ignore",
                ).strip()

                if message:
                    now = time.monotonic()
                    is_telemetry = message.startswith("ENC,")

                    if (
                        not is_telemetry
                        or now - last_telemetry_print
                        >= STM32_TELEMETRY_LOG_INTERVAL
                    ):
                        print(
                            f"[STM32 RX] {message}",
                            flush=True,
                        )
                        if is_telemetry:
                            last_telemetry_print = now

                    if is_telemetry and wheel_csv is not None:
                        m = ENC_LINE_RE.search(message)
                        if m:
                            yaw_now = parse_imu_telemetry(message)[3]
                            gyro_now = parse_imu_gyro(message)
                            seq_now = parse_telemetry_seq(message)
                            if ai_command is not None:
                                c_st, c_sp, _ = ai_command.get()
                            else:
                                c_st = c_sp = float("nan")
                            g = m.groups()
                            try:
                                wheel_csv.write(
                                    "%.3f,%s,%.1f,%.1f,%s,%s,%s,%s,%s,%s,%s"
                                    ",%s,%s,%s,%s,%s,%s,%s,%s\n"
                                    % (time.monotonic() - t_start,
                                       "" if seq_now is None else seq_now,
                                       c_st, c_sp,
                                       g[0], g[1],
                                       g[2], g[3], g[4], g[5], g[6], g[7],
                                       g[8], g[9], g[10],
                                       "" if yaw_now is None else "%.1f" % yaw_now,
                                       *[
                                           "" if v is None else "%.2f" % v
                                           for v in gyro_now
                                       ])
                                )
                                # Xa ngay: run.sh dung bang SIGINT roi SIGKILL
                                # sau 2s, dem trong bo nho co the mat.
                                wheel_csv.flush()
                            except (IOError, ValueError):
                                pass

                    if is_telemetry and imu_status is not None:
                        roll, pitch, accel, yaw = parse_imu_telemetry(message)
                        if roll is not None:
                            imu_status.update(roll, pitch, accel, yaw)

        except Exception as error:
            if not stop_event.is_set():
                print(
                    f"[STM32] LOI DOC: "
                    f"{type(error).__name__}: {error}",
                    flush=True,
                )
                stop_event.set()
            break


def parse_ai_command(raw_data):
    message = raw_data.decode("utf-8", errors="ignore").strip()
    parts = message.split()

    if len(parts) < 2:
        raise ValueError(f"command ngan: {message!r}")

    steer = float(parts[0])
    speed = float(parts[1])
    if not math.isfinite(steer) or not math.isfinite(speed):
        raise ValueError(f"command khong huu han: {message!r}")

    return steer, speed, message


def encode_frame(camera, imu_status=None, blocked=False,
                 max_retries=3, retry_delay=0.02):
    ok = False
    frame = None

    for _attempt in range(max_retries):
        ok, frame = camera.read()
        if ok:
            break
        time.sleep(retry_delay)

    if not ok:
        raise RuntimeError(
            f"mat frame camera (da thu lai {max_retries} lan)"
        )

    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
    ok, buffer = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
    )
    if not ok:
        raise RuntimeError("JPEG encode fail")

    payload = {"Img": base64.b64encode(buffer.tobytes()).decode("ascii")}

    if imu_status is not None:
        imu_snap = imu_status.snapshot()
        if imu_snap["lifted"]:
            payload["lifted"] = True
            payload["lift_since"] = imu_snap["lift_since"]

    # Noi cho AI biet lenh cua no DANG KHONG DEN BANH XE. Khong co co nay,
    # AI khong the phan biet "xe dang chay va chua kip sua" voi "xe dung
    # yen vi bi chan" - va khau tich phan cua no cu don len trong luc xe
    # khong the di chuyen. Do thuc te 2026-09-10: LiDAR giu xe 12 giay,
    # tich phan don steer tu +5 len +12, den luc tha ra xe xoay gap vuot
    # han qua vach (loi tu +26 sang -236).
    if blocked:
        payload["blocked"] = True

    return json.dumps(payload).encode("utf-8")


def run_socket_server(camera, ai_command, safety, stop_event, imu_status=None):
    server = None
    connection = None

    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(1)
        server.settimeout(SOCKET_POLL_TIMEOUT)

        print(f"[NET] Lang nghe tai {HOST}:{PORT}")
        print("[NET] Cho patrol_robot.py ket noi...")

        while connection is None and not stop_event.is_set():
            try:
                connection, address = server.accept()
                print(f"[NET] Da ket noi: {address}")
            except socket.timeout:
                continue

        if connection is None:
            return

        connection.settimeout(SOCKET_POLL_TIMEOUT)

        frame_count = 0
        started = time.monotonic()
        last_printed_command = None

        while not stop_event.is_set():
            try:
                raw_command = connection.recv(1024)
            except socket.timeout:
                # Khong gui frame khi khong co command, tranh lech protocol.
                continue
            except (ConnectionResetError, OSError) as error:
                print(f"[NET] Mat ket noi: {error}")
                break

            if raw_command == b"":
                print("[NET] patrol_robot.py da dong ket noi")
                break

            try:
                steer, speed, command_text = parse_ai_command(raw_command)
            except ValueError as error:
                print(f"[NET] Lenh khong hop le: {error}")
                ai_command.invalidate()
                continue

            ai_command.set(steer, speed)
            if command_text != last_printed_command:
                print(f"[CMD] steer={steer:.1f} speed={speed:.1f}")
                last_printed_command = command_text

            try:
                blocked_now = safety.snapshot()["state"] != "GO"
                frame_data = encode_frame(
                    camera, imu_status=imu_status, blocked=blocked_now
                )
                connection.sendall(frame_data)
            except (BrokenPipeError, ConnectionResetError, socket.timeout, OSError) as error:
                print(f"[NET] Gui frame loi: {error}")
                break

            frame_count += 1
            if frame_count % 100 == 0:
                elapsed = max(time.monotonic() - started, 0.001)
                snapshot = safety.snapshot()
                print(
                    f"[NET] FPS={frame_count / elapsed:.1f} "
                    f"frames={frame_count} LIDAR={snapshot['state']} "
                    f"reason={snapshot['reason']}"
                )

    finally:
        ai_command.invalidate()
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass
        if server is not None:
            try:
                server.close()
            except Exception:
                pass


def main():
    # Ngưỡng LiDAR quyết định xe có được đi hay không, nên phải nhìn thấy
    # ngay lúc khởi động - không bao giờ để chạy đường thật mà tưởng đang
    # ở ngưỡng production.
    if LANE_TEST_NO_LIDAR:
        print("!" * 62, flush=True)
        print("  THU BAM LANE TREN MAT DAT - LIDAR VA NE VAT DANG BI BO QUA", flush=True)
        print("  Chi chay tren doan duong trong; nguoi van hanh san sang STOP.", flush=True)
        print("!" * 62, flush=True)
    elif BENCH_MODE:
        print("=" * 62, flush=True)
        print("  CHE DO BENCH (AGV_BENCH=1) - CHI DUNG KHI XE KE TREN GIA", flush=True)
        print(f"  LiDAR ha xuong {LIDAR_STOP_DIST_M:.2f}m / "
              f"{LIDAR_RESUME_DIST_M:.2f}m, ne vat DA TAT.", flush=True)
        print("  KHONG chay tren duong that o che do nay.", flush=True)
        print("=" * 62, flush=True)
    else:
        print(f"[MODE] Production - LiDAR STOP {LIDAR_STOP_DIST_M:.2f}m / "
              f"RESUME {LIDAR_RESUME_DIST_M:.2f}m, ne vat BAT.", flush=True)

    camera = None
    stm32 = None
    sender_thread = None
    reader_thread = None
    lidar_thread = None

    stop_event = threading.Event()
    ai_command = SharedCommand()
    safety = LidarSafety()
    imu_status = ImuStatus()
    lidar_worker = LidarWorker(safety, stop_event)

    try:
        camera = open_camera()
        stm32 = open_stm32()

        sender_thread = threading.Thread(
            target=sender_thread_fn,
            args=(stm32, ai_command, safety, stop_event),
            daemon=True,
        )
        reader_thread = threading.Thread(
            target=stm32_reader_fn,
            args=(stm32, stop_event, imu_status, ai_command),
            daemon=True,
        )
        if NO_LIDAR:
            # Ghim GO TRUOC khi sender chay, neu khong sender se doc trang
            # thai khoi tao "STOP/startup" va gui STOP ngay khung dau.
            bypass_reason = (
                "run lane, thu bam vach tren duong trong"
                if LANE_TEST_NO_LIDAR else
                "--no-lidar, xe ke tren gia"
            )
            safety.bypass(bypass_reason)
        else:
            lidar_thread = threading.Thread(
                target=lidar_worker.run,
                daemon=True,
            )

        sender_thread.start()
        reader_thread.start()
        if lidar_thread is not None:
            lidar_thread.start()

        run_socket_server(camera, ai_command, safety, stop_event, imu_status)

    except KeyboardInterrupt:
        print("\n[BRIDGE] Ctrl+C - E-STOP", flush=True)
    except Exception as error:
        print(f"[BRIDGE] LOI: {type(error).__name__}: {error}", flush=True)
    finally:
        print("[BRIDGE] Dang shutdown...", flush=True)
        ai_command.invalidate()
        # Go bypass truoc: dang bypass thi force_stop bi lam vo hieu, ma luc
        # shutdown thi ta THUC SU muon dung.
        safety.clear_bypass()
        safety.force_stop("bridge shutdown")
        stop_event.set()

        # Dong LiDAR de danh thuc iter_scans() neu dang block.
        lidar_worker.shutdown()

        if sender_thread is not None:
            sender_thread.join(timeout=2.0)
        if lidar_thread is not None:
            lidar_thread.join(timeout=2.0)
        if reader_thread is not None:
            reader_thread.join(timeout=1.0)

        if stm32 is not None:
            try:
                stm32.close()
            except Exception:
                pass
        if camera is not None:
            camera.release()

        print("[BRIDGE] Da dong camera/socket/serial/LiDAR", flush=True)


if __name__ == "__main__":
    main()
