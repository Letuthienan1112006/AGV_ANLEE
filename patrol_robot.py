"""
patrol_robot.py — Vision pipeline cho AGV Patrol Robot
============================================================
Đã refactor để dùng Confg.py — tất cả constants từ Confg.

Chạy:
    Terminal 1: python3 fake_car_socket.py   # hoặc kết nối với STM32
    Terminal 2: python3 patrol_robot.py
"""

import socket
import math
import cv2
import numpy as np
import time
import json
import base64
import os
import threading
import queue
import logging
from contextlib import ExitStack
from datetime import datetime
from collections import deque

import requests
import paho.mqtt.client as mqtt

from lane_control import LanePID, LaneTrackingGuard
from manual_control import ManualDriver, RawKeyboard
from lane_geometry import select_line_segment

try:
    from secrets_local import DISCORD_WEBHOOK_URL
except ImportError:
    DISCORD_WEBHOOK_URL = None
    print("[DISCORD] secrets_local.py khong ton tai - tat canh bao Discord")

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  IMPORT TẤT CẢ CẤU HÌNH TỪ Confg.py
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
from Confg import (
    # Network
    SOCKET_IP, SOCKET_PORT,
    CAMERA_WARMUP_FRAMES,
    PERSON_DEVICE,
    MQTT_BROKER_IP, MQTT_PORT, MQTT_TOPIC_ALERT, MQTT_TOPIC_STATUS,
    MQTT_CLIENT_ID,
    # Model paths
    SEGMENTATION_MODEL_PATH, PERSON_MODEL_PATH,
    UNIFIED_MODEL_PATH, UNIFIED_ENGINE_PATH,
    UNIFIED_INFERENCE_SIZE, UNIFIED_CONF_THRESHOLD,
    UNIFIED_LANE_CONF_THRESHOLD,
    SEG_INPUT_SIZE, SEG_NORMALIZE_MEAN, SEG_NORMALIZE_STD,
    SEG_NUM_CLASSES, SEG_BACKBONE, CLASS_COLORS,
    # Vision
    PERSON_CONF_THRESHOLD, PERSON_MIN_AREA, PERSON_INFERENCE_SIZE,
    PERSON_CONFIRM_FRAMES, PERSON_CONFIRM_RATIO, PERSON_CLASS_ID,
    VEHICLE_CLASS_ID,
    YOLO_EVERY_N_FRAMES,
    # Control
    BASE_SPEED, MIN_SPEED, MAX_STEER, STEER_CENTER_X, STEER_TRIM,
    MANUAL_SPEED_STEP, MANUAL_STEER_STEP, MANUAL_MAX_SPEED,
    MANUAL_MAX_STEER,
    MANUAL_IDLE_STOP_SEC, MANUAL_STEER_RETURN_SEC,
    RECOVERY_ERROR_THRESHOLD, RECOVERY_SPEED,
    LANE_LOST_MAX_FRAMES, LANE_REACQUIRE_FRAMES,
    PID_KP, PID_KI, PID_KD, PID_MAX_OUTPUT,
    PID_INTEGRAL_LIMIT, PID_STEP_LIMIT,
    PID_RESET_INTEGRAL_ON_SIGN_CROSS,
    PID_INTEGRAL_LEAK_TAU,
    LANE_ERROR_SMOOTHING_ALPHA,
    LANE_LOST_STEER_DECAY,
    ADAPTIVE_ERROR_NORM, ADAPTIVE_POWER,
    ADAPTIVE_HEAVY_THRESHOLD, ADAPTIVE_SEVERE_THRESHOLD,
    SLOPE_BOOST_LIGHT, SLOPE_BOOST_MEDIUM, SLOPE_BOOST_HEAVY,
    # Navigation
    PATROL_LOCATIONS, LOCATION_INTERVAL,
    ARUCO_MIN_AREA, ARUCO_CONFIRM_FRAMES, ARUCO_COMMANDS,
    STOP_DURATION,
    INTERSECTION_SCAN_Y, INTERSECTION_DISPLAY_SCAN_Y, LINE_SCAN_BAND,
    LINE_MIN_PIXELS,
    HEADING_SCAN_Y_FAR, HEADING_ERROR_WEIGHT,
    USE_STANLEY_CONTROLLER, FIT_ROI_MARGIN, FIT_MIN_PIXELS,
    STANLEY_K, STANLEY_MIN_SPEED, STANLEY_STEER_GAIN,
    INTERSECTION_SHIFT_THRESHOLD, INTERSECTION_STABLE_FRAMES,
    INTERSECTION_MAX_FRAMES, INTERSECTION_TIMEOUT_FRAMES,
    SAFE_DIST_FRAMES_AFTER_SIGN,
    LANE_STABLE_OFFSET, LANE_STABLE_WIDTH,
    STABILITY_REQUIRED_FRAMES, STABILITY_CONFIDENCE_THR,
    STABILITY_AGREEMENT_RATIO,
    # Alert
    ALERT_COOLDOWN_SEC, SAVE_ALERT_IMAGE, ALERT_IMAGE_DIR,
    STATUS_INTERVAL_SEC,
    # Debug
    SHOW_DEBUG_WINDOW, DEBUG_HUD_FONT_SCALE, DEBUG_HUD_LINE_HEIGHT,
    # Helper
    _get_aruco_dict,
)

# Cho phep A/B mot engine moi bang `AGV_UNIFIED_ENGINE_PATH=... ./run.sh dry`
# ma khong doi cau hinh production. Chi sau khi benchmark va so mask dat moi
# cap nhat Confg.py sang engine moi.
UNIFIED_ENGINE_PATH = os.environ.get(
    "AGV_UNIFIED_ENGINE_PATH", UNIFIED_ENGINE_PATH)

# Quyết định backend TRƯỚC khi import framework cũ. Run 103220 mất
# 300-390 ms/inference trong patrol, trong khi cùng TensorRT runtime chạy
# riêng chỉ 115 ms/khung (8.69 FPS). Pipeline YOLO26 không dùng torch,
# torchvision, PIL, Ultralytics hay DeepLab; nạp chúng vẫn tạo CUDA context,
# chiếm bộ nhớ hợp nhất và còn chuyển BGR->PIL vô ích mỗi khung.
USE_UNIFIED_YOLO26 = os.environ.get("AGV_USE_YOLO26_UNIFIED", "1") not in ("", "0")

# `run control`: nguoi lai bang WASD, moi thu khac giu nguyen - camera,
# suy luan YOLO26, canh bao nguoi/xe, telemetry, video, va lop an toan
# LiDAR trong bridge. Chi cap (steer, speed) cuoi cung la doi nguon: tu
# ban phim thay vi lane PID. Duong tu dong KHONG bi sua gi.
MANUAL_CONTROL = os.environ.get("AGV_MANUAL_CONTROL", "") not in ("", "0")

torch = None
transforms = None
Image = None
create_deeplabv3 = None
YOLO = None
if not USE_UNIFIED_YOLO26:
    import torch
    import torchvision
    from torchvision import transforms
    from PIL import Image
    from ultralytics import YOLO
    from model import create_deeplabv3

    # torchvision 0.11.1 tren JetPack 4.6 khong co CUDA NMS. Legacy YOLO
    # forward van chay GPU, chi dua NMS nho ve CPU.
    _orig_nms = torchvision.ops.nms

    def _nms_cpu_fallback(boxes, scores, iou_threshold):
        if boxes.is_cuda:
            keep = _orig_nms(
                boxes.detach().cpu(), scores.detach().cpu(), iou_threshold
            )
            return keep.to(boxes.device)
        return _orig_nms(boxes, scores, iou_threshold)

    torchvision.ops.nms = _nms_cpu_fallback

    # Confg chi noi MUON chay o dau; quyet dinh cuoi cung xet CUDA co that.
    if PERSON_DEVICE != "cpu" and not torch.cuda.is_available():
        print("[INIT] Khong thay CUDA - YOLO se chay tren CPU")
        PERSON_DEVICE = "cpu"
    torch.backends.cudnn.benchmark = True

logging.getLogger("ultralytics").setLevel(logging.ERROR)


def recv_json_frame(sock, max_bytes=2_000_000):
    """Nhan tron ven mot JSON frame tu Bridge."""
    data = bytearray()

    while len(data) < max_bytes:
        chunk = sock.recv(65536)

        if not chunk:
            raise ConnectionError("Bridge da dong ket noi")

        data.extend(chunk)

        try:
            return json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue

    raise RuntimeError(
        f"JSON frame vuot qua gioi han {max_bytes} bytes"
    )


# ============================================================
#  KHỞI TẠO ArUco detector (dùng chung)
# ============================================================
aruco_dict     = _get_aruco_dict()

# Compatible with both old and new OpenCV ArUco APIs
if hasattr(cv2.aruco, "DetectorParameters_create"):
    aruco_params = cv2.aruco.DetectorParameters_create()
else:
    aruco_params = cv2.aruco.DetectorParameters()

if hasattr(cv2.aruco, "ArucoDetector"):
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
else:
    aruco_detector = None


# ============================================================
#  MQTT CLIENT
# ============================================================

def send_discord_alert(image_bytes, persons, location):
    """Gui canh bao (anh + text) qua Discord webhook. Luon duoc goi
    tu thread nen (xem send_alert) nen khong chan luong chinh."""
    if not DISCORD_WEBHOOK_URL:
        return
    try:
        confidence = max(p["confidence"] for p in persons)
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        content = (
            "CANH BAO: phat hien " + summarize_detections(persons) +
            " tai '" + location + "'\n" +
            "Tin cay cao nhat: " + format(confidence, ".0%") +
            " - " + ts
        )
        resp = requests.post(
            DISCORD_WEBHOOK_URL,
            data={"content": content},
            files={"file": ("alert.jpg", image_bytes, "image/jpeg")},
            timeout=5,
        )
        if resp.status_code not in (200, 204):
            print("[DISCORD] Loi gui:", resp.status_code, resp.text[:200])
    except Exception as error:
        print("[DISCORD] Loi gui:", error)


def send_discord_lift_alert(image_bytes, location, lift_since):
    """Gui canh bao xe bi nhac len khoi mat dat qua Discord webhook."""
    if not DISCORD_WEBHOOK_URL:
        return
    try:
        content = (
            "CANH BAO: XE BI NHAC LEN KHOI MAT DAT!\n"
            "Vi tri: '" + location + "'\n"
            "Thoi diem nhac len: " + lift_since
        )
        data = {"content": content}
        files = None
        if image_bytes is not None:
            files = {"file": ("lift_alert.jpg", image_bytes, "image/jpeg")}
        resp = requests.post(
            DISCORD_WEBHOOK_URL, data=data, files=files, timeout=5,
        )
        if resp.status_code not in (200, 204):
            print(
                "[DISCORD] Loi gui canh bao nhac len:",
                resp.status_code, resp.text[:200],
            )
    except Exception as error:
        print("[DISCORD] Loi gui canh bao nhac len:", error)


class MQTTClient:
    def __init__(self):
        self.client    = mqtt.Client(client_id=MQTT_CLIENT_ID)
        self.connected = False
        self.client.on_connect    = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self._connect()

    def _connect(self):
        try:
            if not hasattr(self.client, "connect_async"):
                print("[MQTT] Khong co connect_async - chay offline")
                return
            self.client.connect_async(
                MQTT_BROKER_IP, MQTT_PORT, keepalive=60
            )
            self.client.loop_start()
            print(f"[MQTT] Dang ket noi nen toi {MQTT_BROKER_IP}:{MQTT_PORT}")
        except Exception as error:
            print(f"[MQTT] Khong khoi dong duoc: {error} - chay offline")

    def _on_connect(self, client, userdata, flags, rc):
        self.connected = (rc == 0)
        print("[MQTT] Kết nối thành công!" if rc == 0
              else f"[MQTT] Lỗi kết nối rc={rc}")

    def _on_disconnect(self, client, userdata, rc):
        self.connected = False
        print("[MQTT] Mất kết nối, đang thử lại...")

    def publish(self, topic: str, payload: dict) -> bool:
        if not self.connected:
            return False
        try:
            self.client.publish(topic, json.dumps(payload), qos=1)
            return True
        except Exception as e:
            print(f"[MQTT] Lỗi publish: {e}")
            return False

    def send_alert(self, frame: np.ndarray, persons: list, location: str):
        frame_copy = frame.copy()
        for p in persons:
            x1, y1, x2, y2 = map(int, p["bbox"])
            cv2.rectangle(frame_copy, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(frame_copy, f"{p['label']} {p['confidence']:.0%}",
                        (x1, max(y1 - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

        if SAVE_ALERT_IMAGE:
            ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(ALERT_IMAGE_DIR, f"alert_{ts}.jpg")
            cv2.imwrite(path, frame_copy)

        _, buf  = cv2.imencode(".jpg", frame_copy, [cv2.IMWRITE_JPEG_QUALITY, 75])
        img_b64 = base64.b64encode(buf).decode()

        payload = {
            "event":      "intruder_detected",
            "timestamp":  datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "location":   location,
            "count":      len(persons),
            "confidence": round(max(p["confidence"] for p in persons), 3),
            "image":      img_b64,
        }
        if self.publish(MQTT_TOPIC_ALERT, payload):
            print(f"[ALERT] Đã gửi: {summarize_detections(persons)} tại '{location}'")

        send_discord_alert(buf.tobytes(), persons, location)

    def send_status(self, data: dict):
        self.publish(MQTT_TOPIC_STATUS, {
            "event":     "status",
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            **data,
        })

    def send_lift_alert(self, location: str, lift_since: str):
        payload = {
            "event":      "robot_lifted",
            "timestamp":  datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "location":   location,
            "lift_since": lift_since,
        }
        if self.publish(MQTT_TOPIC_ALERT, payload):
            print(f"[ALERT] Da gui: XE BI NHAC LEN tai '{location}'")

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()


# ============================================================
#  ARUCO MARKER DETECTION
# ============================================================

class ArucoCommandFilter:
    """Lọc nhiễu: chỉ nhận lệnh khi thấy cùng 1 marker N frame liên tiếp."""
    def __init__(self):
        self.buffer      = deque(maxlen=ARUCO_CONFIRM_FRAMES)
        self.last_cmd    = None
        self.cmd_sent    = False
        self.missing_cnt = 0

    def update(self, detected_id):
        self.buffer.append(detected_id)

        if detected_id is None:
            self.missing_cnt += 1
            if self.missing_cnt >= ARUCO_CONFIRM_FRAMES:
                self.cmd_sent    = False
                self.last_cmd    = None
                self.missing_cnt = 0
            return None

        self.missing_cnt = 0

        if len(self.buffer) < ARUCO_CONFIRM_FRAMES:
            return None

        ids_in_buf = [x for x in self.buffer if x is not None]
        if len(ids_in_buf) < ARUCO_CONFIRM_FRAMES:
            return None

        most_common_id = max(set(ids_in_buf), key=ids_in_buf.count)
        if ids_in_buf.count(most_common_id) < ARUCO_CONFIRM_FRAMES * 0.8:
            return None

        cmd = ARUCO_COMMANDS.get(most_common_id)
        if cmd is None:
            return None

        if self.cmd_sent and self.last_cmd == cmd:
            return None

        self.cmd_sent = True
        self.last_cmd = cmd
        print(f"[ARUCO] Marker ID={most_common_id} → lệnh: {cmd}")
        return cmd

    @property
    def is_seeing_marker(self):
        return any(x is not None for x in self.buffer)

    def reset(self):
        self.buffer.clear()
        self.last_cmd    = None
        self.cmd_sent    = False
        self.missing_cnt = 0


def detect_aruco(frame):
    """Detect ArUco marker lớn nhất trong frame."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if aruco_detector is not None:
        corners, ids, _ = aruco_detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is None or len(ids) == 0:
        return None, None

    best_id = None
    best_area = 0
    best_corners = None

    for i, corner in enumerate(corners):
        pts = corner[0]
        area = cv2.contourArea(pts)
        if area > best_area and area > ARUCO_MIN_AREA:
            best_area = area
            best_id = int(ids[i][0])
            best_corners = corner

    return best_id, best_corners


# ============================================================
#  PHÁT HIỆN NGƯỜI
# ============================================================

class IntruderConfirmFilter:
    def __init__(self):
        self.buffer        = deque(maxlen=PERSON_CONFIRM_FRAMES)
        self.last_alert_ts = 0.0
        self.alert_count   = 0

    def update(self, detected):
        self.buffer.append(detected)
        if len(self.buffer) < PERSON_CONFIRM_FRAMES:
            return False
        if sum(self.buffer) / PERSON_CONFIRM_FRAMES < PERSON_CONFIRM_RATIO:
            return False
        now = time.time()
        if now - self.last_alert_ts < ALERT_COOLDOWN_SEC:
            return False
        self.last_alert_ts = now
        self.alert_count  += 1
        return True

    @property
    def is_detecting(self):
        return bool(self.buffer) and sum(self.buffer) > 0


def detect_persons(yolo_model, frame):
    """Phat hien nguoi VA xe (COCO person=0, car=2) de canh bao.
    Ten "detect_persons"/"persons" giu nguyen de khong phai doi ten o
    nhieu noi khac trong file, nhung tu day co the la nguoi hoac xe -
    xem field "label" cua tung phan tu tra ve."""
    results = yolo_model(
        frame,
        imgsz=PERSON_INFERENCE_SIZE,
        conf=PERSON_CONF_THRESHOLD,
        classes=[PERSON_CLASS_ID, VEHICLE_CLASS_ID],
        verbose=False,
        device=PERSON_DEVICE,
    )
    persons = []
    for r in results:
        if r.boxes is None:
            continue
        for box in r.boxes:
            bbox = box.xyxy[0].cpu().numpy()
            area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
            if area < PERSON_MIN_AREA:
                continue
            cls_id = int(box.cls[0])
            label = "Nguoi" if cls_id == PERSON_CLASS_ID else "Xe"
            persons.append({
                "bbox":       bbox,
                "confidence": float(box.conf[0]),
                "area":       area,
                "class_id":   cls_id,
                "label":      label,
            })
    return persons


def summarize_detections(persons):
    """'2 nguoi, 1 xe' tu danh sach detection (co ca nguoi lan xe)."""
    counts = {}
    for p in persons:
        label = p.get("label", "Nguoi")
        counts[label] = counts.get(label, 0) + 1
    parts = []
    if counts.get("Nguoi"):
        parts.append(f"{counts['Nguoi']} nguoi")
    if counts.get("Xe"):
        parts.append(f"{counts['Xe']} xe")
    return ", ".join(parts) if parts else "0 doi tuong"


# ============================================================
#  PATROL LOCATION TRACKER
# ============================================================

class PatrolLocationTracker:
    def __init__(self):
        self.start_time = time.time()

    @property
    def current(self):
        elapsed = time.time() - self.start_time
        idx = int(elapsed / LOCATION_INTERVAL) % len(PATROL_LOCATIONS)
        return PATROL_LOCATIONS[idx]


# ============================================================
#  LANE FOLLOWING
# ============================================================

_SEGMAP_LUT = None

def decode_segmap(mask):
    # 2026-09-11: ban cu quet TOAN BO mang mot luot cho MOI lop (6 luot).
    # Tra cuu bang mau 1 luot cho ket qua y het nhung re hon nhieu tren
    # Jetson Nano - overlay debug chay moi khung nen no an vao toc do vong
    # lap, ma vong lap cham chinh la thu sinh ra dao dong.
    global _SEGMAP_LUT
    if _SEGMAP_LUT is None:
        n = max(CLASS_COLORS) + 1
        _SEGMAP_LUT = np.zeros((n, 3), dtype=np.uint8)
        for class_id, color in CLASS_COLORS.items():
            _SEGMAP_LUT[class_id] = color
    return _SEGMAP_LUT[np.clip(mask, 0, len(_SEGMAP_LUT) - 1)]


def scan_lane(mask, y=None, reference_x=None):
    """Nhu get_lane_midpoint() nhung tra ve THEM co from_line cho biet ket
    qua den tu VACH THAT (class 2) hay chi la fallback ve vung road.

    2026-09-09: bat buoc phai phan biet duoc 2 truong hop nay. Do da tren
    duong that: class "road" phu GAN HET chieu rong khung hinh (duong khong
    co le/curb trong tam camera), nen midpoint cua road LUON ~= giua anh
    (319/320) bat ke xe dang lech the nao - tuc la fallback road KHONG mang
    thong tin vi tri, chi la "khong do duoc". Truoc day cac ham goi khong
    phan biet duoc nen coi no nhu 1 phep do that -> gay 2 loi nang (xem
    get_heading_error va vong lap chinh).

    Tra ve (left, right, mid, from_line) hoac None neu khong thay gi.
    """
    if y is None:
        y = INTERSECTION_SCAN_Y
    h = mask.shape[0]
    if y >= h:
        y = h - 1

    # Uu tien dung vach line (class 2) vi chinh xac hon vung road
    # rong (class 1). Vach thuong la net dut -> dung 1 hang y don co
    # the roi dung khoang trong giua 2 net dut, mat tin hieu chinh xac
    # va fallback ve road (khong nhay theo lech that cua xe), gay
    # troi tich luy. Quet 1 dai hang quanh y (LINE_SCAN_BAND) va uu
    # tien hang GAN y nhat co vach, de "nhin" qua khoang trong ma van
    # bam theo vach lien tuc; chi that su fallback road khi ca dai
    # deu khong thay vach.
    y0 = max(0, y - LINE_SCAN_BAND)
    y1 = min(h, y + LINE_SCAN_BAND + 1)
    # Run 20260916_103220 showed the near midpoint barely moving (369->371)
    # while heading jumped -11->+84 px. The old code took the FIRST and LAST
    # line pixel across every detached blob in a row, so the real stripe and
    # a cyan false-positive at the right edge became one fake stripe. Inspect
    # rows nearest to y and choose exactly one contiguous component instead.
    row_order = sorted(range(y0, y1), key=lambda row: (abs(row - y), row))
    for row in row_order:
        segment = select_line_segment(
            mask[row, :], LINE_MIN_PIXELS, reference_x=reference_x,
        )
        if segment is not None:
            left, right, mid = segment
            return left, right, mid, True

    lane_indices = np.where(mask[y, :] == 1)[0]
    if len(lane_indices) < 2:
        return None
    left = lane_indices[0]
    right = lane_indices[-1]
    return left, right, (left + right) // 2, False


def get_lane_midpoint(mask, y=None):
    """Ban rut gon cua scan_lane() - giu nguyen chu ky 3 gia tri cho cac
    cho chi can toa do (ve debug...). Cho nao can biet nguon goc ket qua
    (vach that hay fallback road) thi goi thang scan_lane()."""
    res = scan_lane(mask, y)
    if res is None:
        return None
    left, right, mid, _from_line = res
    return left, right, mid


def get_heading_error(mask, near_y, near_mx):
    """Quet them 1 diem o hang xa hon (near_y - HEADING_SCAN_Y_FAR nam
    phia tren trong anh), noi voi diem gan (near_mx tai near_y) thanh 1
    duong thang, tra ve do lech GOC (pixel-tuong-duong, da nhan
    HEADING_ERROR_WEIGHT) so voi phuong thang dung (di thang).

    Duong > 0 nghia la diem xa lech ve BEN PHAI so voi diem gan (huong
    duong dang re/nghieng phai) - cung dau voi lane_error hien co (mx -
    center_x duong = lech phai) nen cong truc tiep duoc, khong can doi dau.

    Tra ve 0 neu khong tim thay diem xa (giu nguyen lane_error cu, khong
    anh huong gi - dung cho ca truong hop mat doan giua vi 2 hang duoc
    quet doc lap voi nhau).

    2026-09-09 SUA LOI NANG: truoc day dung get_lane_midpoint() cho diem
    xa, ma ham do khi khong thay vach se FALLBACK ve road -> tra ve giua
    vung road (~319 = giua anh) chu khong tra None. Vi road phu het khung
    tren duong that, nhanh "if far_points is None: return 0.0" gan nhu
    KHONG BAO GIO chay, va diem xa BIA RA (giua anh) duoc dung nhu that.
    Hau qua do duoc bang mo phong dung tinh huong that: xe lech phai 71px
    (lane_error=-71), heading bia ra +75.7px -> lane_error cuoi = +4.7px,
    tuc la TRIET TIEU HOAN TOAN loi that va con doi dau -> PID khong be
    lai gi ca. Day chinh la trieu chung "xe lech ma khong thay keo lai".
    Gio chi dung diem xa khi no den tu VACH THAT (from_line=True)."""
    # Associate the far observation with the near stripe. Without this,
    # another detached line-like patch can flip heading by ~100 px in one
    # frame even though the near midpoint has not moved.
    far_points = scan_lane(mask, HEADING_SCAN_Y_FAR, reference_x=near_mx)
    if far_points is None:
        return 0.0
    _, _, far_mx, far_from_line = far_points
    if not far_from_line:
        return 0.0

    dx = float(far_mx - near_mx)
    dy = float(near_y - HEADING_SCAN_Y_FAR)
    if dy <= 0:
        return 0.0

    angle_deg = math.degrees(math.atan2(dx, dy))
    return angle_deg * HEADING_ERROR_WEIGHT


def get_lane_center_robust(mask, near_y, far_y, center_x,
                             roi_margin=None, min_pixels=None):
    """2026-09-05: thay the cho get_lane_midpoint()+get_heading_error()
    dung chung khi USE_STANLEY_CONTROLLER=True. Fit 1 duong thang x=m*y+b
    qua TOAN BO pixel class 2 (line) trong dai ROI quanh ca near_y va
    far_y - dam bao near/far dung CHUNG 1 he quy chieu (khac voi cach cu:
    2 lan goi get_lane_midpoint doc lap, co the 1 lan bat duoc line con
    1 lan roi ve road fallback, gay lech gia). Dai ROI cung tu nhien bam
    duoc qua khoang trong giua 2 net dut (bat pixel tu dash lan can).

    Fallback ve class road (class 1) tai near_y neu qua it pixel line
    trong ROI - giu logic tuong tu ban cu, heading=0.0 khi phai fallback.

    Tra ve (cross_track_error_px, heading_error_deg, near_mx) hoac None
    neu ca line va road deu khong thay gi."""
    if roi_margin is None:
        roi_margin = FIT_ROI_MARGIN
    if min_pixels is None:
        min_pixels = FIT_MIN_PIXELS
    h = mask.shape[0]
    if near_y >= h:
        near_y = h - 1
    if far_y >= h:
        far_y = h - 1
    y0 = max(0, min(near_y, far_y) - roi_margin)
    y1 = min(h, max(near_y, far_y) + roi_margin + 1)

    ys, xs = np.where(mask[y0:y1, :] == 2)
    if len(xs) >= min_pixels:
        ys_abs = ys.astype(np.float64) + y0
        m, b = np.polyfit(ys_abs, xs.astype(np.float64), 1)
        near_mx = m * near_y + b
        cross_track_error = near_mx - center_x
        heading_error_deg = math.degrees(math.atan(m))
        return cross_track_error, heading_error_deg, near_mx, True

    lane_indices = np.where(mask[near_y, :] == 1)[0]
    if len(lane_indices) < 2:
        return None
    left, right = lane_indices[0], lane_indices[-1]
    near_mx = (left + right) / 2.0
    # 2026-09-09: tra ve them from_line=False. Gia tri nay lay tu vung ROAD
    # (phu het khung -> tam luon ~= giua anh), tuc la "khong do duoc" chu
    # khong phai "xe dang o giua". Ben goi PHAI phan biet, neu khong se
    # dinh dung loi da gap o duong PID (xem get_heading_error).
    return near_mx - center_x, 0.0, near_mx, False


def stanley_steer(cross_track_error_px, heading_error_deg, speed_proxy,
                    k=None, min_speed=None):
    """Stanley Controller dang pixel-space (xem ghi chu STANLEY_K trong
    Confg.py - k/gain la uoc luong, CAN TUNE THUC TE tren xe that):
        steer_deg = heading_error_deg + atan(k * cross_track_error / v)
    speed_proxy: dung car_speed (thang raw 0-106, truoc bridge clamp)
    lam proxy van toc - khong phai m/s that."""
    if k is None:
        k = STANLEY_K
    if min_speed is None:
        min_speed = STANLEY_MIN_SPEED
    v = max(speed_proxy, min_speed)
    cross_term_deg = math.degrees(math.atan2(k * cross_track_error_px, v))
    return heading_error_deg + cross_term_deg


def stanley_to_steer_command(stanley_deg, max_steer=None, gain=None):
    """Doi output do cua Stanley sang thang lenh STEER cua firmware."""
    if max_steer is None:
        max_steer = MAX_STEER
    if gain is None:
        gain = STANLEY_STEER_GAIN
    return float(np.clip(stanley_deg * gain, -max_steer, max_steer))


def detect_intersection_points(mask, scan_y=None):
    if scan_y is None:
        scan_y = INTERSECTION_SCAN_Y
    height, width = mask.shape
    info = {
        "center_left": None, "center": None, "center_right": None,
        "left": None, "right": None,
        "is_intersection": False, "lane_count": 0,
        "should_turn_left": False, "should_turn_right": False,
        "left_shift": 0, "right_shift": 0, "confidence": 0.0,
    }
    scan_y      = min(scan_y, height - 1)
    lane_pixels = np.where(mask[scan_y, :] == 1)[0]
    if len(lane_pixels) < 10:
        return info

    ll = lane_pixels[0]; lr = lane_pixels[-1]; lw = lr - ll

    scan_points = {
        "center":       width // 2,
        "center_left":  ll + lw // 4,
        "center_right": ll + 3 * lw // 4,
        "left":         ll,
        "right":        lr,
    }

    if not hasattr(detect_intersection_points, "initial_left"):
        detect_intersection_points.initial_left  = ll
        detect_intersection_points.initial_right = lr

    for name, x_pos in scan_points.items():
        if not (0 <= x_pos < width):
            info[name] = {"detected": False}
            continue
        if name in ("left", "right"):
            info[name] = {
                "detected": bool(mask[scan_y, x_pos] == 1),
                "center": x_pos, "start": x_pos, "end": x_pos, "width": 1,
            }
        else:
            x0 = max(0, x_pos - 30)
            x1 = min(width, x_pos + 30)
            seg = np.where(mask[scan_y, x0:x1] == 1)[0]
            if len(seg) > 10:
                s = seg[0] + x0; e = seg[-1] + x0
                info[name] = {"detected": True, "start": s, "end": e,
                              "center": (s + e) // 2, "width": e - s}
            else:
                info[name] = {"detected": False}

    detected = sum(1 for v in info.values()
                   if isinstance(v, dict) and v.get("detected"))

    if info["left"].get("detected"):
        info["left_shift"]  = (info["left"]["center"]
                                - detect_intersection_points.initial_left)
    if info["right"].get("detected"):
        info["right_shift"] = (info["right"]["center"]
                                - detect_intersection_points.initial_right)

    thr = INTERSECTION_SHIFT_THRESHOLD * (
        1.3 if detected >= 4 else 1.1 if detected == 3 else 0.9
    )
    ls = abs(info["left_shift"]); rs = abs(info["right_shift"])
    conf = min(
        (0.3 if detected >= 3 else 0.0) +
        (0.4 if max(ls, rs) > thr else 0.0) +
        (0.3 if detected >= 4 and max(ls, rs) > thr * 1.2 else 0.0),
        1.0,
    )
    info["confidence"]        = conf
    info["should_turn_left"]  = (info["left_shift"]  >  thr
                                  and detected >= 3 and conf > 0.6)
    info["should_turn_right"] = (info["right_shift"] < -thr
                                  and detected >= 3 and conf > 0.6)
    info["lane_count"]        = detected
    info["is_intersection"]   = detected >= 3
    return info


def calculate_intersection_steering(intersection_info, traffic_sign_command):
    if not intersection_info["is_intersection"]:
        return None
    cx = STEER_CENTER_X
    if not traffic_sign_command:
        if intersection_info["should_turn_left"]:  return -15
        if intersection_info["should_turn_right"]: return  15
        cl = intersection_info.get("center")
        if cl and cl.get("detected"):
            return max(-5, min(5, (cl["center"] - cx) * 0.3))
        return 0

    cl = intersection_info.get("center")
    if not cl or not cl.get("detected"):
        return None
    cur = cl["center"]

    if traffic_sign_command == "turn_left":
        lft = intersection_info.get("left", {})
        tx  = (lft["center"] if lft.get("detected")
               else intersection_info.get("center_left", {}).get("center", cx - 80))
        return max(-15, min(-5, (tx - cx) * 0.4))

    if traffic_sign_command == "turn_right":
        rgt = intersection_info.get("right", {})
        tx  = (rgt["center"] if rgt.get("detected")
               else intersection_info.get("center_right", {}).get("center", cx + 80))
        return max(5, min(15, (tx - cx) * 0.4))

    if traffic_sign_command == "straight":
        return max(-8, min(8, (cur - cx) * 0.25))

    return None


# ============================================================
#  PID & TỐC ĐỘ
# ============================================================

class PIDController(LanePID):
    def __init__(self, kp=None, ki=None, kd=None, max_output=None):
        super().__init__(
            PID_KP if kp is None else kp,
            PID_KI if ki is None else ki,
            PID_KD if kd is None else kd,
            PID_MAX_OUTPUT if max_output is None else max_output,
            PID_INTEGRAL_LIMIT, PID_STEP_LIMIT,
            reset_integral_on_sign_cross=PID_RESET_INTEGRAL_ON_SIGN_CROSS,
            integral_leak_tau=PID_INTEGRAL_LEAK_TAU,
        )


def adaptive_speed_control(error, base_speed=None, min_speed=None):
    base_speed = base_speed if base_speed is not None else BASE_SPEED
    min_speed  = min_speed  if min_speed  is not None else MIN_SPEED
    factor = 1 - (abs(error) / ADAPTIVE_ERROR_NORM) ** ADAPTIVE_POWER
    if abs(error) > ADAPTIVE_HEAVY_THRESHOLD:  factor *= 0.8
    if abs(error) > ADAPTIVE_SEVERE_THRESHOLD: factor *= 0.7
    return int(max(base_speed * factor, min_speed))


def detect_steep_slope(lane_error, lane_points, prev_lane_points=None):
    boost = 1.0
    if abs(lane_error) > 100:  boost = SLOPE_BOOST_MEDIUM
    elif abs(lane_error) > 50: boost = SLOPE_BOOST_LIGHT
    if lane_points and prev_lane_points:
        try:
            pl, pr, pm = prev_lane_points
            cl, cr, cm = lane_points
            if abs(cm - pm) > 30 or abs((cr - cl) - (pr - pl)) > 20:
                boost = max(boost, SLOPE_BOOST_HEAVY)
        except (TypeError, ValueError):
            pass
    return boost


# ============================================================
#  LOC MEM LANE_ERROR (EMA) - giam rung giat do nhieu segmentation
# ============================================================

class ErrorSmoother:
    """Loc trung binh mu (EMA) cho lane_error truoc khi dua vao PID,
    giam rung giat khung-sang-khung ma khong lam tre phan ung nhieu
    nhu loc trung binh cua so cung. ALPHA=1.0 tuong duong khong loc."""

    def __init__(self, alpha=None):
        self.alpha = alpha if alpha is not None else LANE_ERROR_SMOOTHING_ALPHA
        self.value = None

    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value

    def decay(self, factor):
        """MAT VACH: khong co so do moi nao ca. Khong duoc bom lai so do CU
        vao update() - lam vay EMA se hoi tu len dung cai loi da cu, va steer
        con tang dan trong luc xe dang mu (do duoc 2026-09-10, khung 25-34:
        raw_err dong bang 163, steer bo 14 -> 18). Thay vao do keo loi ve 0
        de lenh lai tu nha ra, vi lenh lai cuoi cung chinh la lenh vua day
        vach ra khoi khung hinh."""
        if self.value is None:
            return 0.0
        self.value *= factor
        return self.value

    def reset(self):
        self.value = None


# ============================================================
#  BỘ LỌC ỔN ĐỊNH GIAO LỘ
# ============================================================

class StabilityFilter:
    def __init__(self, required_frames=None, confidence_threshold=None):
        self.required_frames = required_frames or STABILITY_REQUIRED_FRAMES
        self.confidence_threshold = confidence_threshold or STABILITY_CONFIDENCE_THR
        self.turn_left_buffer  = deque(maxlen=self.required_frames)
        self.turn_right_buffer = deque(maxlen=self.required_frames)
        self.confidence_buffer = deque(maxlen=self.required_frames)

    def update(self, should_turn_left, should_turn_right, confidence):
        self.turn_left_buffer.append(should_turn_left)
        self.turn_right_buffer.append(should_turn_right)
        self.confidence_buffer.append(confidence)

    def get_stable_decision(self):
        if len(self.turn_left_buffer) < self.required_frames:
            return False, False, 0.0
        avg_c = sum(self.confidence_buffer) / self.required_frames
        sl = (sum(self.turn_left_buffer)
              >= self.required_frames * STABILITY_AGREEMENT_RATIO
              and avg_c >= self.confidence_threshold)
        sr = (sum(self.turn_right_buffer)
              >= self.required_frames * STABILITY_AGREEMENT_RATIO
              and avg_c >= self.confidence_threshold)
        return sl, sr, avg_c

    def reset(self):
        self.turn_left_buffer.clear()
        self.turn_right_buffer.clear()
        self.confidence_buffer.clear()


SHOW_WINDOW = False  # False khi chạy qua SSH/headless

SHOW_WINDOW = False  # False khi chạy SSH/headless
# Cho phep benchmark phan dieu khien khong bi ma hoa MJPG chen vao. Mac dinh
# van ghi video de dieu tra; dat AGV_RECORD_VIDEO=0 chi khi can do/tiet kiem CPU.
RECORD_VIDEO = os.environ.get("AGV_RECORD_VIDEO", "1") not in ("", "0")
# Full road+line alpha overlay is useful while validating a model, but it
# allocates/converts/blends an entire 640x360 RGB image every control frame.
# Normal driving only needs the line mask plus geometry/HUD in the recording.
# Set AGV_VIDEO_FULL_MASK=1 for the old, heavier diagnostic overlay.
VIDEO_FULL_MASK = os.environ.get("AGV_VIDEO_FULL_MASK", "0") not in ("", "0")
# Vong that sau FP16 + fast overlay co trung vi 8.55 FPS. Ghi 8 FPS de
# playback gan thoi gian that; 6 FPS cu lam clip cham hon chuyen dong ~40%.
VIDEO_FPS = 8
# Thu muc ghi ket qua. Mac dinh nam tren SSD, KHONG phai the nho he thong:
# 2026-09-10 do duoc the nho con 906MB (94% day) va moi khung mat ~800ms chi
# de ghi video - vong dieu khien tut tu ~14fps xuong 1.7fps, cham hon nhieu so
# voi muc can de bam lane. SSD con 411GB va gan nhu khong dung.
_RUN_OUT_DIR = os.environ.get("AGV_RUN_DIR", "/mnt/ssd/agv_runs/loose")
try:
    os.makedirs(_RUN_OUT_DIR, exist_ok=True)
except Exception as _error:
    print(f"[OUT] Khong tao duoc {_RUN_OUT_DIR}: {_error} - ghi vao thu muc hien tai")
    _RUN_OUT_DIR = "."

_RUN_STAMP = time.strftime("%Y%m%d_%H%M%S")
VIDEO_OUTPUT_PATH = os.path.join(
    _RUN_OUT_DIR, "patrol_record_" + _RUN_STAMP + ".avi"
)

# Log so lieu tung khung. Truoc day moi thong tin dieu khien chi nam tren
# HUD cua video, nen muon biet lane_error mot khung nao la phai nhin bang
# mat - kho ket luan, nhat la voi clip 2-3 giay. File nay cho phep do bang
# so: dem ty le LINE/HOLD, xem dau cua lane_error co khop huong lech that,
# va lane_error co giam khi steer da bao hoa hay khong.
TELEMETRY_PATH = os.path.join(
    _RUN_OUT_DIR, "patrol_telem_" + _RUN_STAMP + ".csv"
)
class AsyncVideoRecorder:
    """Encode debug video without stalling the steering loop.

    MJPG took about 21 ms synchronously on the Jetson.  A bounded queue keeps
    at most two completed display frames; when storage/encoding falls behind,
    the oldest debug frame is dropped instead of delaying a motor command.
    Video is diagnostic data, while control freshness is safety-critical.
    """

    def __init__(self, path, fps, frame_size):
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        self.writer = cv2.VideoWriter(path, fourcc, fps, frame_size)
        if not self.writer.isOpened():
            raise RuntimeError("Khong mo duoc VideoWriter: " + path)
        self.frames_written = 0
        self.frames_dropped = 0
        self.error = None
        self._queue = queue.Queue(maxsize=2)
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _worker(self):
        try:
            while True:
                frame = self._queue.get()
                if frame is None:
                    break
                self.writer.write(frame)
                self.frames_written += 1
        except Exception as error:
            self.error = error
        finally:
            self.writer.release()

    def submit(self, frame):
        if self.error is not None:
            return
        try:
            self._queue.put_nowait(frame)
            return
        except queue.Full:
            pass
        # Keep the newest visual evidence. The array is not mutated after
        # record_frame(), so passing ownership to the worker needs no copy.
        try:
            self._queue.get_nowait()
            self.frames_dropped += 1
        except queue.Empty:
            pass
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            self.frames_dropped += 1

    def close(self):
        # At most two frames can be ahead, so this bounded wait is short and
        # happens only after the vehicle loop has already stopped.
        if not self._thread.is_alive():
            return
        while True:
            try:
                self._queue.put_nowait(None)
                break
            except queue.Full:
                try:
                    self._queue.get_nowait()
                    self.frames_dropped += 1
                except queue.Empty:
                    pass
        self._thread.join(timeout=5.0)


video_recorder = None

def record_frame(frame):
    global video_recorder
    if not RECORD_VIDEO:
        return

    if video_recorder is None:
        h, w = frame.shape[:2]
        video_recorder = AsyncVideoRecorder(
            VIDEO_OUTPUT_PATH, VIDEO_FPS, (w, h))
        print(f"[REC] Recording video to: {VIDEO_OUTPUT_PATH}")

    video_recorder.submit(frame)


# ============================================================
#  LOAD MODELS
# ============================================================

img_transform = None
device = None
if not USE_UNIFIED_YOLO26:
    img_transform = transforms.Compose([
        transforms.Resize(SEG_INPUT_SIZE),
        transforms.ToTensor(),
        transforms.Normalize(SEG_NORMALIZE_MEAN, SEG_NORMALIZE_STD),
    ])
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

unified_model = None
seg_model = None
seg_use_fp16 = False
person_model = None

if USE_UNIFIED_YOLO26:
    print("=" * 60)
    print("[INIT] MOT MODEL: YOLO26-seg thay ca segmentation lan YOLO nguoi")
    print("[INIT]   backend mac dinh da xac nhan tren xe that")
    print("=" * 60)
    # 2026-09-16: KHONG chon backend bang try/except ImportError - Jetson
    # nay da co san ultralytics 8.0.20 (dung cho YOLOv8n nguoi/xe tu truoc
    # gio), qua cu de doc duoc checkpoint YOLO26 nhung van import THANH
    # CONG, nen except ImportError khong bao gio chay va no roi thang vao
    # nhanh ultralytics sai. Chon theo file nao THAT SU CO tren may nay:
    # laptop (dev/train) chi co .pt, Jetson chi deploy .engine (khong
    # bao gio dua ca .pt cho Jetson, vi ultralytics o day khong doc noi).
    if os.path.exists(UNIFIED_ENGINE_PATH):
        from yolo26_tensorrt_runtime import TensorRTUnifiedYOLO26
        print(f"[INIT]   backend: TensorRT, {UNIFIED_ENGINE_PATH}")
        unified_model = TensorRTUnifiedYOLO26(
            UNIFIED_ENGINE_PATH,
            conf_thres=UNIFIED_CONF_THRESHOLD,
            lane_conf_thres=UNIFIED_LANE_CONF_THRESHOLD,
            min_area=PERSON_MIN_AREA,
        )
    elif os.path.exists(UNIFIED_MODEL_PATH):
        from yolo26_unified import UnifiedYOLO26
        print(f"[INIT]   backend: ultralytics, {UNIFIED_MODEL_PATH}")
        unified_model = UnifiedYOLO26(
            UNIFIED_MODEL_PATH,
            device=PERSON_DEVICE,
            conf=UNIFIED_CONF_THRESHOLD,
            imgsz=UNIFIED_INFERENCE_SIZE,
            min_area=PERSON_MIN_AREA,
        )
    else:
        raise FileNotFoundError(
            "Khong thay model gop: ca {} lan {} deu khong ton tai".format(
                UNIFIED_ENGINE_PATH, UNIFIED_MODEL_PATH))
    # Cung ly do warmup nhu hai model cu: lan goi GPU dau ton hang chuc
    # giay khoi tao cuDNN, va neu no roi vao vong dieu khien thi xe dung im.
    try:
        _t0 = time.time()
        unified_model.warmup()
        print(f"[INIT] Warmup YOLO26: {(time.time()-_t0)*1000:.0f}ms")
    except Exception as _uerr:
        print(f"[INIT] Warmup YOLO26 bo qua: {type(_uerr).__name__}: {_uerr}")
else:
    print("[INIT] Loading segmentation model...")
    try:
        seg_model = create_deeplabv3(
            num_classes=SEG_NUM_CLASSES, backbone=SEG_BACKBONE, pretrained=False
        ).to(device)
        ckpt = torch.load(SEGMENTATION_MODEL_PATH, map_location=device)
        seg_model.load_state_dict(ckpt.get("state_dict", ckpt), strict=False)
        seg_model.eval()
        if device.type == "cuda":
            seg_model = seg_model.half()
            seg_use_fp16 = True
        print(f"[INIT] Segmentation OK ({device}, fp16={seg_use_fp16})")

        # Chay mot lan suy luan gia de cuDNN khoi tao xong NGAY BAY GIO.
        # 2026-09-10: do duoc khung DAU TIEN cua vong dieu khien mat 25.4 giay
        # (seg=25423ms) - do la chi phi khoi tao cuDNN, chi xay ra mot lan.
        # Truoc day no roi vao trong vong lap: xe dung im 25 giay sau khi bao
        # "Bat dau tuan tra", bridge bao AI timeout, va con so fps trung binh
        # bi keo tu 3.5 xuong 1.70. Lam warmup o day thi vong lap bat dau nong.
        try:
            _t0 = time.time()
            _warm = torch.zeros(
                (1, 3, SEG_INPUT_SIZE[0], SEG_INPUT_SIZE[1]), device=device
            )
            if seg_use_fp16:
                _warm = _warm.half()
            with torch.no_grad():
                seg_model(_warm)
            if device.type == "cuda":
                torch.cuda.synchronize()
            print(f"[INIT] Warmup segmentation: {(time.time()-_t0)*1000:.0f}ms")
            del _warm
        except Exception as _werr:
            print(f"[INIT] Warmup bo qua: {type(_werr).__name__}: {_werr}")
    except FileNotFoundError:
        print(f"[WARN] Segmentation model not found: {SEGMENTATION_MODEL_PATH}")
        print("[WARN] Segmentation disabled. YOLO/person detection can still run.")
        seg_model = None
    except Exception as e:
        print(f"[WARN] Segmentation load failed: {type(e).__name__}: {e}")
        print("[WARN] Segmentation disabled. YOLO/person detection can still run.")
        seg_model = None

    print("[INIT] Loading person detection model...")
    person_model = YOLO(PERSON_MODEL_PATH)
    print(f"[INIT] Person model OK (device={PERSON_DEVICE})")

    # Lan goi GPU dau tien ton 17.8 giay de khoi tao cuDNN - do o day chu khong
    # de no roi vao vong dieu khien (cung ly do nhu warmup segmentation).
    if PERSON_DEVICE != "cpu":
        try:
            _t0 = time.time()
            _dummy = np.zeros((360, 640, 3), dtype=np.uint8)
            person_model(
                _dummy, imgsz=PERSON_INFERENCE_SIZE, conf=PERSON_CONF_THRESHOLD,
                classes=[PERSON_CLASS_ID, VEHICLE_CLASS_ID],
                verbose=False, device=PERSON_DEVICE,
            )
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            print(f"[INIT] Warmup YOLO: {(time.time()-_t0)*1000:.0f}ms")
            del _dummy
        except Exception as _yerr:
            print(f"[INIT] Warmup YOLO bo qua: {type(_yerr).__name__}: {_yerr}")
print("[INIT] ArUco detector sẵn sàng")


# ============================================================
#  MAIN LOOP
# ============================================================

if __name__ == "__main__":
    car_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    car_socket.connect((SOCKET_IP, SOCKET_PORT))
    car_socket.settimeout(3.0)
    print(f"[CAR] Socket xe: {SOCKET_IP}:{SOCKET_PORT}")

    # Camera C270 can vai chuc khung moi tu chinh xong phoi sang: nhung
    # khung dau LUON DEN HOAN TOAN. Model doan bua tren anh den (do duoc:
    # line = 63.2% khung hinh, 145551px) va sinh ra lenh lai that su -
    # trong lan chay 2026-09-10 la steer=-13 speed=19 o 4 khung dau. Lan
    # do xe chua co nguon dong luc nen vo hai, nhung co nguon thi xe se
    # giat sang trai ngay giay dau tien.
    #
    # Gui "0 0" nen day chi la xin anh, banh khong chay.
    print(f"[WARMUP] Bo {CAMERA_WARMUP_FRAMES} khung dau de camera tu chinh sang...")
    for _ in range(CAMERA_WARMUP_FRAMES):
        try:
            car_socket.sendall(b"0 0")
            recv_json_frame(car_socket)
        except Exception as _error:
            print(f"[WARMUP] Loi khi lay khung: {_error}")
            break

    mqtt_client      = MQTTClient()
    time.sleep(1.5)

    lane_pid         = PIDController()
    lane_guard       = LaneTrackingGuard(LANE_REACQUIRE_FRAMES, LANE_LOST_MAX_FRAMES)
    last_tracking_state = None

    manual_driver = None
    if MANUAL_CONTROL:
        manual_driver = ManualDriver(
            speed_step=MANUAL_SPEED_STEP,
            steer_step=MANUAL_STEER_STEP,
            max_speed=MANUAL_MAX_SPEED,
            max_steer=MANUAL_MAX_STEER,
            idle_stop_sec=MANUAL_IDLE_STOP_SEC,
            steer_return_sec=MANUAL_STEER_RETURN_SEC,
        )
        print("=" * 60)
        print("  DIEU KHIEN TAY (run control) - lane PID KHONG lai xe")
        print("  w tien (giu de chay) | s phanh | a trai | d phai")
        print("  x tra lai thang      | SPACE dung khan | q thoat")
        print("  Nha phim {:.1f}s la xe tu dung. KHONG co lui:".format(
            MANUAL_IDLE_STOP_SEC))
        print("  firmware clamp speed >= 0, s chi la phanh.")
        print("  LiDAR + canh bao AI van chay binh thuong.")
        print("=" * 60)
    error_smoother   = ErrorSmoother()
    stability_filter = StabilityFilter()
    aruco_filter     = ArucoCommandFilter()
    intruder_filter  = IntruderConfirmFilter()
    location_tracker = PatrolLocationTracker()

    steer_angle      = 0
    car_speed        = 0
    last_midpoint    = None
    prev_lane_points = None
    lane_lost_count  = 0
    frame_count      = 0
    persons          = []

    start_time = time.time()

    # YOLO chay xuyen qua nhieu khung: giu tham chieu ngoai vong lap de
    # khong phai cho no xong trong cung mot khung (xem muc A).
    yolo_thread = None
    yolo_result = {}
    yolo_frame  = None
    yolo_wait_ms = 0.0

    # buffering=1 (line-buffered): mat dien giua luc chay van con du lieu.
    try:
        telem = open(TELEMETRY_PATH, "w", buffering=1)
        telem.write(
            "frame,t,src,raw_err,final_err,steer,speed,lost,"
            "line_px,mid,heading_px,tracking_state,pid_integral,"
            "auto_steer,auto_speed\n"
        )
        print(f"[TELEM] Ghi so lieu vao: {TELEMETRY_PATH}")
    except Exception as error:
        print(f"[TELEM] Khong mo duoc file log: {error}")
        telem = None
    was_lifted       = False

    intersection_mode    = False
    traffic_sign_command = None
    sign_detected        = False
    passed_sign          = False
    ready_to_turn        = False

    is_stopped      = False
    stop_start_time = None

    last_status_time = time.time()

    print("[MAIN] Bắt đầu tuần tra. Nhấn Q hoặc ESC để dừng.")

    # TensorRT path must not initialize a PyTorch CUDA context. ExitStack is
    # a no-op context manager available on Jetson's Python 3.6.
    # RawKeyboard vao ExitStack chu khong tu quan ly: terminal de o che do
    # cbreak ma khong khoi phuc thi shell sau do khong dung duoc nua, nen
    # viec tra lai phai xay ra ke ca khi vong lap nem loi.
    with (ExitStack() if USE_UNIFIED_YOLO26 else torch.no_grad()), \
            ExitStack() as _manual_stack:
        manual_keyboard = (_manual_stack.enter_context(RawKeyboard())
                           if MANUAL_CONTROL else None)
        try:
            while True:
                # ── Gửi lệnh & nhận frame ───────────────────
                perf_now = time.time()
                perf_interval_ms = (0.0 if "perf_prev_send" not in locals()
                                    else (perf_now - perf_prev_send) * 1000)
                perf_prev_send = perf_now
                perf_t0 = perf_now
                car_socket.sendall(
                    bytes(f"{steer_angle + STEER_TRIM} {car_speed}", "utf-8")
                )
                recv = recv_json_frame(car_socket)
                perf_socket_ms = (time.time() - perf_t0) * 1000

                jpg    = base64.b64decode(recv["Img"])
                bgr    = cv2.imdecode(
                    np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                pil_im = None
                if not USE_UNIFIED_YOLO26:
                    pil_im = Image.fromarray(
                        cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                    )
                current_location = location_tracker.current

                # ── A0. KIEM TRA BI NHAC LEN (IMU qua Bridge) ────
                # Bridge bao lenh lai dang khong den banh xe (LiDAR chan,
                # UART loi...). Dung de khoa tich phan PID.
                cmd_blocked = bool(recv.get("blocked"))

                is_lifted_now = bool(recv.get("lifted"))
                if is_lifted_now and not was_lifted:
                    was_lifted = True
                    lift_since = recv.get(
                        "lift_since",
                        datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    )
                    mqtt_client.send_lift_alert(current_location, lift_since)
                    _, lift_buf = cv2.imencode(
                        ".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 75]
                    )
                    threading.Thread(
                        target=send_discord_lift_alert,
                        args=(lift_buf.tobytes(), current_location, lift_since),
                        daemon=True,
                    ).start()
                elif not is_lifted_now and was_lifted:
                    was_lifted = False

                # ── A. PHÁT HIỆN NGƯỜI ──────────────────────
                # YOLO chay tren GPU o thread rieng, chong lan voi
                # segmentation, nen chi chay 1/YOLO_EVERY_N_FRAMES khung.
                frame_count += 1
                run_yolo_this_frame = (frame_count % YOLO_EVERY_N_FRAMES == 0)
                # YOLO chi phuc vu canh bao nguoi xam nhap, KHONG tham gia
                # dieu khien lai. Truoc day vong lap goi yolo_thread.join()
                # ngay trong cung khung, nen du chay song song voi
                # segmentation, YOLO (828-850ms tren CPU) van quyet dinh
                # thoi gian khung: do duoc 1.85 fps thay vi ~4 fps. Gio de
                # no chay xuyen khung, khung nao no xong thi thu ket qua -
                # canh bao tre 1-2 khung khong anh huong gi.
                # 2026-09-11: truoc day perf_yolo_t0 duoc dat O DAY, roi
                # perf_yolo_ms lai tinh SAU khoi segmentation ben duoi - nen
                # con so "yolo=" bao gom tron ca segmentation, va hai cot
                # yolo= / seg= luon in ra y het nhau. Doc log tuong YOLO ton
                # 200ms moi khung trong khi that ra no chay o thread khac.
                # Gio tach lam ba so doc lap:
                #   yolo_loop  - thoi gian VONG LAP CHINH bo ra cho YOLO
                #                (khoi thread + thu ket qua). Phai rat nho.
                #   yolo_infer - do BEN TRONG thread, la thoi gian suy luan
                #                that. No khong cong vao thoi gian khung,
                #                nhung co gianh GPU voi segmentation.
                #   seg        - rieng segmentation, khong dinh YOLO.
                perf_yolo_loop_ms = 0.0
                _t = time.time()
                # Mot model thi khong con gi de chay song song: cung mot
                # luot suy luan o khoi B tra ve ca mask lan hop nguoi, nen
                # khong khoi thread rieng va YOLO_EVERY_N_FRAMES khong con
                # y nghia - moi khung deu co ket qua nguoi, mien phi.
                if (not USE_UNIFIED_YOLO26
                        and run_yolo_this_frame and yolo_thread is None):
                    yolo_frame = bgr
                    def _run_yolo(_bgr=bgr):
                        _y0 = time.time()
                        yolo_result["persons"] = detect_persons(person_model, _bgr)
                        yolo_result["infer_ms"] = (time.time() - _y0) * 1000
                    yolo_thread = threading.Thread(target=_run_yolo, daemon=True)
                    yolo_thread.start()
                perf_yolo_loop_ms += (time.time() - _t) * 1000

                # ── B. SEGMENTATION (GPU, chay song song voi YOLO) ───
                if USE_UNIFIED_YOLO26:
                    # Mot luot suy luan, hai dau ra. pred_mask ra dung cac
                    # id cua SEG_CLASS_NAMES nen scan_lane()/
                    # detect_intersection_points()/decode_segmap() khong
                    # phai doi gi; hop nguoi/xe lay ngay tai day thay vi
                    # doi thread.
                    perf_seg_t0 = time.time()
                    pred_mask, unified_persons = unified_model.infer(bgr)
                    # Cung ly do nhu ban DeepLabV3 ben duoi: class motobike
                    # hay bi nham voi mat duong sang, va mot lo hong trong
                    # bien "road" thi lam lech huong. Gan lai thanh road -
                    # canh bao xe khong bi anh huong vi no doc tu hop
                    # detection, khong doc mask.
                    pred_mask[pred_mask == 4] = 1
                    perf_seg_ms = (time.time() - perf_seg_t0) * 1000
                    perf_pre_ms = 0.0
                    perf_gpu_ms = perf_seg_ms
                elif seg_model is not None:
                    perf_seg_t0 = time.time()
                    tensor = img_transform(pil_im).unsqueeze(0).to(device)
                    if seg_use_fp16:
                        tensor = tensor.half()
                    # Tien xu ly chay tren CPU (PIL resize + ToTensor +
                    # Normalize); phan con lai la GPU. Gop chung lai thi
                    # khong biet nen cat cho nao khi vong lap cham.
                    perf_pre_ms = (time.time() - perf_seg_t0) * 1000

                    pred_mask_small = (
                        torch.argmax(seg_model(tensor), dim=1)
                        .squeeze(0).cpu().numpy()
                    )

                    frame_h, frame_w = bgr.shape[:2]
                    pred_mask = cv2.resize(
                        pred_mask_small.astype(np.uint8),
                        (frame_w, frame_h),
                        interpolation=cv2.INTER_NEAREST,
                    )
                    # TAM THOI (2026-09-03): class "motobike" (4) chua du du lieu
                    # train de nhan dien dung - hay bi nham vach vang/mat duong
                    # sang mot obike, lam thung/lech bien "road" gay lech huong.
                    # Gan lai thanh road cho toi khi train lai voi du lieu
                    # motobike da dang hon. Khong anh huong canh bao xe (dung
                    # YOLO rieng, khong dung segmentation).
                    pred_mask[pred_mask == 4] = 1
                    perf_seg_ms = (time.time() - perf_seg_t0) * 1000
                    perf_gpu_ms = perf_seg_ms - perf_pre_ms
                else:
                    # No segmentation model available.
                    # Use an empty mask so the rest of the program can keep running.
                    pred_mask = np.zeros((bgr.shape[0], bgr.shape[1]), dtype=np.uint8)
                    perf_seg_ms = 0.0
                    perf_pre_ms = 0.0
                    perf_gpu_ms = 0.0

                # Thu ket qua YOLO neu no da xong - KHONG cho. Anh gui kem
                # canh bao la anh luc YOLO thay nguoi (yolo_frame), khong
                # phai khung hien tai, de anh khop voi hop bounding.
                _t = time.time()
                if USE_UNIFIED_YOLO26:
                    # Ket qua thuoc dung khung nay, nen anh canh bao la bgr
                    # chu khong phai yolo_frame cua mot khung truoc do.
                    yolo_done = True
                    persons = unified_persons
                    should_alert = intruder_filter.update(len(persons) > 0)
                    if should_alert and persons:
                        threading.Thread(
                            target=mqtt_client.send_alert,
                            args=(bgr, persons, current_location),
                            daemon=True,
                        ).start()
                else:
                    yolo_done = yolo_thread is not None and not yolo_thread.is_alive()
                    if yolo_done:
                        yolo_thread = None
                        persons = yolo_result.get("persons", persons)
                        should_alert = intruder_filter.update(len(persons) > 0)
                        if should_alert and persons:
                            threading.Thread(
                                target=mqtt_client.send_alert,
                                args=(yolo_frame if yolo_frame is not None else bgr,
                                      persons, current_location),
                                daemon=True,
                            ).start()
                perf_yolo_loop_ms += (time.time() - _t) * 1000

                perf_subtotal_ms = (time.time() - perf_t0) * 1000
                if USE_UNIFIED_YOLO26:
                    # Khong tai dung nhan "seg=" o day: con so nay bao gom
                    # CA phat hien nguoi, khac han nghia cua seg= o duong
                    # hai model. Dan nay tung doc sai log dung kieu do mot
                    # lan roi (xem ghi chu khoi A), nen dat ten rieng.
                    trt_timing = getattr(unified_model, "last_timing_ms", None)
                    timing_text = ""
                    if trt_timing:
                        timing_text = (" pre={:.0f} h2d={:.0f} exec={:.0f} "
                                       "d2h={:.0f} post={:.0f}").format(
                            trt_timing["pre"], trt_timing["h2d"],
                            trt_timing["execute"], trt_timing["d2h"],
                            trt_timing["post"],
                        )
                    print("[STAGE] interval={:.0f}ms socket={:.0f}ms "
                          "unified={:.0f}ms(mask+nguoi, 1 luot{}) "
                          "subtotal={:.0f}ms".format(
                              perf_interval_ms, perf_socket_ms,
                              perf_seg_ms, timing_text, perf_subtotal_ms,
                          ))
                else:
                    print("[STAGE] interval={:.0f}ms socket={:.0f}ms "
                          "seg={:.0f}ms(pre {:.0f} + gpu {:.0f}) "
                          "yolo_loop={:.1f}ms yolo_infer={:.0f}ms"
                          "(start={} thu={}) subtotal={:.0f}ms".format(
                              perf_interval_ms, perf_socket_ms,
                              perf_seg_ms, perf_pre_ms, perf_gpu_ms,
                              perf_yolo_loop_ms, yolo_result.get("infer_ms", 0.0),
                              run_yolo_this_frame, yolo_done,
                              perf_subtotal_ms,
                          ))

                # ── C. ARUCO ─────────────────────────────────
                perf_aux_t0 = time.time()
                marker_id, marker_corners = detect_aruco(bgr)
                aruco_cmd = aruco_filter.update(marker_id)
                perf_aruco_ms = (time.time() - perf_aux_t0) * 1000

                if aruco_cmd == "stop":
                    is_stopped      = True
                    stop_start_time = time.time()
                    car_speed       = 0
                    steer_angle     = 0
                    print("[ARUCO] Lệnh STOP — dừng xe")
                elif aruco_cmd in ("turn_left", "turn_right", "straight"):
                    traffic_sign_command = aruco_cmd
                    sign_detected        = True
                    passed_sign          = False
                    ready_to_turn        = False
                    print(f"[ARUCO] Lệnh {aruco_cmd} — chờ đến giao lộ")

                if is_stopped:
                    if time.time() - stop_start_time >= STOP_DURATION:
                        is_stopped = False
                        car_speed  = BASE_SPEED
                        print("[ARUCO] Hết thời gian STOP — tiếp tục")
                    else:
                        if SHOW_DEBUG_WINDOW or RECORD_VIDEO:
                            # RGB->BGR truoc khi ve chu do (0,0,255) - xem
                            # ghi chu cung loi o phan DEBUG WINDOW ben duoi.
                            if USE_UNIFIED_YOLO26:
                                display_img = cv2.resize(bgr, (640, 360))
                            else:
                                display_img = cv2.cvtColor(
                                    np.array(pil_im.resize((640, 360))),
                                    cv2.COLOR_RGB2BGR,
                                )
                            cv2.putText(display_img, "STOPPED",
                                        (240, 180), cv2.FONT_HERSHEY_SIMPLEX,
                                        2, (0, 0, 255), 4)
                            record_frame(display_img)
                            if SHOW_WINDOW:
                                cv2.imshow("Patrol Robot", display_img)
                            if SHOW_WINDOW and cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                                break
                        continue

                # ── D. INTERSECTION & LANE ───────────────────
                intersection_info = detect_intersection_points(pred_mask)
                intersection_info_display = detect_intersection_points(
                    pred_mask, scan_y=INTERSECTION_DISPLAY_SCAN_Y
                )
                lane_scan   = scan_lane(pred_mask, reference_x=last_midpoint)
                lane_points = None if lane_scan is None else lane_scan[:3]
                lane_from_line = bool(lane_scan is not None and lane_scan[3])
                lane_guard.update(lane_from_line, blocked=cmd_blocked)
                # Chi diem VACH THAT moi dung de so sanh khung-truoc-khung
                # (detect_steep_slope). Truoc day tron ca fallback road vao:
                # midpoint road (~320) va midpoint vach that (vd 249) chenh
                # nhau rat xa, nen moi lan mat vach 1 khung la bi coi nhu
                # "lane doi dot ngot" -> tang toc (SLOPE_BOOST_HEAVY) dung
                # luc nhan dien dang chap chon, nguoc voi mong muon.
                line_points = lane_points if lane_from_line else None
                center_x    = STEER_CENTER_X

                # 2026-09-09: dem "mat lane" theo VACH THAT, khong theo
                # "co thay gi khong". Truoc day lane_points chi None khi
                # ca road lan line deu khong thay - ma road phu het khung
                # nen gan nhu KHONG BAO GIO None -> nguong LANE_LOST_MAX_
                # FRAMES (dung xe an toan) that ra la code chet, khong bao
                # gio kich hoat. Gio dem theo lan mat vach that.
                if lane_from_line:
                    lane_lost_count = 0
                else:
                    lane_lost_count += 1

                if not hasattr(detect_intersection_points, "stable_count"):
                    detect_intersection_points.stable_count = 0
                if (intersection_info["is_intersection"]
                        and intersection_info["lane_count"] >= 3):
                    detect_intersection_points.stable_count += 1
                else:
                    detect_intersection_points.stable_count = 0
                intersection_truly_stable = (
                    detect_intersection_points.stable_count >= INTERSECTION_STABLE_FRAMES
                )

                stability_filter.update(
                    intersection_info.get("should_turn_left", False),
                    intersection_info.get("should_turn_right", False),
                    intersection_info.get("confidence", 0.0),
                )
                stable_left, stable_right, avg_confidence = (
                    stability_filter.get_stable_decision()
                )

                if sign_detected and not aruco_filter.is_seeing_marker:
                    passed_sign = True

                if passed_sign:
                    if not hasattr(detect_intersection_points, "frames_since_passed"):
                        detect_intersection_points.frames_since_passed = 0
                    detect_intersection_points.frames_since_passed += 1
                    safe_dist = (
                        detect_intersection_points.frames_since_passed
                        >= SAFE_DIST_FRAMES_AFTER_SIGN
                    )
                else:
                    if hasattr(detect_intersection_points, "frames_since_passed"):
                        detect_intersection_points.frames_since_passed = 0
                    safe_dist = True

                strong_signal = (
                    passed_sign
                    and intersection_info["is_intersection"]
                    and intersection_info["lane_count"] >= 3
                    and avg_confidence > 0.8
                    and (
                        (stable_left  and traffic_sign_command == "turn_left") or
                        (stable_right and traffic_sign_command == "turn_right")
                    )
                )

                if not ready_to_turn and (
                    (passed_sign and safe_dist
                     and intersection_info["is_intersection"]
                     and intersection_truly_stable
                     and intersection_info["lane_count"] >= 3
                     and traffic_sign_command in ("turn_left", "turn_right", "straight"))
                    or strong_signal
                ):
                    ready_to_turn     = True
                    intersection_mode = True
                    if not hasattr(detect_intersection_points, "int_frames"):
                        detect_intersection_points.int_frames = 0

                if intersection_mode and ready_to_turn:
                    if not hasattr(detect_intersection_points, "int_frames"):
                        detect_intersection_points.int_frames = 0
                    detect_intersection_points.int_frames += 1
                else:
                    if hasattr(detect_intersection_points, "int_frames"):
                        detect_intersection_points.int_frames = 0

                # Reset logic
                reset_needed = False

                if (intersection_mode and ready_to_turn
                        and not intersection_info["is_intersection"]):
                    reset_needed = True

                if intersection_mode and ready_to_turn and lane_points:
                    lx, rx, mx = lane_points
                    lane_stable = (abs(mx - center_x) < LANE_STABLE_OFFSET
                                   and rx - lx > LANE_STABLE_WIDTH)
                    cmd_done = (
                        (traffic_sign_command == "turn_left"  and mx < center_x - 30) or
                        (traffic_sign_command == "turn_right" and mx > center_x + 30) or
                        (traffic_sign_command == "straight")
                    )
                    if lane_stable and cmd_done:
                        reset_needed = True

                if intersection_mode and ready_to_turn:
                    if (getattr(detect_intersection_points, "int_frames", 0)
                            > INTERSECTION_MAX_FRAMES):
                        reset_needed = True

                if (sign_detected or passed_sign or ready_to_turn) and \
                        not intersection_info["is_intersection"]:
                    if not hasattr(detect_intersection_points, "frames_after_sign"):
                        detect_intersection_points.frames_after_sign = 0
                    detect_intersection_points.frames_after_sign += 1
                    if (detect_intersection_points.frames_after_sign
                            > INTERSECTION_TIMEOUT_FRAMES):
                        reset_needed = True
                else:
                    if hasattr(detect_intersection_points, "frames_after_sign"):
                        detect_intersection_points.frames_after_sign = 0

                if reset_needed:
                    intersection_mode    = False
                    traffic_sign_command = None
                    sign_detected        = False
                    passed_sign          = False
                    ready_to_turn        = False
                    stability_filter.reset()
                    aruco_filter.reset()
                    for attr in ("frames_after_sign", "int_frames",
                                 "frames_since_passed", "stable_count"):
                        if hasattr(detect_intersection_points, attr):
                            setattr(detect_intersection_points, attr, 0)

                # ── E. STEERING ──────────────────────────────
                lane_error = 0
                raw_lane_error = 0

                if (intersection_mode and ready_to_turn
                        and traffic_sign_command in ("turn_left", "turn_right", "straight")):
                    steer = calculate_intersection_steering(
                        intersection_info, traffic_sign_command
                    )
                    if steer is not None:
                        steer_angle = steer
                        # Gia tri 80 tu thang toc do mo phong cu bi bridge
                        # kep am tham ve 25, lam xe vao giao lo nhanh hon
                        # moi cau hinh BASE/MIN_SPEED. Dung toc do cua de
                        # dieu khien cung mot don vi voi lane following.
                        car_speed   = MIN_SPEED
                    else:
                        if USE_STANLEY_CONTROLLER:
                            stanley_result = get_lane_center_robust(
                                pred_mask, INTERSECTION_SCAN_Y, HEADING_SCAN_Y_FAR, center_x
                            )
                            if stanley_result is not None and stanley_result[3]:
                                raw_lane_error, heading_err_deg, near_mx, _ = stanley_result
                                last_midpoint = near_mx
                                lane_error = raw_lane_error
                            elif last_midpoint is not None:
                                raw_lane_error = lane_error = last_midpoint - center_x
                                heading_err_deg = 0.0
                            else:
                                raw_lane_error = lane_error = 0
                                heading_err_deg = 0.0
                            slope_boost = detect_steep_slope(
                                lane_error, line_points, prev_lane_points
                            )
                            car_speed = int(
                                adaptive_speed_control(lane_error) * slope_boost
                            )
                            steer_angle = stanley_to_steer_command(
                                stanley_steer(raw_lane_error, heading_err_deg, car_speed)
                            )
                        else:
                            if lane_from_line:
                                _, _, mx  = lane_points
                                last_midpoint = mx
                                lane_error    = mx - center_x
                                lane_error   += get_heading_error(
                                    pred_mask, INTERSECTION_SCAN_Y, mx
                                )
                            elif last_midpoint is not None:
                                lane_error = last_midpoint - center_x
                            elif lane_points:
                                _, _, mx = lane_points
                                lane_error = mx - center_x
                            slope_boost = detect_steep_slope(
                                lane_error, line_points, prev_lane_points
                            )
                            raw_lane_error = lane_error
                            if lane_from_line:
                                lane_error = error_smoother.update(lane_error)
                            else:
                                lane_error = error_smoother.decay(
                                    LANE_LOST_STEER_DECAY
                                )
                            steer_angle = lane_pid.compute(
                                lane_error,
                                hold_integral=cmd_blocked or not lane_from_line,
                                measured_error=raw_lane_error,
                            )
                            car_speed   = int(
                                adaptive_speed_control(lane_error) * slope_boost
                            )
                else:
                    if USE_STANLEY_CONTROLLER:
                        stanley_result = get_lane_center_robust(
                            pred_mask, INTERSECTION_SCAN_Y, HEADING_SCAN_Y_FAR, center_x
                        )
                        # 2026-09-09: chi nhan ket qua khi den tu VACH THAT
                        # (phan tu [3]). Fallback road cua ham nay cung chi la
                        # "khong do duoc" - xem ghi chu trong get_lane_center_robust.
                        if stanley_result is not None and stanley_result[3]:
                            raw_lane_error, heading_err_deg, near_mx, _ = stanley_result
                            last_midpoint = near_mx
                            lane_error = raw_lane_error
                        elif last_midpoint is not None:
                            raw_lane_error = lane_error = last_midpoint - center_x
                            heading_err_deg = 0.0
                        else:
                            raw_lane_error = lane_error = 0
                            heading_err_deg = 0.0
                        slope_boost = detect_steep_slope(
                            lane_error, line_points, prev_lane_points
                        )
                        car_speed = int(
                            adaptive_speed_control(lane_error) * slope_boost
                        )
                        steer_angle = stanley_to_steer_command(
                            stanley_steer(raw_lane_error, heading_err_deg, car_speed)
                        )
                    else:
                        # 2026-09-09: chi tin va cap nhat last_midpoint khi
                        # do duoc VACH THAT. Khi mat vach (fallback road),
                        # GIU nguyen loi do duoc gan nhat thay vi coi nhu xe
                        # dang o giua duong - vi midpoint cua road luon ~=
                        # giua anh (road phu het khung), tuc la "khong biet"
                        # chu khong phai "dang o giua". Cach cu lam loi nhay
                        # qua lai giua tri that va ~0 moi khi mat vach 1-2
                        # khung -> be lai giat cuc, dung lai, roi be tiep
                        # (dung kieu "luon qua luon lai" da thay khi test).
                        if lane_from_line:
                            _, _, mx  = lane_points
                            last_midpoint = mx
                            lane_error    = mx - center_x
                            lane_error   += get_heading_error(
                                pred_mask, INTERSECTION_SCAN_Y, mx
                            )
                        elif last_midpoint is not None:
                            lane_error = last_midpoint - center_x
                        elif lane_points:
                            # chua tung thay vach lan nao (vua khoi dong)
                            _, _, mx = lane_points
                            lane_error = mx - center_x
                        slope_boost = detect_steep_slope(
                            lane_error, line_points, prev_lane_points
                        )
                        raw_lane_error = lane_error
                        if lane_from_line:
                            lane_error = error_smoother.update(lane_error)
                        else:
                            lane_error = error_smoother.decay(
                                LANE_LOST_STEER_DECAY
                            )
                        steer_angle = lane_pid.compute(
                            lane_error,
                            hold_integral=cmd_blocked or not lane_from_line,
                            measured_error=raw_lane_error,
                        )
                        car_speed   = int(
                            adaptive_speed_control(lane_error) * slope_boost
                        )

                # Lech qua nhieu khoi tam lane: giam toc manh de xoay
                # gap quanh 1 banh thay vi lao tiep (khong doi huong steer,
                # PID da tu day steer ve gan max khi loi lon).
                # Dung RAW error (truoc EMA smoothing) de kiem tra nguong -
                # 2026-09-04: phat hien EMA (alpha=0.5) lam tre tin hieu qua
                # nhieu so voi loi that dang tang nhanh (raw -213 nhung sau
                # EMA chi con -87), khien nguong 150 gan nhu khong bao gio
                # kich hoat dung luc xe dang troi manh nhat - xe cu bam sat
                # 1 ben line ma khong duoc giam toc de xoay gap lai.
                if abs(raw_lane_error) >= RECOVERY_ERROR_THRESHOLD:
                    car_speed = RECOVERY_SPEED

                # Final gate after navigation/adaptive/recovery overrides.
                # A single spurious LINE cannot restart a lost-line stop.
                steer_angle, car_speed = lane_guard.command(steer_angle, car_speed)
                if not lane_guard.ready:
                    lane_pid.reset()
                    error_smoother.reset()

                # Y dinh cua AI, bat TRUOC khi ban phim ghi de. Khong co
                # cap nay thi lan chay tay chi luu lai lenh cua NGUOI, va
                # thu dang gia nhat bi mat: "AI dinh be bao nhieu o dung
                # khung do". Giu duoc no thi run control thanh cong cu thu
                # thap du lieu - lai tay an toan tren duong that, con bo
                # bam vach thi cham diem offline ma khong phai giao xe cho
                # bo dieu khien chua duoc kiem chung.
                # O duong tu dong hai cot nay trung voi steer/speed, nen
                # cot luon co nghia thay vi de trong tuy che do.
                auto_steer, auto_speed = steer_angle, car_speed

                # Dat SAU chot tu dong va TRUOC telemetry: moi lan ghi de
                # cua duong tu dong (recovery, mat vach, adaptive speed) da
                # chay xong, nen ban phim la tieng noi cuoi cung va cot
                # telemetry van la lenh thuc su gui di.
                # Lop an toan LiDAR nam trong bridge (real_car_socket.py),
                # khong phai o day - no van chan lenh nhu binh thuong.
                if manual_driver is not None:
                    for _key in manual_keyboard.pending_keys():
                        manual_driver.feed(_key)
                    steer_angle, car_speed = manual_driver.command()
                    if manual_driver.quit:
                        print("\n[MANUAL] q - dung xe va thoat")
                        car_socket.sendall(b"0 0")
                        break
                if lane_guard.state != last_tracking_state:
                    print(f"[TRACK] {lane_guard.state} (line={lane_from_line}, "
                          f"confirmed={lane_guard.good}/{LANE_REACQUIRE_FRAMES})")
                    last_tracking_state = lane_guard.state

                prev_lane_points = line_points
                perf_control_done = time.time()

                # ── E2. TELEMETRY ───────────────────────
                # Ghi sau khi car_speed da qua HET cac lan ghi de (recovery,
                # mat vach) nen cot speed la lenh thuc su gui di, khong phai
                # gia tri trung gian.
                if telem is not None:
                    mid_val = lane_points[2] if lane_points else -1
                    # heading khong duoc giu rieng o bien nao, nhung
                    # raw_err = (mid - center_x) + heading nen suy ra duoc.
                    if lane_from_line and lane_points:
                        head_px = raw_lane_error - (mid_val - center_x)
                    else:
                        head_px = 0
                    try:
                        telem.write(
                            "%d,%.3f,%s,%.1f,%.1f,%d,%d,%d,%d,%d,%.1f,%s,%.3f,%d,%d\n" % (
                                frame_count,
                                time.time() - start_time,
                                "LINE" if lane_from_line else "HOLD",
                                raw_lane_error,
                                lane_error,
                                steer_angle,
                                car_speed,
                                lane_lost_count,
                                int((pred_mask == 2).sum()),
                                mid_val,
                                head_px,
                                lane_guard.state,
                                lane_pid.integral,
                                auto_steer,
                                auto_speed,
                            )
                        )
                    except Exception:
                        pass

                # ── F. HEARTBEAT ─────────────────────────────
                if time.time() - last_status_time >= STATUS_INTERVAL_SEC:
                    mqtt_client.send_status({
                        "location":        current_location,
                        "speed":           car_speed,
                        "steer":           steer_angle,
                        "intruder_active": intruder_filter.is_detecting,
                        "total_alerts":    intruder_filter.alert_count,
                        "intersection":    intersection_mode,
                        "aruco_command":   traffic_sign_command or "none",
                    })
                    last_status_time = time.time()
                perf_status_done = time.time()

                # ── G. DEBUG WINDOW ──────────────────────────
                if SHOW_DEBUG_WINDOW or RECORD_VIDEO:
                    # 2026-09-08: img_resized (tu pil_im, la anh RGB) va
                    # mask_color (CLASS_COLORS trong Confg.py dat theo ten
                    # mau RGB, vd cam=[255,165,0]) truoc gio duoc blend rồi
                    # ghi thang vao VideoWriter/imshow - ca 2 noi nay LUON
                    # doc mang la BGR (quy uoc cv2), nen toan bo lop
                    # camera+mask bi DAO KENH R/B trong video xuat ra. Cac
                    # lenh ve (circle/rectangle/putText) o duoi dung mau
                    # BGR dung quy uoc (vd (0,0,255)="do") nen KHONG bi anh
                    # huong - chi rieng lop camera+mask la sai. Phat hien
                    # qua giai ma lai 1 video that: class "line" (dinh
                    # nghia cyan) hien ra mau VANG, class "person" (dinh
                    # nghia do) hien mau XANH DUONG trong video da luu -
                    # co the gay hieu lam khi xem lai video debug bang mat.
                    # Doi RGB->BGR ngay tu day de dong bo voi phan ve o duoi.
                    if USE_UNIFIED_YOLO26:
                        if bgr.shape[:2] == (360, 640):
                            img_resized = bgr.copy()
                        else:
                            img_resized = cv2.resize(bgr, (640, 360))
                    else:
                        img_resized = cv2.cvtColor(
                            np.array(pil_im.resize((640, 360))),
                            cv2.COLOR_RGB2BGR,
                        )
                    if VIDEO_FULL_MASK:
                        mask_color = cv2.cvtColor(
                            decode_segmap(pred_mask), cv2.COLOR_RGB2BGR
                        )
                        display_img = cv2.addWeighted(
                            img_resized, 0.5, mask_color, 0.5, 0
                        )
                    else:
                        # Fast recording path: keep the real camera image and
                        # paint only the cyan line pixels used by steering.
                        # Road fill is not used to steer and obscures the raw
                        # scene while costing ~15 ms/frame on Tegra X1.
                        display_img = img_resized
                        display_img[pred_mask == 2] = (255, 255, 0)

                    for pname, pinfo in intersection_info_display.items():
                        if isinstance(pinfo, dict) and pinfo.get("detected"):
                            cx_pt = pinfo["center"]
                            col = {
                                "left":         (255, 0,   0),
                                "center_left":  (255, 255, 0),
                                "center":       (0,   255, 0),
                                "center_right": (0,   255, 255),
                                "right":        (0,   0,   255),
                            }.get(pname, (255, 255, 255))
                            cv2.circle(display_img, (cx_pt, 355), 8, col, -1)
                            cv2.putText(
                                display_img, pname[0].upper(),
                                (cx_pt - 5, 340),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2,
                            )

                    cv2.line(display_img, (0, INTERSECTION_DISPLAY_SCAN_Y),
                             (640, INTERSECTION_DISPLAY_SCAN_Y), (128, 128, 128), 2)
                    cv2.line(display_img, (0, INTERSECTION_SCAN_Y),
                             (640, INTERSECTION_SCAN_Y), (255, 255, 0), 2)
                    cv2.line(display_img, (center_x, 0), (center_x, 360),
                             (128, 128, 128), 1)

                    if lane_points:
                        lx, rx, mx = lane_points
                        # Cham trang/text dung raw_lane_error (dung cho CA 2 he
                        # dieu khien) thay vi mx (chi dung khi USE_STANLEY_
                        # CONTROLLER=False - mx la ket qua cua get_lane_midpoint
                        # rieng, KHAC voi diem Stanley thuc su dung de lai, se
                        # gay hien thi sai lech nhu bug "+heading=" tung gap
                        # phai 2026-09-04). lx/rx (mep trai/phai) van la tu
                        # get_lane_midpoint - chi mang tinh minh hoa, khong feed
                        # vao dieu khien nen khong sao.
                        display_mx = center_x + int(round(raw_lane_error))
                        cv2.circle(display_img, (display_mx, INTERSECTION_SCAN_Y),
                                   6, (255, 255, 255), -1)
                        cv2.circle(display_img, (lx, INTERSECTION_SCAN_Y),
                                   4, (255, 0, 0), -1)
                        cv2.circle(display_img, (rx, INTERSECTION_SCAN_Y),
                                   4, (255, 0, 0), -1)
                        cv2.line(display_img,
                                 (center_x, INTERSECTION_SCAN_Y),
                                 (display_mx, INTERSECTION_SCAN_Y),
                                 (255, 255, 255), 3)
                        # LINE = do duoc vach that khung nay; HOLD = mat vach,
                        # dang giu loi cu (them 2026-09-09 de xem lai video la
                        # biet ngay khung nao co tin hieu that, khung nao khong)
                        src_txt = "LINE" if lane_from_line else "HOLD"
                        src_col = ((0, 255, 0) if lane_from_line
                                   else (0, 165, 255))
                        cv2.putText(
                            display_img,
                            f"raw={int(raw_lane_error)} final={int(lane_error)} "
                            f"{src_txt} lost={lane_lost_count} {lane_guard.state}",
                            (display_mx + 8, INTERSECTION_SCAN_Y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, src_col, 1,
                        )

                    if marker_corners is not None:
                        cv2.aruco.drawDetectedMarkers(display_img, [marker_corners])
                        if marker_id is not None:
                            cmd_label = ARUCO_COMMANDS.get(marker_id, "?")
                            pts = marker_corners[0].astype(int)
                            cv2.putText(
                                display_img,
                                f"ID={marker_id} -> {cmd_label}",
                                (pts[0][0], pts[0][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                            )

                    for p in persons:
                        x1, y1, x2, y2 = map(int, p["bbox"])
                        col = (0, 0, 255) if intruder_filter.is_detecting \
                              else (0, 165, 255)
                        cv2.rectangle(display_img, (x1, y1), (x2, y2), col, 2)
                        cv2.putText(
                            display_img,
                            f"{p['label'].upper()} {p['confidence']:.0%}",
                            (x1, max(y1 - 8, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2,
                        )

                    hud = [
                        f"Mode: {'MANUAL(WASD)' if manual_driver is not None else ('INTERSECTION' if intersection_mode else 'PATROL')}",
                        f"Speed:{car_speed}  Steer:{steer_angle}",
                        # Chi hien o che do tay: o duong tu dong hai so nay
                        # trung nhau nen in ra chi lam roi HUD.
                        *([f"AI muon: Steer:{auto_steer}  Speed:{auto_speed}"]
                          if manual_driver is not None else []),
                        f"ArUco cmd: {traffic_sign_command or 'none'}",
                        f"Sign detected:{sign_detected}  Passed:{passed_sign}",
                        f"Ready to turn:{ready_to_turn}",
                        f"Location: {current_location}",
                        f"INTRUDER: {'>>> YES <<<' if intruder_filter.is_detecting else 'clear'}",
                        f"Alerts: {intruder_filter.alert_count}  "
                        f"MQTT:{'OK' if mqtt_client.connected else 'OFFLINE'}",
                        f"Lane count: {intersection_info.get('lane_count', 0)}",
                    ]
                    for i, line in enumerate(hud):
                        col = (0, 0, 255) if "YES" in line else (0, 255, 255)
                        cv2.putText(
                            display_img, line,
                            (10, 20 + i * DEBUG_HUD_LINE_HEIGHT),
                            cv2.FONT_HERSHEY_SIMPLEX, DEBUG_HUD_FONT_SCALE, col, 2,
                        )

                    record_frame(display_img)
                    if SHOW_WINDOW:
                        cv2.imshow("Patrol Robot", display_img)
                    key = (cv2.waitKey(1) & 0xFF) if SHOW_WINDOW else 255
                    if key in (ord("q"), 27):
                        break

                if frame_count % 10 == 0:
                    perf_end = time.time()
                    print("[AUX] aruco={:.0f}ms control={:.0f}ms "
                          "telem_status={:.0f}ms debug_record={:.0f}ms "
                          "total_after_infer={:.0f}ms".format(
                              perf_aruco_ms,
                              (perf_control_done - perf_aux_t0) * 1000
                              - perf_aruco_ms,
                              (perf_status_done - perf_control_done) * 1000,
                              (perf_end - perf_status_done) * 1000,
                              (perf_end - perf_aux_t0) * 1000,
                          ))

        except KeyboardInterrupt:
            print("\n[MAIN] Dừng...")
        finally:
            if video_recorder is not None:
                video_recorder.close()
                print("[REC] Video saved: {} (written={}, dropped={})".format(
                    VIDEO_OUTPUT_PATH,
                    video_recorder.frames_written,
                    video_recorder.frames_dropped,
                ))
                if video_recorder.error is not None:
                    print("[REC] Loi ghi video: {}".format(video_recorder.error))
            if telem is not None:
                try:
                    telem.close()
                    print(f"[TELEM] Log saved: {TELEMETRY_PATH}")
                except Exception:
                    pass
            if SHOW_WINDOW:
                cv2.destroyAllWindows()
            mqtt_client.disconnect()
            car_socket.close()
            print("[MAIN] Đã đóng tất cả kết nối.")
