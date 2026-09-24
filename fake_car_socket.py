"""
fake_car_socket.py — Simulator xe cho patrol_robot.py
Mục đích: thay xe thật bằng webcam laptop để test code mà không cần hardware
Cách dùng:
    Terminal 1: python3 fake_car_socket.py
    Terminal 2: python3 patrol_robot.py
"""

import socket
import cv2
import base64
import json
import time

# ============================================================
#  CẤU HÌNH
# ============================================================
HOST = "127.0.0.1"
PORT = 54321

CAMERA_INDEX = 0      # Webcam laptop (đổi 1, 2... nếu có nhiều camera)
FRAME_WIDTH = 640
FRAME_HEIGHT = 360
JPEG_QUALITY = 70
LOOP_DELAY = 0.03     # ~33 fps

# ============================================================
#  KHỞI TẠO
# ============================================================
print("[FAKE CAR] Đang mở webcam...")
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if not cap.isOpened():
    print("[FAKE CAR] LỖI: Không mở được webcam!")
    print("[FAKE CAR] Thử đổi CAMERA_INDEX (0, 1, 2...) hoặc check camera permission")
    exit(1)

# Test 1 frame để chắc chắn camera OK
ret, frame = cap.read()
if not ret:
    print("[FAKE CAR] LỖI: Camera mở được nhưng không đọc được frame!")
    exit(1)

print(f"[FAKE CAR] Webcam OK! Frame size: {frame.shape}")

# Socket server
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)

print(f"[FAKE CAR] Đang lắng nghe tại {HOST}:{PORT}")
print("[FAKE CAR] Chờ patrol_robot.py kết nối...")
print("[FAKE CAR] (Nhấn Ctrl+C để dừng)")

conn, addr = server.accept()
print(f"[FAKE CAR] Đã kết nối: {addr}")

# ============================================================
#  MAIN LOOP
# ============================================================
frame_count = 0
start_time = time.time()
last_cmd = ""

try:
    while True:
        # 1. Nhận lệnh điều khiển từ patrol_robot.py
        try:
            cmd = conn.recv(1024).decode(errors="ignore")
        except ConnectionResetError:
            print("[FAKE CAR] patrol_robot.py đã ngắt kết nối")
            break

        if not cmd:
            continue

        # In ra lệnh (rate-limited, không spam)
        if cmd != last_cmd:
            print(f"[CMD] steer_speed = {cmd.strip()}")
            last_cmd = cmd

        # 2. Đọc frame mới từ webcam
        ret, frame = cap.read()
        if not ret:
            print("[FAKE CAR] Mất frame, retry...")
            continue

        # 3. Resize và encode JPEG
        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        ok, buf = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
        )
        if not ok:
            continue

        # 4. Đóng gói JSON và gửi
        payload = {"Img": base64.b64encode(buf).decode()}
        try:
            conn.sendall(json.dumps(payload).encode())
        except (BrokenPipeError, ConnectionResetError):
            print("[FAKE CAR] Connection broken")
            break

        # 5. FPS counter mỗi 100 frame
        frame_count += 1
        if frame_count % 100 == 0:
            elapsed = time.time() - start_time
            fps = frame_count / elapsed
            print(f"[FAKE CAR] FPS: {fps:.1f} | Frames sent: {frame_count}")

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("\n[FAKE CAR] Đang dừng...")

finally:
    conn.close()
    server.close()
    cap.release()
    elapsed = time.time() - start_time
    if frame_count > 0:
        print(f"[FAKE CAR] Tổng: {frame_count} frames trong {elapsed:.1f}s "
              f"({frame_count/elapsed:.1f} FPS trung bình)")
    print("[FAKE CAR] Đã đóng tất cả kết nối.")