"""
generate_markers.py — Tạo ảnh ArUco marker để in ra giấy
============================================================
Đã refactor để dùng Confg.py

Cài đặt: pip install opencv-contrib-python
Chạy:    python generate_markers.py

Sau đó in ra A4, ép plastic, dán tại các ngã rẽ trong khuôn viên.
"""

import cv2
import numpy as np
import os

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  IMPORT TỪ Confg.py
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
from Confg import (
    ARUCO_DICT_NAME,
    ARUCO_MARKER_SIZE,
    ARUCO_BORDER_SIZE,
    ARUCO_OUTPUT_DIR,
    ARUCO_MARKER_INFO,
)


# ── Tạo marker ───────────────────────────────────────────────

def create_marker(marker_id: int, info: dict) -> np.ndarray:
    """Tạo ảnh marker với viền và nhãn tên lệnh."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, ARUCO_DICT_NAME)
    )

    # Tạo marker đen trắng
    marker_img = cv2.aruco.generateImageMarker(
        aruco_dict, marker_id, ARUCO_MARKER_SIZE
    )

    # Thêm viền trắng
    marker_bordered = cv2.copyMakeBorder(
        marker_img,
        ARUCO_BORDER_SIZE, ARUCO_BORDER_SIZE,
        ARUCO_BORDER_SIZE, ARUCO_BORDER_SIZE,
        cv2.BORDER_CONSTANT, value=255,
    )

    # Chuyển sang BGR để vẽ màu
    output = cv2.cvtColor(marker_bordered, cv2.COLOR_GRAY2BGR)

    total_size = ARUCO_MARKER_SIZE + 2 * ARUCO_BORDER_SIZE

    # Vẽ thanh nhãn ở dưới
    label_h = 80
    canvas  = np.ones((total_size + label_h, total_size, 3), dtype=np.uint8) * 255
    canvas[:total_size, :] = output

    # Nền màu cho nhãn
    color = info["color"]
    canvas[total_size:, :] = color

    # Text: symbol + tên lệnh
    symbol = info["symbol"]
    cmd    = info["cmd"]
    label  = f"ID {marker_id}  {symbol}  {cmd}"

    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
    tx = (total_size - tw) // 2
    ty = total_size + (label_h + th) // 2 - 4

    cv2.putText(canvas, label, (tx, ty),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)

    return canvas


def create_placement_guide() -> np.ndarray:
    """Tạo ảnh hướng dẫn dán marker."""
    W, H = 800, 500
    guide = np.ones((H, W, 3), dtype=np.uint8) * 245

    cv2.putText(guide, "HUONG DAN DAN ARUCO MARKER",
                (80, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (30, 30, 30), 2)

    # Sơ đồ đường + xe
    cv2.rectangle(guide, (120, 120), (680, 380), (180, 180, 180), -1)
    cv2.line(guide, (400, 120), (400, 380), (255, 255, 255), 3)

    # Xe
    cv2.rectangle(guide, (160, 230), (240, 280), (50, 120, 200), -1)
    cv2.putText(guide, "Xe", (170, 262),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # Marker minh họa
    cv2.rectangle(guide, (330, 200), (400, 260), (255, 255, 255), -1)
    cv2.rectangle(guide, (340, 210), (390, 250), (0, 0, 0), -1)
    cv2.rectangle(guide, (350, 220), (380, 240), (255, 255, 255), -1)
    cv2.putText(guide, "Marker", (310, 290),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 1)

    # Mũi tên hướng đi
    cv2.arrowedLine(guide, (200, 220), (310, 230),
                    (0, 160, 0), 2, tipLength=0.3)

    # Ghi chú
    notes = [
        "- Dán marker vuông goc voi huong di cua xe",
        "- Chieu cao marker: 40-60cm so voi mat dat",
        "- Dam bao marker khong bi che khuat",
        "- Nen ep plastic de chong nuoc / bui ban",
        "- Khoang cach camera nhin thay tot nhat: 0.5-2m",
    ]
    for i, note in enumerate(notes):
        cv2.putText(guide, note, (80, 410 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (60, 60, 60), 1)

    return guide


# ── Main ─────────────────────────────────────────────────────

if __name__ == "__main__":
    os.makedirs(ARUCO_OUTPUT_DIR, exist_ok=True)

    print(f"Đang tạo {len(ARUCO_MARKER_INFO)} ArUco marker...")

    for marker_id, info in ARUCO_MARKER_INFO.items():
        img = create_marker(marker_id, info)
        filename = f"marker_{marker_id}_{info['cmd'].replace(' ', '_')}.png"
        path = os.path.join(ARUCO_OUTPUT_DIR, filename)
        cv2.imwrite(path, img)
        print(f"  ✓ {path}")

    guide_path = os.path.join(ARUCO_OUTPUT_DIR, "00_HUONG_DAN_DAN_MARKER.png")
    cv2.imwrite(guide_path, create_placement_guide())
    print(f"  ✓ {guide_path}")

    print(f"\nĐã tạo xong! Mở thư mục '{ARUCO_OUTPUT_DIR}/' và in các file PNG ra A4.")
    print("\nTóm tắt lệnh:")
    for mid, info in ARUCO_MARKER_INFO.items():
        print(f"  Marker ID {mid} → {info['symbol']} {info['cmd']}")
    print("\nGợi ý khi in:")
    print("  - In khổ A4 (21x29.7cm), không scale")
    print("  - Ép plastic hoặc bọc nylon nếu để ngoài trời")
    print("  - Dán ở độ cao 40-60cm, vuông góc với hướng đi xe")

    # Xem trước
    print("\nNhấn phím bất kỳ để đóng cửa sổ xem trước...")
    for marker_id, info in ARUCO_MARKER_INFO.items():
        img_path = os.path.join(
            ARUCO_OUTPUT_DIR,
            f"marker_{marker_id}_{info['cmd'].replace(' ', '_')}.png"
        )
        img = cv2.imread(img_path)
        cv2.imshow(f"Marker ID={marker_id} - {info['cmd']}", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()