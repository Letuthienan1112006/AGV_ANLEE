"""
Thu thap anh thuc te de label + train lai model segmentation.

Cach dung:
    python3 collect_images.py

Chay la chup ngay lap tuc, lien tuc moi 0.5s. Nhan Ctrl+C bat cu luc
nao de dung va luu lai toan bo anh da chup.
"""
import cv2
import os
import time
from datetime import datetime

INTERVAL_SEC = 0.5

session = datetime.now().strftime("%Y%m%d_%H%M%S")
out_dir = f"/home/agv/agv_git_clean/train_images_{session}"
os.makedirs(out_dir, exist_ok=True)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
if not cap.isOpened():
    print("KHONG MO DUOC CAMERA (co the con tien trinh khac dang giu /dev/video0)")
    raise SystemExit(1)

# warm up
for _ in range(10):
    cap.read()

print(f"DANG CHUP lien tuc, luu vao: {out_dir}")
print("Nhan Ctrl+C de dung va luu.")

count = 0
next_shot = time.time()

try:
    while True:
        now = time.time()
        if now >= next_shot:
            ok, frame = cap.read()
            if ok:
                fname = os.path.join(out_dir, f"img_{count:04d}.jpg")
                cv2.imwrite(fname, frame)
                count += 1
                if count % 10 == 0:
                    print(f"  ... {count} anh")
            next_shot += INTERVAL_SEC
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nDa dung.")
finally:
    cap.release()

print(f"XONG: {count} anh da luu vao {out_dir}")
