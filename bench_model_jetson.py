"""
bench_model_jetson.py — kiem tra model segmentation CHAY DUOC tren Jetson
va do toc do suy luan that, KHONG can camera/STM32/LiDAR.

Muc dich: bat loi som truoc khi ra xe. Model duoc train tren laptop bang
torch moi (2.14 + CUDA 13); Jetson chay torch cu hon nhieu (JetPack). File
.tar chi chua state_dict (tensor thuan) nen ve ly thuyet tuong thich, nhung
phai chay that moi chac - va con phai do toc do: model qua cham thi vong
dieu khien tre, xe phan ung khong kip.

Chay tren Jetson:
    cd ~/agv_git_clean && python3 bench_model_jetson.py
    python3 bench_model_jetson.py /mnt/ssd/agv_models/segmentation_mobilenetv2.tar
"""
import glob
import os
import sys
import time

import numpy as np
import torch
from PIL import Image
from torchvision import transforms

from model import create_deeplabv3
from Confg import (
    SEGMENTATION_MODEL_PATH, SEG_INPUT_SIZE, SEG_NORMALIZE_MEAN,
    SEG_NORMALIZE_STD, SEG_NUM_CLASSES, SEG_BACKBONE,
    INTERSECTION_SCAN_Y, HEADING_SCAN_Y_FAR, LINE_MIN_PIXELS,
)

CKPT = sys.argv[1] if len(sys.argv) > 1 else SEGMENTATION_MODEL_PATH
IMG_DIR = "./bench_imgs"

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"torch={torch.__version__}  device={device}")
print(f"Model: {CKPT}")

t0 = time.time()
model = create_deeplabv3(num_classes=SEG_NUM_CLASSES, backbone=SEG_BACKBONE,
                          pretrained=False).to(device)
ckpt = torch.load(CKPT, map_location=device)
state = ckpt.get("state_dict", ckpt)
missing, unexpected = model.load_state_dict(state, strict=False)
model.eval()
use_fp16 = device.type == "cuda"
if use_fp16:
    model = model.half()
print(f"Nap model: {time.time()-t0:.1f}s  fp16={use_fp16}")
if missing:
    print(f"  [CANH BAO] {len(missing)} tham so THIEU trong file: {list(missing)[:5]}")
if unexpected:
    print(f"  [CANH BAO] {len(unexpected)} tham so THUA trong file: {list(unexpected)[:5]}")
if not missing and not unexpected:
    print("  Khop hoan toan state_dict - khong thieu/thua tham so nao")

transform = transforms.Compose([
    transforms.Resize(SEG_INPUT_SIZE),
    transforms.ToTensor(),
    transforms.Normalize(SEG_NORMALIZE_MEAN, SEG_NORMALIZE_STD),
])

files = sorted(glob.glob(os.path.join(IMG_DIR, "*.jpg")))
if not files:
    print(f"[LOI] khong co anh trong {IMG_DIR}")
    sys.exit(1)

times, near_px, far_px = [], [], []
with torch.no_grad():
    for i, f in enumerate(files):
        img = Image.open(f).convert("RGB")
        x = transform(img).unsqueeze(0).to(device)
        if use_fp16:
            x = x.half()
        if device.type == "cuda":
            torch.cuda.synchronize()
        t = time.time()
        out = model(x)
        pred_small = torch.argmax(out, dim=1).squeeze(0).cpu().numpy().astype(np.uint8)
        if device.type == "cuda":
            torch.cuda.synchronize()
        dt = (time.time() - t) * 1000
        if i > 0:              # bo lan dau (warm-up)
            times.append(dt)

        import cv2
        w, h = img.size
        pred = cv2.resize(pred_small, (w, h), interpolation=cv2.INTER_NEAREST)
        pred[pred == 4] = 1
        sy = min(INTERSECTION_SCAN_Y, h - 1)
        fy = min(HEADING_SCAN_Y_FAR, h - 1)
        near_px.append(int((pred[sy] == 2).sum()))
        far_px.append(int((pred[fy] == 2).sum()))

t = np.array(times)
print(f"\n=== TOC DO ({len(t)} lan, da bo warm-up) ===")
print(f"  trung binh={t.mean():.0f}ms  min={t.min():.0f}ms  max={t.max():.0f}ms"
      f"  -> {1000/t.mean():.1f} khung/giay (chi rieng segmentation)")

n = np.array(near_px); fp = np.array(far_px)
print(f"\n=== DAU RA (nguong LINE_MIN_PIXELS={LINE_MIN_PIXELS}) ===")
print(f"  pixel vach tai hang gan y={INTERSECTION_SCAN_Y}: min={n.min()} median={np.median(n):.0f} max={n.max()}")
print(f"  pixel vach tai hang xa  y={HEADING_SCAN_Y_FAR}: min={fp.min()} median={np.median(fp):.0f} max={fp.max()}")
print(f"  so anh dat nguong o hang gan: {(n >= LINE_MIN_PIXELS).sum()}/{len(n)}")
print(f"  so anh dat nguong o hang xa : {(fp >= LINE_MIN_PIXELS).sum()}/{len(fp)}")
