"""
2026-09-08: So sanh model CU (segmentation_mobilenetv2.tar, dang chay
tren xe) voi model MOI (segmentation_bridged.tar, train tren dataset da
noi khoang trong giua net dut) - chay ca 2 tren cung 1 tap anh test
(chua tung dung de train/valid ca 2 model), do do phu/thua cua class
"line" de xem viec noi label co lam model doan "line" day/on dinh hon
khong.

Chay: .venv_train/bin/python compare_models.py
"""
import json
import os
import sys

import numpy as np
import torch
from PIL import Image, ImageDraw
from torchvision import transforms

sys.path.insert(0, "/mnt/big/AGV_ANLEE")
from model import create_deeplabv3

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
SEG_INPUT_SIZE = (256, 448)
MEAN = [0.485, 0.456, 0.406]
STD = [0.229, 0.224, 0.225]
CLASS_NAMES = ["background", "road", "line", "car", "motobike", "person"]

OLD_CKPT = "/mnt/big/AGV_ANLEE/train_data/segmentation_mobilenetv2.tar"
NEW_CKPT = "/mnt/big/AGV_ANLEE/train_data/segmentation_bridged.tar"
TEST_DIR = "/mnt/big/AGV_ANLEE/train_data/dataset_v1_coco/test"
OUT_DIR = "/mnt/big/AGV_ANLEE/train_data/model_compare_preview"

os.makedirs(OUT_DIR, exist_ok=True)

transform = transforms.Compose([
    transforms.Resize(SEG_INPUT_SIZE),
    transforms.ToTensor(),
    transforms.Normalize(MEAN, STD),
])


def load_model(ckpt_path):
    model = create_deeplabv3(num_classes=len(CLASS_NAMES), backbone="mobilenetv2",
                              pretrained=False).to(DEVICE)
    ckpt = torch.load(ckpt_path, map_location=DEVICE)
    model.load_state_dict(ckpt.get("state_dict", ckpt), strict=False)
    model.eval()
    return model


def predict(model, pil_img):
    w, h = pil_img.size
    x = transform(pil_img).unsqueeze(0).to(DEVICE)
    with torch.no_grad():
        out = model(x)
    pred = torch.argmax(out, dim=1).squeeze(0).cpu().numpy().astype(np.uint8)
    pred_full = np.array(Image.fromarray(pred).resize((w, h), Image.NEAREST))
    return pred_full


def line_stats(pred_mask):
    line = pred_mask == 2
    total = line.sum()
    rows_with_line = int((line.sum(axis=1) > 0).sum())
    return int(total), rows_with_line, pred_mask.shape[0]


def overlay(pil_img, pred_mask):
    colors = {1: (128, 0, 128), 2: (0, 255, 255), 3: (0, 255, 0),
              4: (255, 165, 0), 5: (255, 0, 0)}
    base = pil_img.convert("RGB")
    color_layer = Image.new("RGB", base.size, (0, 0, 0))
    arr = np.array(color_layer)
    for cid, col in colors.items():
        arr[pred_mask == cid] = col
    color_layer = Image.fromarray(arr)
    return Image.blend(base, color_layer, 0.5)


def main():
    print("Loading models...")
    old_model = load_model(OLD_CKPT)
    new_model = load_model(NEW_CKPT)

    files = sorted(f for f in os.listdir(TEST_DIR) if f.endswith(".jpg"))
    step = max(1, len(files) // 20)
    sample = files[::step][:20]

    old_totals, new_totals = [], []
    old_rows, new_rows = [], []

    for i, fname in enumerate(sample):
        img = Image.open(os.path.join(TEST_DIR, fname)).convert("RGB")
        pred_old = predict(old_model, img)
        pred_new = predict(new_model, img)

        ot, orow, h = line_stats(pred_old)
        nt, nrow, _ = line_stats(pred_new)
        old_totals.append(ot); new_totals.append(nt)
        old_rows.append(orow); new_rows.append(nrow)

        if i < 12:
            combo = Image.new("RGB", (img.width * 2 + 4, img.height), (0, 0, 0))
            combo.paste(overlay(img, pred_old), (0, 0))
            combo.paste(overlay(img, pred_new), (img.width + 4, 0))
            combo.save(os.path.join(OUT_DIR, f"{i:02d}_{fname}.png"))

    print(f"\n=== KET QUA tren {len(sample)} anh test (chua tung train/valid) ===")
    print(f"So pixel class 'line' trung binh: CU={np.mean(old_totals):.0f}  "
          f"MOI={np.mean(new_totals):.0f}  "
          f"(x{np.mean(new_totals)/max(np.mean(old_totals),1):.2f})")
    print(f"So hang co it nhat 1 pixel 'line' (/{h} hang), trung binh: "
          f"CU={np.mean(old_rows):.1f}  MOI={np.mean(new_rows):.1f}  "
          f"(x{np.mean(new_rows)/max(np.mean(old_rows),1):.2f})")
    print(f"\nAnh so sanh (trai=CU, phai=MOI): {OUT_DIR}")


if __name__ == "__main__":
    main()
