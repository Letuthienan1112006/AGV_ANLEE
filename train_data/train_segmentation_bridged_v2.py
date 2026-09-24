"""
2026-09-08 v2: BAN SAO cua train_segmentation_bridged.py - v1 (30 epoch,
trong so "line" tu tinh ~1.02, gan nhu khong uu tien) so sanh voi model
cu cho ket qua GAN NHU KHONG DOI (+3% pixel trung binh, co ca ca giam).
v2 nay THU: (1) ep trong so "line" cao han han thay vi de tu tinh, (2)
tang so epoch vi val_loss o epoch 30 van dang giam, chua ro da hoi tu.
CHUA CHAC se tot hon v1 - day la thi nghiem, so sanh lai bang
compare_models.py sau khi xong truoc khi quyet dinh dua len xe.

BAN SAO cua train_segmentation.py, CHI doi DATA_ROOT/OUT_PATH
de train tren dataset_v1_coco_bridged (da noi cac doan net dut cung 1
vach thanh 1 polygon lien tuc - xem bridge_line_gaps.py). Khong dung de
len file train_segmentation.py goc/model dang deploy - de so sanh truoc/
sau truoc khi quyet dung ban nao.

Train lai model segmentation (DeepLabV3-MobileNetV2) tu dataset COCO
(dataset_v1_coco_bridged/{train,valid,test}/_annotations.coco.json).

Cach dung:
    python3 train_segmentation_bridged.py

Output: segmentation_bridged.tar (dung format {"state_dict": ...} khop
voi cach patrol_robot.py dang load model hien tai).
"""
import json
import os
import time

import numpy as np
import torch
import torch.nn as nn
from PIL import Image, ImageDraw
from pycocotools import mask as coco_mask
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms

import sys
sys.path.insert(0, "/mnt/big/AGV_ANLEE")
from model import create_deeplabv3


# ============================================================
#  CAU HINH
# ============================================================
DATA_ROOT = "/mnt/big/AGV_ANLEE/train_data/dataset_v1_coco_bridged"
OUT_PATH = "/mnt/big/AGV_ANLEE/train_data/segmentation_bridged_v2.tar"

# v2: ep trong so class "line" cao han han so voi tu tinh (~1.02 o v1,
# gan nhu khong uu tien gi). LINE_WEIGHT_BOOST nhan them vao trong so tu
# tinh - CHUA CO CO SO KHOA HOC cho con so 4.0, la diem khoi dau de thu
# nghiem, can xem lai ket qua compare_models.py sau khi train xong.
LINE_CLASS_INDEX  = 2
LINE_WEIGHT_BOOST = 4.0

SEG_INPUT_SIZE = (256, 448)  # (H, W) - khop voi Confg.py hien tai tren Jetson
SEG_NORMALIZE_MEAN = [0.485, 0.456, 0.406]
SEG_NORMALIZE_STD = [0.229, 0.224, 0.225]
SEG_BACKBONE = "mobilenetv2"

# Thu tu class - PHAI khop voi thu tu nay khi cap nhat CLASS_COLORS trong
# Confg.py sau khi train xong. 0 luon la background.
CLASS_NAMES = ["background", "road", "line", "car", "motobike", "person"]
NUM_CLASSES = len(CLASS_NAMES)

# Thu tu ve mask: ve truoc se bi ve de len boi lop sau (lop sau "thang").
# road ve truoc (nen lon), cac vat the cu the ve sau (de len tren road).
DRAW_ORDER = ["road", "line", "car", "motobike", "person"]

BATCH_SIZE = 8
NUM_EPOCHS = 60
LEARNING_RATE = 1e-4
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


# ============================================================
#  DATASET: doc COCO JSON, rasterize polygon -> mask
# ============================================================
class CocoSegDataset(Dataset):
    def __init__(self, split_dir, augment=False):
        self.split_dir = split_dir
        ann_path = os.path.join(split_dir, "_annotations.coco.json")
        with open(ann_path) as f:
            coco = json.load(f)

        # category_id (Roboflow) -> ten class
        self.cat_id_to_name = {c["id"]: c["name"] for c in coco["categories"]}
        # ten class -> label id (thu tu co dinh trong CLASS_NAMES)
        self.name_to_label = {name: i for i, name in enumerate(CLASS_NAMES)}

        # gom annotation theo image_id
        self.anns_by_image = {}
        for ann in coco["annotations"]:
            self.anns_by_image.setdefault(ann["image_id"], []).append(ann)

        self.images = coco["images"]
        self.augment = augment

        self.img_transform = transforms.Compose([
            transforms.Resize(SEG_INPUT_SIZE),
            transforms.ToTensor(),
            transforms.Normalize(SEG_NORMALIZE_MEAN, SEG_NORMALIZE_STD),
        ])

    def __len__(self):
        return len(self.images)

    def _rasterize_mask(self, img_info, anns):
        w, h = img_info["width"], img_info["height"]
        # Tat ca thao tac ve deu gop vao 1 mang numpy duy nhat (mask_np) de
        # tranh 2 buffer lech nhau giua PIL polygon va RLE numpy.
        mask_np = np.zeros((h, w), dtype=np.uint8)

        # gom annotation theo ten class de ve dung thu tu DRAW_ORDER
        anns_by_class = {}
        for ann in anns:
            cname = self.cat_id_to_name.get(ann["category_id"])
            if cname not in self.name_to_label:
                continue
            anns_by_class.setdefault(cname, []).append(ann)

        for cname in DRAW_ORDER:
            label_id = self.name_to_label[cname]
            for ann in anns_by_class.get(cname, []):
                seg = ann.get("segmentation", [])

                if isinstance(seg, dict):
                    # RLE (Roboflow xuat RLE cho mot so vung phuc tap)
                    rle = seg
                    if isinstance(rle["counts"], list):
                        rle = coco_mask.frPyObjects(rle, rle["size"][0], rle["size"][1])
                    m = coco_mask.decode(rle)
                    mask_np[m.astype(bool)] = label_id
                    continue

                # Polygon: list cac list [x1,y1,x2,y2,...].
                # Ve vao 1 canvas PIL tam thoi roi gop vao mask_np, khong
                # ve truc tiep vao mask_np de tranh lech buffer.
                for poly in seg:
                    if not isinstance(poly, list) or len(poly) < 6:
                        continue
                    pts = [(poly[i], poly[i + 1]) for i in range(0, len(poly), 2)]
                    tmp = Image.new("L", (w, h), 0)
                    ImageDraw.Draw(tmp).polygon(pts, fill=1)
                    tmp_np = np.array(tmp, dtype=bool)
                    mask_np[tmp_np] = label_id

        return Image.fromarray(mask_np, mode="L")

    def __getitem__(self, idx):
        img_info = self.images[idx]
        img_path = os.path.join(self.split_dir, img_info["file_name"])
        image = Image.open(img_path).convert("RGB")

        anns = self.anns_by_image.get(img_info["id"], [])
        mask = self._rasterize_mask(img_info, anns)

        image_t = self.img_transform(image)

        mask_resized = mask.resize(
            (SEG_INPUT_SIZE[1], SEG_INPUT_SIZE[0]), resample=Image.NEAREST
        )
        mask_t = torch.from_numpy(np.array(mask_resized, dtype=np.int64))

        return image_t, mask_t


# ============================================================
#  TRAIN
# ============================================================
def compute_class_weights(dataset, num_classes, max_samples=300):
    """Uoc luong ti le pixel moi class tu mot phan dataset de can bang loss
    (road/line se chiem da so pixel, car/motobike hiem hon nhieu)."""
    counts = np.zeros(num_classes, dtype=np.int64)
    n = min(len(dataset), max_samples)
    step = max(1, len(dataset) // n)
    for i in range(0, len(dataset), step):
        _, mask = dataset[i]
        vals, cnts = np.unique(mask.numpy(), return_counts=True)
        for v, c in zip(vals, cnts):
            counts[v] += c

    counts = np.maximum(counts, 1)
    freq = counts / counts.sum()
    weights = 1.0 / np.log(1.02 + freq)
    weights = weights / weights.sum() * num_classes
    return torch.tensor(weights, dtype=torch.float32)


def run_epoch(model, loader, criterion, optimizer=None):
    is_train = optimizer is not None
    model.train() if is_train else model.eval()

    total_loss = 0.0
    n_batches = 0

    with torch.set_grad_enabled(is_train):
        for images, masks in loader:
            images = images.to(DEVICE)
            masks = masks.to(DEVICE)

            outputs = model(images)
            loss = criterion(outputs, masks)

            if is_train:
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

            total_loss += loss.item()
            n_batches += 1

    return total_loss / max(n_batches, 1)


def main():
    print(f"Device: {DEVICE}")
    print(f"Classes ({NUM_CLASSES}): {CLASS_NAMES}")

    train_ds = CocoSegDataset(os.path.join(DATA_ROOT, "train"))
    valid_ds = CocoSegDataset(os.path.join(DATA_ROOT, "valid"))
    print(f"Train: {len(train_ds)} anh | Valid: {len(valid_ds)} anh")

    train_loader = DataLoader(
        train_ds, batch_size=BATCH_SIZE, shuffle=True,
        num_workers=4, pin_memory=True,
    )
    valid_loader = DataLoader(
        valid_ds, batch_size=BATCH_SIZE, shuffle=False,
        num_workers=2, pin_memory=True,
    )

    print("Dang uoc luong ti le class de can bang loss...")
    class_weights = compute_class_weights(train_ds, NUM_CLASSES).to(DEVICE)
    class_weights[LINE_CLASS_INDEX] *= LINE_WEIGHT_BOOST
    print(f"(v2: da nhan trong so '{CLASS_NAMES[LINE_CLASS_INDEX]}' voi "
          f"LINE_WEIGHT_BOOST={LINE_WEIGHT_BOOST})")
    for name, w in zip(CLASS_NAMES, class_weights.tolist()):
        print(f"  {name}: weight={w:.3f}")

    model = create_deeplabv3(
        num_classes=NUM_CLASSES, backbone=SEG_BACKBONE, pretrained=True
    ).to(DEVICE)

    criterion = nn.CrossEntropyLoss(weight=class_weights)
    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="min", factor=0.5, patience=3
    )

    best_val_loss = float("inf")
    for epoch in range(1, NUM_EPOCHS + 1):
        t0 = time.time()
        train_loss = run_epoch(model, train_loader, criterion, optimizer)
        val_loss = run_epoch(model, valid_loader, criterion)
        scheduler.step(val_loss)
        dt = time.time() - t0

        marker = ""
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save({"state_dict": model.state_dict()}, OUT_PATH)
            marker = " <- luu (val_loss tot nhat)"

        print(
            f"Epoch {epoch:3d}/{NUM_EPOCHS} | "
            f"train_loss={train_loss:.4f} | val_loss={val_loss:.4f} | "
            f"{dt:.0f}s{marker}"
        )

    print(f"\nXONG. Model tot nhat da luu tai: {OUT_PATH}")
    print(f"Best val_loss: {best_val_loss:.4f}")


if __name__ == "__main__":
    main()
