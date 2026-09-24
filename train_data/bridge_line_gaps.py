"""
bridge_line_gaps.py — Noi cac doan net dut CUNG 1 vach "line" thanh 1
polygon lien tuc (bao gom ca khoang trong giua 2 net dut), thay vi de
nguyen nhieu polygon nho roi rac nhu dataset goc.

BOI CANH (2026-09-08): kiem tra lai train_data/dataset_v1_coco cho thay
moi doan net dut duoc gan nhan RIENG LE (vd anh img_0044 co 4 annotation
"line" tach roi cho CUNG 1 vach, dien tich giam dan theo xa: 5534 -> 199
-> 35 -> 15px^2 - doan xa nhat chi 15 pixel vuong, gan nhu 1 diem cham o
do phan giai train 256x448). Nghi day la ly do model that ra "line" thua/
roi rac khi chay that (khong han la nhieu - co the model dang hoc DUNG
nhung gi duoc day: doan net dut roi rac). Script nay THU noi cac doan
cung 1 vach lai truoc khi phai label lai bang tay toan bo.

Cach lam (khong dung nhan tay, tu dong theo hinh hoc):
1. Voi moi anh, gom cac annotation category "line".
2. "Chain" (noi chuoi) cac doan theo quy dao: xet tu GAN (y lon, duoi anh)
   den XA (y nho, tren anh) - dung vi day la goc nhin xe huong toi truoc,
   vat gan hon o duoi anh. Doan moi duoc gan vao chain co diem CUOI du
   doan (ngoai suy tuyen tinh tu 2 diem gan nhat) gan voi tam cua doan do
   nhat, VA khoang cach theo y khong qua MAX_BRIDGE_Y_GAP (tranh noi nham
   2 vach khac nhau, hoac noi xuyen qua giao lo/cho vach that su ket thuc).
3. Voi moi chain >=2 doan: giu nguyen polygon goc cua tung doan (khong mat
   do chinh xac vien), THEM cac polygon "cau noi" = convex hull cua tung
   CAP doan lien tiep (khong lay hull toan bo chain - tranh phinh to qua o
   cac khuc cong, chi noi cuc bo tung cap mot).
4. Gop tat ca thanh 1 annotation "line" duy nhat (segmentation = list nhieu
   polygon con), xoa cac annotation doan le da gop. Anh KHONG bi doi (chi
   copy sang thu muc dich), cac category khac (road/car/motobike/person)
   giu nguyen 100%.

Dau ra: 1 dataset MOI (khong dung de len dataset_v1_coco goc) tai
DST_ROOT, dung dinh dang COCO y het dataset goc -> train_segmentation.py
chi can doi DATA_ROOT la dung duoc ngay, khong can sua gi them.

Chay:
    python3 bridge_line_gaps.py            # xu ly ca train/valid/test
    python3 bridge_line_gaps.py --preview N # xuat them N anh so sanh
                                             # truoc/sau de kiem tra bang mat
"""
import argparse
import copy
import json
import os
import shutil

import numpy as np
from PIL import Image, ImageDraw

SRC_ROOT = "/mnt/big/AGV_ANLEE/train_data/dataset_v1_coco"
DST_ROOT = "/mnt/big/AGV_ANLEE/train_data/dataset_v1_coco_bridged"
SPLITS = ["train", "valid", "test"]
LINE_CATEGORY_NAME = "line"

# --- Nguong noi chain (CAN kiem tra lai bang mat qua --preview truoc khi
# dung de train that) ---
MAX_BRIDGE_Y_GAP   = 60   # px (o anh 256 cao) - qua nguong nay coi nhu
                           # doan khac / het vach, KHONG noi
MAX_BRIDGE_X_DRIFT = 40   # px - sai lech cho phep giua vi tri du doan
                           # (ngoai suy tuyen tinh) va vi tri thuc te


def poly_xy(poly):
    return list(zip(poly[0::2], poly[1::2]))


def centroid(poly):
    pts = poly_xy(poly)
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return cx, cy


def convex_hull(points):
    """Andrew's monotone chain. points: list[(x,y)] -> list[(x,y)] (CCW)."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def build_chains(items):
    """items: list of dict(cx,cy,...). Tra ve list cac chain (list cac
    item), sap theo thu tu GAN->XA trong tung chain."""
    ordered = sorted(items, key=lambda a: -a["cy"])
    chains = []
    for it in ordered:
        best_chain, best_dev = None, None
        for ch in chains:
            last = ch[-1]
            y_gap = last["cy"] - it["cy"]
            if y_gap <= 0 or y_gap > MAX_BRIDGE_Y_GAP:
                continue
            if len(ch) >= 2:
                prev = ch[-2]
                dy = last["cy"] - prev["cy"]
                slope = 0.0 if abs(dy) < 1e-6 else (last["cx"] - prev["cx"]) / dy
                pred_x = last["cx"] + slope * (it["cy"] - last["cy"])
            else:
                pred_x = last["cx"]
            dev = abs(pred_x - it["cx"])
            if dev > MAX_BRIDGE_X_DRIFT:
                continue
            if best_dev is None or dev < best_dev:
                best_dev, best_chain = dev, ch
        if best_chain is not None:
            best_chain.append(it)
        else:
            chains.append([it])
    return chains


def rasterize_polys(polys, w, h):
    im = Image.new("L", (w, h), 0)
    draw = ImageDraw.Draw(im)
    for poly in polys:
        pts = poly_xy(poly)
        if len(pts) >= 3:
            draw.polygon(pts, fill=1)
    return np.array(im)


def merge_chain(chain, img_w, img_h, next_ann_id):
    """chain: list item(anns=1 annotation goc). Tra ve 1 annotation moi da
    gop (segmentation = doan goc + cac hull cau noi giua cap lien tiep)."""
    base = chain[0]["ann"]
    segs = [it["ann"]["segmentation"][0] for it in chain]
    for a, b in zip(chain, chain[1:]):
        pts = poly_xy(a["ann"]["segmentation"][0]) + poly_xy(b["ann"]["segmentation"][0])
        hull = convex_hull(pts)
        if len(hull) >= 3:
            flat = [c for p in hull for c in p]
            segs.append(flat)

    mask = rasterize_polys(segs, img_w, img_h)
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        bbox = base["bbox"]
        area = base["area"]
    else:
        x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
        bbox = [float(x0), float(y0), float(x1 - x0 + 1), float(y1 - y0 + 1)]
        area = float(mask.sum())

    merged = copy.deepcopy(base)
    merged["id"] = next_ann_id
    merged["segmentation"] = segs
    merged["bbox"] = bbox
    merged["area"] = area
    merged["iscrowd"] = 0
    return merged


def process_split(split, stats, preview_n=0, preview_dir=None):
    src_dir = os.path.join(SRC_ROOT, split)
    dst_dir = os.path.join(DST_ROOT, split)
    os.makedirs(dst_dir, exist_ok=True)

    ann_path = os.path.join(src_dir, "_annotations.coco.json")
    if not os.path.isfile(ann_path):
        print(f"[SKIP] khong thay {ann_path}")
        return

    with open(ann_path) as f:
        coco = json.load(f)

    cat_id_to_name = {c["id"]: c["name"] for c in coco["categories"]}
    line_cat_id = next(
        (cid for cid, n in cat_id_to_name.items() if n == LINE_CATEGORY_NAME),
        None,
    )
    img_by_id = {im["id"]: im for im in coco["images"]}

    anns_by_img = {}
    for a in coco["annotations"]:
        anns_by_img.setdefault(a["image_id"], []).append(a)

    next_ann_id = max((a["id"] for a in coco["annotations"]), default=0) + 1
    new_annotations = []
    n_images_changed = 0
    n_chains_bridged = 0
    n_dashes_absorbed = 0
    preview_done = 0

    for img_id, anns in anns_by_img.items():
        img_info = img_by_id[img_id]
        w, h = img_info["width"], img_info["height"]

        line_anns = [a for a in anns if a["category_id"] == line_cat_id]
        other_anns = [a for a in anns if a["category_id"] != line_cat_id]
        new_annotations.extend(other_anns)

        if not line_anns:
            continue

        items = []
        for a in line_anns:
            if not isinstance(a.get("segmentation"), list) or not a["segmentation"]:
                new_annotations.append(a)  # RLE hoac rong - giu nguyen, khong gop
                continue
            cx, cy = centroid(a["segmentation"][0])
            items.append({"ann": a, "cx": cx, "cy": cy})

        chains = build_chains(items)
        image_changed = False
        chain_polys_before, chain_polys_after = [], []

        for ch in chains:
            if len(ch) == 1:
                new_annotations.append(ch[0]["ann"])
                continue
            merged = merge_chain(ch, w, h, next_ann_id)
            next_ann_id += 1
            new_annotations.append(merged)
            n_chains_bridged += 1
            n_dashes_absorbed += len(ch)
            image_changed = True
            chain_polys_before.extend(it["ann"]["segmentation"][0] for it in ch)
            chain_polys_after.extend(merged["segmentation"])

        if image_changed:
            n_images_changed += 1
            if preview_n and preview_done < preview_n and preview_dir:
                _save_preview(
                    os.path.join(src_dir, img_info["file_name"]),
                    chain_polys_before, chain_polys_after, w, h,
                    os.path.join(preview_dir, f"{split}_{preview_done:03d}_{img_info['file_name']}.png"),
                )
                preview_done += 1

        # copy anh sang dataset moi (chi khi chua co, tranh copy lai nhieu lan)
        src_img = os.path.join(src_dir, img_info["file_name"])
        dst_img = os.path.join(dst_dir, img_info["file_name"])
        if not os.path.isfile(dst_img) and os.path.isfile(src_img):
            shutil.copy2(src_img, dst_img)

    # copy not qua cac anh KHONG co line annotation (chua duoc copy o tren)
    for img_info in coco["images"]:
        src_img = os.path.join(src_dir, img_info["file_name"])
        dst_img = os.path.join(dst_dir, img_info["file_name"])
        if not os.path.isfile(dst_img) and os.path.isfile(src_img):
            shutil.copy2(src_img, dst_img)

    coco["annotations"] = new_annotations
    with open(os.path.join(dst_dir, "_annotations.coco.json"), "w") as f:
        json.dump(coco, f)

    stats[split] = {
        "n_images": len(coco["images"]),
        "n_images_changed": n_images_changed,
        "n_chains_bridged": n_chains_bridged,
        "n_dashes_absorbed": n_dashes_absorbed,
    }


def _save_preview(img_path, before_polys, after_polys, w, h, out_path):
    if not os.path.isfile(img_path):
        return
    base = Image.open(img_path).convert("RGB").resize((w, h))

    def overlay(polys, color):
        im = base.copy()
        draw = ImageDraw.Draw(im, "RGBA")
        for poly in polys:
            pts = poly_xy(poly)
            if len(pts) >= 3:
                draw.polygon(pts, fill=color)
        return im

    before_im = overlay(before_polys, (255, 255, 0, 120))
    after_im = overlay(after_polys, (0, 255, 255, 120))
    combo = Image.new("RGB", (w * 2 + 4, h), (0, 0, 0))
    combo.paste(before_im, (0, 0))
    combo.paste(after_im, (w + 4, 0))
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    combo.save(out_path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--preview", type=int, default=0,
                     help="So anh so sanh truoc/sau xuat ra de kiem tra bang mat")
    args = ap.parse_args()

    preview_dir = os.path.join(DST_ROOT, "_preview") if args.preview else None
    if preview_dir and os.path.isdir(preview_dir):
        shutil.rmtree(preview_dir)

    stats = {}
    for split in SPLITS:
        process_split(split, stats, preview_n=args.preview, preview_dir=preview_dir)

    print("\n=== KET QUA ===")
    for split, s in stats.items():
        print(f"[{split}] {s['n_images']} anh, {s['n_images_changed']} anh co gop "
              f"chain, {s['n_chains_bridged']} chain duoc noi "
              f"(gom {s['n_dashes_absorbed']} doan net dut rieng le -> "
              f"{s['n_chains_bridged']} vach lien tuc)")
    print(f"\nDataset moi: {DST_ROOT}")
    if preview_dir:
        print(f"Anh preview (trai=truoc/vang, phai=sau/cyan): {preview_dir}")


if __name__ == "__main__":
    main()
