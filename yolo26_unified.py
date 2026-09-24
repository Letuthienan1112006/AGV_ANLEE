"""One YOLO26-seg model standing in for the two-model pipeline
(DeepLabV3 segmentation + YOLOv8n person/vehicle detection).

Output contracts match what patrol_robot.py already consumes from the two
separate models, so callers need no changes:
  - pred_mask: HxW uint8 array using the SAME ids as Confg.SEG_CLASS_NAMES
    (0=background, 1=road, 2=line, 3=car, 4=motobike, 5=person) - the
    exact array scan_lane()/detect_intersection_points()/decode_segmap()
    already expect.
  - persons: list of dicts shaped like detect_persons()'s return value
    (bbox, confidence, area, class_id, label) for summarize_detections()
    and the MQTT alert path.

The training dataset numbers its classes differently: 0=car, 1=line,
2=motobike, 3=person, 4=road. Those ids are one below the COCO json's
because Ultralytics' convert_coco writes `class = category_id - 1`, and
this Roboflow export numbers categories from 0 rather than COCO's 1 -
verified against per-class instance counts (road in 2673/2673 train
images, motobike in only 150). TRAIN_ID_TO_SEG_ID below is the only place
that mapping is spelled out - nothing else in this file or its caller
needs to know the training id order.

Status 2026-09-16: model da train 100 epoch, TensorRT engine da doi chieu
voi ONNX va da chay end-to-end voi camera/bridge/STM32 tren xe. YOLO26 la
backend mac dinh; AGV_USE_YOLO26_UNIFIED=0 chi danh cho doi chieu pipeline
cu co chu y.
"""
import numpy as np

# id in the trained YOLO26 model -> id in Confg.SEG_CLASS_NAMES
TRAIN_ID_TO_SEG_ID = {
    0: 3,  # car
    1: 2,  # line
    2: 4,  # motobike
    3: 5,  # person
    4: 1,  # road
}
ROAD_SEG_ID  = 1
LINE_SEG_ID  = 2
PERSON_TRAIN_ID   = 3
CAR_TRAIN_ID      = 0
MOTOBIKE_TRAIN_ID = 2
DETECTION_TRAIN_IDS = (PERSON_TRAIN_ID, CAR_TRAIN_ID, MOTOBIKE_TRAIN_ID)


def build_pred_mask(classes, polys, height, width):
    """Rasterise instance polygons into the dense class mask scan_lane()
    expects. Road is painted first and line second, so a pixel claimed by
    both ends up line - the same priority scan_lane() applies when it
    checks class 2 before falling back to class 1, rather than leaving the
    outcome to whichever instance the model happened to emit last."""
    import cv2
    mask = np.zeros((height, width), dtype=np.uint8)
    for target in (ROAD_SEG_ID, LINE_SEG_ID):
        for cls_id, poly in zip(classes, polys):
            if TRAIN_ID_TO_SEG_ID.get(int(cls_id)) != target:
                continue
            poly = np.asarray(poly)
            if len(poly) < 3:
                continue
            cv2.fillPoly(mask, [poly.astype(np.int32)], int(target))
    return mask


def build_detections(classes, boxes_xyxy, confs, min_area):
    """Shape detections like detect_persons() did, so summarize_detections()
    and the MQTT alert path are unchanged. Motobike joins car under "Xe":
    the old YOLOv8n filter only ever asked COCO for person and car, so
    keeping two labels avoids changing what the alert text can say."""
    detections = []
    for cls_id, bbox, confidence in zip(classes, boxes_xyxy, confs):
        cls_id = int(cls_id)
        if cls_id not in DETECTION_TRAIN_IDS:
            continue
        area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
        if area < min_area:
            continue
        detections.append({
            "bbox":       bbox,
            "confidence": float(confidence),
            "area":       float(area),
            "class_id":   cls_id,
            "label":      "Nguoi" if cls_id == PERSON_TRAIN_ID else "Xe",
        })
    return detections


class UnifiedYOLO26:
    """Wraps one ultralytics YOLO(-seg) model; one call gets both outputs."""

    def __init__(self, weights_path, device="cuda:0", conf=0.55,
                 imgsz=640, min_area=1500):
        from ultralytics import YOLO
        self.model = YOLO(weights_path)
        self.device = device
        self.conf = conf
        self.imgsz = imgsz
        self.min_area = min_area

    def warmup(self, height=360, width=640):
        dummy = np.zeros((height, width, 3), dtype=np.uint8)
        self.model(dummy, imgsz=self.imgsz, conf=self.conf,
                   verbose=False, device=self.device)

    def infer(self, frame_bgr):
        """Returns (pred_mask, persons) - see module docstring for shapes."""
        h, w = frame_bgr.shape[:2]
        results = self.model(frame_bgr, imgsz=self.imgsz, conf=self.conf,
                             verbose=False, device=self.device)
        r = results[0]
        if r.boxes is None or len(r.boxes) == 0:
            return np.zeros((h, w), dtype=np.uint8), []

        classes = r.boxes.cls.cpu().numpy().astype(int)
        polys = [] if r.masks is None else r.masks.xy
        return (
            build_pred_mask(classes, polys, h, w),
            build_detections(classes, r.boxes.xyxy.cpu().numpy(),
                             r.boxes.conf.cpu().numpy(), self.min_area),
        )
