"""Pure-numpy YOLO26-seg postprocessing for raw TensorRT/ONNX Runtime output.

Jetson's JetPack 4.6.3 ships Python 3.6.9; Ultralytics requires 3.8+, so
yolo26_unified.py (which imports `from ultralytics import YOLO`) cannot run
on the car. This module reimplements just enough of Ultralytics' own
postprocessing - box decode, per-class NMS, mask assembly - to consume the
two raw tensors a TensorRT engine returns directly, using nothing but
numpy. Verified against ultralytics' own decoded output on the same image
(see tests/test_yolo26_raw_decode.py) rather than assumed correct from the
math alone.

Input size is NOT fixed here. Every entry point takes model_h/model_w,
and TensorRTUnifiedYOLO26 passes the engine's own binding shape, so a
re-export at a different resolution cannot leave a stale constant
disagreeing with the engine that is actually loaded. (Confg.py's
UNIFIED_INFERENCE_SIZE still drives the ultralytics/.pt path only.)

  640x640 square : output0 (1,41,8400),  output1 (1,32,160,160)
  512x288 rect   : output0 (1,41,3024),  output1 (1,32,72,128)

The rectangular export matters more than it looks: a 640x360 camera
letterboxed into a square 640x640 spends 44% of the input on grey
padding, and the anchor count and prototype area shrink with it, so the
numpy postprocessing drops too. Measured on the test images, 288x512
against square 640: inference -42%, postprocessing -79%, with mask IoU
0.97 (road) / 0.93 (line) against the square result.

Class order comes from yolo26_unified.py's TRAIN_ID_TO_SEG_ID
(0=car,1=line,2=motobike,3=person,4=road) - imported, not redefined
here, so the two files cannot drift apart on which id means what.
"""
import numpy as np

from yolo26_unified import (
    CAR_TRAIN_ID,
    DETECTION_TRAIN_IDS,
    LINE_SEG_ID,
    MOTOBIKE_TRAIN_ID,
    PERSON_TRAIN_ID,
    ROAD_SEG_ID,
    TRAIN_ID_TO_SEG_ID,
)

NUM_CLASSES = 5
NUM_MASK_COEFFS = 32
# Default kept square for callers that predate the rectangular export. The
# TensorRT runtime does NOT use these: it reads the engine's own binding
# shape and passes it down, so a re-export at another size cannot leave a
# stale number here disagreeing with the engine actually loaded.
MODEL_INPUT_H = 640
MODEL_INPUT_W = 640
MODEL_INPUT_SIZE = 640


def letterbox_params(orig_h, orig_w, new_h=MODEL_INPUT_H, new_w=None):
    """Same convention as ultralytics: scale to fit, pad the remainder,
    padding split evenly on both sides. Returns (ratio, pad_x, pad_y).

    Accepts a rectangular target. A model exported at the camera's own
    aspect (288x512 for a 640x360 frame) carries almost no padding, so it
    spends none of its compute on grey pixels - unlike a square 640x640,
    where 44% of the input is padding for this camera.
    new_w defaults to new_h, which keeps every square caller unchanged."""
    if new_w is None:
        new_w = new_h
    ratio = min(new_h / orig_h, new_w / orig_w)
    scaled_h, scaled_w = round(orig_h * ratio), round(orig_w * ratio)
    pad_x = (new_w - scaled_w) / 2
    pad_y = (new_h - scaled_h) / 2
    return ratio, pad_x, pad_y


def letterbox_image(frame_bgr, new_h=MODEL_INPUT_H, new_w=None,
                    destination=None):
    """Returns (blob, ratio, pad_x, pad_y). blob is (1,3,H,W) float32 RGB
    0-1, matching Ultralytics' own preprocessing (BGR->RGB, HWC->CHW,
    /255), so a model trained through ultralytics decodes correctly.

    ``destination`` lets the TensorRT path write straight into its reusable
    input host buffer.  The old path allocated a 640x640 padded uint8 image,
    then a second 4.9 MB float blob, then copied that blob once more into the
    TensorRT buffer every frame.  Filling the final CHW buffer directly both
    removes the large copy and only converts the real image band (640x360 on
    the vehicle); the padding is filled as a scalar.
    """
    import cv2
    if new_w is None:
        new_w = new_h
    h, w = frame_bgr.shape[:2]
    ratio, pad_x, pad_y = letterbox_params(h, w, new_h, new_w)
    scaled_h, scaled_w = round(h * ratio), round(w * ratio)
    if (scaled_h, scaled_w) == (h, w):
        resized = frame_bgr
    else:
        resized = cv2.resize(
            frame_bgr, (scaled_w, scaled_h), interpolation=cv2.INTER_LINEAR)
    left, top = round(pad_x - 0.1), round(pad_y - 0.1)
    right = new_w - scaled_w - left
    bottom = new_h - scaled_h - top

    if destination is not None:
        expected = (1, 3, new_h, new_w)
        if destination.shape != expected or destination.dtype != np.float32:
            raise ValueError(
                "destination must be float32 with shape {}".format(expected)
            )
        destination.fill(np.float32(114.0 / 255.0))
        dst = destination[0, :, top:top + scaled_h, left:left + scaled_w]
        scale = np.float32(1.0 / 255.0)
        # BGR HWC -> RGB CHW, directly into the TensorRT host allocation.
        np.multiply(resized[:, :, 2], scale, out=dst[0], casting="unsafe")
        np.multiply(resized[:, :, 1], scale, out=dst[1], casting="unsafe")
        np.multiply(resized[:, :, 0], scale, out=dst[2], casting="unsafe")
        return destination, ratio, left, top

    padded = cv2.copyMakeBorder(resized, top, bottom, left, right,
                                cv2.BORDER_CONSTANT, value=(114, 114, 114))
    blob = padded[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
    return np.ascontiguousarray(blob[None]), ratio, left, top


def xywh_to_xyxy(boxes):
    cx, cy, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    return np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], axis=1)


def nms(boxes_xyxy, scores, class_ids, iou_thres=0.7):
    """Greedy per-class NMS (matches ultralytics' default agnostic_nms=False
    used to train this model). Returns indices to keep, highest score first
    within each class."""
    keep = []
    for cls in np.unique(class_ids):
        idx = np.where(class_ids == cls)[0]
        idx = idx[np.argsort(-scores[idx])]
        b = boxes_xyxy[idx]
        area = (b[:, 2] - b[:, 0]).clip(0) * (b[:, 3] - b[:, 1]).clip(0)
        suppressed = np.zeros(len(idx), dtype=bool)
        for i in range(len(idx)):
            if suppressed[i]:
                continue
            keep.append(idx[i])
            if i + 1 >= len(idx):
                break
            xx1 = np.maximum(b[i, 0], b[i + 1:, 0])
            yy1 = np.maximum(b[i, 1], b[i + 1:, 1])
            xx2 = np.minimum(b[i, 2], b[i + 1:, 2])
            yy2 = np.minimum(b[i, 3], b[i + 1:, 3])
            inter = (xx2 - xx1).clip(0) * (yy2 - yy1).clip(0)
            iou = inter / (area[i] + area[i + 1:] - inter + 1e-9)
            suppressed[i + 1:][iou > iou_thres] = True
    return np.array(sorted(keep), dtype=int)


def decode_detections(output0, conf_thres=0.4, iou_thres=0.7,
                      lane_conf_thres=None):
    """output0: (1, 41, N) raw model output.
    Returns (boxes_xyxy_model_space, class_ids, confidences, mask_coeffs) -
    boxes are still in the LETTERBOXED model space; rescale_boxes() maps
    them back to the original frame.

    lane_conf_thres applies to road/line only; None means "same as
    conf_thres". The two outputs have opposite false-positive costs, so one
    shared threshold has to be wrong for one of them:

      * A missed LINE stops the car. Measured on the night run 131525, at
        conf 0.40 only 22% of frames produced any line pixels and the car
        stood still 52% of the time; at 0.10 that becomes 92%. The model
        was seeing the line all along and the threshold discarded it.
        A spurious line costs one frame, because LANE_REACQUIRE_FRAMES
        needs three consecutive detections before motion resumes.
      * A spurious PERSON sends a real intruder alert over MQTT/Discord.
        Nothing debounces that on the receiving end.

    On daytime test images the threshold changes nothing at all (line and
    road mask IoU 1.0000 from 0.40 down to 0.10, same four detections), so
    this only takes effect where it is needed."""
    pred = output0[0].T  # (8400, 41)
    boxes = pred[:, :4]
    cls_scores = pred[:, 4:4 + NUM_CLASSES]
    mask_coeffs = pred[:, 4 + NUM_CLASSES:]

    class_ids = cls_scores.argmax(1)
    confidences = cls_scores.max(1)
    if lane_conf_thres is None:
        mask = confidences > conf_thres
    else:
        is_lane = np.isin(
            class_ids,
            [t for t, seg in TRAIN_ID_TO_SEG_ID.items()
             if seg in (ROAD_SEG_ID, LINE_SEG_ID)],
        )
        mask = confidences > np.where(is_lane, lane_conf_thres, conf_thres)
    if not mask.any():
        return (np.zeros((0, 4)), np.zeros(0, dtype=int),
               np.zeros(0), np.zeros((0, NUM_MASK_COEFFS)))

    boxes_xyxy = xywh_to_xyxy(boxes[mask])
    class_ids = class_ids[mask]
    confidences = confidences[mask]
    mask_coeffs = mask_coeffs[mask]

    keep = nms(boxes_xyxy, confidences, class_ids, iou_thres)
    return boxes_xyxy[keep], class_ids[keep], confidences[keep], mask_coeffs[keep]


def rescale_boxes(boxes_xyxy_640, ratio, pad_x, pad_y, orig_h, orig_w):
    """Undo letterbox_image()'s scale+pad, matching ultralytics'
    scale_boxes(). Clips to the original frame."""
    boxes = boxes_xyxy_640.copy()
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_x) / ratio
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_y) / ratio
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, orig_w)
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, orig_h)
    return boxes


def decode_masks(mask_coeffs, proto, boxes_xyxy_640, ratio, pad_x, pad_y,
                 orig_h, orig_w, mask_thr=0.5, model_h=MODEL_INPUT_H,
                 model_w=None):
    """proto: (1, 32, Hp, Wp) prototype masks (output1).
    Returns a list of (orig_h, orig_w) bool arrays, one per detection,
    already cropped to that detection's box - same shape ultralytics'
    own `masks.data` produces after retina_masks resize."""
    import cv2
    if model_w is None:
        model_w = model_h
    if len(mask_coeffs) == 0:
        return []
    proto = proto[0]  # (32, Hp, Wp)
    c, hp, wp = proto.shape
    combined = (mask_coeffs @ proto.reshape(c, -1)).reshape(-1, hp, wp)
    combined = 1 / (1 + np.exp(-np.clip(combined, -80, 80)))

    # Ultralytics crops every prototype mask to its detection box. The old
    # raw decoder accepted boxes_xyxy_640 but never used them, allowing mask
    # "ghosts" outside the instance box. Those ghosts were visible as cyan
    # fragments in run 103220 and could be joined to the real stripe by the
    # heading scan. Crop in 160x160 prototype space before upsampling.
    boxes_proto = boxes_xyxy_640.astype(np.float32).copy()
    boxes_proto[:, [0, 2]] *= float(wp) / model_w
    boxes_proto[:, [1, 3]] *= float(hp) / model_h
    xs = np.arange(wp, dtype=np.float32)[None, None, :]
    ys = np.arange(hp, dtype=np.float32)[None, :, None]
    x1 = boxes_proto[:, 0, None, None]
    y1 = boxes_proto[:, 1, None, None]
    x2 = boxes_proto[:, 2, None, None]
    y2 = boxes_proto[:, 3, None, None]
    combined *= ((xs >= x1) & (xs < x2) & (ys >= y1) & (ys < y2))

    # Remove letterbox padding while the masks are still only 160x160, then
    # resize the real image band directly to 640x360.  The old implementation
    # first expanded to 640x640, cropped 140 rows above/below, and resized a
    # second time.  That did two large OpenCV operations for the same result.
    proto_left = int(round(pad_x * wp / model_w))
    proto_top = int(round(pad_y * hp / model_h))
    content_w = max(1, int(round(orig_w * ratio * wp / model_w)))
    content_h = max(1, int(round(orig_h * ratio * hp / model_h)))
    proto_right = min(wp, proto_left + content_w)
    proto_bottom = min(hp, proto_top + content_h)
    combined = combined[:, proto_top:proto_bottom, proto_left:proto_right]

    # Resize all lane instances together. OpenCV drops the channel dimension
    # for N=1, so restore it before slicing below.
    stack = np.ascontiguousarray(combined.transpose(1, 2, 0))
    stack = cv2.resize(
        stack, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
    if stack.ndim == 2:
        stack = stack[:, :, None]
    return [stack[:, :, i] > mask_thr for i in range(stack.shape[2])]


def build_pred_mask_from_masks(class_ids, binary_masks, height, width):
    """Same priority rule as yolo26_unified.build_pred_mask (road, then
    line on top) but from full-resolution boolean masks instead of
    polygons - the raw decode path has no polygon step to reuse."""
    out = np.zeros((height, width), dtype=np.uint8)
    for target in (ROAD_SEG_ID, LINE_SEG_ID):
        for cls_id, m in zip(class_ids, binary_masks):
            if TRAIN_ID_TO_SEG_ID.get(int(cls_id)) == target:
                out[m] = target
    return out


def build_detections_from_boxes(class_ids, boxes_xyxy, confidences, min_area):
    detections = []
    for cls_id, bbox, confidence in zip(class_ids, boxes_xyxy, confidences):
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


def infer_from_raw_outputs(output0, output1, orig_h, orig_w, ratio, pad_x,
                            pad_y, conf_thres=0.4, iou_thres=0.7,
                            min_area=1500, model_h=MODEL_INPUT_H,
                            model_w=None, lane_conf_thres=None):
    """One call from the two tensors a TensorRT/ONNX Runtime session
    returns to the (pred_mask, persons) contract patrol_robot.py expects -
    the same contract yolo26_unified.UnifiedYOLO26.infer() produces on the
    training/laptop side."""
    boxes640, class_ids, confidences, mask_coeffs = decode_detections(
        output0, conf_thres, iou_thres, lane_conf_thres)
    boxes_orig = rescale_boxes(boxes640, ratio, pad_x, pad_y, orig_h, orig_w)
    # Lane control chi doc mask road/line. Car, person va motobike chi can
    # bbox cho canh bao; giai ma mask full-resolution cua chung tung chiem
    # CPU nhung ket qua bi build_pred_mask_from_masks bo ngay sau do.
    lane_keep = np.array([
        TRAIN_ID_TO_SEG_ID.get(int(cls_id)) in (ROAD_SEG_ID, LINE_SEG_ID)
        for cls_id in class_ids
    ], dtype=bool)
    lane_classes = class_ids[lane_keep]
    lane_masks = decode_masks(
        mask_coeffs[lane_keep], output1, boxes640[lane_keep],
        ratio, pad_x, pad_y, orig_h, orig_w,
        model_h=model_h, model_w=model_w,
    )
    pred_mask = build_pred_mask_from_masks(
        lane_classes, lane_masks, orig_h, orig_w)
    persons = build_detections_from_boxes(class_ids, boxes_orig, confidences,
                                          min_area)
    return pred_mask, persons
