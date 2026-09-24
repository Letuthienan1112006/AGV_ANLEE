"""Inspect stationary camera frames; calibration history is explicit opt-in.

Preferred entrypoint (starts/stops the bridge):
    run snap
    run snap --session mount-a-20260911 --confirm-placement

Direct use with an already-running bridge:
    python3 snapshot_check.py 3 --session mount-a-20260911 --confirm-placement

Each confirmed invocation must follow a NEW measured placement of the VEHICLE
center over a straight line, body parallel to the line. Camera must stay fixed.
Use a NEW session after remounting the camera. Never infer this pose from the
image itself. Inspection alone never appends calibration history or changes code.
"""
import argparse
import base64
import hashlib
import json
import os
import socket
import time
import uuid

from camera_calibration import (
    HISTORY_VERSION, MIN_VALID_SAMPLES, MAX_HEADING_DEG,
    MAX_CENTER_SPREAD_PX, measure_line, propose_center,
    session_placements, summarize_placement,
)


# Do not pool the old center_calib.csv, which had no pose/config/session checks.
CALIB_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "center_calib_v2.jsonl")
OUT_DIR = "./snapshots"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("count", nargs="?", type=int, default=MIN_VALID_SAMPLES,
                        help="so anh (mac dinh: 3); chi quan sat neu khong xac nhan")
    parser.add_argument("--session", help="ten phien; doi ten sau khi doi gia/camera")
    parser.add_argument("--confirm-placement", action="store_true",
                        help="xac nhan VUA DAT LAI xe dung tam bang thuoc, song song vach thang; camera khong doi")
    args = parser.parse_args(argv)
    if args.count < 1:
        parser.error("count phai >= 1")
    if args.session is not None:
        args.session = args.session.strip()
        if not args.session:
            parser.error("--session khong duoc rong")
    if args.confirm_placement and not args.session:
        parser.error("--confirm-placement can --session TEN_PHIEN")
    if args.confirm_placement and args.count < MIN_VALID_SAMPLES:
        parser.error("ghi hieu chinh can it nhat %d anh" % MIN_VALID_SAMPLES)
    return args


def recv_json_frame(sock, max_bytes=2_000_000):
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
    raise RuntimeError("JSON frame vuot qua gioi han")


def _ask_frame(sock):
    # This helper only submits STOP, never a positive speed.
    sock.sendall(b"0 0")
    return recv_json_frame(sock)


class SnapshotInspector:
    def __init__(self):
        # Lazy imports keep --help and offline unit tests free of camera/GPU work.
        import cv2
        import numpy as np
        import Confg

        self.cv2, self.np = cv2, np
        self.config = Confg
        self.use_unified = (
            os.environ.get("AGV_USE_YOLO26_UNIFIED", "1") not in ("", "0")
        )
        self.fp16 = False
        self.device = None
        self.transform = None
        self.model_path = None

        if self.use_unified:
            if os.path.exists(Confg.UNIFIED_ENGINE_PATH):
                from yolo26_tensorrt_runtime import TensorRTUnifiedYOLO26
                self.backend = "yolo26_tensorrt"
                self.model_path = Confg.UNIFIED_ENGINE_PATH
                self.model = TensorRTUnifiedYOLO26(
                    self.model_path,
                    conf_thres=Confg.UNIFIED_CONF_THRESHOLD,
                    min_area=Confg.PERSON_MIN_AREA,
                )
            elif os.path.exists(Confg.UNIFIED_MODEL_PATH):
                from yolo26_unified import UnifiedYOLO26
                self.backend = "yolo26_ultralytics"
                self.model_path = Confg.UNIFIED_MODEL_PATH
                self.model = UnifiedYOLO26(
                    self.model_path,
                    device=Confg.PERSON_DEVICE,
                    conf=Confg.UNIFIED_CONF_THRESHOLD,
                    imgsz=Confg.UNIFIED_INFERENCE_SIZE,
                    min_area=Confg.PERSON_MIN_AREA,
                )
            else:
                raise FileNotFoundError(
                    "Khong thay model YOLO26: %s hoac %s" % (
                        Confg.UNIFIED_ENGINE_PATH, Confg.UNIFIED_MODEL_PATH))
            print("[INIT] Hieu chinh bang %s (%s)..." %
                  (self.backend, self.model_path))
            self.model.warmup()
        else:
            # Chi giu de doi chieu co chu y. Khong duoc tron ket qua nay
            # vao phien hieu chinh YOLO26: hai model dat mask line khac nhau.
            import torch
            from PIL import Image
            from torchvision import transforms
            from model import create_deeplabv3

            self.torch, self.Image = torch, Image
            self.backend = "legacy_deeplab"
            self.model_path = Confg.SEGMENTATION_MODEL_PATH
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.fp16 = self.device.type == "cuda"
            print("[INIT] Hieu chinh bang DeepLab CU (%s)..." % self.model_path)
            self.model = create_deeplabv3(
                num_classes=Confg.SEG_NUM_CLASSES,
                backbone=Confg.SEG_BACKBONE,
                pretrained=False,
            ).to(self.device)
            ckpt = torch.load(self.model_path, map_location=self.device)
            self.model.load_state_dict(ckpt.get("state_dict", ckpt), strict=False)
            self.model.eval()
            if self.fp16:
                self.model = self.model.half()
            self.transform = transforms.Compose([
                transforms.Resize(Confg.SEG_INPUT_SIZE),
                transforms.ToTensor(),
                transforms.Normalize(Confg.SEG_NORMALIZE_MEAN,
                                     Confg.SEG_NORMALIZE_STD),
            ])
        os.makedirs(OUT_DIR, exist_ok=True)

    def identity(self):
        """Reject pooling changed model/config; physical remount needs a new ID."""
        config = self.config
        digest = hashlib.sha256()
        with open(self.model_path, "rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return {
            "measurement": "controller-line-band-heading-v1",
            "vision_backend": self.backend,
            "source": "bridge-jpeg",
            "camera_index": config.CAMERA_INDEX,
            "width": config.CAMERA_WIDTH, "height": config.CAMERA_HEIGHT,
            "near_y": config.INTERSECTION_SCAN_Y,
            "far_y": config.HEADING_SCAN_Y_FAR,
            "band": config.LINE_SCAN_BAND, "min_pixels": config.LINE_MIN_PIXELS,
            "heading_weight": config.HEADING_ERROR_WEIGHT,
            "input_size": (config.UNIFIED_INFERENCE_SIZE if self.use_unified
                           else list(config.SEG_INPUT_SIZE)),
            "confidence": (config.UNIFIED_CONF_THRESHOLD
                           if self.use_unified else None),
            "normalize_mean": (None if self.use_unified
                               else list(config.SEG_NORMALIZE_MEAN)),
            "normalize_std": (None if self.use_unified
                              else list(config.SEG_NORMALIZE_STD)),
            "backbone": (None if self.use_unified else config.SEG_BACKBONE),
            "classes": list(config.SEG_CLASS_NAMES),
            "model_sha256": digest.hexdigest(), "fp16": self.fp16,
            "min_samples": MIN_VALID_SAMPLES, "max_heading": MAX_HEADING_DEG,
            "max_spread": MAX_CENTER_SPREAD_PX,
        }

    def one_shot(self, sock):
        cv2, np, config = self.cv2, self.np, self.config
        recv = _ask_frame(sock)
        jpg = base64.b64decode(recv["Img"])
        bgr = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None or bgr.shape[:2] != (config.CAMERA_HEIGHT, config.CAMERA_WIDTH):
            raise ValueError("anh bridge khong dung CAMERA_WIDTH/HEIGHT; khong hieu chinh")
        infer_started = time.monotonic()
        if self.use_unified:
            pred, _detections = self.model.infer(bgr)
        else:
            pil_im = self.Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            tensor = self.transform(pil_im).unsqueeze(0).to(self.device)
            if self.fp16:
                tensor = tensor.half()
            with self.torch.no_grad():
                small = self.torch.argmax(
                    self.model(tensor), dim=1).squeeze(0).cpu().numpy()
            pred = cv2.resize(
                small.astype(np.uint8),
                (config.CAMERA_WIDTH, config.CAMERA_HEIGHT),
                interpolation=cv2.INTER_NEAREST,
            )
        infer_ms = (time.monotonic() - infer_started) * 1000.0
        timing = getattr(self.model, "last_timing_ms", None)
        if timing:
            print("  infer=%.0f ms (pre=%.0f, H2D=%.0f, execute=%.0f, "
                  "D2H=%.0f, post=%.0f)" % (
                infer_ms, timing["pre"], timing["h2d"], timing["execute"],
                timing["d2h"], timing["post"],
            ))
        else:
            print("  infer=%.0f ms" % infer_ms)
        pred[pred == 4] = 1
        print("  road=%d px; line=%d px%s" %
              (int((pred == 1).sum()), int((pred == 2).sum()),
               "; IMU bao LIFTED - kiem tra xe da dat tren mat dat" if recv.get("lifted") else ""))
        sample = measure_line(pred, config.INTERSECTION_SCAN_Y,
                              config.HEADING_SCAN_Y_FAR, config.LINE_SCAN_BAND,
                              config.LINE_MIN_PIXELS, config.HEADING_ERROR_WEIGHT)
        color = np.zeros((*pred.shape, 3), dtype=np.uint8)
        for class_id, rgb in config.CLASS_COLORS.items():
            color[pred == class_id] = rgb
        overlay = cv2.addWeighted(bgr, 0.5, cv2.cvtColor(color, cv2.COLOR_RGB2BGR), 0.5, 0)
        for y in (config.INTERSECTION_SCAN_Y, config.HEADING_SCAN_Y_FAR):
            cv2.line(overlay, (0, y), (config.CAMERA_WIDTH - 1, y), (255, 0, 255), 1)
        cv2.line(overlay, (config.STEER_CENTER_X, 0),
                 (config.STEER_CENTER_X, config.CAMERA_HEIGHT - 1), (255, 255, 255), 1)
        ts = time.strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6]
        for suffix, frame in (("raw", bgr), ("overlay", overlay)):
            path = os.path.join(OUT_DIR, "%s_%s.jpg" % (ts, suffix))
            if not cv2.imwrite(path, frame):
                raise IOError("khong luu duoc " + path)
            print("  -> " + path)
        print("  tam vach=%s; goc=%s do; heading=%s px" %
              (sample["mid"], sample["heading_deg"], sample["heading_px"]))
        if sample["mid"] is not None and sample["heading_px"] is not None:
            error = sample["mid"] - config.STEER_CENTER_X + sample["heading_px"]
            print("  sai so truoc loc/PID = mid - %s + heading = %+.1f px" %
                  (config.STEER_CENTER_X, error))
        return sample


def save_placement(samples, session, identity):
    """Called only after explicit operator confirmation and valid measurements."""
    summary = summarize_placement(samples)
    if not summary["valid"]:
        raise ValueError(summary["reason"])
    try:
        with open(CALIB_LOG) as stream:
            placements = session_placements(stream, session, identity)
    except FileNotFoundError:
        placements = []
    record = {
        "schema": HISTORY_VERSION, "session": session,
        "placement_id": uuid.uuid4().hex,
        "captured_at": time.strftime("%Y%m%d_%H%M%S"),
        "confirmed": True, "identity": identity, "samples": samples,
    }
    with open(CALIB_LOG, "a") as stream:
        stream.write(json.dumps(record, sort_keys=True) + "\n")
    return placements + [summary]


def main(argv=None):
    args = parse_args(argv)
    print("[CHU Y] Khong doi code/thong so; khong gui lenh cho banh chay.")
    print("Dat TAM XE (khong phai tam camera) tren vach THANG, than song song.")
    print("Moi lan ghi can nhac/dat LAI va do bang thuoc; doi camera -> session MOI.")
    if not args.confirm_placement:
        print("[QUAN SAT] Khong ghi lich su; dung --session TEN --confirm-placement de ghi.")
    inspector = SnapshotInspector()
    config = inspector.config
    samples = []
    try:
        with socket.create_connection((config.SOCKET_IP, config.SOCKET_PORT), timeout=5) as sock:
            print("[WARMUP] Bo %d khung dau..." % config.CAMERA_WARMUP_FRAMES)
            for _ in range(config.CAMERA_WARMUP_FRAMES):
                _ask_frame(sock)
            for index in range(args.count):
                samples.append(inspector.one_shot(sock))
                if index < args.count - 1:
                    time.sleep(1)
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print("[LOI] %s; khong ghi hieu chinh. Dung 'run snap' de bridge tu khoi dong." % error)
        return 1
    summary = summarize_placement(samples)
    if not summary["valid"]:
        print("[CHUA DAT] " + summary["reason"] + "; khong ghi lich su.")
        return 1 if args.confirm_placement else 0
    print("[LAN NAY] mid=%s px; ung vien mid+heading=%s px; "
          "tan mat=%s px; %d anh hop le." %
          (summary["mid"], summary["center"], summary["spread"],
           summary["sample_count"]))
    if not args.confirm_placement:
        print("[XONG] Phep do on dinh KHONG chung minh tu the xe dung hay da hieu chuan.")
        return 0
    try:
        placements = save_placement(samples, args.session, inspector.identity())
    except (OSError, ValueError) as error:
        print("[KHONG GHI] " + str(error))
        return 1
    print("[LICH SU] phien %s: %d lan dat xe hop le (%s)" %
          (args.session, len(placements), CALIB_LOG))
    proposal = propose_center(placements)
    if proposal["valid"]:
        print("[UNG VIEN] STEER_CENTER_X = %s (hien tai %s); chua tu dong thay doi." %
              (proposal["center"], config.STEER_CENTER_X))
        print("Chi co gia tri voi cac tu the DA DO DUNG; chua chung minh xe bam tam khi chay.")
    else:
        print("[CHUA KET LUAN] " + proposal["reason"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
