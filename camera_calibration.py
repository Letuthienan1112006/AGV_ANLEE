"""Pure calibration measurements/validation; no camera, model or project imports."""
import json
import math
import statistics

from lane_geometry import select_line_segment


MIN_VALID_SAMPLES = 3
MIN_PLACEMENTS = 3
MAX_HEADING_DEG = 3.0
MAX_CENTER_SPREAD_PX = 25
# Version 4 changes line association from the span of all row pixels to one
# contiguous component. Old center records used different geometry and must
# never be pooled with measurements from this algorithm.
HISTORY_VERSION = 4


def scan_line_mid(mask, y, band, min_pixels, reference_x=None):
    """Match patrol.scan_lane's contiguous LINE-component selection.

    Accepts either a NumPy mask or nested lists, so synthetic tests need no
    camera/GPU dependencies. ROAD is deliberately never a calibration fallback.
    """
    height = len(mask)
    if not height or y < 0 or band < 0 or min_pixels < 1:
        return None
    y = min(y, height - 1)
    candidates = range(max(0, y - band), min(height, y + band + 1))
    for row in sorted(candidates, key=lambda row: (abs(row - y), row)):
        segment = select_line_segment(
            mask[row], min_pixels, reference_x=reference_x,
        )
        if segment is not None:
            return segment[2]
    return None


def measure_line(mask, near_y, far_y, band, min_pixels, heading_weight):
    """Return near midpoint and the same image-space heading used by patrol."""
    mid = scan_line_mid(mask, near_y, band, min_pixels)
    far_mid = scan_line_mid(mask, far_y, band, min_pixels, reference_x=mid)
    angle = None
    if mid is not None and far_mid is not None and near_y > far_y:
        angle = math.degrees(math.atan2(float(far_mid - mid), near_y - far_y))
    return {
        "mid": mid,
        "heading_deg": angle,
        "heading_px": None if angle is None else angle * heading_weight,
    }


def _finite_number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def summarize_placement(samples):
    """Reject missing/skewed samples, not just an apparently good median."""
    if len(samples) < MIN_VALID_SAMPLES:
        return {"valid": False, "reason": "can it nhat %d mau hop le" % MIN_VALID_SAMPLES}
    for sample in samples:
        if (not isinstance(sample, dict)
                or not _finite_number(sample.get("mid"))
                or not _finite_number(sample.get("heading_deg"))
                or not _finite_number(sample.get("heading_px"))):
            return {"valid": False, "reason": "co mau thieu vach gan/xa hoac goc khong hop le"}
        if abs(sample["heading_deg"]) > MAX_HEADING_DEG:
            return {"valid": False, "reason": "co mau goc vach vuot %.1f do; kiem tra tu the/hinh hoc camera" % MAX_HEADING_DEG}
    mids = [sample["mid"] for sample in samples]
    # Controller zero is: mid - STEER_CENTER_X + heading_px == 0.
    # Therefore each stationary sample proposes center=mid+heading_px,
    # not mid alone. Keeping calibration and runtime equations identical
    # prevents a repeat of run 012523 (mid~403, heading~-6.4 => center~397).
    centers = [sample["mid"] + sample["heading_px"] for sample in samples]
    spread = max(centers) - min(centers)
    if spread > MAX_CENTER_SPREAD_PX:
        return {"valid": False, "reason": "tam vach dao dong qua %d px" % MAX_CENTER_SPREAD_PX}
    return {
        "valid": True,
        "mid": statistics.median(mids),
        "heading_deg": statistics.median(sample["heading_deg"] for sample in samples),
        "center": statistics.median(centers),
        "spread": spread,
        "sample_count": len(samples),
    }


def session_placements(lines, session, identity):
    """Read only confirmed, compatible, unique placements in this session.

    Old unversioned CSV is never accepted. Camera movement cannot be detected
    from metadata: the operator must choose a NEW session after remounting.
    """
    placements, seen = [], set()
    for number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except (TypeError, ValueError):
            raise ValueError("lich su dong %d khong phai JSON hop le" % number)
        if not isinstance(record, dict) or record.get("schema") != HISTORY_VERSION:
            raise ValueError("lich su dong %d khong dung phien ban" % number)
        if record.get("session") != session:
            continue
        if record.get("identity") != identity:
            raise ValueError("cau hinh/model trong phien nay da doi; hay dat --session MOI")
        placement_id = record.get("placement_id")
        if not isinstance(placement_id, str) or not placement_id or placement_id in seen:
            raise ValueError("lich su co lan dat xe thieu ID hoac bi lap")
        seen.add(placement_id)
        if record.get("confirmed") is not True or not isinstance(record.get("samples"), list):
            raise ValueError("lich su co lan dat xe chua xac nhan/do khong day du")
        summary = summarize_placement(record["samples"])
        if not summary["valid"]:
            raise ValueError("lich su co lan dat xe khong hop le: " + summary["reason"])
        placements.append(summary)
    return placements


def propose_center(placements):
    if len(placements) < MIN_PLACEMENTS:
        return {"valid": False, "reason": "can it nhat %d lan DAT LAI xe da xac nhan" % MIN_PLACEMENTS}
    centers = [placement["center"] for placement in placements]
    spread = max(centers) - min(centers)
    if spread > MAX_CENTER_SPREAD_PX:
        return {"valid": False, "reason": "cac lan dat khong lap lai duoc; kiem tra tu the, camera va nhan dien"}
    return {"valid": True,
            "center": int(round(statistics.median(centers))), "spread": spread}
