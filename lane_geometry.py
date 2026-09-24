"""Pure lane-mask geometry shared by runtime and camera calibration."""

try:
    import numpy as _np
except ImportError:  # calibration unit tests can still use nested lists
    _np = None


def contiguous_line_segments(row, line_value=2, min_pixels=1):
    """Return inclusive ``(left, right, midpoint)`` runs in one mask row.

    Treating every line pixel in a row as one object is unsafe: a real lane
    stripe and a detached false-positive then become one extremely wide fake
    stripe whose midpoint lies between them.
    """
    if _np is not None and isinstance(row, _np.ndarray):
        matches = _np.asarray(row) == line_value
        padded = _np.empty(matches.size + 2, dtype=_np.bool_)
        padded[0] = False
        padded[-1] = False
        padded[1:-1] = matches
        edges = _np.flatnonzero(padded[1:] != padded[:-1])
        starts, ends = edges[0::2], edges[1::2]  # end is exclusive
        return [
            (int(left), int(right - 1), int((left + right - 1) // 2))
            for left, right in zip(starts, ends)
            if right - left >= min_pixels
        ]

    segments = []
    start = None
    for x, value in enumerate(row):
        if value == line_value:
            if start is None:
                start = x
        elif start is not None:
            if x - start >= min_pixels:
                right = x - 1
                segments.append((start, right, (start + right) // 2))
            start = None
    if start is not None and len(row) - start >= min_pixels:
        right = len(row) - 1
        segments.append((start, right, (start + right) // 2))
    return segments


def select_line_segment(row, min_pixels, reference_x=None, line_value=2):
    """Select one contiguous stripe, never the span across detached blobs.

    With a reference, track the component nearest the already-measured lane.
    Without one (startup/calibration), prefer the widest component and use
    image-centre proximity only as a deterministic tie-breaker.
    """
    segments = contiguous_line_segments(row, line_value, min_pixels)
    if not segments:
        return None
    if reference_x is not None:
        return min(segments, key=lambda seg: (
            abs(seg[2] - reference_x), -(seg[1] - seg[0] + 1)
        ))
    image_center = (len(row) - 1) / 2.0
    return min(segments, key=lambda seg: (
        -(seg[1] - seg[0] + 1), abs(seg[2] - image_center)
    ))
