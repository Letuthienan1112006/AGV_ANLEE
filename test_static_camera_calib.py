"""Stationary calibration using the SAME bridge image and measurement as snap.

Preferred: run snap --session mount-a-20260911 --confirm-placement
This starts/stops the bridge for you. Put the VEHICLE center over a straight
line using a ruler, body parallel, camera fixed. Re-place and re-measure before
each confirmed invocation; start a NEW session after moving/remounting camera.

With a bridge already running, this compatibility entrypoint also works:
    python3 test_static_camera_calib.py 3 --session mount-a --confirm-placement
Without confirmation it only inspects; no calibration history/config is changed.

No direct VideoCapture defaults, ROAD fallback or different scan row: delegation
uses the bridge's resized/JPEG image, configured LINE band/minimum and heading.
"""


def main(argv=None):
    from snapshot_check import main as inspect_stationary
    return inspect_stationary(argv)


if __name__ == "__main__":
    raise SystemExit(main())
