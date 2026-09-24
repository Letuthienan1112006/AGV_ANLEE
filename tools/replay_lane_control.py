"""Replay recorded measurements offline; does NOT predict a new trajectory.

Usage: python3 -B tools/replay_lane_control.py runs_pulled/<run>
Print CSV to stdout. No files are changed and no hardware is accessed.
The PID comparison holds recorded filtered/raw errors fixed. The guard
comparison holds recorded detections/commands fixed, without aligning
bridge timestamps or inferring an unrecorded blocked flag.
"""
import ast
import csv
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from lane_control import LanePID, LaneTrackingGuard


def constants(path):
    result = {}
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign):
            try:
                value = ast.literal_eval(node.value)
            except (ValueError, TypeError):
                continue
            for name in node.targets:
                if isinstance(name, ast.Name):
                    result[name.id] = value
    return result


def replay(run):
    config = constants(ROOT / 'Confg.py')
    now = [0.0]
    pid = LanePID(*(config[k] for k in (
        'PID_KP', 'PID_KI', 'PID_KD', 'PID_MAX_OUTPUT',
        'PID_INTEGRAL_LIMIT', 'PID_STEP_LIMIT')), clock=lambda: now[0])
    guard = LaneTrackingGuard(config['LANE_REACQUIRE_FRAMES'],
                              config['LANE_LOST_MAX_FRAMES'])
    files = sorted(run.glob('patrol_telem_*.csv'))
    if len(files) != 1:
        raise ValueError('expected exactly one patrol telemetry file')
    with files[0].open() as handle:
        for row in csv.DictReader(handle):
            now[0] = float(row['t'])
            line = row['src'] == 'LINE'
            guard.update(line)
            if line:
                steer = pid.compute(float(row['final_err']),
                                    measured_error=float(row['raw_err']))
            else:
                pid.reset()
                steer = 0
            # Keep the two experiments separate: no claim to simulate the
            # changed EMA, actuator response, or future camera observations.
            _, speed = guard.command(int(row['steer']), int(row['speed']))
            yield (row['frame'], row['t'], row['src'], row['raw_err'],
                   row['steer'], steer, row['speed'], speed, guard.state,
                   round(pid.integral, 3))


if __name__ == '__main__':
    writer = csv.writer(sys.stdout)
    writer.writerow(('frame', 't', 'src', 'raw_error', 'logged_steer',
                     'pid_only_replay_steer', 'logged_speed',
                     'guard_only_replay_speed', 'tracking_state', 'integral'))
    writer.writerows(replay(Path(sys.argv[1])))
