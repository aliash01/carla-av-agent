"""Measure lateral execution error from an agent trace.

The planner's MARGIN exists to absorb the gap between the path it draws and the
path the controller actually drives. This reports that gap, per manoeuvre state,
so MARGIN can be set from a measurement instead of a guess.

track_err = cross - lane_offset: achieved offset from the route line minus the
commanded one, both in metres, signed the same way.

    python -m scripts.track_error results/traces/route5_demagic.csv
"""

import csv
import sys
from collections import defaultdict


def summarise(path):
    rows = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            if 'track_err' not in r:
                sys.exit(f'{path}: no track_err column - trace predates the '
                         'tracking instrumentation, re-run the route')
            rows.append(r)
    if not rows:
        sys.exit(f'{path}: empty trace')

    by_state = defaultdict(list)
    for r in rows:
        # reversing has its own control path and never calls _lateral_control,
        # so its rows carry a stale cross - exclude rather than pretend
        if r['state'] == 'REVERSING':
            continue
        by_state[r['state']].append((int(r['tick']),
                                     float(r['track_err']),
                                     float(r['speed'])))

    print(f'{path}: {len(rows)} ticks\n')
    print(f'{"state":<12} {"ticks":>6} {"peak |err|":>11} {"at tick":>8} '
          f'{"speed":>7} {"mean |err|":>11}')
    worst = 0.0
    for state in ('FOLLOWING', 'PREPARING', 'CHANGING', 'RETURNING'):
        s = by_state.get(state)
        if not s:
            continue
        tick, err, speed = max(s, key=lambda t: abs(t[1]))
        mean = sum(abs(e) for _, e, _ in s) / len(s)
        print(f'{state:<12} {len(s):>6} {abs(err):>11.3f} {tick:>8} '
              f'{speed:>7.2f} {mean:>11.3f}')
        if state in ('CHANGING', 'RETURNING'):
            worst = max(worst, abs(err))

    print(f'\nmanoeuvre peak |err| (CHANGING + RETURNING): {worst:.3f} m')
    print('this is the clearance MARGIN must cover; planner.py MARGIN = 0.2')
    if worst > 0.2:
        print(f'-> MARGIN is {worst / 0.2:.1f}x too small')
    else:
        print('-> MARGIN covers the measured error')


if __name__ == '__main__':
    summarise(sys.argv[1] if len(sys.argv) > 1
              else 'results/traces/route5_demagic.csv')
