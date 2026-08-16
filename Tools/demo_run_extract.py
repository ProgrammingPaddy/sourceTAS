"""demo_run_extract - isolate the RUN segment inside a captured demo trace.

The in-game captures include pre/post-run frames (respawns, teleports,
sparse session-clock stretches). The run itself is the longest CONTIGUOUS
stretch: snapshot dt <= 0.1s and no teleport jumps (|dpos| < 500u).
Emits <name>_run.csv beside each input and prints a self-check against
the run time encoded in the filename (e.g. 0-14-75 = 14.75s).

Usage: python demo_run_extract.py <trace_dir>
"""
import csv
import os
import re
import sys

def load(path):
    with open(path) as f:
        return [r for r in csv.DictReader(f)]

def segments(rows):
    # The RUN has a signature the rest of the session lacks: DENSE
    # snapshots AND SUSTAINED MOTION. Mark each row, then take the longest
    # stretch of marked rows tolerating holes up to 1s (brief slow moments
    # on a board). Pure gap/jump segmentation either shattered coarse
    # captures or glued sparse session frames - this selects the run
    # directly.
    n = len(rows)
    good = [False] * n
    for i in range(1, n):
        dt = float(rows[i]['sim']) - float(rows[i - 1]['sim'])
        if dt <= 0 or dt > 0.25:
            continue
        dx = float(rows[i]['x']) - float(rows[i - 1]['x'])
        dy = float(rows[i]['y']) - float(rows[i - 1]['y'])
        dz = float(rows[i]['z']) - float(rows[i - 1]['z'])
        d = (dx * dx + dy * dy + dz * dz) ** 0.5
        v = d / dt
        if 100.0 < v < 4000.0:   # moving at legal speed, not teleporting
            good[i] = True
    best, cur = [], []
    last_good_sim = None
    for i in range(n):
        if good[i]:
            if (cur and last_good_sim is not None
                and float(rows[i]['sim']) - last_good_sim > 1.0):
                if len(cur) > len(best):
                    best = cur
                cur = []
            cur.append(i)
            last_good_sim = float(rows[i]['sim'])
    if len(cur) > len(best):
        best = cur
    return [best] if best else [[0]]

def main():
    d = sys.argv[1]
    for name in sorted(os.listdir(d)):
        if not name.endswith('.csv') or name.endswith('_run.csv'):
            continue
        rows = load(os.path.join(d, name))
        if len(rows) < 50:
            print(f'{name}: too small, skipped')
            continue
        segs = segments(rows)
        best = max(segs, key=len)
        seg = [rows[i] for i in best]
        span = float(seg[-1]['sim']) - float(seg[0]['sim'])
        m = re.search(r'(\d+)-(\d+)-(\d+)', name)
        expect = None
        if m:
            expect = int(m.group(1)) * 60 + int(m.group(2)) + int(m.group(3)) / 100.0
        out = os.path.join(d, name[:-4] + '_run.csv')
        with open(out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=rows[0].keys())
            w.writeheader()
            w.writerows(seg)
        chk = ''
        if expect:
            chk = (f' | filename says {expect:.2f}s -> '
                   + ('SPAN COVERS RUN' if span >= expect * 0.9
                      else 'SEGMENT SHORTER THAN RUN - inspect'))
        print(f'{name}: {len(rows)} rows -> run segment {len(seg)} rows, '
              f'{span:.2f}s{chk}')

if __name__ == '__main__':
    sys.exit(main())
