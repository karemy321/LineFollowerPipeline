"""Weave table from results/wobble/{real,stress}_<tag>.csv: per track (both floors) and x."""
import csv, sys
tags = sys.argv[1:]
for reg in ('real', 'stress'):
    print(f'\n### {reg}  (c = 1.0; per cell: completed/2 · sign changes per s · steering std · mean / max lateral cm)')
    rows = {t: list(csv.DictReader(open(f'results/wobble/{reg}_{t}.csv'))) for t in tags}
    print('| track | x | ' + ' | '.join(tags) + ' |')
    print('|---|---|' + '---|' * len(tags))
    for track in ('straight', 'gentle', 'scurve', 'sharp90', 'oval'):
        for x in ('0', '1'):
            cells = []
            for t in tags:
                rs = [r for r in rows[t] if r['track'] == track and r['x'] == x]
                done = sum(r['result'] == 'DONE' for r in rs)
                sc = sum(float(r['signChg_per_s']) for r in rs) / len(rs)
                sd = sum(float(r['steerStd']) for r in rs) / len(rs)
                ml = sum(float(r['meanLat_cm']) for r in rs) / len(rs)
                mx = max(float(r['maxLat_cm']) for r in rs)
                cells.append(f'{done}/{len(rs)} · {sc:.2f} · {sd:.1f} · {ml:.2f}/{mx:.1f}')
            print(f'| {track} | {x} | ' + ' | '.join(cells) + ' |')
