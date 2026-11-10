#!/usr/bin/env python3
"""Summarise a LineFollowerPipeline.Recorder CSV (lf_*.csv) for ROBOT_TEST_CHECKLIST.md.

    py tools/analyze_recording.py lf_20261110_142233.csv               # whole file
    py tools/analyze_recording.py lf_....csv --from 12.5 --to 20       # one pass (seconds)
    py tools/analyze_recording.py recordings/                          # every CSV in a folder

Reports frame rate, steering smoothness (largest step, jumps >= 25, frames at +-50 and how many
of them had glare in view), tracking, coasting / hold / rebuilt-base / rejected-band frames and
the times of the events, so the pass criteria can be read off directly. Standard library only.
"""
import argparse
import csv
import os
import statistics
import sys

JUMP = 25          # steering change between two frames that counts as a jump
GLARE = 0.02       # masked_frac above this = glare in view


def load(path):
    with open(path, newline='', encoding='ascii') as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k, v in r.items():
            if k in ('line_color', 'floor_color'):
                continue
            r[k] = float(v) if v not in ('', 'nan') else float('nan')
    return rows


def spells(rows, key):
    """(start_s, duration_ms) of each run of frames with rows[key] == 1."""
    out, start = [], None
    for r in rows:
        on = r[key] == 1
        if on and start is None:
            start = r['t_ms']
        if not on and start is not None:
            out.append((start / 1000, r['t_ms'] - start))
            start = None
    if start is not None:
        out.append((start / 1000, rows[-1]['t_ms'] - start))
    return out


def fmt_times(ts, n=12):
    s = ', '.join(f'{t:.2f}s' for t in ts[:n])
    return s + (f' ... (+{len(ts) - n})' if len(ts) > n else '') if ts else '-'


def report(path, t_from, t_to):
    rows = load(path)
    run = [r for r in rows if r['processing'] == 1
           and (t_from is None or r['t_ms'] >= t_from * 1000)
           and (t_to is None or r['t_ms'] <= t_to * 1000)]
    print(f'== {os.path.basename(path)}')
    calib = sum(1 for r in rows if r['processing'] == 0)
    print(f'   {len(rows)} lines ({calib} calibration), frame {int(rows[0]["img_w"])}x{int(rows[0]["img_h"])}'
          if rows else '   empty file')
    if len(run) < 2:
        print('   fewer than 2 processing frames in the window')
        return
    t0, t1 = run[0]['t_ms'] / 1000, run[-1]['t_ms'] / 1000
    dts = [b['t_ms'] - a['t_ms'] for a, b in zip(run, run[1:])]
    dts_sorted = sorted(dts)
    print(f'   window {t0:.2f}-{t1:.2f} s, {len(run)} frames, '
          f'{(len(run) - 1) / max(1e-9, t1 - t0):.1f} fps, frame interval median {statistics.median(dts):.1f} ms, '
          f'95% {dts_sorted[int(0.95 * (len(dts) - 1))]:.1f} ms, max {max(dts):.1f} ms')

    lib = sorted(r['lib_ms'] for r in run if r.get('lib_ms', 0) > 0)
    if lib:
        print(f'   library time per frame (lib_ms): median {statistics.median(lib):.2f} ms, '
              f'95% {lib[int(0.95 * (len(lib) - 1))]:.2f} ms, max {lib[-1]:.2f} ms')
    flips = sum(1 for a, b in zip(run, run[1:]) if a['steering'] * b['steering'] < 0)
    print(f'   steering sign changes: {flips} ({flips / max(1e-9, t1 - t0):.2f} per s), '
          f'std {statistics.pstdev(r["steering"] for r in run):.1f}')
    steps = [(abs(b['steering'] - a['steering']), b) for a, b in zip(run, run[1:])]
    jumps = [r for s, r in steps if s >= JUMP]
    pins = [r for r in run if abs(r['steering']) >= 50]
    pins_glare = [r for r in pins if r['masked_frac'] > GLARE]
    jumps_glare = [r for r in jumps if r['masked_frac'] > GLARE]
    steer = [r['steering'] for r in run]
    print(f'   steering: min {min(steer):+.0f}, max {max(steer):+.0f}, largest step {max(s for s, _ in steps):.0f}, '
          f'jumps >= {JUMP}: {len(jumps)} (with glare in view: {len(jumps_glare)})')
    print(f'   frames at +-50: {len(pins)} (with glare in view: {len(pins_glare)}) at {fmt_times([r["t_ms"] / 1000 for r in pins])}')
    if jumps:
        print(f'   jumps at {fmt_times([r["t_ms"] / 1000 for r in jumps])}')

    trk = [r['tracking'] for r in run]
    lost = sum(1 for r in run if r['track_found'] == 0)
    print(f'   tracking: min {min(trk):.0f}, median {statistics.median(trk):.0f}; line not found in {lost} frames')
    glare = [r for r in run if r['masked_frac'] > GLARE]
    if glare:
        print(f'   glare in view: {len(glare)} frames, masked max {max(r["masked_frac"] for r in run) * 100:.0f}%; '
              f'steering there {min(r["steering"] for r in glare):+.0f} .. {max(r["steering"] for r in glare):+.0f}')
    for key, name in (('coasting', 'coast'), ('line_held', 'hold (incl. coast)'), ('base_recon', 'rebuilt base')):
        sp = spells(run, key)
        if sp:
            print(f'   {name}: {sum(1 for r in run if r[key] == 1)} frames in {len(sp)} spells, longest '
                  f'{max(d for _, d in sp):.0f} ms; starts at {fmt_times([s for s, _ in sp])}')
        else:
            print(f'   {name}: none')
    rej = [r for r in run if r['rejected_bands'] > 0]
    print(f'   rejected bands: {len(rej)} frames' + (f' at {fmt_times([r["t_ms"] / 1000 for r in rej])}' if rej else ''))
    corr = [r['corr_coverage'] for r in run if r['corr_coverage'] > 0]
    if corr:
        print(f'   corridor used in {len(corr)} frames, coverage median {statistics.median(corr):.2f}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('paths', nargs='+', help='lf_*.csv files or folders')
    ap.add_argument('--from', dest='t_from', type=float, help='start of the window, seconds from the file start')
    ap.add_argument('--to', dest='t_to', type=float, help='end of the window, seconds')
    a = ap.parse_args()
    files = []
    for p in a.paths:
        if os.path.isdir(p):
            files += sorted(os.path.join(p, f) for f in os.listdir(p) if f.startswith('lf_') and f.endswith('.csv'))
        else:
            files.append(p)
    if not files:
        sys.exit('no lf_*.csv files found')
    for f in files:
        report(f, a.t_from, a.t_to)


if __name__ == '__main__':
    main()
