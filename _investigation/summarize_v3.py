"""Summarise the v3 closed-loop campaign (results/v3/<regime>_<lib>.csv) as Markdown tables.

Usage:  py -X utf8 summarize_v3.py [results/v3] > results/v3/summary.md
"""
import csv
import sys
from collections import OrderedDict, defaultdict
from pathlib import Path

D = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent / "results" / "v3")
REGIMES = OrderedDict([
    ("real", "realistic timing (150 ms latency, 20 Hz Blockly loop, dead zone 2), 5–17 cm/s"),
    ("stress", "3× faster (power 17 ≈ 0.5 m/s), 200 ms latency, 15 Hz, dead zone 3"),
    ("glare", "realistic timing + glare over the near strip in 3 lamp zones (15 % of the track each)"),
    ("glarestress", "3× faster + glare zones"),
])


def load(reg, lib):
    p = D / f"{reg}_{lib}.csv"
    if not p.exists():
        return None
    with open(p, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def key(r):
    return (r["track"], r["scheme"], r["distract"], r["c"], r["x"], r["vmax"])


def agg(rows):
    n = len(rows)
    done = sum(r["result"] == "DONE" for r in rows)
    f = lambda k: sum(float(r[k]) for r in rows) / max(1, n)
    mx = lambda k: max((float(r[k]) for r in rows), default=0)
    return dict(n=n, done=done, meanLat=f("meanLat_cm"), maxLat=mx("maxLat_cm"), sign=f("signChg_per_s"),
                lost=f("lost_pct"), wrong=f("wrongLock_pct"), ms=f("lib_ms"))


def pair(a, b, k, fmt="%.2f"):
    return (fmt % a[k]) + " → " + (fmt % b[k])


out = []
w = out.append
w("# v3 closed-loop campaign — v2 (202e70f) → v3 (working copy), identical scenarios\n")
w("Tracks: straight 2 m, gentle curve R 0.8 m, S-curve R 0.4 m, sharp ~90° corner (R 4 cm), oval with "
  "R 0.35 m ends; floors white_on_dark and black_on_wood; with / without distractors (not in the glare "
  "regimes); Blockly gain c and power mode x as listed. A run fails when the axle is > 12 cm from the line. "
  "v3 runs with `frameIntervalMs = 16.667` (the simulator renders at 60 Hz, not in real time). "
  "`lib ms` is the library time per frame while 18 runs share the CPU (not comparable with `gradlew perf`).\n")
for reg, desc in REGIMES.items():
    v2, v3 = load(reg, "v2"), load(reg, "v3")
    if not v2 or not v3:
        continue
    w(f"\n## Regime `{reg}` — {desc}\n")
    a, b = agg(v2), agg(v3)
    w("| runs | completed v2 → v3 | mean lateral err cm | worst lateral err cm | steering sign changes /s | lost % of frames | wrong-lock % | lib ms/frame |")
    w("|---:|---|---|---|---|---|---|---|")
    w(f"| {a['n']} | {a['done']} → {b['done']} | {pair(a, b, 'meanLat')} | {pair(a, b, 'maxLat', '%.1f')} | "
      f"{pair(a, b, 'sign')} | {pair(a, b, 'lost', '%.1f')} | {pair(a, b, 'wrong')} | {pair(a, b, 'ms')} |")
    # by track × distractor
    w("\n| track | distractors | completed v2 → v3 | mean lat cm | worst lat cm | lost % | wrong-lock % |")
    w("|---|---|---|---|---|---|---|")
    g2, g3 = defaultdict(list), defaultdict(list)
    for r in v2: g2[(r["track"], r["distract"])].append(r)
    for r in v3: g3[(r["track"], r["distract"])].append(r)
    for k in sorted(g2):
        x, y = agg(g2[k]), agg(g3.get(k, []))
        w(f"| {k[0]} | {'yes' if k[1] == '1' else 'no'} | {x['done']}/{x['n']} → {y['done']}/{y['n']} | "
          f"{pair(x, y, 'meanLat')} | {pair(x, y, 'maxLat', '%.1f')} | {pair(x, y, 'lost', '%.1f')} | {pair(x, y, 'wrong')} |")
    # by c × x
    w("\n| c | x | completed v2 → v3 | mean lat cm | worst lat cm | sign changes /s |")
    w("|---|---|---|---|---|---|")
    h2, h3 = defaultdict(list), defaultdict(list)
    for r in v2: h2[(r["c"], r["x"])].append(r)
    for r in v3: h3[(r["c"], r["x"])].append(r)
    for k in sorted(h2, key=lambda t: (float(t[0]), t[1])):
        x, y = agg(h2[k]), agg(h3.get(k, []))
        w(f"| {k[0]} | {k[1]} | {x['done']}/{x['n']} → {y['done']}/{y['n']} | {pair(x, y, 'meanLat')} | "
          f"{pair(x, y, 'maxLat', '%.1f')} | {pair(x, y, 'sign')} |")
    # changed outcomes
    m3 = {key(r): r for r in v3}
    diffs = [(r, m3.get(key(r))) for r in v2 if m3.get(key(r)) and (r["result"] == "DONE") != (m3[key(r)]["result"] == "DONE")]
    if diffs:
        w("\nRuns whose outcome changed:\n")
        w("| track | floor | distractors | c | x | v2 | v3 |")
        w("|---|---|---|---|---|---|---|")
        for r, s in diffs:
            w(f"| {r['track']} | {r['scheme']} | {'yes' if r['distract'] == '1' else 'no'} | {r['c']} | {r['x']} | {r['result']} | {s['result']} |")
print("\n".join(out))
