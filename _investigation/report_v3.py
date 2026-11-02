"""Build the v2-vs-v3 result tables (Markdown) from results/final and results/exp.

Usage:  py -X utf8 report_v3.py  [> results/report_v3.md]
"""
import csv
import math
import os
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
F = os.path.join(HERE, "results", "final")
E = os.path.join(HERE, "results", "exp")

# Pass criteria of the disturbance matrix (task: STEER / POWER nearly identical to the undisturbed run)
MAX_DS, MEAN_DS, MAX_DROP, MEAN_DROP = 3, 1.0, 10, 3.0


def load(path):
    if not os.path.exists(path):
        return []
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def num(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError, TypeError):
        return float("nan")


# ── disturbance matrix ─────────────────────────────────────────────────────
def mx_ok(r):
    if r.get("frames") == "NOCAL" or r.get("track") is None:
        return False
    try:
        return (num(r, "maxDeltaSteer") <= MAX_DS and num(r, "meanDeltaSteer") <= MEAN_DS
                and num(r, "maxPowerDrop") <= MAX_DROP and num(r, "meanPowerDrop") <= MEAN_DROP
                and num(r, "lostFrames") == 0)
    except Exception:
        return False


def mx_rows(rows):
    out = []
    for r in rows:
        if r.get("frames") == "NOCAL" or "NOCAL" in r.values():
            r = dict(r); r["nocal"] = True
        out.append(r)
    return out


def mean(xs):
    xs = [x for x in xs if not math.isnan(x)]
    return sum(xs) / len(xs) if xs else float("nan")


def matrix_summary(rows):
    valid = [r for r in rows if not r.get("nocal")]
    return dict(n=len(rows), nocal=len(rows) - len(valid), ok=sum(mx_ok(r) for r in valid),
                mds=mean([num(r, "meanDeltaSteer") for r in valid]),
                xds=max([num(r, "maxDeltaSteer") for r in valid] or [0]),
                mdr=mean([num(r, "meanPowerDrop") for r in valid]),
                xdr=max([num(r, "maxPowerDrop") for r in valid] or [0]),
                lost=sum(num(r, "lostFrames") for r in valid))


def fmt(x, d=2):
    return "–" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{d}f}"


def matrix_tables(v2, v3):
    print("### Disturbance matrix — overall\n")
    print(f"Pass = max |ΔSTEER| ≤ {MAX_DS}, mean |ΔSTEER| ≤ {MEAN_DS}, max POWER drop ≤ {MAX_DROP}, mean POWER drop ≤ {MEAN_DROP}, "
          "no frame at the minimum power where the clean run is not (60 measured frames per case, same poses clean vs disturbed).\n")
    print("| | cases | pass | calibration failed | mean \\|ΔSTEER\\| | max \\|ΔSTEER\\| | mean POWER drop | max POWER drop | frames lost |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for name, rows in (("v2", v2), ("v3", v3)):
        s = matrix_summary(rows)
        print(f"| {name} | {s['n']} | {s['ok']} | {s['nocal']} | {fmt(s['mds'])} | {fmt(s['xds'],0)} | {fmt(s['mdr'])} | {fmt(s['xdr'],0)} | {int(s['lost'])} |")
    print()
    # by type × scheme (pass counts)
    def key(r):
        return (r["type"], r["scheme"])
    agg = {}
    for name, rows in (("v2", v2), ("v3", v3)):
        d = defaultdict(list)
        for r in rows:
            if r.get("track") is None:
                continue
            d[key(r)].append(r)
        agg[name] = d
    schemes = ["black_on_wood", "white_on_dark", "black_on_white", "red_on_green"]
    types = ["glare_part", "glare_full", "glare_big", "shadow_room", "shadow_robot", "uneven", "patch", "cover", "stripe", "cast"]
    print("### Disturbance matrix — passed cases per disturbance type and colour pair (v2 → v3)\n")
    print("Each cell: 7 cases (near / mid / far × straight / curve, plus the disturbance present during calibration).\n")
    print("| disturbance | " + " | ".join(schemes) + " |")
    print("|---|" + "---|" * len(schemes))
    for t in types:
        cells = []
        for sc in schemes:
            a = agg["v2"].get((t, sc), []); b = agg["v3"].get((t, sc), [])
            cells.append(f"{sum(mx_ok(r) for r in a if not r.get('nocal'))}/{len(a)} → {sum(mx_ok(r) for r in b if not r.get('nocal'))}/{len(b)}")
        print(f"| {t} | " + " | ".join(cells) + " |")
    print()
    print("### Disturbance matrix — mean |ΔSTEER| and mean POWER drop per disturbance type (all colour pairs, positions, tracks; v2 → v3)\n")
    print("| disturbance | mean \\|ΔSTEER\\| | max \\|ΔSTEER\\| | mean POWER drop | max POWER drop |")
    print("|---|---|---|---|---|")
    for t in types:
        a = [r for r in v2 if r.get("type") == t and not r.get("nocal")]
        b = [r for r in v3 if r.get("type") == t and not r.get("nocal")]
        print(f"| {t} | {fmt(mean([num(r,'meanDeltaSteer') for r in a]))} → {fmt(mean([num(r,'meanDeltaSteer') for r in b]))} | "
              f"{fmt(max([num(r,'maxDeltaSteer') for r in a] or [0]),0)} → {fmt(max([num(r,'maxDeltaSteer') for r in b] or [0]),0)} | "
              f"{fmt(mean([num(r,'meanPowerDrop') for r in a]))} → {fmt(mean([num(r,'meanPowerDrop') for r in b]))} | "
              f"{fmt(max([num(r,'maxPowerDrop') for r in a] or [0]),0)} → {fmt(max([num(r,'maxPowerDrop') for r in b] or [0]),0)} |")
    print()
    print("### Disturbance matrix — by position and track (pass v2 → v3)\n")
    print("| | near | mid | far | during calibration |")
    print("|---|---|---|---|---|")
    for tr in ("straight3", "arcL"):
        cells = []
        for pos in ("near", "mid", "far"):
            a = [r for r in v2 if r.get("track") == tr and r.get("pos") == pos and r.get("calib") == "0"]
            b = [r for r in v3 if r.get("track") == tr and r.get("pos") == pos and r.get("calib") == "0"]
            cells.append(f"{sum(mx_ok(r) for r in a)}/{len(a)} → {sum(mx_ok(r) for r in b)}/{len(b)}")
        a = [r for r in v2 if r.get("track") == tr and r.get("calib") == "1"]
        b = [r for r in v3 if r.get("track") == tr and r.get("calib") == "1"]
        cells.append(f"{sum(mx_ok(r) for r in a)}/{len(a)} → {sum(mx_ok(r) for r in b)}/{len(b)}" if a or b else "–")
        print(f"| {'straight' if tr == 'straight3' else 'curve (R 0.5 m)'} | " + " | ".join(cells) + " |")
    print()
    fails = [r for r in v3 if not mx_ok(r)]
    if fails:
        print("### Disturbance matrix — v3 cases outside the pass criteria\n")
        print("| colour pair | track | disturbance | position | calib | max \\|ΔS\\| | mean \\|ΔS\\| | max drop | mean drop | lost | v2: max \\|ΔS\\| / max drop / lost |")
        print("|---|---|---|---|---|---:|---:|---:|---:|---:|---|")
        idx = {(r.get("scheme"), r.get("track"), r.get("type"), r.get("pos"), r.get("calib")): r for r in v2}
        for r in fails:
            k = (r.get("scheme"), r.get("track"), r.get("type"), r.get("pos"), r.get("calib"))
            o = idx.get(k, {})
            if r.get("nocal"):
                print(f"| {k[0]} | {k[1]} | {k[2]} | {k[3]} | {k[4]} | calibration failed | | | | | |")
                continue
            print(f"| {k[0]} | {k[1]} | {k[2]} | {k[3]} | {k[4]} | {fmt(num(r,'maxDeltaSteer'),0)} | {fmt(num(r,'meanDeltaSteer'))} | "
                  f"{fmt(num(r,'maxPowerDrop'),0)} | {fmt(num(r,'meanPowerDrop'))} | {fmt(num(r,'lostFrames'),0)} | "
                  f"{fmt(num(o,'maxDeltaSteer'),0)} / {fmt(num(o,'maxPowerDrop'),0)} / {fmt(num(o,'lostFrames'),0)} |")
        print()


# ── closed loop ────────────────────────────────────────────────────────────
def cl_summary(rows):
    n = len(rows)
    if n == 0:
        return None
    done = [r for r in rows if r["result"] == "DONE"]
    return dict(n=n, done=len(done),
                mean=mean([num(r, "meanLat_cm") for r in rows]),
                rms=mean([num(r, "rmsLat_cm") for r in rows]),
                worst=max([num(r, "maxLat_cm") for r in done] or [float("nan")]),
                sc=mean([num(r, "signChg_per_s") for r in rows]),
                sd=mean([num(r, "steerStd") for r in rows]),
                pj=mean([num(r, "powerJerk") for r in rows]),
                lp=mean([num(r, "lowPower_pct") for r in rows]),
                t=mean([num(r, "time_s") for r in done]),
                ms=mean([num(r, "lib_ms") for r in rows]))


def cl_line(label, a, b):
    if a is None or b is None:
        return
    print(f"| {label} | {a['n']} | {a['done']} → {b['done']} | {fmt(a['rms'])} → {fmt(b['rms'])} | {fmt(a['mean'])} → {fmt(b['mean'])} | "
          f"{fmt(a['worst'],1)} → {fmt(b['worst'],1)} | {fmt(a['sc'],3)} → {fmt(b['sc'],3)} | {fmt(a['pj'])} → {fmt(b['pj'])} | "
          f"{fmt(a['lp'],1)} → {fmt(b['lp'],1)} | {fmt(a['t'],1)} → {fmt(b['t'],1)} |")


CL_HEAD = ("| | runs | completed | RMS lateral error cm | mean lateral error cm | worst lateral error cm (completed) | steering sign changes /s | "
           "power jerk (mean \\|ΔPOWER\\| per frame) | % frames power ≤ 10 | time per run s (completed) |\n"
           "|---|---:|---|---|---|---|---|---|---|---|")


def closed_tables():
    print("### Closed loop — regimes (11 tracks × 2 floors × ± distractors × c ∈ {0.5, 1} × x ∈ {0, 1}; v2 → v3)\n")
    print(CL_HEAD)
    for reg in ("ideal", "real", "stress", "tilt35"):
        cl_line(reg, cl_summary(load(os.path.join(F, f"{reg}_v2.csv"))), cl_summary(load(os.path.join(F, f"{reg}_v3.csv"))))
    print()
    for reg in ("real",):
        a = load(os.path.join(F, f"{reg}_v2.csv")); b = load(os.path.join(F, f"{reg}_v3.csv"))
        if not a or not b:
            continue
        print(f"### Closed loop — regime `{reg}` by track (both floors, ± distractors, all gains; v2 → v3)\n")
        print(CL_HEAD)
        for tr in ("straight", "gentle", "gentleR", "scurve", "scurveR", "tightL", "tightR", "sharp90L", "sharp90", "oval", "lap"):
            cl_line(tr, cl_summary([r for r in a if r["track"] == tr]), cl_summary([r for r in b if r["track"] == tr]))
        print()
        print(f"### Closed loop — regime `{reg}` by steering gain c and power mode x (all tracks; v2 → v3)\n")
        print(CL_HEAD)
        for c in ("0.50", "1.00"):
            for x in ("0", "1"):
                cl_line(f"c = {c}, x = {x}", cl_summary([r for r in a if r["c"] == c and r["x"] == x]),
                        cl_summary([r for r in b if r["c"] == c and r["x"] == x]))
        print()
        fails = [r for r in b if r["result"] != "DONE"]
        fa = [r for r in a if r["result"] != "DONE"]
        print(f"Runs not completed in `{reg}`: v2 {len(fa)} ({', '.join(sorted(set(r['track'] + (' +d' if r['distract']=='1' else '') + ' c' + r['c'] + ' x' + r['x'] for r in fa))) or '–'}); "
              f"v3 {len(fails)} ({', '.join(sorted(set(r['track'] + (' +d' if r['distract']=='1' else '') + ' c' + r['c'] + ' x' + r['x'] for r in fails))) or '–'}).\n")
    print("### Closed loop — start at an angle or offset (straight and gentle curve, realistic timing; v2 → v3)\n")
    print("| start | runs | completed | settle time s (last time outside 1 cm / 3°) | RMS lateral error cm | worst lateral error cm | sign changes /s |")
    print("|---|---:|---|---|---|---|---|")
    for st, label in (("yaw15", "turned 15° left"), ("yawm15", "turned 15° right"), ("yaw30", "turned 30° left"), ("lat3", "3 cm left"), ("latm3", "3 cm right")):
        a = load(os.path.join(F, f"start_{st}_v2.csv")); b = load(os.path.join(F, f"start_{st}_v3.csv"))
        if not a or not b:
            continue
        sa, sb = cl_summary(a), cl_summary(b)
        print(f"| {label} | {sa['n']} | {sa['done']} → {sb['done']} | {fmt(mean([num(r,'settle_s') for r in a]))} → {fmt(mean([num(r,'settle_s') for r in b]))} | "
              f"{fmt(sa['rms'])} → {fmt(sb['rms'])} | {fmt(sa['worst'],1)} → {fmt(sb['worst'],1)} | {fmt(sa['sc'],3)} → {fmt(sb['sc'],3)} |")
    print()
    print("### Closed loop — laps through disturbances (lap + oval, 2 floors, c = 1, x ∈ {0, 1}, realistic timing; v2 → v3)\n")
    print("Each run drives through the disturbance for several dozen frames. Compare with the clean row.\n")
    print(CL_HEAD)
    for d in ("clean", "shadowRobot", "glareStraight", "glareCurve", "shadowRoom", "uneven", "patch", "cover", "stripe", "cast", "edgeclean", "edge"):
        label = {"edge": "table edge + background (oval)", "edgeclean": "oval only, clean (reference for the table edge)"}.get(d, d)
        cl_line(label, cl_summary(load(os.path.join(F, f"dist_{d}_v2.csv"))), cl_summary(load(os.path.join(F, f"dist_{d}_v3.csv"))))
    print()


def experiment_table():
    print("### \"Top to bottom\" — trace direction and floor vs image coordinates (v3 variants)\n")
    print("Full disturbance matrix (278 calibrated cases) and the `real` closed-loop set (176 runs), same code apart from the switch.\n")
    print("| variant | Config | matrix pass | matrix mean \\|ΔSTEER\\| | matrix max \\|ΔSTEER\\| | matrix mean POWER drop | frames lost | closed loop completed | RMS lateral cm | sign changes /s | power jerk |")
    print("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for v, cfg in (("both", "`TRACE_BOTH` (default)"), ("bottomup", "`TRACE_BOTTOM_UP`"), ("topdown", "`TRACE_TOP_DOWN`"),
                   ("image", "`pathFloorSpace = false`"), ("nomemory", "`pathMemory = false`")):
        m = mx_rows(load(os.path.join(E, f"matrix_{v}.csv"))); c = load(os.path.join(E, f"real_{v}.csv"))
        if not m or not c:
            continue
        s = matrix_summary(m); k = cl_summary(c)
        print(f"| {v} | {cfg} | {s['ok']} | {fmt(s['mds'])} | {fmt(s['xds'],0)} | {fmt(s['mdr'])} | {int(s['lost'])} | {k['done']}/{k['n']} | "
              f"{fmt(k['rms'])} | {fmt(k['sc'],3)} | {fmt(k['pj'])} |")
    print()


if __name__ == "__main__":
    v2 = mx_rows(load(os.path.join(F, "matrix_v2.csv")))
    v3 = mx_rows(load(os.path.join(F, "matrix_v3.csv")))
    if v2 and v3:
        matrix_tables(v2, v3)
    closed_tables()
    experiment_table()
