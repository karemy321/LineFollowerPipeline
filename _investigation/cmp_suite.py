"""Compare evaluation suites: v2 and the old v3 (results/final/*_v2/_v3.csv) against
one or more new suite directories (results/suite/<tag>/*.csv, see run_suite.sh).

Usage:  py -X utf8 cmp_suite.py results/suite/v3c1 [more dirs...]
"""
import csv, math, os, sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
FIN = os.path.join(HERE, "results", "final")
MAX_DS, MEAN_DS, MAX_DROP, MEAN_DROP = 3, 1.0, 10, 3.0


def load(p):
    if not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def num(r, k):
    try:
        return float(r[k])
    except Exception:
        return float("nan")


def mean(xs):
    xs = [x for x in xs if not math.isnan(x)]
    return sum(xs) / len(xs) if xs else float("nan")


def mx_ok(r):
    if r.get("frames") in (None, "NOCAL"):
        return False
    return (num(r, "maxDeltaSteer") <= MAX_DS and num(r, "meanDeltaSteer") <= MEAN_DS and num(r, "maxPowerDrop") <= MAX_DROP
            and num(r, "meanPowerDrop") <= MEAN_DROP and num(r, "lostFrames") == 0)


def sources(dirs):
    s = [("v2", lambda n: os.path.join(FIN, n + "_v2.csv")), ("v3 old", lambda n: os.path.join(FIN, n + "_v3.csv"))]
    for d in dirs:
        s.append((os.path.basename(d.rstrip("/\\")), lambda n, d=d: os.path.join(d, n + ".csv")))
    return s


def matrix(srcs):
    print("## Disturbance matrix\n")
    print("| | pass | no cal | mean dS | max dS | mean drop | max drop | lost | straight near/mid/far/calib | curve near/mid/far |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|---|---|")
    for name, f in srcs:
        rows = load(f("matrix"))
        if not rows:
            continue
        valid = [r for r in rows if r.get("frames") not in (None, "NOCAL")]
        ok = sum(mx_ok(r) for r in valid)
        cell = defaultdict(int)
        for r in valid:
            if mx_ok(r):
                key = ("calib" if r["calib"] == "1" else r["pos"], r["track"])
                cell[key] += 1
        st = "/".join(str(cell[(p, "straight3")]) for p in ("near", "mid", "far", "calib"))
        cu = "/".join(str(cell[(p, "arcL")]) for p in ("near", "mid", "far"))
        print(f"| {name} | {ok} | {len(rows)-len(valid)} | {mean([num(r,'meanDeltaSteer') for r in valid]):.2f} | "
              f"{max(num(r,'maxDeltaSteer') for r in valid):.0f} | {mean([num(r,'meanPowerDrop') for r in valid]):.2f} | "
              f"{max(num(r,'maxPowerDrop') for r in valid):.0f} | {int(sum(num(r,'lostFrames') for r in valid))} | {st} | {cu} |")
    print()


def cl_summary(rows):
    rows = [r for r in rows if r.get("result") not in (None, "NOCAL")]
    n = len(rows)
    if n == 0:
        return None
    done = sum(r["result"] == "DONE" for r in rows)
    sj = mean([num(r, "steerJerk") for r in rows]) if "steerJerk" in rows[0] else float("nan")
    return dict(n=n, done=done, rms=mean([num(r, "rmsLat_cm") for r in rows]), sc=mean([num(r, "signChg_per_s") for r in rows]),
                sj=sj, pj=mean([num(r, "powerJerk") for r in rows]), lp=mean([num(r, "lowPower_pct") for r in rows]),
                settle=mean([num(r, "settle_s") for r in rows]))


def cl_table(title, names, srcs):
    print(f"## {title}\n")
    print("| set | " + " | ".join(n for n, _ in srcs) + " |")
    print("|---|" + "---|" * len(srcs))
    for nm in names:
        cells = []
        for _, f in srcs:
            rows = load(f(nm))
            s = cl_summary(rows) if rows else None
            if not s:
                cells.append("–"); continue
            sj = "" if math.isnan(s["sj"]) else f" sj {s['sj']:.2f}"
            cells.append(f"{s['done']}/{s['n']} rms {s['rms']:.2f} sc {s['sc']:.3f}{sj} pj {s['pj']:.2f} lp {s['lp']:.1f}")
        print(f"| {nm} | " + " | ".join(cells) + " |")
    print()


def main():
    srcs = sources(sys.argv[1:])
    matrix(srcs)
    cl_table("Closed loop regimes", ["ideal", "real", "stress", "tilt35", "phone30"], srcs)
    cl_table("Starts", ["start_yaw15", "start_yaw30", "start_yawm15", "start_lat3", "start_latm3"], srcs)
    cl_table("Disturbance laps", ["dist_clean", "dist_shadowRobot", "dist_glareStraight", "dist_glareCurve", "dist_shadowRoom",
                                  "dist_uneven", "dist_patch", "dist_cover", "dist_stripe", "dist_cast", "dist_edge", "dist_edgeclean"], srcs)
    cl_table("Realistic glare laps (30 fps, vibration)", ["glare_clean", "glare_glareMid", "glare_glareNear", "glare_glareBig", "glare_glareStreak",
                                                          "glarecal_glareMid", "glarecal_glareNear", "glarecal_glareBig", "glarecal_glareStreak"], srcs)


main()
