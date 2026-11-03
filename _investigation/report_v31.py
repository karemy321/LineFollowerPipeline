"""Tables for the v3.1 section of CHANGES_v3.md (Markdown on stdout).

Sources: v2 and v3 before v3.1 (results/final/*_v2/_v3.csv, results/jit/j1_*.csv,
results/glcl1/*_v3base.csv), v3.1 (results/suite/<tag>/), v2 parts measured with the
new harness (results/suite/v2/), the "top to bottom" rerun (results/exp_final/).

Usage:  py -X utf8 report_v31.py [tag=v3c3]
"""
import csv, math, os, sys
from collections import defaultdict

H = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(H, "results")
TAG = sys.argv[1] if len(sys.argv) > 1 else "v3c3"
S31, SV2, FIN, EXP = (os.path.join(R, "suite", TAG), os.path.join(R, "suite", "v2"), os.path.join(R, "final"), os.path.join(R, "exp_final"))
MAX_DS, MEAN_DS, MAX_DROP, MEAN_DROP = 3, 1.0, 10, 3.0


def load(p):
    if not p or not os.path.exists(p):
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


def f(x, d=2):
    return "–" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{d}f}"


# ── jitter ───────────────────────────────────────────────────────────────
def jit_summary(rows, cal):
    rs = [r for r in rows if r.get("calib") == str(cal)]
    ok = [r for r in rs if r.get("jitS") not in (None, "", "NOCAL")]
    if not rs:
        return None
    return dict(n=len(rs), nocal=len(rs) - len(ok), jS=mean([num(r, "jitS") for r in ok]), cjS=mean([num(r, "clean_jitS") for r in ok]),
                jT=mean([num(r, "jitT") for r in ok]), cjT=mean([num(r, "clean_jitT") for r in ok]),
                jumps=sum(num(r, "jumpS") for r in ok), cjumps=sum(num(r, "clean_jumpS") for r in ok),
                hf=mean([num(r, "hfS") for r in ok]), chf=mean([num(r, "clean_hfS") for r in ok]),
                bias=mean([num(r, "biasS") for r in ok]), lost=sum(num(r, "lost") for r in ok),
                tmin=sum(1 for r in ok if num(r, "minT") <= 5))


def jitter_rows(rows):
    """Jitter CSV rows: the first columns (lib, scheme, track, glare, calib) then values; NOCAL rows are short."""
    out = []
    for r in rows or []:
        out.append(r)
    return out


def jitter_table():
    srcs = [("v2", load(os.path.join(SV2, "jitter.csv")) or load(os.path.join(R, "jit", "j1_v2.csv"))),
            ("v3 before", load(os.path.join(R, "jit", "j1_v3.csv"))),
            ("v3.1", load(os.path.join(S31, "jitter.csv")))]
    print("| calibration | | cases (no calibration) | STEER change / frame, glare (clean) | POWER change / frame, glare (clean) | STEER jumps ≥ 5, glare (clean) | STEER noise (RMS around 7-frame mean), glare (clean) | mean \\|STEER glare − clean\\| | frames lost | cases with POWER at the minimum |")
    print("|---|---|---:|---|---|---|---|---:|---:|---:|")
    for cal, cname in ((0, "clean (glare after)"), (1, "with the glare in view")):
        for name, rows in srcs:
            if not rows:
                continue
            s = jit_summary(rows, cal)
            if not s:
                continue
            print(f"| {cname} | {name} | {s['n']} ({s['nocal']}) | {f(s['jS'])} ({f(s['cjS'])}) | {f(s['jT'])} ({f(s['cjT'])}) | "
                  f"{int(s['jumps'])} ({int(s['cjumps'])}) | {f(s['hf'])} ({f(s['chf'])}) | {f(s['bias'])} | {int(s['lost'])} | {s['tmin']} |")
    print()
    # per glare variant, calibration with the glare in view
    print("STEER change per frame with glare (without) and STEER jumps, by glare variant — calibrated with the glare in view (all 4 colour pairs × straight / curve / S-curve):\n")
    variants = ["near", "mid", "far", "big", "beside", "streak", "weak"]
    print("| glare | " + " | ".join(n for n, r in srcs if r) + " |")
    print("|---|" + "---|" * sum(1 for n, r in srcs if r))
    for g in variants:
        cells = []
        for name, rows in srcs:
            if not rows:
                continue
            rs = [r for r in rows if r.get("glare") == g and r.get("calib") == "1" and r.get("jitS") not in (None, "", "NOCAL")]
            nocal = sum(1 for r in rows if r.get("glare") == g and r.get("calib") == "1") - len(rs)
            if not rs:
                cells.append(f"no calibration ({nocal})"); continue
            cells.append(f"{f(mean([num(r,'jitS') for r in rs]))} ({f(mean([num(r,'clean_jitS') for r in rs]))}), jumps {int(sum(num(r,'jumpS') for r in rs))}"
                         + (f", no cal. {nocal}" if nocal else ""))
        print(f"| {g} | " + " | ".join(cells) + " |")
    print()


# ── closed loop ──────────────────────────────────────────────────────────
def cl(rows):
    if not rows:
        return None
    rows = [r for r in rows if r.get("result") not in (None, "NOCAL")]
    if not rows:
        return None
    sj = mean([num(r, "steerJerk") for r in rows]) if "steerJerk" in rows[0] else float("nan")
    return dict(n=len(rows), done=sum(r["result"] == "DONE" for r in rows), rms=mean([num(r, "rmsLat_cm") for r in rows]),
                sc=mean([num(r, "signChg_per_s") for r in rows]), sj=sj, pj=mean([num(r, "powerJerk") for r in rows]),
                lp=mean([num(r, "lowPower_pct") for r in rows]), settle=mean([num(r, "settle_s") for r in rows]))


def cl_cell(s, settle=False):
    if not s:
        return "–"
    t = f"**{s['done']}/{s['n']}** · {f(s['rms'])} cm · {f(s['sc'],3)}/s · S {f(s['sj'])} · P {f(s['pj'])} · min {f(s['lp'],1)} %"
    if settle:
        t += f" · settle {f(s['settle'],1)} s"
    return t


def cl_table(names, settle=False):
    print("Each cell: **completed** · RMS lateral error · steering sign changes per second · S = mean |ΔSTEER| per frame · P = mean |ΔPOWER| per frame · share of frames at the minimum POWER" + (" · time to settle" if settle else "") + ".\n")
    print("| set | v2 | v3 before | v3.1 |")
    print("|---|---|---|---|")
    for nm, label in names:
        v2 = load(os.path.join(FIN, nm + "_v2.csv")) or load(os.path.join(SV2, nm + ".csv"))
        v3 = load(os.path.join(FIN, nm + "_v3.csv"))
        v31 = load(os.path.join(S31, nm + ".csv"))
        print(f"| {label} | {cl_cell(cl(v2), settle)} | {cl_cell(cl(v3), settle)} | {cl_cell(cl(v31), settle)} |")
    print()


def glare_laps():
    print("Each cell: **completed** / 32 · S = mean |ΔSTEER| per frame · P = mean |ΔPOWER| per frame · share of frames at the minimum POWER.\n")
    print("| realistic glare, laps | calibrated | v2 | v3 before | v3.1 |")
    print("|---|---|---|---|---|")
    names = [("clean", "no glare"), ("glareMid", "spot mid-view"), ("glareNear", "spot near"), ("glareBig", "large spot (hides most of the line)"), ("glareStreak", "lamp-tube streak along the line")]
    def cell(rows):
        s = cl(rows)
        return "–" if not s else f"**{s['done']}**/{s['n']} · S {f(s['sj'])} · P {f(s['pj'])} · min {f(s['lp'],1)} %"
    for nm, label in names:
        for pre, cal in (("glare_", "with the glare"), ("glarecal_", "clean")):
            if nm == "clean" and pre == "glarecal_":
                continue
            v2 = load(os.path.join(SV2, pre + nm + ".csv"))
            old = load(os.path.join(R, "glcl1", nm + "_v3base.csv")) if pre == "glarecal_" or nm == "clean" else None
            v31 = load(os.path.join(S31, pre + nm + ".csv"))
            print(f"| {label} | {cal if nm != 'clean' else '–'} | {cell(v2)} | {cell(old)} | {cell(v31)} |")
    print()


# ── matrix ───────────────────────────────────────────────────────────────
def mx_ok(r):
    if r.get("frames") in (None, "NOCAL"):
        return False
    return (num(r, "maxDeltaSteer") <= MAX_DS and num(r, "meanDeltaSteer") <= MEAN_DS and num(r, "maxPowerDrop") <= MAX_DROP
            and num(r, "meanPowerDrop") <= MEAN_DROP and num(r, "lostFrames") == 0)


def matrix():
    print("| | pass (of 280) | calibration failed | mean \\|ΔSTEER\\| | max \\|ΔSTEER\\| | mean POWER drop | max POWER drop | frames lost | straight near / mid / far / during calibration (of 40) | curve near / mid / far (of 40) |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|---|---|")
    for name, p in (("v2", os.path.join(FIN, "matrix_v2.csv")), ("v3 before", os.path.join(FIN, "matrix_v3.csv")), ("v3.1", os.path.join(S31, "matrix.csv"))):
        rows = load(p)
        if not rows:
            continue
        valid = [r for r in rows if r.get("frames") not in (None, "NOCAL")]
        cell = defaultdict(int)
        for r in valid:
            if mx_ok(r):
                cell[("calib" if r["calib"] == "1" else r["pos"], r["track"])] += 1
        st = " / ".join(str(cell[(q, "straight3")]) for q in ("near", "mid", "far", "calib"))
        cu = " / ".join(str(cell[(q, "arcL")]) for q in ("near", "mid", "far"))
        print(f"| {name} | **{sum(mx_ok(r) for r in valid)}** | {len(rows)-len(valid)} | {f(mean([num(r,'meanDeltaSteer') for r in valid]))} | "
              f"{max(num(r,'maxDeltaSteer') for r in valid):.0f} | {f(mean([num(r,'meanPowerDrop') for r in valid]))} | "
              f"{max(num(r,'maxPowerDrop') for r in valid):.0f} | {int(sum(num(r,'lostFrames') for r in valid))} | {st} | {cu} |")
    print()


def experiment():
    print("| variant | matrix pass (of 280) | mean \\|ΔSTEER\\| | max \\|ΔSTEER\\| | mean POWER drop | closed loop `real`: completed | RMS lateral | glare jitter: STEER / frame (clean) | STEER jumps |")
    print("|---|---:|---:|---:|---:|---|---:|---|---:|")
    labels = [("both", "near → far + gaps re-read from both sides (default)"), ("bottomup", "near → far only"), ("topdown", "far → near only"),
              ("image", "image coordinates instead of the floor"), ("nomemory", "no path memory")]
    for nm, label in labels:
        mx = load(os.path.join(EXP, f"matrix_{nm}.csv"))
        rl = load(os.path.join(EXP, f"real_{nm}.csv"))
        jt = load(os.path.join(EXP, f"jitter_{nm}.csv"))
        if not mx and not rl:
            print(f"| {label} | – | – | – | – | – | – | – | – |"); continue
        valid = [r for r in (mx or []) if r.get("frames") not in (None, "NOCAL")]
        s = cl(rl)
        js = jit_summary(jt, 1) if jt else None
        print(f"| {label} | {sum(mx_ok(r) for r in valid) if mx else '–'} | {f(mean([num(r,'meanDeltaSteer') for r in valid])) if mx else '–'} | "
              f"{(max(num(r,'maxDeltaSteer') for r in valid) if valid else float('nan')):.0f} | {f(mean([num(r,'meanPowerDrop') for r in valid])) if mx else '–'} | "
              f"{(str(s['done']) + '/' + str(s['n'])) if s else '–'} | {f(s['rms']) if s else '–'} | "
              f"{(f(js['jS']) + ' (' + f(js['cjS']) + ')') if js else '–'} | {int(js['jumps']) if js else '–'} |")
    print()


if __name__ == "__main__":
    print("#### Jitter with realistic glare (scripted drive, 30 fps, vibration)\n"); jitter_table()
    print("#### Closed-loop laps with realistic glare (30 fps, vibration, realistic delay)\n"); glare_laps()
    print("#### Disturbance matrix\n"); matrix()
    print("#### Closed-loop regimes\n")
    cl_table([("ideal", "ideal (60 fps, no delay)"), ("real", "realistic (150 ms delay, 20 Hz loop, motor dead zone)"),
              ("stress", "stress (3× speed, 200 ms, 15 Hz)"), ("tilt35", "camera tilt off by 10°"), ("phone30", "phone: 30 fps + vibration")])
    print("#### Starts at an angle or offset\n")
    cl_table([("start_yaw15", "turned 15° left"), ("start_yaw30", "turned 30° left"), ("start_yawm15", "turned 15° right"),
              ("start_lat3", "3 cm left"), ("start_latm3", "3 cm right")], settle=True)
    print("#### Laps through disturbances\n")
    cl_table([("dist_clean", "clean"), ("dist_shadowRobot", "robot shadow"), ("dist_glareStraight", "glare (smooth) on the straight"),
              ("dist_glareCurve", "glare (smooth) on the curve"), ("dist_shadowRoom", "room shadows"), ("dist_uneven", "strong light gradient"),
              ("dist_patch", "colour patches"), ("dist_cover", "covering objects"), ("dist_stripe", "bright stripes"), ("dist_cast", "colour casts"),
              ("dist_edge", "table edge"), ("dist_edgeclean", "table edge — reference without")])
    print("#### \"Top to bottom\" on the final code\n"); experiment()
