"""Cross-check the code, README.md and the guide PDF.

For every public Config field: type + default in the CODE vs the README table row,
and whether the field is documented in README and in the PDF (old and new).
For every public FrameData field: documented in README and PDF (old and new).
Also: every README ```java example appears in the PDF, and outdated values are gone.
Writes a Markdown report (stdout) used in DOCS_UPDATE.md; exit code 1 on any mismatch.

Usage: py -X utf8 check_docs.py <LineFollowerPipeline.java> <README.md> <old.pdf> <new.pdf>
"""
import re
import sys
from pathlib import Path
import pymupdf

src, readme_p, old_pdf, new_pdf = (Path(a) for a in sys.argv[1:5])
code = src.read_text(encoding="utf-8")
readme = readme_p.read_text(encoding="utf-8")


def pdf_text(p):
    t = "\n".join(pg.get_text() for pg in pymupdf.open(p))
    return re.sub(r"\s+", " ", t)


old_t, new_t = pdf_text(old_pdf), pdf_text(new_pdf)


def block(name):
    i = code.index(f"public static final class {name}")
    depth, j = 0, code.index("{", i)
    for k in range(j, len(code)):
        depth += {"{": 1, "}": -1}.get(code[k], 0)
        if depth == 0:
            return code[j:k]


cfg_fields = re.findall(r"public\s+(boolean|int|float)\s+(\w+)\s*=\s*([^;]+);", block("Config"))
fd_fields = [m[2] for m in re.findall(r"public\s+(final\s+)?([\w\[\]]+)\s+(\w+)\s*(=|;)", block("FrameData"))]

# README table rows keyed by the field in the first cell
rows = {}
for line in readme.splitlines():
    if line.startswith("| `"):
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        for name in re.findall(r"`(\w+)(?:\[\])?`", cells[0]):
            rows.setdefault(name, cells)


def num(s):
    s = s.strip("`").strip()
    if s in ("true", "false"):
        return s
    return float(s.rstrip("f"))


def in_pdf(text, name):
    return re.search(rf"(?<![A-Za-z]){re.escape(name)}(?![A-Za-z])", text) is not None


bad = 0
out = ["### Config fields (code vs README vs PDF)", "",
       "| Field | Type (code) | Default (code) | README row: type / default | README | PDF (new) | PDF (old) |",
       "|---|---|---|---|---|---|---|"]
for typ, name, default in cfg_fields:
    r = rows.get(name)
    ok_row = r is not None and r[1].strip("`") == typ and num(r[2]) == num(default)
    rd, pn, po = r is not None, in_pdf(new_t, name), in_pdf(old_t, name)
    bad += (not ok_row) + (not pn)
    out.append(f"| `{name}` | {typ} | `{default.strip()}` | "
               f"{'✅ ' + r[1].strip('`') + ' / ' + r[2].strip('`') if ok_row else '❌ ' + (str(r[1:3]) if r else 'missing')} | "
               f"{'✅' if rd else '❌'} | {'✅' if pn else '❌'} | {'yes' if po else '**no — added**'} |")
out += ["", f"{len(cfg_fields)} public Config fields.", "",
        "### FrameData fields (README vs PDF)", "",
        "| Field | README | PDF (new) | PDF (old) |", "|---|---|---|---|"]
for name in fd_fields:
    rd, pn, po = name in rows, in_pdf(new_t, name), in_pdf(old_t, name)
    bad += (not rd) + (not pn)
    out.append(f"| `{name}` | {'✅' if rd else '❌'} | {'✅' if pn else '❌'} | {'yes' if po else '**no — added**'} |")
out += ["", f"{len(fd_fields)} public FrameData fields.", ""]

# README examples present in the PDF (first and last non-empty line of each block)
blocks = re.findall(r"^```java\n(.*?)^```", readme, re.S | re.M)
missing = []
for b in blocks:
    ls = [l.strip() for l in b.splitlines() if l.strip()]
    for l in (ls[0], ls[-1]):
        if re.sub(r"\s+", " ", l) not in new_t:
            missing.append(l)
bad += len(missing)
out.append(f"### README examples in the PDF\n\n{len(blocks)} README `java` blocks; "
           + ("all present in the new PDF (first and last line of each found)." if not missing
              else f"missing lines: {missing}"))

# Outdated values that must be gone from both documents
stale = {"-100 … +100 steering": r"-100\s*…\s*\+100(?! *\)?\s*\(was)",
         "tracking minimum 2 (\"2 … 100\")": r"\b2\s*…\s*100",
         "1280, 720 frame size": r"1280,\s*720",
         "data.someXFrac placeholder": r"someXFrac",
         "STEER bar −100…+100": r"STEER bar \(−100"}
out += ["", "### Outdated values", "", "| Pattern | README | PDF (new) | PDF (old) |", "|---|---|---|---|"]
for label, pat in stale.items():
    # the changelog legitimately says "(was -100 … +100)" — only that form is allowed
    def hits(t):
        return [m for m in re.finditer(pat, t) if "was" not in t[max(0, m.start() - 6):m.start()]]
    r_, n_, o_ = len(hits(readme)), len(hits(new_t)), len(hits(old_t))
    bad += r_ + n_
    out.append(f"| {label} | {r_} | {n_} | {o_} |")

print("\n".join(out))
print(f"\nRESULT: {'OK — no mismatches' if bad == 0 else f'{bad} problem(s)'}")
sys.exit(1 if bad else 0)
