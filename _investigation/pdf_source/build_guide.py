"""Build LineFollowerPipeline_Integration_Guide.pdf from guide_template.html + README.md.

The guide's prose lives in guide_template.html. Every table and every code
example is copied from README.md at build time, so the PDF and the README cannot
drift apart:

  <!--CODE n "prefix"-->   n-th ```java block of README.md (asserts it starts with prefix)
  <!--FENCE "prefix"-->    first fenced block (any language) whose first line starts with prefix
  <!--TABLE "heading" plain="i,j" class="c" nowrap="k"-->
                           first Markdown table after the README heading line "heading";
                           columns in `plain` lose their `code` formatting (Type / Default /
                           Range look like the original guide), `nowrap` columns do not wrap.

The filled HTML is printed with headless Microsoft Edge (Chromium "print to PDF",
Skia/PDF — the same producer as the original guide), A4, no header/footer.

Usage:  py -X utf8 build_guide.py [README.md] [out.pdf]
"""
import html
import re
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
README = Path(sys.argv[1]) if len(sys.argv) > 1 else REPO / "README.md"
OUT_PDF = (Path(sys.argv[2]) if len(sys.argv) > 2 else REPO / "LineFollowerPipeline_Integration_Guide.pdf").resolve()
TEMPLATE = HERE / "guide_template.html"
OUT_HTML = HERE / "guide.html"
EDGE = Path(r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe")

# Badges in the first column of tables (field names / row labels).
NEW = {"rejectDistractors", "bandMaxWidthMult", "bandMinWidthMult", "trackHoldFrames",
       "steerOffsetWeight", "steerHeadingWeight", "steerLookAhead", "steerHeadingMinRows",
       "steerHeadingCalibMaxDeg", "steerOffsetTerm", "steerHeadingTerm", "headingDeg",
       "headingConfidence", "lineHeld", "rejectedBands",
       "Robot at an angle to a straight line", "Dark object / shadow beside or touching the line",
       "Line hidden for a moment in the middle of the image"}
# CHANGED = existing items whose behaviour changed in v2.
CHANGED = {"trackMemory", "trackMemoryGateMult", "trackRelockFrames", "cameraTiltDeg",
           "vRoiHalfWidthPx", "calibFrames", "steering", "tracking"}
MAX_CODE_CHARS = 103   # A4 code block holds ~104 Consolas 8.8pt characters (503 pt / 4.84 pt)

md = README.read_text(encoding="utf-8")
lines = md.splitlines()


# ---------------------------------------------------------------- inline Markdown
def inline(text, plain=False):
    codes = []

    def keep(m):
        codes.append(m.group(1))
        return f"\x00{len(codes) - 1}\x00"

    t = re.sub(r"`([^`]*)`", keep, text)
    t = html.escape(t, quote=False)
    t = re.sub(r"\[([^\]]+)\]\([^)]*\)", r"<em>\1</em>", t)           # links -> italic name
    t = re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", t)
    t = re.sub(r"(?<![\w*])\*(?!\s)(.+?)(?<!\s)\*(?![\w*])", r"<em>\1</em>", t)

    def back(m):
        c = html.escape(codes[int(m.group(1))], quote=False)
        return c if plain else f"<code>{c}</code>"

    return re.sub(r"\x00(\d+)\x00", back, t)


def plain_text(cell):
    return re.sub(r"[`*]", "", cell).strip()


# ---------------------------------------------------------------- code blocks
fences = []   # (lang, code)
for m in re.finditer(r"^```(\w*)\n(.*?)^```", md, re.S | re.M):
    fences.append((m.group(1), m.group(2).rstrip("\n")))
java_blocks = [c for lang, c in fences if lang == "java"]

TOKEN = re.compile(r'"(?:\\.|[^"\\])*"|//[^\n]*|/\*.*?\*/', re.S)


def highlight(code):
    for l in code.splitlines():
        if len(l) > MAX_CODE_CHARS:
            raise SystemExit(f"code line too wide for the PDF ({len(l)} > {MAX_CODE_CHARS}): {l}")
    out, pos = [], 0
    for m in TOKEN.finditer(code):
        out.append(html.escape(code[pos:m.start()], quote=False))
        tok = html.escape(m.group(0), quote=False)
        out.append(tok if m.group(0).startswith('"') else f'<span class="c">{tok}</span>')
        pos = m.end()
    out.append(html.escape(code[pos:], quote=False))
    return "<pre><code>" + "".join(out) + "</code></pre>"


def code_marker(m):
    n, prefix = int(m.group(1)), m.group(2)
    code = java_blocks[n - 1]
    first = code.lstrip().splitlines()[0]
    if not first.startswith(prefix):
        raise SystemExit(f"CODE {n}: README java block starts with {first!r}, expected {prefix!r}")
    used_code.add(n)
    return highlight(code)


def fence_marker(m):
    prefix = m.group(1)
    for lang, code in fences:
        if code.lstrip().startswith(prefix):
            return highlight(code)
    raise SystemExit(f"FENCE: no fenced block starts with {prefix!r}")


# ---------------------------------------------------------------- tables
def table_after(heading):
    try:
        i = next(k for k, l in enumerate(lines) if l.strip() == heading)
    except StopIteration:
        raise SystemExit(f"TABLE: heading {heading!r} not found in README")
    while not lines[i].startswith("|"):
        i += 1
    rows = []
    while i < len(lines) and lines[i].startswith("|"):
        rows.append([c.strip() for c in lines[i].strip().strip("|").split("|")])
        i += 1
    return rows[0], rows[2:]          # header, body (row 1 is the --- separator)


def cols(spec):
    return {int(x) for x in spec.split(",") if x.strip()}


def table_marker(m):
    heading, plain, cls, nowrap = m.group(1), cols(m.group(2)), m.group(3), cols(m.group(4))
    head, body = table_after(heading)
    used_tables.append((heading, head, body))
    out = [f'<table class="{cls}">' if cls else "<table>", "<thead><tr>"]
    out += [f"<th>{inline(h, plain=True)}</th>" for h in head]
    out.append("</tr></thead><tbody>")
    for row in body:
        out.append("<tr>")
        for j, cell in enumerate(row):
            content = inline(cell, plain=(j in plain))
            if j == 0:
                key = plain_text(cell).split(",")[0].split("[")[0].strip()
                key2 = plain_text(cell).split(" (")[0].strip()
                if key in NEW or key2 in NEW:
                    content += ' <span class="badge">NEW</span>'
                elif key in CHANGED:
                    content += ' <span class="badge changed">CHANGED</span>'
            attr = ' class="nowrap"' if j in nowrap else ""
            out.append(f"<td{attr}>{content}</td>")
        out.append("</tr>")
    out.append("</tbody></table>")
    return "".join(out)


used_code, used_tables = set(), []
tpl = TEMPLATE.read_text(encoding="utf-8")
tpl = re.sub(r'<!--CODE (\d+) "([^"]*)"-->', code_marker, tpl)
tpl = re.sub(r'<!--FENCE "([^"]*)"-->', fence_marker, tpl)
tpl = re.sub(r'<!--TABLE "([^"]*)" plain="([^"]*)" class="([^"]*)" nowrap="([^"]*)"-->', table_marker, tpl)
if "<!--CODE" in tpl or "<!--TABLE" in tpl or "<!--FENCE" in tpl:
    raise SystemExit("unfilled marker left in template")
# Segoe UI has no "⊥": draw it with Cambria Math at regular weight (a faked bold would become a Type3 font).
parts = re.split(r"(<pre>.*?</pre>|<style>.*?</style>|<title>.*?</title>)", tpl, flags=re.S)
tpl = "".join(p if p.startswith(("<pre>", "<style>", "<title>")) else p.replace("⊥", '<span class="sym">⊥</span>')
              for p in parts)
OUT_HTML.write_text(tpl, encoding="utf-8")

skipped = sorted(set(range(1, len(java_blocks) + 1)) - used_code)
print(f"README: {len(java_blocks)} java blocks, used {len(used_code)}"
      + (f" (not in the PDF: {skipped})" if skipped else ""))
print(f"README: {len(used_tables)} tables copied")

# ---------------------------------------------------------------- print with Edge
with tempfile.TemporaryDirectory() as profile:
    cmd = [str(EDGE), "--headless", "--disable-gpu", "--no-first-run", "--no-pdf-header-footer",
           f"--user-data-dir={profile}", f"--print-to-pdf={OUT_PDF}", OUT_HTML.as_uri()]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    if r.returncode != 0 or not OUT_PDF.exists():
        raise SystemExit(f"Edge failed ({r.returncode}): {r.stderr[-2000:]}")
print(f"PDF written: {OUT_PDF}")
