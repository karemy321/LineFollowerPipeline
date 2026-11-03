"""Insert the v3.1 section (changes_v31_text.md + summary + report_v31.py tables + open points)
into CHANGES_v3.md, after the header and before "## 1. Results in short". Re-runnable: an
existing v3.1 section is replaced. Usage: py -X utf8 fill_changes_v31.py [tag]"""
import os, subprocess, sys
H = os.path.dirname(os.path.abspath(__file__))
tag = sys.argv[1] if len(sys.argv) > 1 else "v3c3"
doc = open(os.path.join(H, "CHANGES_v3.md"), encoding="utf-8").read()
text = open(os.path.join(H, "changes_v31_text.md"), encoding="utf-8").read().rstrip() + "\n"
summary = open(os.path.join(H, "changes_v31_summary.md"), encoding="utf-8").read().rstrip() + "\n"
tables = subprocess.run([sys.executable, "-X", "utf8", os.path.join(H, "report_v31.py"), tag], capture_output=True, text=True, encoding="utf-8").stdout
section = text + "\n" + summary + "\n### Measured tables (v3.1)\n\n" + tables.replace("#### ", "#### ") + "\n---\n\n"
start = doc.find("## v3.1")
end = doc.find("## 1. Results in short")
if start < 0:
    start = end
doc = doc[:start] + section + doc[end:]
open(os.path.join(H, "CHANGES_v3.md"), "w", encoding="utf-8", newline="\n").write(doc)
print("CHANGES_v3.md: v3.1 section", len(section), "chars")
