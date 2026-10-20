"""Render OLD and NEW guide pages to PNG and build side-by-side comparison sheets.

  old/page_NN.png   every page of the OLD PDF (docs_backup)
  new/page_NN.png   every page of the NEW PDF (repo)
  side_by_side/old_NN__new_MM.png   OLD page NN (left) next to the NEW page MM
                    that holds the same section (right), drawn as vectors, 100 dpi

Usage: py -X utf8 compare.py <old.pdf> <new.pdf> <out_dir>
"""
import sys
from pathlib import Path
import pymupdf

old_path, new_path, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
old, new = pymupdf.open(old_path), pymupdf.open(new_path)
for sub in ("old", "new", "side_by_side"):
    d = out / sub
    d.mkdir(parents=True, exist_ok=True)
    for f in d.glob("*.png"):
        f.unlink()

for name, doc in (("old", old), ("new", new)):
    for p in doc:
        p.get_pixmap(dpi=110).save(out / name / f"page_{p.number + 1:02d}.png")

# Match each OLD page to the NEW page that contains its first heading / line.
ANCHORS = {1: "LineFollowerPipeline", 2: "Have colour frames?", 3: "Line Colour", 4: "Track Memory",
           5: "Calibration has two phases", 6: "Runtime Utility Methods", 7: "Drawing the overlay",
           8: "Dynamic road band", 9: "Full Example", 10: "Notes"}
pairs = []
for o in range(old.page_count):
    key = ANCHORS.get(o + 1)
    m = next((i for i, p in enumerate(new) if key and key in p.get_text()), None)
    pairs.append((o, m))

W, H = old[0].rect.width, old[0].rect.height
gap, top = 24, 28
for o, m in pairs:
    sheet = pymupdf.open()
    pg = sheet.new_page(width=2 * W + 3 * gap, height=H + top + gap)
    for col, (doc, idx, label) in enumerate(((old, o, "OLD"), (new, m, "NEW"))):
        x0 = gap + col * (W + gap)
        if idx is None:
            continue
        r = pymupdf.Rect(x0, top, x0 + W, top + H)
        pg.draw_rect(r, color=(0.6, 0.6, 0.6), width=0.8)
        pg.show_pdf_page(r, doc, idx)
        pg.insert_text((x0, top - 9), f"{label}  page {idx + 1}", fontsize=13, color=(0.1, 0.1, 0.1))
    fn = out / "side_by_side" / f"old_{o + 1:02d}__new_{(m or 0) + 1:02d}.png"
    pg.get_pixmap(dpi=100).save(fn)
    print(f"OLD p{o + 1:2d}  <->  NEW p{(m or 0) + 1:2d}   {fn.name}")
print(f"rendered {old.page_count} old + {new.page_count} new pages")
