"""Print bbox of lines starting with given strings in two PDFs (old vs new)."""
import sys, pymupdf
def lines(path):
    out = []
    for p in pymupdf.open(path):
        for b in p.get_text('dict')['blocks']:
            for l in b.get('lines', []):
                t = ''.join(s['text'] for s in l['spans'])
                out.append((p.number + 1, t, l['bbox'], l['spans'][0]['size']))
    return out
old, new = lines(sys.argv[1]), lines(sys.argv[2])
for key in sys.argv[3:]:
    o = next((x for x in old if x[1].startswith(key)), None)
    n = next((x for x in new if x[1].startswith(key)), None)
    fmt = lambda x: 'missing' if x is None else f'p{x[0]} x={x[2][0]:6.1f} y={x[2][1]:6.1f}..{x[2][3]:6.1f} w={x[2][2]-x[2][0]:6.1f} size={x[3]:.2f}'
    print(f'{key[:28]:28s} OLD {fmt(o)}\n{"":28s} NEW {fmt(n)}')
