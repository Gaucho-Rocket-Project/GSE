#!/usr/bin/env python3
"""build_ecr_pdf.py -- render ECR-01 markdown to the print PDF.

The ECR markdown is the source of truth.  This script converts it to HTML with
the package's own styling (heading rules, table chrome, palette) and renders it
with WeasyPrint, always from docs/ so that the figure paths in `docs/fig_*.png`
resolve.

    python3 tools/build_ecr_pdf.py            # -> ECR-01_GSE_radio_replacement.pdf
"""
import pathlib
import sys

import markdown
from weasyprint import HTML

ROOT = pathlib.Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"
SRC = ROOT / "ECR-01_GSE_radio_replacement.md"
OUT = ROOT / "ECR-01_GSE_radio_replacement.pdf"

CSS = """
@page {
  size: Letter; margin: 22mm 18mm 20mm 18mm;
  @bottom-right { content: counter(page); font-family: "DejaVu Serif";
                  font-size: 9pt; color:#555; }
  @bottom-left  { content: "ECR-01 · GSE radio replacement · rev 02 draft";
                  font-family:"DejaVu Sans"; font-size:7pt; color:#999; }
}
body { font-family: "DejaVu Serif", serif; font-size: 9.2pt; line-height: 1.45;
       color:#111; }
h1 { font-family:"DejaVu Sans"; font-size: 17pt; color:#b71234;
     border-bottom:2pt solid #b71234; padding-bottom:4pt; }
h2 { font-family:"DejaVu Sans"; font-size: 12.5pt; margin-top: 16pt;
     border-bottom:0.6pt solid #ccc; padding-bottom:2pt; }
h3 { font-family:"DejaVu Sans"; font-size: 10.2pt; margin-top: 11pt; }
h4 { font-family:"DejaVu Sans"; font-size: 9.4pt; margin-top: 9pt; }
table { border-collapse: collapse; width: 100%; font-family:"DejaVu Sans";
        font-size: 7.3pt; margin: 6pt 0 9pt 0; }
th { background:#f0f1f3; text-align:left; padding:3pt 4pt;
     border-bottom:0.8pt solid #b71234; }
td { padding:2.6pt 4pt; border-bottom:0.4pt solid #dde1e4; vertical-align: top; }
tr:nth-child(even) td { background:#fafbfc; }
thead { display: table-header-group; }
tr { page-break-inside: avoid; }
h2, h3 { page-break-after: avoid; }
code { font-family:"DejaVu Sans Mono"; font-size: 7.6pt; background:#f4f6f8;
       padding:0.5pt 2pt; }
pre { background:#f4f6f8; border:0.5pt solid #d5dbe0; padding:6pt;
      font-size:7.2pt; white-space: pre-wrap; }
pre code { background:none; padding:0; }
img { max-width: 100%; }
blockquote { border-left:3pt solid #b71234; margin:6pt 0; padding:2pt 8pt;
             color:#333; }
hr { border:none; border-top:0.5pt solid #ccc; margin:12pt 0; }
em { color:#333; }
"""


def main() -> int:
    text = SRC.read_text(encoding="utf-8")
    body = markdown.markdown(
        text,
        extensions=["tables", "fenced_code", "sane_lists", "attr_list"],
    )
    html = f"<html><head><meta charset='utf-8'><style>{CSS}</style></head>" \
           f"<body>{body}</body></html>"

    # images are referenced as fig_*.png, which live in docs/ -- render from there
    tmp = DOCS / "_ecr_render.html"
    tmp.write_text(html, encoding="utf-8")
    try:
        HTML(filename=str(tmp)).write_pdf(str(OUT))
    finally:
        tmp.unlink(missing_ok=True)

    import pymupdf

    n = len(pymupdf.open(str(OUT)))
    print(f"wrote {OUT.relative_to(ROOT)} ({n} pages)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
