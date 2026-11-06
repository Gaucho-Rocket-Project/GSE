#!/usr/bin/env python3
"""
finish_insert.py -- turn the WeasyPrint body of the Section VI insert into a
page-for-page facsimile of the HCR-5100 layout.

WeasyPrint 70 cannot tint a @page background without painting the whole sheet,
so the chrome is stamped afterwards with PyMuPDF, measured from the original
HCR-5100 PDF (612 x 792 pt):

    header bar     #B71234, y = 0 .. 64.8 pt, 1.4 pt #7D0C24 rule below
    header text    DejaVu Sans Bold 11 pt, white, x = 72.7, baseline y = 40
    footer rule    #000000 0.7 pt, full width, y = 707
    page number    serif 9.5 pt, baseline 701, right aligned at x = 540
    footer address three lines, right aligned at x = 540, baselines 718/728/737

Usage:  python tools/finish_insert.py docs/INSERT_SECTION_VI.pdf
"""
from __future__ import annotations

import os
import sys

import fitz

RED = (0.7176, 0.0706, 0.2039)          # #B71234
DARKRED = (0.4902, 0.0471, 0.1412)      # #7D0C24
GREY = (0.29, 0.29, 0.29)
LINK = (0.10, 0.42, 0.71)
WHITE = (1, 1, 1)
LEFT = 72.0
RIGHT = 540.0
WIDTH = 612.0
BAR_H = 64.8
HEADER = "MOJAVE SPHINX  |  Build, Integration, and Launch Guidebook  |  HCR-5100"


def stamp(path_in: str, path_out: str | None = None, first_page: int = 1,
          page_offset: int = 0) -> str:
    doc = fitz.open(path_in)
    for i, page in enumerate(doc):
        # ---------------------------------------------------- header bar
        page.draw_rect(fitz.Rect(0, 0, WIDTH, BAR_H), color=None, fill=RED, width=0)
        page.draw_rect(fitz.Rect(0, BAR_H, WIDTH, BAR_H + 1.5), color=None,
                       fill=DARKRED, width=0)
        page.insert_text((72.7, 40.5), HEADER, fontname="hebo", fontsize=11,
                         color=WHITE)
        # -------------------------------------------------- footer rule
        page.draw_line(fitz.Point(0, 707.0), fitz.Point(WIDTH, 707.0),
                       color=(0.0, 0.0, 0.0), width=0.7)
        # --------------------------------------------------- page number
        num = str(i + first_page + page_offset)
        w = fitz.get_text_length(num, fontname="tiro", fontsize=9.5)
        page.insert_text((RIGHT - w, 701.0), num, fontname="tiro", fontsize=9.5,
                         color=GREY)
        # ------------------------------------------------ footer address
        for txt, font, size, col, y in (
                ("Half Cat Rocketry", "hebo", 8.8, (0.05, 0.05, 0.05), 718.0),
                ("HalfCatRocketry.com", "helv", 8.3, LINK, 728.0),
                ("contact@halfcatrocketry.com", "helv", 7.8, LINK, 737.0)):
            w = fitz.get_text_length(txt, fontname=font, fontsize=size)
            page.insert_text((RIGHT - w, y), txt, fontname=font, fontsize=size,
                             color=col)
        # ------------------------------------------- left footer identity
        page.insert_text((72.0, 718.0), "HCR-5100 rev 02 draft", fontname="hebo",
                         fontsize=7.6, color=(0.25, 0.25, 0.25))
        page.insert_text((72.0, 728.0), "Section VI \u2014 Ground Support Equipment (ECR-01)",
                         fontname="helv", fontsize=6.8, color=(0.45, 0.45, 0.45))
        page.insert_text((72.0, 737.0), "Replaces rev 01 pages 235\u2013283",
                         fontname="helv", fontsize=6.4, color=(0.55, 0.55, 0.55))
    out = path_out or path_in
    doc.save(out, garbage=3, deflate=True)
    doc.close()
    return out


if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 else "docs/INSERT_SECTION_VI.pdf"
    tmp = src.replace(".pdf", ".raw.pdf")
    if not os.path.exists(tmp):
        os.replace(src, tmp)          # keep the weasyprint output for re-stamping
    stamp(tmp, src)
    print("stamped", src)
