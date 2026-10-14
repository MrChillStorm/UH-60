"""TM 1-1520-237-10 figure 2-9, its two sheets joined into one image in sheet 1's frame, to draw textures from.

The figure's line art is black on white; light() turns a piece of it into light lines on a dark ground.
"""
import os, subprocess, tempfile
import numpy as np
from PIL import Image, ImageDraw

import panel_layout as L

PAGES = (61, 62)                # figure 2-9 sheets 1 and 2 in the PDF (Change 10 edition)
# the figure's callouts, erased: (sheet, polygon) for the arrowheads, (sheet, line) for the shafts
ARROWHEADS = [(1, [(85.7, 226.9), (83.8, 240.6), (81.9, 237.4), (78.4, 238.6)]),
              (2, [(255.6, 122.6), (265.6, 113.1), (265.2, 116.8), (268.7, 118.0)]),
              (2, [(338.1, 261.4), (333.6, 274.5), (332.3, 271.0), (328.7, 271.5)])]
SHAFTS = [(1, (81.9, 237.4), (67.8, 276.3)), (2, (265.2, 116.8), (342.9, 69.2)), (2, (332.3, 271.0), (306.9, 313.2))]
X0, X1, Y0, Y1, CUT = 55.0, 1005.0, 70.0, 300.0, 527.0     # the joined image, sheet 1 pt; sheet 2 from x = CUT on


class Art:
    def __init__(s, pdf, dpi=400):
        s.k = k = dpi / 72
        tmp = tempfile.mkdtemp()
        subprocess.run(['pdftoppm', '-q', '-gray', '-r', str(dpi), '-f', str(PAGES[0]), '-l', str(PAGES[1]), '-png',
                        pdf, os.path.join(tmp, 's')], check=True)
        sheets = [Image.open(os.path.join(tmp, f)).convert('L') for f in sorted(os.listdir(tmp)) if f.endswith('.png')]
        for sheet, poly in ARROWHEADS:
            ImageDraw.Draw(sheets[sheet - 1]).polygon([(x * k, y * k) for x, y in poly], fill=255, outline=255,
                                                      width=int(k))
        for sheet, a, b in SHAFTS:
            ImageDraw.Draw(sheets[sheet - 1]).line([(a[0] * k, a[1] * k), (b[0] * k, b[1] * k)], fill=255, width=int(2 * k))
        j = L.JOIN
        s.img = Image.new('L', (int((X1 - X0) * k), int((Y1 - Y0) * k)), 255)
        s.img.paste(sheets[0].crop((int(X0 * k), int(Y0 * k), int(CUT * k), int(Y1 * k))), (0, 0))
        s.img.paste(sheets[1].crop((int((CUT - j[0]) * k), int((Y0 - j[1]) * k), int((X1 - j[0]) * k),
                                    int((Y1 - j[1]) * k))), (int((CUT - X0) * k), 0))

    def box(s, x0, x1, y0, y1, sheet=1):
        """A box given in a sheet's pt, in sheet 1's pt."""
        if sheet == 2:
            j = L.JOIN
            return x0 + j[0], x1 + j[0], y0 + j[1], y1 + j[1]
        return x0, x1, y0, y1

    def crop(s, x0, x1, y0, y1, sheet=1, size=None):
        x0, x1, y0, y1 = s.box(x0, x1, y0, y1, sheet)
        im = s.img.crop((int(round((x0 - X0) * s.k)), int(round((y0 - Y0) * s.k)),
                         int(round((x1 - X0) * s.k)), int(round((y1 - Y0) * s.k))))
        return im.resize(size, Image.LANCZOS) if size else im


class Vec:
    """The figure's vector art, rendered per box (see vector)."""
    def __init__(s, pdf):
        s.pdf = pdf

    def vector(s, x0, x1, y0, y1, sheet, W, H):
        return vector(s.pdf, x0, x1, y0, y1, sheet, W, H)


def vector(pdf, x0, x1, y0, y1, sheet, W, H):
    """A box (sheet pt) rendered straight from the PDF's vector art to exactly W x H pixels, at the resolution that
    takes (no resampling of a raster: as sharp as the drawing), the callout arrows erased."""
    dpi = W / (x1 - x0) * 72
    k = dpi / 72
    px, py = int(round(x0 * k)), int(round(y0 * k))
    tmp = tempfile.mkdtemp()
    page = PAGES[sheet - 1]
    subprocess.run(['pdftoppm', '-q', '-gray', '-r', '%.3f' % dpi, '-f', str(page), '-l', str(page), '-x', str(px), '-y', str(py),
                    '-W', str(W), '-H', str(H), '-png', pdf, os.path.join(tmp, 'v')], check=True)
    im = Image.open(os.path.join(tmp, sorted(os.listdir(tmp))[0])).convert('L')
    d = ImageDraw.Draw(im)
    for s, poly in ARROWHEADS:
        if s == sheet:
            d.polygon([(x * k - px, y * k - py) for x, y in poly], fill=255, outline=255, width=max(1, int(k / 2)))
    for s, a, b in SHAFTS:
        if s == sheet:
            d.line([(a[0] * k - px, a[1] * k - py), (b[0] * k - px, b[1] * k - py)], fill=255, width=max(2, int(2 * k)))
    return im.resize((W, H)) if im.size != (W, H) else im


def light(im, ground, line):
    """Black-on-white art -> lines of grey `line` on grey `ground`."""
    a = np.asarray(im.convert('L'), float)
    return Image.fromarray(np.clip(ground + (255 - a) / 255 * (line - ground), 0, 255).astype(np.uint8))
