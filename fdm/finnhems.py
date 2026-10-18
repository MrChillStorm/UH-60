"""A fictional FinnHEMS livery (FH 60) for the UH-60A: Liveries/2048x2048/FinnHEMS.png.

Styled on FinnHEMS's helicopters: fluorescent lime-yellow all over, a green and yellow chequered band along the lower
fuselage (two rows) and the tail boom (one), a row along the cabin's roof edge, green stripes along the tail pylon,
"FinnHEMS" in green on the aft cabin and the call sign in black on the boom (the nose's paint between the band and
the windscreen is too narrow for it), the registration under a Finnish flag on the pylon. The radar cone at the nose
is yellow as well.

The paint goes on over N7601C.png's: where that livery is white or blue the new colour takes its shading (panel
lines, rivets, soiling) as the ratio of its brightness to the paint's; its dark details (windows, exhausts, warning
decals) stay, its registration is filled in. Where things go is worked out on the airframe itself: every texel of
the texture is traced back to its point on uh60.ac (the triangles rasterised in texture space), so the bands and the
lettering lie by position (ac frame: x aft, y up, z left, m) across the doors and both sides, reading correctly on
each.

    python3 finnhems.py         writes UH-60A/Models/Liveries/2048x2048/FinnHEMS.png
"""
import os, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'UH-60A', 'Models')
N = 2048

YELLOW = np.array([222, 232, 40]) / 255.0         # fluorescent lime-yellow
GREEN = np.array([0, 125, 60]) / 255.0
BLACK = np.array([22, 22, 22]) / 255.0

PAINTED = ['fuselage', 'fuselage2', 'fuselage3', 'fuselage4', 'doorL', 'doorR', 'porteL', 'porteR', 'windowL1',
           'windowL2', 'windowR1', 'windowR2', 'noze', 'cachetail', 'elevator', 'wings', 'caches', 'barriere']
SIDES = ['fuselage', 'doorL', 'doorR', 'porteL', 'porteR', 'windowL1', 'windowL2', 'windowR1', 'windowR2']

SQ = 0.16                                         # m: the chequers
BAND_TOP, CABIN_END = -1.10, 0.40                 # the lower band's top edge; aft of this, one row along the boom
BOOM_Y = (0.40, -1.10 - SQ, 5.10, -1.05 - SQ)     # the boom row's top edge from (x, y) to (x, y)
ROOF = (-6.10, -0.62, -0.16)                      # the roof-edge row: from x to x, its top edge


# ---------------------------------------------------------------- the airframe in texture space

def parse_ac(path):
    """uh60.ac's objects on texture.png: name, vertices (with their loc offsets) and surfaces [(vertex, u, v)]."""
    lines = open(path, encoding='latin-1').read().split('\n')
    out = []

    def obj(i, parent):
        o = {'name': None, 'texture': None, 'loc': [0.0] * 3, 'verts': [], 'surfs': []}
        i += 1
        while i < len(lines):
            l = lines[i]
            if l.startswith('name'):
                o['name'] = l.split('"')[1]
            elif l.startswith('texture'):
                o['texture'] = l.split('"')[1]
            elif l.startswith('loc'):
                o['loc'] = [float(x) for x in l.split()[1:4]]
            elif l.startswith('numvert'):
                n = int(l.split()[1]); off = [a + b for a, b in zip(o['loc'], parent)]
                o['verts'] = [[float(x) + c for x, c in zip(lines[i + 1 + k].split()[:3], off)] for k in range(n)]
                i += n
            elif l.startswith('numsurf'):
                i += 1
                for _ in range(int(l.split()[1])):
                    while not lines[i].startswith('refs'):
                        i += 1
                    r = int(lines[i].split()[1])
                    o['surfs'].append([(int(a), float(b), float(c)) for a, b, c in (lines[i + 1 + k].split() for k in range(r))])
                    i += r + 1
                continue
            elif l.startswith('kids'):
                out.append(o); i += 1
                for _ in range(int(l.split()[1])):
                    i = obj(i, [a + b for a, b in zip(o['loc'], parent)])
                return i
            i += 1
        out.append(o)
        return i

    i = next(k for k, l in enumerate(lines) if l.startswith('OBJECT'))
    obj(i, [0.0] * 3)
    return [o for o in out if o['texture'] == 'texture.png' and o['surfs']]


def texel_map(objs):
    """Per texel: the object (-1: none), its point on the airframe and the face's normal."""
    oid = np.full((N, N), -1, np.int16); pos = np.zeros((N, N, 3), np.float32); nrm = np.zeros((N, N, 3), np.float32)
    for k, o in enumerate(objs):
        V = np.array(o['verts'])
        for r in o['surfs']:
            for t in range(1, len(r) - 1):
                tri = (r[0], r[t], r[t + 1])
                P = V[[q[0] for q in tri]]
                n = np.cross(P[1] - P[0], P[2] - P[0]); ln = np.linalg.norm(n)
                if ln == 0:
                    continue
                U = np.array([[q[1] * N, (1 - q[2]) * N] for q in tri])
                (x0, y0), (x1, y1) = np.floor(U.min(0)).astype(int), np.ceil(U.max(0)).astype(int)
                x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, N - 1), min(y1, N - 1)
                d = (U[1, 1] - U[2, 1]) * (U[0, 0] - U[2, 0]) + (U[2, 0] - U[1, 0]) * (U[0, 1] - U[2, 1])
                if x1 < x0 or y1 < y0 or abs(d) < 1e-12:
                    continue
                xs, ys = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
                a = ((U[1, 1] - U[2, 1]) * (xs - U[2, 0]) + (U[2, 0] - U[1, 0]) * (ys - U[2, 1])) / d
                b = ((U[2, 1] - U[0, 1]) * (xs - U[2, 0]) + (U[0, 0] - U[2, 0]) * (ys - U[2, 1])) / d
                m = (a >= -0.01) & (b >= -0.01) & (1 - a - b >= -0.01)
                yy, xx = np.nonzero(m)
                oid[yy + y0, xx + x0] = k
                pos[yy + y0, xx + x0] = a[m][:, None] * P[0] + b[m][:, None] * P[1] + (1 - a - b)[m][:, None] * P[2]
                nrm[yy + y0, xx + x0] = n / ln
    return oid, pos, nrm


# ---------------------------------------------------------------- lettering

def font(size, italic=False):
    """Arial Bold (Italic), found by its file name among the system's fonts, which Pillow searches on each OS."""
    for name in (('Arial Bold Italic.ttf', 'arialbi.ttf') if italic else ('Arial Bold.ttf', 'arialbd.ttf')):
        try:
            return ImageFont.truetype(name, int(size))
        except OSError:
            pass
    sys.exit('Arial Bold%s is not among the system fonts' % (' Italic' if italic else ''))


class Decal:
    """Artwork laid flat on the sides: a box in (x, y) m, drawn at RES; on the left side x runs aft as you read it,
    on the right forward, so it reads the same on both."""
    RES = 0.004                                   # m per pixel

    def __init__(self, x0, y0, x1, y1):
        self.box = (x0, y0, x1, y1)
        self.im = Image.new('RGBA', (int((x1 - x0) / self.RES), int((y1 - y0) / self.RES)), (0, 0, 0, 0))
        self.draw = ImageDraw.Draw(self.im)

    def text(self, s, height, rgb, italic=False):
        """s centred in the box, its capitals `height` m tall."""
        f = font(height / self.RES / 0.72, italic)
        l, t, r, b = self.draw.textbbox((0, 0), s, font=f, anchor='ls')
        W, H = self.im.size
        self.draw.text(((W - (r - l)) / 2 - l, H / 2 + (height / self.RES) / 2), s, font=f, anchor='ls',
                       fill=tuple(int(255 * c) for c in rgb) + (255,))
        return self

    def sample(self, pos, nrm, mirror=False):
        """Its colour and coverage at each side texel; mirrored on the right side (a flag, its hoist forward on both)."""
        x0, y0, x1, y1 = self.box
        a = np.asarray(self.im).astype(float) / 255
        left = (nrm[..., 2] > 0) | mirror
        u = np.where(left, pos[..., 0] - x0, x1 - pos[..., 0]) / self.RES
        v = (y1 - pos[..., 1]) / self.RES
        inside = (u >= 0) & (u < a.shape[1] - 1) & (v >= 0) & (v < a.shape[0] - 1)
        ui = np.clip(u, 0, a.shape[1] - 1).astype(int); vi = np.clip(v, 0, a.shape[0] - 1).astype(int)
        px = a[vi, ui]
        return px[..., :3], np.where(inside, px[..., 3], 0.0)


def finnish_flag(w, h):
    """The Finnish flag, w x h px (18:11), the cross's upright toward the hoist (left)."""
    im = Image.new('RGBA', (w, h), (255, 255, 255, 255)); d = ImageDraw.Draw(im)
    blue = (0, 47, 108, 255); u = w / 18.0
    d.rectangle([5 * u, 0, 8 * u, h], fill=blue); d.rectangle([0, 4 * u * h / (11 * u), w, 7 * u * h / (11 * u)], fill=blue)
    return im


# ---------------------------------------------------------------- the paint

def main():
    objs = parse_ac(os.path.join(MODELS, 'uh60.ac'))
    names = [o['name'] for o in objs]
    oid, pos, nrm = texel_map(objs)
    base = np.asarray(Image.open(os.path.join(MODELS, 'Liveries', '2048x2048', 'N7601C.png')).convert('RGB')).astype(float) / 255
    out = base.copy()

    painted = np.isin(oid, [names.index(n) for n in PAINTED])
    side = np.isin(oid, [names.index(n) for n in SIDES]) & (np.abs(nrm[..., 2]) > 0.35)
    X, Y = pos[..., 0], pos[..., 1]

    # N7601C's paint: white (or near it) and its pale blue stripe; the rest is detail to keep
    L = base.mean(-1); sat = base.max(-1) - base.min(-1)
    blue = (base[..., 2] - base[..., 0] > 0.12) & (L > 0.35)
    white = (sat < 0.10) & (L > 0.55)
    # its registration (dark letters on the aft cabin), with their soft edges: filled in with the paint round it
    box = np.isin(oid, [names.index('fuselage')]) & (np.abs(nrm[..., 2]) > 0.35) & (X > 0.0) & (X < 2.8) & \
        (Y > -1.20) & (Y < -0.65)
    reg = box & (L < 0.55)
    for _ in range(3):
        reg = reg | np.roll(reg, 1, 0) | np.roll(reg, -1, 0) | np.roll(reg, 1, 1) | np.roll(reg, -1, 1)
    reg &= box
    white |= reg; blue &= ~reg
    paint = painted & (white | blue)
    # the paint's shading: its brightness over the paint's own (the brightest of it), the letters' from round them
    ref_w = np.percentile(L[painted & white & ~reg], 97); ref_b = np.percentile(L[painted & blue], 97)
    shade = np.where(blue, L / ref_b, L / ref_w)
    shade[reg] = np.nan
    for _ in range(60):                            # diffuse the shading into the letters
        s = np.where(np.isnan(shade), 0, shade); w = (~np.isnan(shade)).astype(float)
        num = sum(np.roll(np.roll(s, dy, 0), dx, 1) for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)))
        den = sum(np.roll(np.roll(w, dy, 0), dx, 1) for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)))
        fill = np.isnan(shade) & (den > 0)
        shade[fill] = num[fill] / den[fill]
    shade = np.clip(np.nan_to_num(shade, nan=1.0), 0.0, 1.15)[..., None]

    colour = np.broadcast_to(YELLOW, base.shape).copy()

    # the chequered bands: squares on (x, the height above the band's lower edge), green where i + j is even
    def chequers(mask, x, t, rows):
        i = np.floor(x / SQ); j = np.floor(t / SQ)
        on = mask & (t >= 0) & (t < rows * SQ) & ((i + j) % 2 == 0)
        colour[on] = GREEN

    lower = BAND_TOP - 2 * SQ
    chequers(side & (X < CABIN_END), X, Y - lower, 2)
    bx0, by0, bx1, by1 = BOOM_Y
    boom_edge = by0 + (np.clip(X, bx0, bx1) - bx0) / (bx1 - bx0) * (by1 - by0)
    chequers(side & (X >= CABIN_END) & (X < bx1), X, Y - boom_edge, 1)
    rx0, rx1, rtop = ROOF
    chequers(side & (X > rx0) & (X < rx1), X, Y - (rtop - SQ), 1)

    # the pylon: green stripes along it, above the registration
    pylon = side & (X > 5.25) & (Y > 0.75)
    ang = np.radians(48.0)                         # its leading edge's slope
    across = -np.sin(ang) * X + np.cos(ang) * Y
    colour[pylon & (np.floor(across / 0.14) % 2 == 0)] = GREEN

    # lettering
    for decal in (Decal(0.10, -1.08, 3.10, -0.70).text('FinnHEMS', 0.27, GREEN, italic=True),
                  Decal(3.25, -1.02, 4.55, -0.80).text('FH 60', 0.17, BLACK),
                  Decal(5.55, 0.10, 6.95, 0.30).text('OH-HBH', 0.15, BLACK)):
        rgb, a = decal.sample(pos, nrm)
        a = np.where(side, a, 0.0)[..., None]
        colour = colour * (1 - a) + rgb * a
    flag = Decal(6.02, 0.40, 6.47, 0.675)
    fw, fh = flag.im.size; flag.im.paste(finnish_flag(fw, fh), (0, 0))
    rgb, a = flag.sample(pos, nrm, mirror=True)
    a = np.where(side, a, 0.0)[..., None]
    colour = colour * (1 - a) + rgb * a

    out[paint] = np.clip(colour * shade, 0, 1)[paint]
    Image.fromarray((out * 255 + 0.5).astype(np.uint8)).save(
        os.path.join(MODELS, 'Liveries', '2048x2048', 'FinnHEMS.png'), optimize=True)
    print('FinnHEMS.png written')


if __name__ == '__main__':
    main()
