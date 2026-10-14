"""Builds the UH-60A instrument panel from the layout measured in panel_layout.py (TM 1-1520-237-10 fig. 2-9):

- the face plate, object panel2 of Panels/panel.ac: the old plate's outline (it fits the cockpit), cut out where
  the round instruments' dials sit behind its plane;
- the plate's texture, Panels/panel-face.png: the TM figure's line art, light on the plate's grey, for what is
  painted on the plate (placards and labels), and plain plate everywhere else;
- the instruments' places in Panels/panel.xml: each 3-D instrument on its TM item, scaled where its bezel differs
  from the TM's. The torque dials, glass displays and moving maps of the old panel aren't in a UH-60A and are gone
  (the files stay in Instruments/). The instruments added to the panel are built by instruments.py.

    python3 panel_face.py [TM.pdf]     the PDF (Change 10 edition) is only needed to redraw the texture
"""
import os, sys
import numpy as np

import panel_layout as L
import tmart

PANELS = L.PANELS

# the old face plate's outline in panel.xml's frame (y, z; m), counter-clockwise seen from the cockpit
OUTER = [(0.0000, -0.2891), (0.2489, -0.2891), (0.4120, -0.2413), (0.8324, -0.2412), (0.8329, -0.1302),
         (0.8336, -0.0741), (0.8298, -0.0207), (0.8195, 0.0268), (0.8079, 0.0565), (0.7857, 0.0712),
         (0.3137, 0.0836), (0.0000, 0.0835), (-0.3137, 0.0836), (-0.7857, 0.0712), (-0.8079, 0.0565),
         (-0.8195, 0.0268), (-0.8298, -0.0207), (-0.8334, -0.0741), (-0.8327, -0.1302), (-0.8324, -0.2412),
         (-0.4120, -0.2413), (-0.2489, -0.2891)]


def plate_x(y, z):
    """The plate's plane (least squares through the old plate, rms 0.4 mm): pitched 1 deg, top away from the crew."""
    return 0.04300 - 0.00010 * y + 0.01741 * z


# mesh -> (legend no., scale, dz, dx, hole)
#   scale: of the mesh about its origin, where its bezel differs from the TM's (the HSI keeps the old panel's 1.2:
#          its bezel is then the TM's height; the altimeters' and airspeed indicators' bezels are the TM's 3.25 in
#          as they are, the hood above them is extra);
#   dz:    mesh origin (dial centre) above the TM bezel's centre, m (the VSI's sphere sits above its knobs, the
#          HSI's card below its course and distance windows);
#   dx:    origin in front of the plate, m, as the old panel had it;
#   hole:  cut-out in the plate around the dial, half width, half height and corner chamfer in the mesh's own
#          metres, inside its bezel; for the meshes whose dial lies behind the plate's plane.
# The instruments of instruments.py are built on their TM bezel's centre with their back on the plate.
KINDS = {
    'ai': (5, 1.0, 0.004, -0.0036, (0.062, 0.0635, 0.012)),
    'hsi': (6, 1.2, -0.006, -0.0001, (0.047, 0.047, 0.012)),
    'asi': (7, 1.0, 0.0, 0.0035, (0.040, 0.040, 0.010)),
    'alt': (2, 1.0, 0.0, 0.0028, (0.040, 0.040, 0.010)),
    'RadarAlt': (1, 1.0, 0.0, 0.0012, (0.037, 0.037, 0.010)),
    'prop-rpm': (13, 1.054, 0.0, 0.0006, None),
    'caution': (4, 0.89, 0.0, 0.0040, None),
    'combimeter': (19, 1.0, 0.0, -0.0031, None),
    'vsi': (3, 1.0, 0.0, 0.0035, (0.040, 0.040, 0.010)),       # the airspeed indicator's mesh
    'clock': (14, 1.0, 0.0, 0.0003, None),
    'stab': (9, 1.0, 0.0, 0.0003, None),
    'cap': (24, 1.0, 0.0, 0.0003, None),
    'modesel': (11, 1.0, 0.0, 0.0003, None),
    'cismode': (10, 1.0, 0.0, 0.0003, None),
    'afms': (21, 1.0, 0.0, 0.0003, None),
    'ignition': (22, 1.0, 0.0, 0.0003, None),
    'rwr': (20, 1.0, 0.0, 0.0003, None),
    'deice': (0, 1.0, 0.0, 0.0003, None),
    'nvgdim': (26, 1.0, 0.0, 0.0003, None),
    'radaltdim': (27, 1.0, 0.0, 0.0003, None),
}
# (panel.xml path, mesh kind, side, comment)
PLACES = [
    ('Instruments/ai/ai.xml', 'ai', 'pilot', 'VERTICAL SITUATION INDICATOR'),
    ('Instruments/ai/ai.xml', 'ai', 'copilot', None),
    ('Instruments/hsi/hsi.xml', 'hsi', 'pilot', 'HORIZONTAL SITUATION INDICATOR'),
    ('Instruments/hsi/hsi-copilot.xml', 'hsi', 'copilot', None),
    ('Instruments/asi/asi.xml', 'asi', 'pilot', 'AIRSPEED INDICATOR'),
    ('Instruments/asi/asi.xml', 'asi', 'copilot', None),
    ('Instruments/alt/alt.xml', 'alt', 'pilot', 'BAROMETRIC ALTIMETER'),
    ('Instruments/alt/alt.xml', 'alt', 'copilot', None),
    ('Instruments/RadarAlt/RadarAlt.xml', 'RadarAlt', 'pilot', 'RADAR ALTIMETER'),
    ('Instruments/RadarAlt/RadarAlt.xml', 'RadarAlt', 'copilot', None),
    ('Instruments/prop-rpm/prop-rpm.xml', 'prop-rpm', 'pilot', "PILOT'S DISPLAY UNIT"),
    ('Instruments/prop-rpm/prop-rpm.xml', 'prop-rpm', 'copilot', None),
    ('Instruments/caution/caution.xml', 'caution', 'pilot', 'MASTER WARNING PANEL'),
    ('Instruments/caution/caution.xml', 'caution', 'copilot', None),
    ('Instruments/combimeter/combimeter.xml', 'combimeter', 'centre', 'CENTRAL DISPLAY UNIT'),
    ('Instruments/vsi/vsi.xml', 'vsi', 'pilot', 'VERTICAL SPEED INDICATOR'),
    ('Instruments/vsi/vsi.xml', 'vsi', 'copilot', None),
    ('Instruments/clock/clock.xml', 'clock', 'pilot', 'CLOCK'),
    ('Instruments/clock/clock.xml', 'clock', 'copilot', None),
    ('Instruments/stab/stab.xml', 'stab', 'pilot', 'STABILATOR INDICATOR'),
    ('Instruments/stab/stab.xml', 'stab', 'copilot', None),
    ('Instruments/modesel/modesel.xml', 'modesel', 'pilot', 'VSI/HSI MODE SELECTOR'),
    ('Instruments/modesel/modesel.xml', 'modesel', 'copilot', None),
    ('Instruments/cismode/cismode.xml', 'cismode', 'pilot', 'CIS MODE SELECTOR'),
    ('Instruments/radaltdim/radaltdim.xml', 'radaltdim', 'pilot', 'RAD ALT DIMMING'),
    ('Instruments/radaltdim/radaltdim.xml', 'radaltdim', 'copilot', None),
    ('Instruments/nvgdim/nvgdim.xml', 'nvgdim', 'copilot', 'NVG DIMMING CONTROL PANEL'),
    ('Instruments/cap/cap.xml', 'cap', 'centre', 'CAUTION/ADVISORY PANEL'),
    ('Instruments/afms/afms.xml', 'afms', 'centre', 'AUXILIARY FUEL MANAGEMENT PANEL'),
    ('Instruments/ignition/ignition.xml', 'ignition', 'centre', 'ENGINE IGNITION SWITCH'),
    ('Instruments/rwr/rwr.xml', 'rwr', 'centre', 'RADAR WARNING INDICATOR'),
    ('Instruments/deice/deice.xml', 'deice', 'centre', 'ICE RATE METER, BLADE DEICE AND IRCM PANELS'),
]
MESHED = {k[0] for k in KINDS.values()}

# TM art outside the items' bezels that belongs on the plate (sheet 1 pt): the ignition switch's OFF/ON and placard
LABELS = [(474.0, 512.0, 218.0, 262.0)]

TEX = 'panel-face.png'
TEX_W, TEX_H = 4096, 1024
GREY, LINE = 54, 165            # the old plate's grey; the art's lines
YMIN, YMAX = min(p[0] for p in OUTER), max(p[0] for p in OUTER)
ZMIN, ZMAX = min(p[1] for p in OUTER), max(p[1] for p in OUTER)


def tm_item(num, side):
    return next(it for it in L.items() if it['num'] == num and it['side'] == side)


def places():
    out = []
    for path, kind, side, comment in PLACES:
        num, s, dz, dx, hole = KINDS[kind]
        it = tm_item(num, side)
        y, z = it['y'], it['z'] + dz
        poly = None
        if hole:
            a, b, c = (v * s for v in hole)
            poly = [(y + p, z + q) for p, q in [(-a + c, -b), (a - c, -b), (a, -b + c), (a, b - c), (a - c, b),
                                                 (-a + c, b), (-a, b - c), (-a, -b + c)]]
        out.append(dict(path=path, kind=kind, side=side, comment=comment, num=num, scale=s,
                        x=plate_x(y, z) + dx, y=y, z=z, hole=poly))
    return out


# ---------------------------------------------------------------- face plate

def chains(poly):
    """Lower and upper chains of a polygon that is monotone in y (counter-clockwise), each sorted by y."""
    n = len(poly); i0 = min(range(n), key=lambda i: (poly[i][0], poly[i][1])); i1 = max(range(n), key=lambda i: poly[i][0])
    lower, i = [poly[i0]], i0
    while i != i1:
        i = (i + 1) % n; lower.append(poly[i])
    upper, i = [poly[i0]], i0
    while i != i1:
        i = (i - 1) % n; upper.append(poly[i])
    return lower, upper


def line_at(chain, ya, yb):
    """z of the chain's edge over the slab (ya, yb), at ya and at yb."""
    m = (ya + yb) / 2
    for (y0, z0), (y1, z1) in zip(chain, chain[1:]):
        if y0 <= m <= y1 and y1 > y0:
            f = lambda y: z0 + (z1 - z0) * (y - y0) / (y1 - y0)
            return f(ya), f(yb)
    raise ValueError('slab outside chain')


def inside(p, poly):
    y, z = p; c = False
    for (y0, z0), (y1, z1) in zip(poly, poly[1:] + poly[:1]):
        if (z0 > z) != (z1 > z) and y < y0 + (y1 - y0) * (z - z0) / (z1 - z0):
            c = not c
    return c


def plate(holes):
    """The plate with its holes as quads (and triangles) in vertical slabs, without T-junctions: (y, z) polygons,
    counter-clockwise seen from the cockpit."""
    for h in holes:
        assert all(inside(p, OUTER) for p in h), 'hole outside the plate'
    for i, h in enumerate(holes):
        for g in holes[i + 1:]:
            assert not any(inside(p, g) for p in h) and not any(inside(p, h) for p in g), 'holes overlap'
    ys = sorted({round(p[0], 6) for poly in [OUTER] + holes for p in poly})
    oc = chains(OUTER); hc = [chains(h) for h in holes]
    pieces = []
    for ya, yb in zip(ys, ys[1:]):
        lo, hi = line_at(oc[0], ya, yb), line_at(oc[1], ya, yb)
        cuts = sorted((line_at(c[0], ya, yb), line_at(c[1], ya, yb)) for h, c in zip(holes, hc)
                      if min(p[0] for p in h) <= ya + 1e-6 and max(p[0] for p in h) >= yb - 1e-6)
        bottom = lo
        for hlo, hhi in cuts:
            pieces.append((ya, yb, bottom, hlo)); bottom = hhi
        pieces.append((ya, yb, bottom, hi))
    # every vertex on each slab line, so neighbouring slabs share their edges' points
    on_line = {}
    for ya, yb, b, t in pieces:
        for y, zs in ((ya, (b[0], t[0])), (yb, (b[1], t[1]))):
            on_line.setdefault(round(y, 6), set()).update(round(z, 7) for z in zs)
    polys = []
    for ya, yb, b, t in pieces:
        left = sorted(z for z in on_line[round(ya, 6)] if b[0] - 1e-7 < z < t[0] + 1e-7)
        right = sorted(z for z in on_line[round(yb, 6)] if b[1] - 1e-7 < z < t[1] + 1e-7)
        p = [(ya, b[0])]
        p += [(yb, z) for z in [b[1]] + [z for z in right if b[1] + 1e-7 < z < t[1] - 1e-7] + [t[1]]]
        p += [(ya, z) for z in [t[0]] + [z for z in left[::-1] if b[0] + 1e-7 < z < t[0] - 1e-7]]
        q = []
        for v in p:                                     # drop repeats (slabs that end in a point)
            if not q or abs(v[0] - q[-1][0]) > 1e-9 or abs(v[1] - q[-1][1]) > 1e-9:
                q.append(v)
        if len(q) > 1 and abs(q[0][0] - q[-1][0]) < 1e-9 and abs(q[0][1] - q[-1][1]) < 1e-9:
            q.pop()
        if len(q) >= 3:
            polys.append(q)
    return polys


def ac_object(polys):
    V, idx = [], {}
    def vid(p):
        k = (round(p[0], 7), round(p[1], 7))
        if k not in idx:
            idx[k] = len(V); V.append(p)
        return idx[k]
    S = [[vid(p) for p in poly] for poly in polys]
    out = ['OBJECT poly', 'name "panel2"', 'loc 0 0 -0', 'data 6', 'panel2', 'texture "%s"' % TEX,
           'texrep 1.000000 1.000000', 'crease 30.000001', 'numvert %d' % len(V)]
    out += ['%.6f %.6f %.6f' % (plate_x(y, z), z, -y) for y, z in V]
    out.append('numsurf %d' % len(S))
    for s in S:
        out += ['SURF 0x10', 'mat 0', 'refs %d' % len(s)]
        out += ['%d %.6f %.6f' % (i, (V[i][0] - YMIN) / (YMAX - YMIN), (V[i][1] - ZMIN) / (ZMAX - ZMIN)) for i in s]
    out.append('kids 0')
    return out


def write_plate(pl):
    path = os.path.join(PANELS, 'panel.ac')
    lines = open(path, encoding='latin-1').read().split('\n')
    i = next(k for k, s in enumerate(lines) if s.strip() == 'name "panel2"') - 1
    j = next(k for k in range(i + 1, len(lines)) if lines[k].startswith('kids'))
    polys = plate([p['hole'] for p in pl if p['hole']])
    lines[i:j + 1] = ac_object(polys)
    open(path, 'w', encoding='latin-1').write('\n'.join(lines))
    return polys


# ---------------------------------------------------------------- panel.xml

HEAD = """<?xml version="1.0" encoding="ISO-8859-1"?>

<PropertyList>

  <path>panel.ac</path>

  <animation>
    <object-name>panel1</object-name>
    <object-name>panel2</object-name>
  </animation>

  <!-- PEDESTAL -->
  <model>
    <path>Pedestal/pedestal.xml</path>
    <offsets>
      <x-m>  0.29576 </x-m>
      <y-m>  0.00000 </y-m>
      <z-m> -0.56926 </z-m>
      <pitch-deg> 30.540 </pitch-deg>
    </offsets>
  </model>

  <!-- OVERHEAD -->
  <model>
    <path>Overhead/overhead.xml</path>
    <offsets>
      <x-m> 0.93298 </x-m>
      <y-m> 0.00000 </y-m>
      <z-m> 0.16245 </z-m>
      <pitch-deg> 30.540 </pitch-deg>
    </offsets>
  </model>

  <!-- Instruments on their places in the UH-60A panel, TM 1-1520-237-10 figure 2-9 (legend numbers below), written
       by fdm/cockpit/panel_face.py: change them there. The face plate (panel2) is cut out and textured to match. -->
"""


def write_xml(pl):
    out = [HEAD]
    for p in pl:
        if p['comment']:
            out.append('  <!-- %s (%d) -->' % (p['comment'], p['num']))
        out += ['  <model>', '    <path>%s</path>' % p['path'], '    <offsets>',
                '      <x-m> %7.4f </x-m>' % p['x'], '      <y-m> %7.4f </y-m>' % p['y'],
                '      <z-m> %7.4f </z-m>' % p['z'], '      <pitch-deg> -1 </pitch-deg>', '    </offsets>']
        if p['scale'] != 1:
            out += ['    <overlay>', '      <animation n="100">', '        <type>scale</type>',
                    '        <property>constants/zero</property>'] + \
                   ['        <%s-offset> %.3f </%s-offset>' % (a, p['scale'], a) for a in 'xyz'] + \
                   ['      </animation>', '    </overlay>']
        out += ['  </model>', '']
    out.append('</PropertyList>\n')
    open(os.path.join(PANELS, 'panel.xml'), 'w').write('\n'.join(out))


# ---------------------------------------------------------------- texture

def write_texture(pdf, dpi=400):
    from PIL import Image, ImageDraw
    art = tmart.Art(pdf, dpi); k = art.k; x0, y0 = tmart.X0, tmart.Y0
    W, H = art.img.size
    # where the art shows: the items without a 3-D instrument, and the labels, less the 3-D instruments' bezels
    mask = Image.new('L', (W, H), 0); md = ImageDraw.Draw(mask)
    rect = lambda b, m: [(b[0] - m - x0) * k, (b[2] - m - y0) * k, (b[1] + m - x0) * k, (b[3] + m - y0) * k]
    for num, name, side, sheet, a, b, c, d in L.ITEMS:
        if num not in MESHED:
            md.rectangle(rect(art.box(a, b, c, d, sheet), 1.5), fill=255)
    for b in LABELS:
        md.rectangle(rect(b, 0), fill=255)
    for num, name, side, sheet, a, b, c, d in L.ITEMS:
        if num in MESHED:
            md.rectangle(rect(art.box(a, b, c, d, sheet), 0), fill=0)
    a = np.asarray(art.img, float); m = np.asarray(mask, float) / 255
    tex = GREY + (255 - a) / 255 * (LINE - GREY) * m
    img = Image.fromarray(np.clip(tex, 0, 255).astype(np.uint8))
    # texture pixel -> panel (y, z) -> figure pt -> art pixel (affine)
    sy, sz = (YMAX - YMIN) / TEX_W, (ZMAX - ZMIN) / TEX_H
    ax = sy / L.S * k; cx = (YMIN / L.S + L.X_CL - x0) * k
    ey = sz / L.S * k; fy = (L.Y_BOT - (ZMAX - L.Z_BOT) / L.S - y0) * k
    out = img.transform((TEX_W, TEX_H), Image.AFFINE, (ax, 0, cx, 0, ey, fy), resample=Image.BICUBIC, fillcolor=GREY)
    out = out.convert('RGB').quantize(colors=32)
    out.save(os.path.join(PANELS, TEX), optimize=True)


if __name__ == '__main__':
    pl = places()
    polys = write_plate(pl)
    write_xml(pl)
    if len(sys.argv) > 1:
        write_texture(sys.argv[1])
    print('plate: %d polygons; instruments:' % len(polys))
    for p in pl:
        print('  %-36s %-7s x %.4f y %+.4f z %+.4f scale %.3f%s' % (p['path'], p['side'], p['x'], p['y'], p['z'],
              p['scale'], ' hole' if p['hole'] else ''))
