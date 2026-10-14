"""Builds the UH-60A instrument panel from the layout measured in panel_layout.py (TM 1-1520-237-10 fig. 2-9):

- the face plate, object panel2 of Panels/panel.ac: the old plate's outline (it fits the cockpit), cut out where
  the round instruments' dials sit behind its plane;
- the plate's texture, Panels/panel-face.png: plain paint (the placards are decals, instruments.py);
- the old glareshield (panel1), raised where it hid the panel's top from the design eyes, the plate's top edge
  with it;
- the instruments' places in Panels/panel.xml: each 3-D instrument on its TM item, scaled where its bezel differs
  from the TM's. The torque dials, glass displays and moving maps of the old panel aren't in a UH-60A and are gone
  (the files stay in Instruments/). The instruments added to the panel are built by instruments.py.

    python3 panel_face.py
"""
import math, os, sys
import numpy as np

import panel_layout as L

PANELS = L.PANELS

# the old face plate's outline in panel.xml's frame (y, z; m), counter-clockwise seen from the cockpit
OUTER = [(0.0000, -0.2891), (0.2489, -0.2891), (0.4120, -0.2413), (0.8324, -0.2412), (0.8329, -0.1302),
         (0.8336, -0.0741), (0.8298, -0.0207), (0.8195, 0.0268), (0.8079, 0.0565), (0.7857, 0.0712),
         (0.3137, 0.0836), (0.0000, 0.0835), (-0.3137, 0.0836), (-0.7857, 0.0712), (-0.8079, 0.0565),
         (-0.8195, 0.0268), (-0.8298, -0.0207), (-0.8334, -0.0741), (-0.8327, -0.1302), (-0.8324, -0.2412),
         (-0.4120, -0.2413), (-0.2489, -0.2891)]


def plate_x(y, z):
    """The plate's plane (least squares through the old plate, rms 0.4 mm): pitched 1 deg, top toward the crew (+x)."""
    return 0.04300 - 0.00010 * y + 0.01741 * z


PITCH = math.degrees(math.atan(0.01741))    # the instruments' pitch to lie in the plate (a positive pitch tips +z to +x)


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
    'hsi60': (6, 1.0, -0.0077, 0.0003, None),                    # avionics.py, the card 7.7 mm below the bezel's centre
    'asi60': (7, 1.0, 0.0, 0.0003, None),                       # gauges.py: on the plate, no cut-out
    'alt60': (2, 1.0, 0.0, 0.0003, None),
    'apn209': (1, 1.0, 0.003, 0.0003, None),                    # its dial 3 mm above the case's centre
    'vsi60': (3, 1.0, 0.0, 0.0003, None),
    'prop-rpm': (13, 1.054, 0.0, 0.0006, None),
    'caution': (4, 1.0, 0.0, 0.0003, None),                     # cap.py: the master warning panel, figure 2-23
    'combimeter': (19, 1.0, 0.0, -0.0031, None),
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
    'stabplac': (8, 1.0, 0.0, 0.0003, None),                    # placards: decals on the plate
    'radiocall': (12, 1.0, 0.0, 0.0003, None),
    'securepl': (25, 1.0, 0.0, 0.0003, None),
    'radiosel': (23, 1.0, 0.0, 0.0003, None),
}
# (panel.xml path, mesh kind, side, comment)
PLACES = [
    ('Instruments/ai/ai.xml', 'ai', 'pilot', 'VERTICAL SITUATION INDICATOR'),
    ('Instruments/ai/ai-copilot.xml', 'ai', 'copilot', None),
    ('Instruments/hsi60/hsi60.xml', 'hsi60', 'pilot', 'HORIZONTAL SITUATION INDICATOR'),
    ('Instruments/hsi60/hsi60-copilot.xml', 'hsi60', 'copilot', None),
    ('Instruments/asi60/asi60.xml', 'asi60', 'pilot', 'AIRSPEED INDICATOR'),
    ('Instruments/asi60/asi60-copilot.xml', 'asi60', 'copilot', None),
    ('Instruments/alt60/alt60.xml', 'alt60', 'pilot', 'BAROMETRIC ALTIMETER'),
    ('Instruments/alt60/alt60-copilot.xml', 'alt60', 'copilot', None),
    ('Instruments/apn209/apn209.xml', 'apn209', 'pilot', 'RADAR ALTIMETER AN/APN-209'),
    ('Instruments/apn209/apn209-copilot.xml', 'apn209', 'copilot', None),
    ('Instruments/prop-rpm/prop-rpm.xml', 'prop-rpm', 'pilot', "PILOT'S DISPLAY UNIT"),
    ('Instruments/prop-rpm/prop-rpm.xml', 'prop-rpm', 'copilot', None),
    ('Instruments/caution/caution.xml', 'caution', 'pilot', 'MASTER WARNING PANEL'),
    ('Instruments/caution/caution.xml', 'caution', 'copilot', None),
    ('Instruments/combimeter/combimeter.xml', 'combimeter', 'centre', 'CENTRAL DISPLAY UNIT'),
    ('Instruments/vsi60/vsi60.xml', 'vsi60', 'pilot', 'VERTICAL SPEED INDICATOR'),
    ('Instruments/vsi60/vsi60-copilot.xml', 'vsi60', 'copilot', None),
    ('Instruments/clock/clock.xml', 'clock', 'pilot', 'CLOCK'),
    ('Instruments/clock/clock.xml', 'clock', 'copilot', None),
    ('Instruments/stab/stab.xml', 'stab', 'pilot', 'STABILATOR INDICATOR'),
    ('Instruments/stab/stab.xml', 'stab', 'copilot', None),
    ('Instruments/modesel/modesel.xml', 'modesel', 'pilot', 'VSI/HSI MODE SELECTOR'),
    ('Instruments/modesel/modesel-copilot.xml', 'modesel', 'copilot', None),
    ('Instruments/cismode/cismode.xml', 'cismode', 'pilot', 'CIS MODE SELECTOR'),
    ('Instruments/radaltdim/radaltdim.xml', 'radaltdim', 'pilot', 'RAD ALT DIMMING'),
    ('Instruments/radaltdim/radaltdim.xml', 'radaltdim', 'copilot', None),
    ('Instruments/nvgdim/nvgdim.xml', 'nvgdim', 'copilot', 'NVG DIMMING CONTROL PANEL'),
    ('Instruments/cap/cap.xml', 'cap', 'centre', 'CAUTION/ADVISORY PANEL'),
    ('Instruments/afms/afms.xml', 'afms', 'centre', 'AUXILIARY FUEL MANAGEMENT PANEL'),
    ('Instruments/ignition/ignition.xml', 'ignition', 'centre', 'ENGINE IGNITION SWITCH'),
    ('Instruments/rwr/rwr.xml', 'rwr', 'centre', 'RADAR WARNING INDICATOR'),
    ('Instruments/deice/deice.xml', 'deice', 'centre', 'ICE RATE METER, BLADE DEICE AND IRCM PANELS'),
    ('Instruments/stabplac/stabplac.xml', 'stabplac', 'pilot', 'STABILATOR POSITION PLACARD'),
    ('Instruments/stabplac/stabplac.xml', 'stabplac', 'copilot', None),
    ('Instruments/radiocall/radiocall.xml', 'radiocall', 'pilot', 'RADIO CALL PLACARD'),
    ('Instruments/radiocall/radiocall.xml', 'radiocall', 'copilot', None),
    ('Instruments/securepl/securepl.xml', 'securepl', 'centre', 'SECURE WARNING PLACARD'),
    ('Instruments/radiosel/radiosel.xml', 'radiosel', 'centre', 'RADIO SELECT PLACARD'),
]
MESHED = {k[0] for k in KINDS.values()}


TEX = 'panel-face.png'
GREY = 54                       # the old plate's grey
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


# ---------------------------------------------------------------- glareshield

# the design eyes (Systems/views.xml: BL 24, WL 257, 5.72479 m back) in panel.xml's frame (Panels/all-panels.xml: its
# origin 6.554 m back and 0.746 m down, pitched -30.54 deg)
_TH = math.radians(-30.54)
EYE_X = math.cos(_TH) * (6.554 - 5.72479) - math.sin(_TH) * (0.746 - 0.26416)
EYE_Z = math.sin(_TH) * (6.554 - 5.72479) + math.cos(_TH) * (0.746 - 0.26416)
EYE_Y = 0.6096
LIP_CLEAR = 0.004               # m, the lip above the eyes' line to the plate's top edge


REAR_CLEAR = 0.003              # m, the glareshield's rear edge (the plate's top) above the instruments' tops


def item_top(y):
    """The top of the TM items at butt line y or its mirror image (the hood stays symmetric); None if there are none."""
    tops = [it['z1'] for it in L.items() if it['y0'] <= y <= it['y1'] or it['y0'] <= -y <= it['y1']]
    return max(tops) if tops else None


def top_z(y):
    """How high the panel reaches at y: its plate's top edge or the top of a TM item there, whichever is higher. None
    off the plate."""
    top = sorted(p for p in OUTER if p[1] > 0.05)
    ys, zs = zip(*top)
    if not ys[0] <= y <= ys[-1]:
        return None
    return max([float(np.interp(y, ys, zs))] + [it['z1'] for it in L.items() if it['y0'] <= y <= it['y1']])


def sight_z(x, y):
    """The lowest z at (x, y) in front of the plate that keeps the panel in sight from both design eyes, up to its top
    (None where neither eye looks past (x, y) at the panel)."""
    need = None
    for ey in (EYE_Y, -EYE_Y):
        t = (EYE_X - x) / (EYE_X - plate_x(0, ZMAX))       # how far along the line from the eye to the plate
        zt = top_z(ey + (y - ey) / t)
        if zt is not None:
            z = EYE_Z + t * (zt - EYE_Z)
            need = z if need is None else max(need, z)
    return need


def read_hood():
    """panel1 of Panels/panel.ac: its lines, the line of its first vertex and its vertices (x toward the crew, z up,
    -y: the .ac frame)."""
    lines = open(os.path.join(PANELS, 'panel.ac'), encoding='latin-1').read().split('\n')
    i = next(k for k, s in enumerate(lines) if s.strip() == 'name "panel1"')
    n0 = next(k for k in range(i, len(lines)) if lines[k].startswith('numvert')) + 1
    return lines, n0, [list(map(float, lines[k].split())) for k in range(n0, n0 + int(lines[n0 - 1].split()[1]))]


def hood_columns(V):
    """The hood's vertices (in front of the plate, above z 0.04) by butt line, a column and its mirror image together:
    [(|y|, [vertex numbers])], inboard first: the centreline, 0.31 m, 0.79 m and the end caps at 0.81 m."""
    ks = sorted((abs(V[k][2]), k) for k in range(len(V)) if V[k][0] > 0.03 and V[k][1] > 0.04 and abs(V[k][2]) < 0.815)
    cols = []
    for a, k in ks:
        if cols and a - cols[-1][2] < 0.008:
            cols[-1][1].append(k); cols[-1][2] = a
        else:
            cols.append([a, [k], a])
    return [(c[0], c[1]) for c in cols]


def sync_outline(V):
    """The plate's top edge (OUTER, z > 0) where the hood's rear underside edge is: the lowest vertex at the plate
    (x < 0.05) on each of OUTER's butt lines."""
    global ZMAX
    for n, (y, z) in enumerate(OUTER):
        if z > 0:
            at = [V[k][1] for k in range(len(V)) if V[k][0] < 0.05 and V[k][1] > 0 and abs(-V[k][2] - y) < 0.0005]
            if at:
                OUTER[n] = (y, min(at))
    ZMAX = max(p[1] for p in OUTER)


def write_hood(lines, n0, V, moved):
    for k in moved:
        lines[n0 + k] = '%.6f %.6f %s' % (V[k][0], V[k][1], lines[n0 + k].split()[2])
    open(os.path.join(PANELS, 'panel.ac'), 'w', encoding='latin-1').write('\n'.join(lines))


def lift_glareshield():
    """The old glareshield (panel1) sits about 4 cm lower over the plate than the TM's panel outline: the master
    warning panels' top row reached into its underside, and its drooping front lip hid the panel's top 1-2 cm from
    the design eyes. Raised by the least that clears them, symmetric, in two steps (again, nothing changes):

    - the hood: its columns of vertices lifted whole (the end caps with the outboard column) so that its rear edge,
      the plate's top, is REAR_CLEAR above every instrument's top;
    - its front lip: lifted so that the lower edge, sampled between the columns, is LIP_CLEAR above the eyes' lines
      of sight to the panel's top.

    Returns the lifts (m) by butt line."""
    from scipy.optimize import linprog
    lines, n0, V = read_hood()
    cols = hood_columns(V)
    var = list(range(len(cols)))
    var[-1] = var[-2]                                             # the end caps go with the outboard column
    nv = max(var) + 1
    rear = [min((k for k in ks if V[k][0] < 0.05), key=lambda k: V[k][1]) for _, ks in cols]
    A, b = [], []
    for a in range(len(cols) - 1):
        ya, yb = cols[a][0], cols[a + 1][0]
        for f in np.linspace(0, 1, 41):
            top = item_top((1 - f) * ya + f * yb)
            z = (1 - f) * V[rear[a]][1] + f * V[rear[a + 1]][1]
            if top is not None and top + REAR_CLEAR > z:
                row = [0.0] * nv; row[var[a]] -= 1 - f; row[var[a + 1]] -= f
                A.append(row); b.append(-(top + REAR_CLEAR - z))
    hood = [0.0] * nv; moved = set()
    if A:
        hood = list(linprog(np.ones(nv), A_ub=A, b_ub=b, bounds=[(0, None)] * nv).x)
        for (y, ks), v in zip(cols, var):
            if hood[v] > 1e-6:
                for k in ks:
                    V[k][1] += hood[v]; moved.add(k)
    sync_outline(V)

    lip = [(y, [k for k in ks if V[k][0] > 0.2]) for y, ks in cols[:-1]]      # its lower and upper edge per column
    low = [min(ks, key=lambda k: V[k][1]) for _, ks in lip]
    order = sorted(range(len(lip)), key=lambda c: -lip[c][0])                 # from the left end round to the right
    path = [(c, -1) for c in order] + [(c, 1) for c in reversed(order) if lip[c][0] > 0.001]
    A, b = [], []
    for (ca, sa), (cb, sb) in zip(path, path[1:]):
        pa, pb = V[low[ca]], V[low[cb]]
        for f in np.linspace(0, 1, 41):
            x, z = (1 - f) * pa[0] + f * pb[0], (1 - f) * pa[1] + f * pb[1]
            y = (1 - f) * sa * abs(pa[2]) + f * sb * abs(pb[2])
            need = sight_z(x, y)
            if need is not None and need + LIP_CLEAR > z:
                row = [0.0] * len(lip); row[ca] -= 1 - f; row[cb] -= f
                A.append(row); b.append(-(need + LIP_CLEAR - z))
    edge = [0.0] * len(lip)
    if A:
        edge = list(linprog(np.ones(len(lip)), A_ub=A, b_ub=b, bounds=[(0, None)] * len(lip)).x)
        for (y, ks), d in zip(lip, edge):
            if d > 1e-6:
                for k in ks:
                    V[k][1] += d; moved.add(k)
    if moved:
        write_hood(lines, n0, V, moved)
    return {round(y, 3): (hood[v], edge[c] if c < len(edge) else 0.0) for c, ((y, _), v) in enumerate(zip(cols, var))}


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
                '      <z-m> %7.4f </z-m>' % p['z'], '      <pitch-deg> %.3f </pitch-deg>' % PITCH, '    </offsets>']
        if p['scale'] != 1:
            out += ['    <overlay>', '      <animation n="100">', '        <type>scale</type>',
                    '        <property>constants/zero</property>'] + \
                   ['        <%s-offset> %.3f </%s-offset>' % (a, p['scale'], a) for a in 'xyz'] + \
                   ['      </animation>', '    </overlay>']
        out += ['  </model>', '']
    out.append('</PropertyList>\n')
    open(os.path.join(PANELS, 'panel.xml'), 'w').write('\n'.join(out))


# ---------------------------------------------------------------- texture

def write_texture():
    """The plate's paint: everything on it is a 3-D part or a decal now."""
    from PIL import Image
    Image.new('RGB', (64, 16), (GREY,) * 3).save(os.path.join(PANELS, TEX), optimize=True)


if __name__ == '__main__':
    pl = places()
    lifts = lift_glareshield()
    polys = write_plate(pl)
    write_xml(pl)
    write_texture()
    print('glareshield lifted (hood, lip) at |y|: ' + ', '.join('%.2f %.1f + %.1f mm' % (y, h * 1000, e * 1000)
                                                           for y, (h, e) in sorted(lifts.items())))
    print('plate: %d polygons; instruments:' % len(polys))
    for p in pl:
        print('  %-36s %-7s x %.4f y %+.4f z %+.4f scale %.3f%s' % (p['path'], p['side'], p['x'], p['y'], p['z'],
              p['scale'], ' hole' if p['hole'] else ''))
