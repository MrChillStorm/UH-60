"""UH-60A instrument panel layout measured from TM 1-1520-237-10, figure 2-9 sheets 1 and 2 (the UH panel;
sheets 3 and 4 are the EH-60A), and compared with the cockpit model's panel.

The figure is vector line art. The boxes below are the outer bezel outlines of each item in PDF points, read
from the page's paths (pdftocairo -svg), except the stabilator indicator, which is drawn as loose strokes and
was read from a 300 dpi render. Sheet 1 holds the copilot's side and the centre, sheet 2 the centre right and the
pilot's side; the central display unit runs over the join.

Frames:
- sheet 2 -> sheet 1: a pure shift, from the OFF DIGITS label printed on both (x and y of both words agree to
  0.001 pt). The panel's bottom edge then lies at the same height on both sheets.
- scale: 3.25 in per 3ATI bezel (barometric altimeter and vertical speed indicator, 46.2 pt on both sheets). With
  it the panel's outer outline is 1.669 m wide; the model's face plate is 1.667 m.
- lateral: the centreline is the middle of the outer outline, which is also the middle of the two VSIs (within
  0.3 pt). y is panel.xml's (and FlightGear's): + right, the pilot's side.
- vertical: the panel's bottom edge outboard (flat on both sheets) on the face plate's, z = -0.241 m in
  panel.xml's frame. The face plate is the y-z plane of that frame, the plane of the figure.

Writes panel-layout.md (the table) and panel-layout.png (the layout over the model's face plate and instruments).
"""
import os, re, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PANELS = os.path.join(HERE, '..', '..', 'UH-60A', 'Models', 'Interior', 'Panels')

JOIN = (437.681, -27.591)       # sheet 2 -> sheet 1, pt: OFF DIGITS at (512.593, 164.483) and (74.912, 192.074)
S = 0.0254 * 3.25 / 46.2        # m per pt
X_CL = 530.2                    # centreline, pt (outer outline 63.2 .. 997.2 after the join)
Y_BOT, Z_BOT = 265.7, -0.241    # bottom edge outboard: pt in the figure, m on the face plate

# (legend no., item, side, sheet, x0, x1, y0, y1 in pt on that sheet)
ITEMS = [
    (7, 'Airspeed indicator', 'copilot', 1, 83.1, 129.7, 105.0, 151.7),
    (5, 'Vertical situation indicator', 'copilot', 1, 144.45, 226.15, 102.15, 187.25),
    (6, 'Horizontal situation indicator', 'copilot', 1, 144.2, 227.4, 190.0, 260.2),
    (4, 'Master warning panel', 'copilot', 1, 153.5, 216.6, 85.3, 101.4),
    (1, 'Radar altimeter', 'copilot', 1, 238.75, 287.05, 102.2, 150.4),
    (2, 'Barometric altimeter', 'copilot', 1, 239.5, 285.7, 158.2, 204.4),
    (3, 'Vertical speed indicator', 'copilot', 1, 239.4, 285.6, 213.6, 259.8),
    (13, "Pilot's display unit", 'copilot', 1, 293.15, 351.85, 102.1, 206.7),
    (14, 'Clock', 'copilot', 1, 303.3, 335.8, 215.2, 247.7),
    (12, 'Radio call placard', 'copilot', 1, 255.8, 276.6, 91.5, 100.4),
    (27, 'Rad alt dimming', 'copilot', 1, 295.5, 308.7, 87.9, 99.5),
    (26, 'NVG dimming control panel', 'copilot', 1, 322.1, 350.2, 87.0, 99.1),
    (9, 'Stabilator indicator', 'copilot', 1, 81.6, 118.2, 164.9, 191.9),
    (8, 'Stabilator position placard', 'copilot', 1, 121.2, 138.9, 165.3, 197.4),
    (11, 'VSI/HSI mode selector', 'copilot', 1, 81.5, 133.8, 201.6, 256.7),
    (24, 'Caution/advisory panel', 'centre', 1, 362.6, 451.4, 87.3, 241.0),
    (19, 'Central display unit', 'centre', 1, 456.6, 154.3 + JOIN[0], 86.2, 173.7),
    (21, 'Auxiliary fuel management panel', 'centre', 1, 462.1, 516.2, 175.8, 215.3),
    (22, 'Engine ignition switch', 'centre', 1, 477.5, 497.7, 228.7, 248.9),
    (25, 'Secure warning placard', 'centre', 1, 381.4, 443.8, 268.4, 280.9),
    (23, 'Radio select placard', 'centre', 1, 452.2, 521.7, 268.4, 280.8),
    (20, 'Radar warning indicator', 'centre', 2, 92.9, 146.4, 205.0, 258.6),
    (0, 'De-ice/IRCM mounting', 'centre', 2, 162.0, 255.6, 121.6, 270.6),
    (15, 'Ice rate meter', 'centre', 2, 191.8, 225.3, 126.6, 160.1),
    (16, 'Blade deice control panel', 'centre', 2, 170.1, 248.2, 168.5, 207.0),
    (17, 'Blade deice test panel', 'centre', 2, 169.5, 247.6, 210.5, 245.7),
    (18, 'IRCM control panel', 'centre', 2, 164.4, 253.5, 250.1, 267.1),
    (13, "Pilot's display unit", 'pilot', 2, 268.9, 326.4, 129.9, 231.8),
    (7, 'Airspeed indicator', 'pilot', 2, 334.7, 380.4, 125.3, 171.1),
    (5, 'Vertical situation indicator', 'pilot', 2, 395.85, 477.55, 128.85, 213.95),
    (6, 'Horizontal situation indicator', 'pilot', 2, 395.3, 478.5, 217.8, 288.0),
    (4, 'Master warning panel', 'pilot', 2, 404.8, 468.0, 112.1, 128.2),
    (1, 'Radar altimeter', 'pilot', 2, 489.9, 538.2, 129.2, 177.6),
    (2, 'Barometric altimeter', 'pilot', 2, 490.9, 537.1, 185.6, 231.7),
    (3, 'Vertical speed indicator', 'pilot', 2, 490.8, 537.0, 241.6, 287.8),
    (14, 'Clock', 'pilot', 2, 280.0, 312.5, 242.8, 275.3),
    (12, 'Radio call placard', 'pilot', 2, 285.5, 306.4, 115.4, 124.4),
    (27, 'Rad alt dimming', 'pilot', 2, 312.85, 329.95, 114.95, 124.65),
    (10, 'CIS mode selector', 'pilot', 2, 331.2, 389.4, 209.2, 234.8),
    (11, 'VSI/HSI mode selector', 'pilot', 2, 333.9, 386.3, 236.6, 290.6),
    (9, 'Stabilator indicator', 'pilot', 2, 332.2, 368.6, 177.6, 204.5),
    (8, 'Stabilator position placard', 'pilot', 2, 371.3, 388.9, 175.4, 207.2),
]

# copilot's half of the panel's outer outline, pt on sheet 1, from the top at the centreline round the outboard
# edge to the bottom at the centreline (the top line is the glareshield's; the bottom slopes down to the centre)
OUTLINE = [(X_CL, 62.2), (326.9, 62.2), (106.1, 78.8), (83.5, 86.5), (68.2, 103.5), (63.2, 120.9), (63.2, 231.1),
           (72.7, 245.9), (93.1, 265.7), (274.9, 265.7), (X_CL, 289.0)]

# the model's instruments that stand for figure items (panel.xml file -> legend number)
MODEL_FOR = {'ai': 5, 'hsi': 6, 'hsi60': 6, 'alt': 2, 'asi': 7, 'RadarAlt': 1, 'asi60': 7, 'alt60': 2, 'apn209': 1, 'vsi60': 3, 'prop-rpm': 13,
             'combimeter': 19, 'caution': 4, 'vsi': 3, 'clock': 14, 'stab': 9, 'cap': 24, 'modesel': 11, 'cismode': 10,
             'afms': 21, 'ignition': 22, 'rwr': 20, 'deice': 0, 'nvgdim': 26, 'radaltdim': 27}


def to_panel(x, y, sheet=1):
    """Figure pt (sheet 1 or 2) -> panel.xml y, z in m."""
    if sheet == 2:
        x, y = x + JOIN[0], y + JOIN[1]
    return (x - X_CL) * S, Z_BOT - (y - Y_BOT) * S


def items():
    out = []
    for num, name, side, sheet, x0, x1, y0, y1 in ITEMS:
        ya, za = to_panel(x0, y1, sheet); yb, zb = to_panel(x1, y0, sheet)
        out.append(dict(num=num, name=name, side=side, y0=ya, y1=yb, z0=za, z1=zb,
                        y=(ya + yb) / 2, z=(za + zb) / 2, w=yb - ya, h=zb - za))
    return out


def ac_objects(path):
    """AC3D file -> {object name: world vertices (x, y, z in the .ac frame)}, children placed by their loc."""
    L = open(path, encoding='latin-1').read().split('\n'); out = {}
    def parse(i, off):
        name, loc, V = None, np.zeros(3), None
        while i < len(L):
            t = L[i].split()
            if not t:
                pass
            elif t[0] == 'name':
                name = L[i].split(None, 1)[1].strip('"')
            elif t[0] == 'loc':
                loc = np.array(list(map(float, t[1:4])))
            elif t[0] == 'numvert':
                n = int(t[1]); V = np.array([list(map(float, L[i + 1 + j].split())) for j in range(n)]); i += n
            elif t[0] == 'kids':
                if name is not None and V is not None:
                    out[name] = V + off + loc
                i += 1
                for _ in range(int(t[1])):
                    while not L[i].startswith('OBJECT'):
                        i += 1
                    i = parse(i + 1, off + loc)
                return i
            i += 1
        return i
    i = 0
    while i < len(L):
        i = parse(i + 1, np.zeros(3)) if L[i].startswith('OBJECT') else i + 1
    return out


def face_edges():
    """Boundary edges of the face plate (panel.ac 'panel2') as (y, z) pairs in panel.xml's frame (y = -z_ac)."""
    L = open(os.path.join(PANELS, 'panel.ac'), encoding='latin-1').read().split('\n')
    i = next(k for k, s in enumerate(L) if s.strip() == 'name "panel2"')
    while not L[i].startswith('numvert'):
        i += 1
    n = int(L[i].split()[1]); V = np.array([list(map(float, s.split())) for s in L[i + 1:i + 1 + n]]); i += n + 1
    ns = int(L[i].split()[1]); i += 1; count = {}
    for _ in range(ns):
        while not L[i].startswith('refs'):
            i += 1
        k = int(L[i].split()[1]); r = [int(L[i + 1 + j].split()[0]) for j in range(k)]; i += k + 1
        for a, b in zip(r, r[1:] + r[:1]):
            e = (min(a, b), max(a, b)); count[e] = count.get(e, 0) + 1
    return [((-V[a, 2], V[a, 1]), (-V[b, 2], V[b, 1])) for (a, b), c in count.items() if c == 1]


def model_instruments(xml=None):
    """Instruments placed in panel.xml: name, y, z (origin) and the mesh's front extents y0..y1, z0..z1 (m), with
    the overlay scale if any."""
    s = open(xml or os.path.join(PANELS, 'panel.xml'), encoding='latin-1').read()
    out = []
    for m in re.finditer(r'<model>(.*?)</model>', s, re.S):
        b = m.group(1)
        p = re.search(r'<path>Instruments/([^<]*)\.xml</path>', b)
        if not p:
            continue
        name = p.group(1).split('/')[-1]
        y, z = (float(re.search(r'<%s>([^<]*)</' % k, b).group(1)) for k in ('y-m', 'z-m'))
        sc = re.search(r'<type>scale</type>.*?<x-offset>([^<]*)</x-offset>', b, re.S)
        sc = float(sc.group(1)) if sc else 1.0
        ac = os.path.join(PANELS, 'Instruments', p.group(1).split('/')[0], name.replace('-copilot', '') + '.ac')
        if name in ('pfd', 'mfd'):
            ac = os.path.join(PANELS, 'Instruments', 'zkv1000', 'mfd.ac')
        V = np.vstack(list(ac_objects(ac).values())) * sc
        out.append(dict(name=name, y=y, z=z, y0=y - V[:, 2].max(), y1=y - V[:, 2].min(), z0=z + V[:, 1].min(),
                        z1=z + V[:, 1].max(), w=np.ptp(V[:, 2]), h=np.ptp(V[:, 1])))
    return out


def table(its, model):
    inch = lambda m: m / 0.0254
    rows = ['| No. | Item | Side | Size, in | Size, mm | Centre y, m | Centre z, m | Model: y, z; size, mm |',
            '|---|---|---|---|---|---|---|---|']
    for it in sorted(its, key=lambda it: it['y']):
        now = [m for m in model if MODEL_FOR.get(m['name'].replace('-copilot', '')) == it['num'] and
               (it['side'] == 'centre' or np.sign(m['y']) == np.sign(it['y']))]
        now = '; '.join('%s %+.3f, %+.3f; %.0f x %.0f' % (m['name'], m['y'], m['z'], m['w'] * 1e3, m['h'] * 1e3)
                        for m in now) or '-'
        rows.append('| %s | %s | %s | %.2f x %.2f | %.0f x %.0f | %+.3f | %+.3f | %s |' % (
            it['num'] or '', it['name'], it['side'], inch(it['w']), inch(it['h']), it['w'] * 1e3, it['h'] * 1e3,
            it['y'], it['z'], now))
    rest = [m for m in model if m['name'].replace('-copilot', '') not in MODEL_FOR]
    if rest:
        rows += ['', 'In the model but not in a UH-60A panel: ' +
                 ', '.join('%s (%+.3f, %+.3f)' % (m['name'], m['y'], m['z']) for m in rest) + '.']
    return '\n'.join(rows)


def figure(its, model, path, background=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, ax = plt.subplots(figsize=(16, 5.6))
    if background is not None:
        img, ext = background
        ax.imshow(img, extent=ext, cmap='gray', alpha=0.45, zorder=0)
    for (a, b) in face_edges():
        ax.plot([a[0], b[0]], [a[1], b[1]], color='0.55', lw=0.8, zorder=1)
    for sgn in (-1, 1):
        P = [to_panel(X_CL + sgn * (x - X_CL), y) for x, y in OUTLINE]
        ax.plot([p[0] for p in P], [p[1] for p in P], color='k', lw=1.2, zorder=2)
    for it in its:
        ax.add_patch(Rectangle((it['y0'], it['z0']), it['w'], it['h'], fill=False, ec='tab:red', lw=1.3, zorder=3))
        if it['num']:
            ax.text(it['y'], it['z'], str(it['num']), color='tab:red', ha='center', va='center', fontsize=9,
                    fontweight='bold', zorder=5)
    for m in model:
        ax.add_patch(Rectangle((m['y0'], m['z0']), m['w'], m['h'], fill=False, ec='tab:blue', ls='--', lw=1.0,
                               zorder=4))
        ax.text((m['y0'] + m['y1']) / 2, m['z0'] - 0.004, m['name'], color='tab:blue', ha='center', va='top',
                fontsize=6.5, zorder=5)
    ax.set_aspect('equal'); ax.set_xlim(-0.86, 0.86); ax.set_ylim(-0.31, 0.15)
    ax.set_xlabel('y, m (panel.xml; + right, pilot)'); ax.set_ylabel('z, m')
    ax.set_title('UH-60A instrument panel: TM 1-1520-237-10 fig. 2-9 (red, legend numbers) over the model '
                 '(face plate grey, instruments blue)', fontsize=10)
    fig.tight_layout(); fig.savefig(path, dpi=150); plt.close(fig)


def tm_background(page1_png, page2_png, dpi=300):
    """The two sheets' renders joined into one image in panel.xml coordinates (for checking the boxes)."""
    from PIL import Image
    k = dpi / 72; x0, x1, y0, y1, cut = 55, 1005, 50, 300, 527.0
    W, H = int((x1 - x0) * k), int((y1 - y0) * k)
    out = Image.new('L', (W, H), 255)
    a = Image.open(page1_png).convert('L').crop((int(x0 * k), int(y0 * k), int(cut * k), int(y1 * k)))
    out.paste(a, (0, 0))
    b = Image.open(page2_png).convert('L').crop((int((cut - JOIN[0]) * k), int((y0 - JOIN[1]) * k),
                                                int((x1 - JOIN[0]) * k), int((y1 - JOIN[1]) * k)))
    out.paste(b, (int((cut - x0) * k), 0))
    (ya, za), (yb, zb) = to_panel(x0, y1), to_panel(x1, y0)
    return np.array(out), (ya, yb, za, zb)


if __name__ == '__main__':
    its, model = items(), model_instruments()
    md = table(its, model)
    open(os.path.join(HERE, 'panel-layout.md'), 'w').write(
        '# UH-60A instrument panel layout\n\nMeasured from TM 1-1520-237-10 figure 2-9 sheets 1-2 by '
        '`panel_layout.py`; y, z in panel.xml\'s frame (m), sizes are the outer bezels.\n\n' + md + '\n')
    figure(its, model, os.path.join(HERE, 'panel-layout.png'))
    if len(sys.argv) == 4:      # sheet renders at 300 dpi and an output path: the check figure over the TM art
        figure(its, model, sys.argv[3], tm_background(sys.argv[1], sys.argv[2]))
    print(md)
