"""Builds the UH-60A instruments the old panel lacked; panel_face.py puts them on their TM 1-1520-237-10 figure 2-9
places.

- Vertical speed indicators (the airspeed indicator's case, glass and needle with a vertical speed dial), clocks
  and stabilator position indicators, working.
- The caution/advisory panel with its capsules, which Systems/uh60a-cap.xml lights from what the simulation has
  (table 2-3 of the TM gives each capsule's condition). The failures the simulation doesn't have stay dark. Its
  BRT/DIM-TEST switch tests the capsules and the master warning panel; MASTER CAUTION lights when an amber capsule
  goes on and stays on until pressed (Nasal/caution.nas).
- The panels with nothing to drive yet (mode selectors, auxiliary fuel management panel, radar warning indicator,
  ice rate meter, blade deice and IRCM panels, dimming controls): the TM's face art on a raised panel, with 3-D
  knobs, switches, lamps and buttons. The engine ignition switch works (/controls/engines/ignition).

Sizes and places are the TM figure's (panel_layout.py); everything is in metres in the instrument's own frame
(acmesh.py), its origin on the TM bezel's centre and its back on the face plate.

    python3 instruments.py TM.pdf
"""
import math, os, re, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import acmesh as A
import panel_layout as L
import tmart

INSTR = os.path.join(L.PANELS, 'Instruments')
SYSTEMS = os.path.join(L.PANELS, '..', '..', '..', 'Systems')
NASAL = os.path.join(L.PANELS, '..', '..', '..', 'Nasal')
PT = L.S                                    # m per figure pt
MM = 0.001
FONTS = '/System/Library/Fonts/Supplemental/'
GROUND, LETTER = 38, 215                    # the panels' paint and lettering


def font(size, narrow=False):
    try:
        return ImageFont.truetype(FONTS + ('Arial Narrow Bold.ttf' if narrow else 'Arial Bold.ttf'), int(size))
    except OSError:
        return ImageFont.load_default()


class Tex:
    """A texture: drawing areas mapped onto the instrument, and plain colours (swatches) along its bottom row."""
    def __init__(s, w, h, bg=(0, 0, 0)):
        s.im = Image.new('RGB', (w, h), bg); s.d = ImageDraw.Draw(s.im); s.n = 0

    def swatch(s, rgb):
        x = s.im.width - 16 * (s.n + 1); y = s.im.height - 16; s.n += 1
        s.d.rectangle([x, y, x + 15, y + 15], fill=rgb)
        return A.const((x + 8) / s.im.width, 1 - (y + 8) / s.im.height)

    def area(s, px0, py0, px1, py1, y0, y1, z0, z1):
        """uv for the instrument's rectangle y0..y1, z0..z1 drawn in the pixels px0..px1, py0..py1 (y down)."""
        W, H = s.im.size
        return A.planar(y0, y1, z0, z1, px0 / W, px1 / W, 1 - py1 / H, 1 - py0 / H)

    def save(s, path):
        s.im.save(path, optimize=True)


def item(num, side):
    return next(it for it in L.ITEMS if it[0] == num and it[2] == side)


def tm_size(num, side):
    it = item(num, side)
    return (it[5] - it[4]) * PT, (it[7] - it[6]) * PT


def text(d, xy, s, f, fill, anchor='mm'):
    d.text(xy, s, font=f, fill=fill, anchor=anchor)


def common_xml(objs, lit=(), glass=(), extra=''):
    """The animations every instrument has: no shadows, the glass effect, the instrument lighting on `lit`."""
    names = ''.join('    <object-name>%s</object-name>\n' % o for o in objs)
    out = '<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- written by fdm/cockpit/instruments.py -->\n\n'
    out += '<PropertyList>\n\n  <path>%%s</path>\n\n  <animation>\n    <type>noshadow</type>\n%s  </animation>\n' % names
    for g in glass:
        out += ('\n  <effect>\n    <inherits-from>../../../../Effects/Glass/glass</inherits-from>\n'
                '    <object-name>%s</object-name>\n  </effect>\n' % g)
    if lit:
        out += ('\n  <animation>\n    <type>material</type>\n%s    <condition>\n      <property>controls/electric/'
                'battery-switch</property>\n    </condition>\n    <emission>\n      <factor-prop>controls/lighting/'
                'instruments-norm</factor-prop>\n      <red>   0.50 </red>\n      <green> 0.40 </green>\n'
                '      <blue>  0.30 </blue>\n    </emission>\n  </animation>\n'
                % ''.join('    <object-name>%s</object-name>\n' % o for o in lit))
    return out + extra + '\n</PropertyList>\n'


def rotate_xml(obj, prop, factor=None, table=None, centre=(0, 0, 0), lo=None, hi=None):
    out = '\n  <animation>\n    <type>rotate</type>\n    <object-name>%s</object-name>\n    <property>%s</property>\n' % (obj, prop)
    if table:
        out += '    <interpolation>\n' + ''.join('      <entry><ind> %g </ind><dep> %g </dep></entry>\n' % e for e in table) \
               + '    </interpolation>\n'
    else:
        out += '    <factor> %g </factor>\n' % factor
        if lo is not None:
            out += '    <min-deg> %g </min-deg>\n    <max-deg> %g </max-deg>\n' % (lo, hi)
    out += '    <center>\n      <x-m> %.4f </x-m>\n      <y-m> %.4f </y-m>\n      <z-m> %.4f </z-m>\n    </center>\n' % centre
    return out + '    <axis>\n      <x> -1 </x>\n      <y> 0 </y>\n      <z> 0 </z>\n    </axis>\n  </animation>\n'


def write_instrument(name, objs, xml):
    folder = os.path.join(INSTR, name); os.makedirs(folder, exist_ok=True)
    A.write(os.path.join(folder, name + '.ac'), objs)
    open(os.path.join(folder, name + '.xml'), 'w').write(xml.replace('%s', name + '.ac', 1))


# ---------------------------------------------------------------- vertical speed indicator

# TM dial (labels of figure 2-9): 0 at 9 o'clock, then clockwise up (anticlockwise down) 0.5, 1, 2, 4 and 6 thousand
# ft/min at 21, 43, 85, 130 and 180 deg
VSI_SCALE = [(0, 0), (500, 21), (1000, 43), (2000, 85), (4000, 130), (6000, 180)]


def vsi_sweep(v):
    a = abs(v)
    for (v0, s0), (v1, s1) in zip(VSI_SCALE, VSI_SCALE[1:]):
        if a <= v1:
            return s0 + (s1 - s0) * (a - v0) / (v1 - v0)
    return 180


def ac_uvs(path, obj):
    s = open(path).read()
    block = s[s.index('name "%s"' % obj):]
    block = block[block.index('numsurf'):block.index('kids')]
    return np.array([[float(a), float(b)] for a, b in re.findall(r'^\d+ (\S+) (\S+)$', block, re.M)])


def vsi():
    """The airspeed indicator's mesh, with the TM's vertical speed dial painted into its face."""
    src = os.path.join(INSTR, 'asi'); folder = os.path.join(INSTR, 'vsi'); os.makedirs(folder, exist_ok=True)
    ac = open(os.path.join(src, 'asi.ac')).read().replace('texture "asi.png"', 'texture "vsi.png"')
    open(os.path.join(folder, 'vsi.ac'), 'w').write(ac)
    im = Image.open(os.path.join(src, 'asi.png')).convert('RGB'); W, H = im.size
    uv = ac_uvs(os.path.join(src, 'asi.ac'), 'face')
    cx, cy = uv[:, 0].mean() * W, (1 - uv[:, 1].mean()) * H
    R = (np.hypot(uv[:, 0] * W - cx, (1 - uv[:, 1]) * H - cy)).max()
    d = ImageDraw.Draw(im)
    d.ellipse([cx - R, cy - R, cx + R, cy + R], fill=(8, 8, 8))
    pt = lambda s, r, up: (cx + r * math.cos(math.radians(180 - s)), cy - r * math.sin(math.radians(180 - s)) * up)
    for up in (1, -1):
        ticks = [(v, 0.12 if v % 1000 == 0 else 0.07) for v in range(0, 1001, 100)] + \
                [(v, 0.12 if v % 1000 == 0 else 0.07) for v in range(1250, 2001, 250)] + \
                [(v, 0.12 if v % 1000 == 0 else 0.07) for v in range(2500, 6001, 500)]
        for v, ln in ticks:
            if up < 0 and v == 6000:
                continue
            s = vsi_sweep(v); w = 6 if ln > 0.1 else 3
            d.line([pt(s, R * 0.97, up), pt(s, R * (0.97 - ln), up)], fill=(240, 240, 240), width=w)
        for v, lab in ((1000, '1'), (2000, '2'), (4000, '4')) + (((6000, '6'),) if up > 0 else ()):
            text(d, pt(vsi_sweep(v), R * 0.66, up), lab, font(R * 0.30), (240, 240, 240))
        text(d, pt(vsi_sweep(500), R * 0.72, up), '.5', font(R * 0.13), (240, 240, 240))
        text(d, pt(9, R * 0.66, up), 'UP' if up > 0 else 'DOWN', font(R * 0.09), (240, 240, 240))
    text(d, (cx, cy - R * 0.30), 'VERTICAL SPEED', font(R * 0.09), (240, 240, 240))
    text(d, (cx, cy + R * 0.28), '1000 FT PER', font(R * 0.09), (240, 240, 240))
    text(d, (cx, cy + R * 0.40), 'MIN', font(R * 0.09), (240, 240, 240))
    im.save(os.path.join(folder, 'vsi.png'), optimize=True)
    xml = open(os.path.join(src, 'asi.xml')).read().replace('asi.ac', 'vsi.ac')
    i = xml.index('<animation>\n    <type>rotate</type>'); j = xml.index('</animation>', i) + len('</animation>')
    table = [(-9999, -272)] + [(-v, -90 - vsi_sweep(v)) for v in (6000, 4000, 2000, 1000, 500)] + \
            [(v, -90 + vsi_sweep(v)) for v in (0, 500, 1000, 2000, 4000, 6000)] + [(9999, 92)]
    rot = ('<animation>\n    <type>rotate</type>\n    <object-name>needle</object-name>\n'
           '    <property>instrumentation/vertical-speed-indicator/indicated-speed-fpm</property>\n'
           '    <interpolation>\n' + ''.join('      <entry><ind> %6d </ind><dep> %4.0f </dep></entry>\n' % e for e in table) +
           '    </interpolation>\n    <axis>\n      <x> -1 </x>\n      <y>  0 </y>\n      <z>  0 </z>\n    </axis>\n  </animation>')
    xml = xml[:i] + rot + xml[j:]
    xml = xml.replace('<PropertyList>', '<!-- vertical speed indicator: the airspeed indicator\'s case and needle with the TM '
                      'figure 2-9 dial; written by fdm/cockpit/instruments.py -->\n\n<PropertyList>', 1)
    open(os.path.join(folder, 'vsi.xml'), 'w').write(xml)


# ---------------------------------------------------------------- round and window instruments

def case(o, outline, window, d0, d1, dial_d, uv_case, cy=0.0, cz=0.0):
    """A case: its sides, the bezel round the window at the front and the window's bore down to the dial."""
    A.walls(o, outline, d0, d1, uv_case)
    A.bezel(o, outline, window, d1, uv_case, cy, cz)
    A.walls(o, window, dial_d, d1, uv_case, outward=False)


def hand(o, length, width, tail, d, uv, y0=0.0, z0=0.0, along=(0, 1)):
    """A flat hand from (y0, z0) along `along`, tapering to a point."""
    ay, az = along; ny, nz = -az, ay
    p = lambda a, b: (y0 + a * ay + b * ny, z0 + a * az + b * nz)
    A.flat(o, [p(-tail, -width / 2), p(length * 0.85, -width / 2), p(length, 0), p(length * 0.85, width / 2),
               p(-tail, width / 2)], d, uv)


def clock():
    w, h = tm_size(14, 'pilot')                               # 58 mm square case
    t = Tex(512, 512); R = 230; c = 240                        # dial drawn in a 460 px circle
    d = t.d; d.ellipse([c - R, c - R, c + R, c + R], fill=(8, 8, 8))
    for m in range(60):
        a = math.radians(90 - 6 * m); ln = 26 if m % 5 == 0 else 12
        d.line([(c + R * 0.97 * math.cos(a), c - R * 0.97 * math.sin(a)),
                (c + (R * 0.97 - ln) * math.cos(a), c - (R * 0.97 - ln) * math.sin(a))], fill=(240, 240, 240),
               width=6 if m % 5 == 0 else 3)
        if m % 5 == 0:
            text(d, (c + R * 0.80 * math.cos(a), c - R * 0.80 * math.sin(a)), str(m or 60), font(24), (240, 240, 240))
    for hr in range(1, 13):
        a = math.radians(90 - 30 * hr)
        text(d, (c + R * 0.58 * math.cos(a), c - R * 0.58 * math.sin(a)), str(hr), font(54), (240, 240, 240))
    r = 0.0205                                                 # dial radius
    dial = t.area(c - R, c - R, c + R, c + R, -r, r, -r, r)
    black, white, grey = t.swatch((22, 22, 22)), t.swatch((235, 235, 235)), t.swatch((60, 60, 60))
    body = A.Obj('case', 'clock.png'); face = A.Obj('face', 'clock.png')
    case(body, A.rrect(w, h, 0.005), A.circle(0.022, n=40), 0, 0.014, 0.006, black)
    A.flat(face, A.circle(r, n=40), 0.006, dial)
    for y, z, dia in ((0.0230, 0.0239, 0.0110), (-0.0230, -0.0239, 0.0090)):    # setting and elapsed time knobs
        A.cylinder(body, dia / 2, y, z, 0.014, 0.020, grey, black)
    hh, mh, sh = A.Obj('hour', 'clock.png'), A.Obj('minute', 'clock.png'), A.Obj('second', 'clock.png')
    hand(hh, 0.0120, 0.0030, 0.003, 0.0066, white); hand(mh, 0.0185, 0.0022, 0.003, 0.0070, white)
    hand(sh, 0.0195, 0.0007, 0.005, 0.0074, white)
    A.cylinder(sh, 0.0015, 0, 0, 0.0074, 0.0078, white, white)
    glass = A.Obj('glass', None, A.GLASS); A.flat(glass, A.circle(0.0215, n=40), 0.0130, A.const(0, 0))
    tsec = '/sim/time/local-day-seconds'
    xml = common_xml(['case', 'face', 'hour', 'minute', 'second', 'glass'], lit=['face', 'hour', 'minute', 'second'],
                     glass=['glass'],
                     extra=rotate_xml('hour', tsec, 1 / 120) + rotate_xml('minute', tsec, 0.1) + rotate_xml('second', tsec, 6))
    write_instrument('clock', [body, face, hh, mh, sh, glass], xml.replace('<PropertyList>', '<!-- clock: local time -->\n<PropertyList>', 1))
    t.save(os.path.join(INSTR, 'clock', 'clock.png'))


# stabilator position indicator, measured on figure 2-9: the needle's pivot 10.1 mm left of and 7.1 mm above the
# case's centre; trailing edge 10 deg up at 10 deg above horizontal, 0 horizontal, 40 down at 40 below
STAB_PIVOT = (-0.0101, 0.0071)


def stab():
    w, h = tm_size(9, 'pilot')                                # 65 x 48 mm case
    ww, wh = 26.8 * PT, 21.7 * PT                              # its window, 48 x 39 mm
    t = Tex(512, 512); px = 512 / ww * 0.96                    # px per m in the window drawing
    W0 = 4; d = t.d
    to = lambda y, z: (W0 + (y + ww / 2) * px, W0 + (wh / 2 - z) * px)
    d.rectangle([W0, W0, W0 + ww * px, W0 + wh * px], fill=(8, 8, 8))
    py, pz = STAB_PIVOT
    for deg in range(-10, 41, 5):
        a = math.radians(-deg); r0 = 0.0338 if deg % 10 else 0.0316
        d.line([to(py + r0 * math.cos(a), pz + r0 * math.sin(a)), to(py + 0.0363 * math.cos(a), pz + 0.0363 * math.sin(a))],
               fill=(240, 240, 240), width=6 if deg % 10 == 0 else 3)
        if deg % 10 == 0:
            text(d, to(py + 0.0285 * math.cos(a), pz + 0.0285 * math.sin(a)), str(abs(deg)), font(26), (240, 240, 240))
    a = math.radians(47.7); text(d, to(py + 0.0300 * math.cos(-a), pz + 0.0300 * math.sin(-a)), 'DN', font(24), (240, 240, 240))
    text(d, to(-0.0150, 0.0113), 'STAB', font(24), (240, 240, 240)); text(d, to(-0.0152, 0.0076), 'POS', font(24), (240, 240, 240))
    text(d, to(-0.0152, -0.0122), 'DEG', font(24), (240, 240, 240))
    face_uv = t.area(W0, W0, W0 + ww * px, W0 + wh * px, -ww / 2, ww / 2, -wh / 2, wh / 2)
    black, white = t.swatch((22, 22, 22)), t.swatch((235, 235, 235))
    window = A.rrect(ww, wh, 0.003)
    body = A.Obj('case', 'stab.png'); face = A.Obj('face', 'stab.png'); needle = A.Obj('needle', 'stab.png')
    case(body, A.rrect(w, h, 0.004), window, 0, 0.012, 0.005, black)
    A.flat(face, window, 0.005, face_uv)
    hand(needle, 0.0305, 0.0016, 0.004, 0.0056, white, py, pz, along=(1, 0))
    A.cylinder(needle, 0.0022, py, pz, 0.0056, 0.0060, white, white)
    glass = A.Obj('glass', None, A.GLASS); A.flat(glass, window, 0.0115, A.const(0, 0))
    xml = common_xml(['case', 'face', 'needle', 'glass'], lit=['face', 'needle'], glass=['glass'],
                     extra=rotate_xml('needle', '/fdm/jsbsim/stab/pos-deg', 1, centre=(0, py, pz), lo=-12, hi=45))
    write_instrument('stab', [body, face, needle, glass],
                     xml.replace('<PropertyList>', '<!-- stabilator position indicator: /fdm/jsbsim/stab/pos-deg, trailing '
                                 'edge down + -->\n<PropertyList>', 1))
    t.save(os.path.join(INSTR, 'stab', 'stab.png'))


# ---------------------------------------------------------------- panels with the TM's face art

class Panel:
    """A raised panel with the TM figure's art on its face, and 3-D parts on it (positions in figure pt from the
    bezel's centre, y down, as measured on the figure)."""
    def __init__(s, name, art, num, side, depth, texw=1024, box=None):
        it = item(num, side)
        s.name, s.sheet = name, it[3]
        s.x0, s.x1, s.y0, s.y1 = box or it[4:8]
        s.cx, s.cy = (s.x0 + s.x1) / 2, (s.y0 + s.y1) / 2
        s.w, s.h = (s.x1 - s.x0) * PT, (s.y1 - s.y0) * PT
        s.depth = depth
        pw = texw; ph = int(round(texw * s.h / s.w))
        H = 1 << max(6, math.ceil(math.log2(ph + 20)))
        s.t = Tex(texw, H, (GROUND,) * 3)
        im = tmart.light(art.crop(s.x0, s.x1, s.y0, s.y1, s.sheet, (pw, ph)), GROUND, LETTER)
        s.t.im.paste(im.convert('RGB'), (0, 0))
        s.face_uv = s.t.area(0, 0, pw, ph, -s.w / 2, s.w / 2, -s.h / 2, s.h / 2)
        s.paint = s.t.swatch((30, 30, 30)); s.knob_side = s.t.swatch((20, 20, 20)); s.knob_top = s.t.swatch((48, 48, 48))
        s.metal = s.t.swatch((170, 170, 172)); s.white = s.t.swatch((230, 230, 230))
        s.body = A.Obj('panel', name + '.png'); s.parts = {}
        A.prism(s.body, A.rrect(s.w, s.h, 0.002), 0, depth, s.face_uv, s.paint)
        s.lit = ['panel']

    def at(s, x, y):
        return x * PT, -y * PT

    def obj(s, name):
        if name not in s.parts:
            s.parts[name] = A.Obj(name, s.name + '.png')
        return s.parts[name]

    def knob(s, x, y, dia, height=0.008, name='knobs', pointer=True):
        """A round knob (dia in mm) with a white index line."""
        cy, cz = s.at(x, y); o = s.obj(name); r = dia * MM / 2
        A.cylinder(o, r, cy, cz, s.depth, s.depth + height, s.knob_top, s.knob_side)
        if pointer:
            A.box(o, cy - 0.0006, cy + 0.0006, cz + r * 0.25, cz + r * 0.95, s.depth + height, s.depth + height + 0.0003,
                  s.white, s.white)
        return cy, cz

    def toggle(s, x, y, name='toggles', up=True):
        """A toggle switch: nut and lever, the lever up (or down)."""
        cy, cz = s.at(x, y); o = s.obj(name)
        A.cylinder(o, 0.0035, cy, cz, s.depth, s.depth + 0.003, s.metal, s.metal, n=6)
        tilt = 0.006 if up else -0.006
        lo, hi = s.depth + 0.003, s.depth + 0.014
        A.beam(o, (lo, cy, cz), (hi, cy, cz + tilt), 0.0022, s.metal)
        return cy, cz

    def button(s, x, y, bw, bh, height=0.003, name='buttons'):
        """A push button (bw x bh pt) with the face art on its top."""
        cy, cz = s.at(x, y); o = s.obj(name)
        A.prism(o, A.rrect(bw * PT, bh * PT, 0.0008, cy, cz), s.depth, s.depth + height, s.face_uv, s.knob_side)
        if name not in s.lit:
            s.lit.append(name)

    def lamp(s, x, y, dia, rgb, name='lamps'):
        cy, cz = s.at(x, y); o = s.obj(name)
        A.cylinder(o, dia * MM / 2, cy, cz, s.depth, s.depth + 0.003, s.t.swatch(rgb), s.knob_side)

    def glass(s, x, y, dia):
        cy, cz = s.at(x, y); o = s.parts.setdefault('glass', A.Obj('glass', None, A.GLASS))
        A.flat(o, A.circle(dia * PT / 2, cy, cz, 40), s.depth + 0.0015, A.const(0, 0))

    def write(s, comment, extra=''):
        objs = [s.body] + list(s.parts.values())
        glass = ['glass'] if 'glass' in s.parts else []
        xml = common_xml([o.name for o in objs], lit=s.lit, glass=glass, extra=extra)
        write_instrument(s.name, objs, xml.replace('<PropertyList>', '<!-- %s -->\n<PropertyList>' % comment, 1))
        s.t.save(os.path.join(INSTR, s.name, s.name + '.png'))


def panels(art):
    p = Panel('modesel', art, 11, 'pilot', 0.005)
    for x in (-17.9, -6.4, 5.8, 17.6):
        for y in (-5.9, 6.8):
            p.button(x, y, 8.4, 8.2)
    p.write('VSI/HSI mode selector (the course and steering modes are not simulated)')

    p = Panel('cismode', art, 10, 'pilot', 0.005)
    for x in (-14.4, -0.2, 14.1):
        p.button(x, -0.5, 8.9, 8.7)
    p.write('CIS mode selector (the command instrument system is not simulated)')

    p = Panel('afms', art, 21, 'centre', 0.005)
    for x, y in ((-19.1, 10.4), (-5.3, 10.4), (7.0, 9.8)):
        p.toggle(x, y)
    p.knob(21.0, 10.3, 9)
    p.write('auxiliary fuel management panel (no external tanks are simulated)')

    p = Panel('rwr', art, 20, 'centre', 0.006)
    p.knob(20.2, -17.6, 9); p.lamp(-21.5, -18.1, 5, (70, 20, 15)); p.glass(0, 0, 34.0)
    p.write('radar warning indicator (no threats are simulated)')

    p = Panel('deice', art, 0, 'centre', 0.003)
    p.glass(-0.2, -52.7, 29.6)                                 # ice rate meter
    p.button(-29.6, -52.8, 8.3, 12.0)                          # PRESS TO TEST
    p.toggle(-19.7, -0.7)                                      # blade deice POWER
    p.lamp(0.9, -0.6, 10, (60, 55, 20))                        # TEST IN PROGRESS
    p.knob(20.0, -1.1, 14)                                     # MODE
    p.knob(-15.2, 32.2, 14)                                    # BLADE DE-ICE TEST selector
    p.lamp(9.8, 32.3, 10, (25, 50, 25)); p.lamp(26.3, 32.3, 10, (25, 50, 25))     # PWR MAIN RTR, TAIL RTR
    p.toggle(23.2, 61.4, up=False)                             # IRCM ON/OFF
    p.write('ice rate meter, blade deice control and test panels, IRCM control panel (not simulated)')

    p = Panel('nvgdim', art, 26, 'copilot', 0.0015, texw=512)
    p.knob(-6.7, 0.4, 7, 0.007); p.knob(6.4, 0.2, 7, 0.007)
    p.write('NVG dimming controls')

    p = Panel('radaltdim', art, 27, 'copilot', 0.0015, texw=256)
    p.knob(0, 0, 7, 0.007)
    p.write('radar altimeter dimming control')

    # engine ignition: a key switch, OFF up and ON to the right
    p = Panel('ignition', art, 22, 'centre', 0.004, texw=256)
    o = p.obj('key')
    A.box(o, -0.0035, 0.0035, -0.012, 0.012, p.depth, p.depth + 0.010, p.knob_top, p.knob_side)
    A.box(o, -0.0007, 0.0007, 0.004, 0.0115, p.depth + 0.010, p.depth + 0.0103, p.white, p.white)
    pick = ('\n  <animation>\n    <type>pick</type>\n    <object-name>key</object-name>\n    <action>\n'
            '      <button>0</button>\n      <repeatable>false</repeatable>\n      <binding>\n'
            '        <command>property-toggle</command>\n        <property>/controls/engines/ignition</property>\n'
            '      </binding>\n    </action>\n    <hovered>\n      <binding>\n        <command>set-tooltip</command>\n'
            '        <tooltip-id>uh60-ignition</tooltip-id>\n        <label>Engine ignition: %s</label>\n'
            '        <property>/controls/engines/ignition</property>\n        <mapping>on-off</mapping>\n'
            '      </binding>\n    </hovered>\n  </animation>\n')
    p.write('engine ignition switch: /controls/engines/ignition (the engines light off only with it ON)',
            rotate_xml('key', '/controls/engines/ignition', 90) + pick)


if __name__ == '__main__':
    art = tmart.Art(sys.argv[1], 600)
    vsi(); clock(); stab(); panels(art)
    import cap
    cap.build(art)
