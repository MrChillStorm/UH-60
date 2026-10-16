"""The UH-60A's round flight instruments, after TM 1-1520-237-10 figure 2-9 (and 3.29, figure 3-34), built new so that
their faces are sharp: dials drawn at 1024 px (about 13 px/mm), supersampled; counters are text nodes.

- airspeed indicator (Instruments/asi60): 20-250 kt;
- vertical speed indicator (vsi60): 0 to 6000 ft/min up and down, the TM's scale;
- barometric altimeter (alt60): counter-pointer, 1000 ft a turn, 1000 FT counter with its 10,000 ft flag, IN. HG counter,
  baro knob;
- radar altimeter AN/APN-209 (apn209): 0-200 ft linear at 10 ft per mark, 200-1500 ft at 100, dial mask, OFF flag, L and
  H bugs with LO and HI lights, four-digit readout, LO SET (with OFF) and HI SET (PUSH TO TEST) knobs; each pilot's own
  (Nasal/avionics.nas works it);
- the VSI's attitude sphere (Instruments/ai/ai.png): redrawn at 4x, its case black, the slip indicator's tube on the
  ball's path; the sphere's and the slip ball's meshes, smooth (ball);
- the pilot's and central display units' faces (prop-rpm.png 4x, combimeter.png 2x): redrawn in their old layout,
  the labels re-typeset and the digits in a seven-segment font, the tapes' band copied pixel for pixel so that
  their calibration holds (the originals are in src/).

    python3 gauges.py
"""
import math, os, re
import numpy as np
from PIL import Image, ImageDraw

import acmesh as A
import instruments as I

SS = 2                                          # supersampling of the dials
FG_FONT_DIGITS = 'DSEG/DSEG7/Classic/DSEG7Classic-Bold.ttf'
WHITE, BLACK = (238, 238, 238), (10, 10, 10)


class Dial:
    """A dial drawn supersampled: positions in metres from its centre, angles clockwise from 12 o'clock."""
    def __init__(s, px, radius, bg=(12, 12, 12)):
        s.px, s.R = px, radius
        s.N = px * SS
        s.im = Image.new('RGB', (s.N, s.N), (0, 0, 0))
        s.d = ImageDraw.Draw(s.im)
        s.k = s.N / (2 * radius)
        s.d.ellipse([0, 0, s.N - 1, s.N - 1], fill=bg)

    def p(s, r, a):
        a = math.radians(a)
        return (s.N / 2 + r * math.sin(a) * s.k, s.N / 2 - r * math.cos(a) * s.k)

    def xy(s, y, z):
        return (s.N / 2 + y * s.k, s.N / 2 - z * s.k)

    def tick(s, a, r0, r1, w, fill=WHITE):
        s.d.line([s.p(r0, a), s.p(r1, a)], fill=fill, width=max(1, int(w * s.k)))

    def text(s, y, z, txt, size, fill=WHITE, narrow=False, anchor='mm'):
        s.d.text(s.xy(y, z), txt, font=I.font(size * s.k, narrow), fill=fill, anchor=anchor)

    def text_at(s, r, a, txt, size, fill=WHITE):
        x, y = s.p(r, a); s.d.text((x, y), txt, font=I.font(size * s.k), fill=fill, anchor='mm')

    def rect(s, y0, y1, z0, z1, fill=None, outline=None, w=0.0004):
        s.d.rectangle([s.xy(y0, z1), s.xy(y1, z0)], fill=fill, outline=outline, width=max(1, int(w * s.k)) if outline else 0)

    def done(s):
        return s.im.resize((s.px, s.px), Image.LANCZOS)


class Gauge:
    """A square-cased round gauge: dial drawn by the caller; needles, bugs, windows and knobs added; the parts' XML."""
    def __init__(s, name, case_w, case_h, dial_r, case_c=(0.0, 0.0), dial_d=0.008, front=0.020):
        s.name, s.R, s.dd, s.front = name, dial_r, dial_d, front
        s.t = I.Tex(2048, 1024)
        s.black, s.white, s.dark = s.t.swatch((16, 16, 16)), s.t.swatch(WHITE), s.t.swatch((4, 4, 4))
        s.grey = s.t.swatch((60, 60, 60))
        s.objs = {}; s.xml = ''; s.lit = []; s.free = [1040, 8, 0]
        body = s.obj('case')
        cy, cz = case_c
        I.case(body, A.rrect(case_w, case_h, 0.006, cy, cz), A.circle(dial_r + 0.0006, n=64), 0, front, dial_d, s.black)
        s.case_c, s.case_w, s.case_h = case_c, case_w, case_h

    def obj(s, name, glass=False):
        if name not in s.objs:
            s.objs[name] = A.Obj(name, None if glass else s.name + '.png', A.GLASS if glass else 0)
        return s.objs[name]

    def alloc(s, w, h):
        x, y, rh = s.free
        if x + w > 2040:
            x, y, rh = 1040, y + rh + 8, 0
        s.free = [x + w + 8, y, max(rh, h)]
        return x, y, x + w, y + h

    def dial(s, img):
        s.t.im.paste(img, (0, 0))
        A.flat(s.obj('dial'), A.circle(s.R + 0.0006, n=72), s.dd, s.t.area(0, 0, img.width, img.height, -s.R, s.R, -s.R, s.R))
        s.lit.append('dial')

    def patch(s, draw, w_px, h_px, y0, y1, z0, z1, d, name, lit=True):
        """A textured quad: `draw(ImageDraw, box)` paints it into the texture."""
        x0, py0, x1, py1 = s.alloc(w_px, h_px)
        sub = Image.new('RGB', (w_px, h_px), (0, 0, 0))                     # drawn on its own, so it can't spill
        draw(ImageDraw.Draw(sub), (0, 0, w_px, h_px))
        s.t.im.paste(sub, (x0, py0))
        A.flat(s.obj(name), [(y0, z0), (y1, z0), (y1, z1), (y0, z1)], d, s.t.area(x0, py0, x1, py1, y0, y1, z0, z1))
        if lit and name not in s.lit:
            s.lit.append(name)

    def needle(s, name, length, width, tail, d, colour=None, hub=0.0030):
        """A needle pointing up at zero, with its hub."""
        o = s.obj(name); uv = colour or s.white
        A.flat(o, [(-width / 2, -tail), (width / 2, -tail), (width / 2, length * 0.88), (0, length), (-width / 2, length * 0.88)], d, uv)
        A.cylinder(o, hub, 0, 0, d, d + 0.0006, s.black, s.black, n=20)
        s.lit.append(name)

    def knob(s, name, y, z, dia, h=0.010):
        o = s.obj(name)
        A.cylinder(o, dia / 2, y, z, s.front, s.front + h, s.grey, s.black, n=28)
        A.box(o, y - 0.0006, y + 0.0006, z + dia * 0.12, z + dia * 0.46, s.front + h, s.front + h + 0.0003, s.white, s.white)

    def glass(s):
        A.flat(s.obj('glass', True), A.circle(s.R + 0.0006, n=64), s.front - 0.0012, A.const(0, 0))

    def write(s, comment, variants=None, light='non-flt'):
        objs = list(s.objs.values())
        x = I.common_xml([o.name for o in objs], lit=s.lit, glass=['glass'], extra=s.xml, light=light,
                         bezel=((s.dd, 0.0, 0.0), (s.R, s.R), True, s.front - 0.0012))
        x = x.replace('<PropertyList>', '<!-- %s; written by fdm/cockpit/gauges.py -->\n<PropertyList>' % comment, 1)
        I.write_instrument(s.name, objs, x)
        for fname, subs in variants or ():
            v = open(os.path.join(I.INSTR, s.name, s.name + '.xml')).read()
            for a, b in subs.items():
                v = v.replace(a, b)
            open(os.path.join(I.INSTR, s.name, fname), 'w').write(v)
        s.t.save(os.path.join(I.INSTR, s.name, s.name + '.png'))


def rot(obj, prop, table=None, factor=None, centre=(0, 0, 0), axis=(-1, 0, 0)):
    out = '\n  <animation>\n    <type>rotate</type>\n    <object-name>%s</object-name>\n    <property>%s</property>\n' % (obj, prop)
    if table:
        out += '    <interpolation>\n' + ''.join('      <entry><ind> %g </ind><dep> %g </dep></entry>\n' % e for e in table) + '    </interpolation>\n'
    else:
        out += '    <factor> %g </factor>\n' % factor
    out += ('    <center>\n      <x-m> %.4f </x-m>\n      <y-m> %.4f </y-m>\n      <z-m> %.4f </z-m>\n    </center>\n'
            '    <axis>\n      <x> %g </x>\n      <y> %g </y>\n      <z> %g </z>\n    </axis>\n  </animation>\n' % (tuple(centre) + tuple(axis)))
    return out


def text_node(name, prop, fmt, size, y, z, d, font=FG_FONT_DIGITS, scale=None, truncate=False, width=0.03):
    sc = '    <scale type="double">%g</scale>\n' % scale if scale else ''
    return ('\n  <text>\n    <name>%s</name>\n    <type type="string">number-value</type>\n    <property>%s</property>\n%s'
            '    <format type="string">%s</format>\n    <truncate type="bool">%s</truncate>\n    <font>%s</font>\n'
            '    <character-size type="double">%g</character-size>\n    <character-aspect-ratio type="double">0.75</character-aspect-ratio>\n'
            '    <axis-alignment>yz-plane</axis-alignment>\n    <max-height>0.03</max-height>\n    <max-width>%g</max-width>\n'
            '    <alignment>center-center</alignment>\n    <offsets>\n      <x-m> %.4f </x-m>\n      <y-m> %.4f </y-m>\n'
            '      <z-m> %.4f </z-m>\n    </offsets>\n  </text>\n' % (name, prop, sc, fmt, 'true' if truncate else 'false', font, size,
                                                                      width, d, y, z))


def emissive(objs, rgb, cond=None, factor=None):
    c = '    <condition>\n      %s\n    </condition>\n' % cond if cond else ''
    f = '      <factor-prop>%s</factor-prop>\n' % factor if factor else ''
    return ('\n  <animation>\n    <type>material</type>\n%s%s    <emission>\n%s      <red> %g </red>\n      <green> %g </green>\n'
            '      <blue> %g </blue>\n    </emission>\n  </animation>\n' % ((''.join('    <object-name>%s</object-name>\n' % o for o in objs), c, f) + tuple(rgb)))


PWR = '<property>/controls/electric/battery-switch</property>'
CASE = 0.0825                                   # 3ATI case, 3.25 in
DIAL_R = 0.0375


# ---------------------------------------------------------------- airspeed

ASI_TABLE = [(0, 0), (10, 10), (50, 68), (100, 139), (150, 209), (200, 282), (250, 354)]


def interp(table, x):
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return table[-1][1]


def asi():
    g = Gauge('asi60', CASE, CASE, DIAL_R)
    D = Dial(1024, DIAL_R)
    for kt in range(20, 251, 5):
        a = interp(ASI_TABLE, kt); big = kt % 10 == 0
        D.tick(a, DIAL_R * 0.97, DIAL_R * (0.80 if big else 0.87), 0.0009 if big else 0.0006)
    for kt in (50, 100, 150, 200, 250):
        D.text_at(DIAL_R * 0.62, interp(ASI_TABLE, kt), str(kt), 0.0072)
    D.text_at(DIAL_R * 0.68, interp(ASI_TABLE, 20), '20', 0.0045)
    D.text(0, -0.006, 'KNOTS', 0.0045)
    g.dial(D.done())
    g.needle('needle', DIAL_R * 0.90, 0.0022, 0.009, s_d(g, 1))
    g.glass()
    g.xml += rot('needle', 'instrumentation/airspeed-indicator/indicated-speed-kt', [(-999, 0)] + ASI_TABLE + [(400, 359)])
    g.write('airspeed indicator, TM 1-1520-237-10 figure 2-9', variants=[('asi60-copilot.xml', {'lights/plt-flt': 'lights/cplt-flt'})], light='plt-flt')


def s_d(g, k):
    return g.dd + 0.0008 * k


# ---------------------------------------------------------------- vertical speed

def vsi():
    g = Gauge('vsi60', CASE, CASE, DIAL_R)
    D = Dial(1024, DIAL_R)
    sweep = I.vsi_sweep
    for up in (1, -1):
        ticks = [(v, v % 1000 == 0) for v in range(0, 1001, 100)] + [(v, v % 1000 == 0) for v in range(1250, 2001, 250)] + \
                [(v, v % 1000 == 0) for v in range(2500, 6001, 500)]
        for v, big in ticks:
            if up < 0 and v == 6000:
                continue
            a = -90 + up * sweep(v) if up > 0 else -90 - sweep(v)
            D.tick(a, DIAL_R * 0.97, DIAL_R * (0.80 if big else 0.87), 0.0009 if big else 0.0006)
        for v, lab in ((1000, '1'), (2000, '2'), (4000, '4')) + (((6000, '6'),) if up > 0 else ()):
            a = -90 + sweep(v) if up > 0 else -90 - sweep(v)
            D.text_at(DIAL_R * 0.64, a, lab, 0.0085)
        a = -90 + sweep(500) if up > 0 else -90 - sweep(500)
        D.text_at(DIAL_R * 0.70, a, '.5', 0.0042)
        D.text(-DIAL_R * 0.62, 0.0045 * up, 'UP' if up > 0 else 'DOWN', 0.0032)
    D.text(0, 0.010, 'VERTICAL SPEED', 0.0034)
    D.text(0, -0.010, '1000 FT PER', 0.0034); D.text(0, -0.0145, 'MIN', 0.0034)
    g.dial(D.done())
    g.needle('needle', DIAL_R * 0.90, 0.0022, 0.009, s_d(g, 1))
    g.glass()
    table = [(-9999, -272)] + [(-v, -90 - sweep(v)) for v in (6000, 4000, 2000, 1000, 500)] + \
            [(v, -90 + sweep(v)) for v in (0, 500, 1000, 2000, 4000, 6000)] + [(9999, 92)]
    g.xml += rot('needle', 'instrumentation/vertical-speed-indicator/indicated-speed-fpm', table)
    g.write('vertical speed indicator, TM 1-1520-237-10 figure 2-9', variants=[('vsi60-copilot.xml', {'lights/plt-flt': 'lights/cplt-flt'})], light='plt-flt')


# ---------------------------------------------------------------- altimeter

def alt():
    g = Gauge('alt60', CASE, CASE, DIAL_R)
    D = Dial(1024, DIAL_R)
    for k in range(50):
        a = k * 7.2; big = k % 5 == 0
        D.tick(a, DIAL_R * 0.97, DIAL_R * (0.80 if big else 0.88), 0.0010 if big else 0.0006)
    for n in range(10):
        D.text_at(DIAL_R * 0.66, n * 36, str(n), 0.0090)
    D.text(0.004, 0.012, 'ALT', 0.0050)
    # windows: 1000 FT (its 10,000 ft flag and digit), IN. HG
    D.rect(-0.0175, -0.0025, -0.0035, 0.0050, fill=(0, 0, 0), outline=WHITE)
    D.text(-0.0100, -0.0068, '1000 FT', 0.0026)
    D.rect(0.0010, 0.0170, -0.0140, -0.0078, fill=(0, 0, 0), outline=WHITE)
    D.text(0.0090, -0.0057, 'IN. HG', 0.0026)
    g.dial(D.done())

    def stripes(d, box):
        x0, y0, x1, y1 = box; w, h = x1 - x0, y1 - y0
        im = Image.new('RGB', (w, h), (235, 235, 235)); dd = ImageDraw.Draw(im)
        for k in range(-h, w, 16):
            dd.polygon([(k, h), (k + 8, h), (k + 8 + h, 0), (k + h, 0)], fill=(20, 20, 20))
        d._image.paste(im, (x0, y0))
    g.patch(stripes, 120, 120, -0.0170, -0.0105, -0.0030, 0.0045, g.dd + 0.0004, 'flag10k')
    g.needle('needle', DIAL_R * 0.88, 0.0040, 0.010, s_d(g, 2), hub=0.0045)
    g.knob('knob', -0.0325, -0.0325, 0.012)
    g.glass()
    g.xml += rot('needle', 'instrumentation/altimeter/indicated-altitude-ft', factor=0.36)
    g.xml += I.select_xml(['flag10k'], '<less-than><property>instrumentation/altimeter/indicated-altitude-ft</property><value>10000</value></less-than>')
    g.xml += text_node('thousands', 'instrumentation/altimeter/indicated-altitude-ft', '%2.0f', 0.0062, -0.0100, 0.0007, g.dd + 0.0006,
                       font='LiberationFonts/LiberationSans-Bold.ttf', scale=0.001, truncate=True)
    g.xml += text_node('inhg', 'instrumentation/altimeter/setting-inhg', '%4.0f', 0.0050, 0.0090, -0.0109, g.dd + 0.0006,
                       font='LiberationFonts/LiberationSans-Bold.ttf', scale=100)
    g.xml += I.light_xml(['thousands', 'inhg'], 'plt-flt')                  # its counters, lit with the dial
    step = 'instrumentation/altimeter/setting-inhg'
    acts = ''
    for button, sgn in ((0, 1), (2, -1), (3, 1), (4, -1)):
        for mods, k in (('<not><property>devices/status/keyboard/shift</property></not>', 0.01),
                        ('<property>devices/status/keyboard/shift</property>', 0.1)):
            acts += ('    <action>\n      <button>%d</button>\n      <repeatable>true</repeatable>\n      <binding>\n        <condition>%s</condition>\n'
                     '        <command>property-adjust</command>\n        <property>%s</property>\n        <step>%g</step>\n'
                     '        <min>28.10</min>\n        <max>31.00</max>\n      </binding>\n    </action>\n' % (button, mods, step, sgn * k))
    g.xml += ('\n  <animation>\n    <type>pick</type>\n    <object-name>knob</object-name>\n%s    <hovered>\n      <binding>\n'
              '        <command>set-tooltip</command>\n        <tooltip-id>alt-baro</tooltip-id>\n        <label>Baro setting: %%.2f inHg</label>\n'
              '        <property>%s</property>\n      </binding>\n    </hovered>\n  </animation>\n' % (acts, step))
    g.xml += rot('knob', step, factor=400, centre=(0, -0.0325, -0.0325))
    g.write('barometric altimeter (counter-pointer), TM 1-1520-237-10 figure 2-9', variants=[('alt60-copilot.xml', {'lights/plt-flt': 'lights/cplt-flt'})], light='plt-flt')


# ---------------------------------------------------------------- radar altimeter AN/APN-209

def ra_angle(ft):
    """Pointer angle, clockwise from 12 o'clock: 0 ft at the bottom, 200 ft at the top, 1500 ft at 3 o'clock."""
    ft = max(0.0, min(1500.0, ft))
    return -180 + 0.9 * ft if ft <= 200 else (ft - 200) * 90 / 1300


def apn209():
    w, h = I.tm_size(1, 'pilot')                         # 86 mm
    R = 0.0315
    g = Gauge('apn209', w, h, R, case_c=(0.0, -0.003))
    D = Dial(1024, R)
    for ft in range(0, 201, 10):
        D.tick(ra_angle(ft), R * 0.97, R * (0.80 if ft % 50 == 0 else 0.87), 0.0009 if ft % 50 == 0 else 0.0006)
    for ft in range(300, 1501, 100):
        D.tick(ra_angle(ft), R * 0.97, R * (0.80 if ft % 500 == 0 else 0.87), 0.0009 if ft % 500 == 0 else 0.0006)
    for ft, lab, size in ((0, '0', 0.0072), (100, '1', 0.0072), (200, '2', 0.0072), (500, '5', 0.0050), (1000, '10', 0.0050),
                          (1500, '15', 0.0050)):
        D.text_at(R * 0.66, ra_angle(ft), lab, size)
    D.text(0.0, 0.0125, 'FT X 100', 0.0032)
    D.text(-0.0060, -0.0040, 'ABS', 0.0030); D.text(0.0075, -0.0040, 'ALT', 0.0030)
    D.rect(-0.0130, 0.0140, -0.0145, -0.0060, fill=(0, 0, 0), outline=(90, 90, 90))
    D.text(-0.0090, -0.0185, 'FEET', 0.0030)
    g.dial(D.done())
    # case face: LO, OFF, HI and PUSH TO TEST by the knobs
    for k, (lab, y, z, wd) in enumerate((('LO', -0.0385, -0.0165, 0.007), ('OFF', -0.0215, -0.0405, 0.009), ('HI', 0.0400, -0.0165, 0.007),
                                         ('PUSH TO TEST', 0.0200, -0.0410, 0.026))):
        def label(d, box, lab=lab):
            d.rectangle(box, fill=(16, 16, 16)); d.text(((box[0] + box[2]) / 2, (box[1] + box[3]) / 2), lab, font=I.font(34), fill=(215, 215, 215), anchor='mm')
        g.patch(label, int(wd * 15000), 56, y - wd / 2, y + wd / 2, z - 0.0023, z + 0.0023, g.front + 0.0002, 'label%d' % k)
    # dial mask: the outer ring between 1500 ft and 0 (the pointer goes out of sight there), the OFF flag on it
    o = g.obj('mask')
    for a0 in range(98, 172, 4):
        a1 = a0 + 4
        p = lambda r, a: (r * math.sin(math.radians(a)), r * math.cos(math.radians(a)))
        A.flat(o, [p(0.022, a0), p(R * 1.01, a0), p(R * 1.01, a1), p(0.022, a1)], g.dd + 0.0030, g.black)
    def offflag(d, box):
        x0, y0, x1, y1 = box; d.rectangle(box, fill=(225, 225, 225))
        for k in range(x0 - (y1 - y0), x1, 20):
            d.polygon([(k, y1), (k + 10, y1), (k + 10 + (y1 - y0), y0), (k + (y1 - y0), y0)], fill=(200, 30, 30))
        d.rectangle([x0 + 18, y0 + 12, x1 - 18, y1 - 12], fill=(225, 225, 225))
        d.text(((x0 + x1) / 2, (y0 + y1) / 2), 'OFF', font=I.font(46), fill=(15, 15, 15), anchor='mm')
    g.patch(offflag, 220, 110, 0.0130, 0.0250, -0.0250, -0.0190, g.dd + 0.0036, 'off-flag')
    # pointer (under the mask), L and H bugs on the rim, LO and HI lights
    g.needle('pointer', R * 0.92, 0.0030, 0.006, g.dd + 0.0010, hub=0.0040)
    for name, lab in (('lo-bug', 'L'), ('hi-bug', 'H')):
        def bug(d, box, lab=lab):
            x0, y0, x1, y1 = box; d.rectangle(box, fill=(225, 225, 225))
            d.text(((x0 + x1) / 2, (y0 + y1) / 2), lab, font=I.font(70), fill=(10, 10, 10), anchor='mm')
        g.patch(bug, 100, 120, -0.0022, 0.0022, R * 0.94, R * 1.10, g.dd + 0.0042, name)
    for name, lab, y, z in (('lo-light', 'LO', -0.0095, 0.0010), ('hi-light', 'HI', -0.0095, 0.0205)):
        def lamp(d, box, lab=lab, on=False):
            x0, y0, x1, y1 = box                                      # lit, on black: in HDR the whole texture emits
            d.rectangle(box, fill=(0, 0, 0) if on else (14, 14, 14), outline=(0, 0, 0) if on else (120, 120, 120), width=6)
            d.text(((x0 + x1) / 2, (y0 + y1) / 2), lab, font=I.font(70), fill=(255, 150, 20) if on else (70, 55, 40), anchor='mm')
        g.patch(lamp, 200, 110, y - 0.0040, y + 0.0040, z - 0.0022, z + 0.0022, g.dd + 0.0004, name + '-off', lit=False)
        g.patch(lambda d, box, lab=lab: lamp(d, box, lab, True), 200, 110, y - 0.0040, y + 0.0040, z - 0.0022, z + 0.0022,
                g.dd + 0.0006, name, lit=False)
    g.knob('lo-knob', -0.0310, -0.0280, 0.013)
    g.knob('hi-knob', 0.0330, -0.0280, 0.013)
    g.glass()
    RA = '/instrumentation/uh60/radalt[0]/'
    table = [(-50, -180)] + [(ft, ra_angle(ft)) for ft in (0, 200, 1500)]
    g.xml += rot('pointer', RA + 'pointer-ft', table)
    g.xml += I.select_xml(['pointer'], '<property>%spointer-vis</property>' % RA)
    g.xml += rot('lo-bug', RA + 'lo-ft', [(-50, -180)] + [(ft, ra_angle(ft)) for ft in (0, 200, 1500)])
    g.xml += rot('hi-bug', RA + 'hi-ft', [(0, -180)] + [(ft, ra_angle(ft)) for ft in (200, 1500)])
    g.xml += I.select_xml(['off-flag'], '<property>%soff-flag</property>' % RA)
    g.xml += I.select_xml(['lo-light'], '<property>%slo-light</property>' % RA)
    g.xml += I.select_xml(['hi-light'], '<property>%shi-light</property>' % RA)
    g.xml += emissive(['lo-light', 'hi-light'], (1, 1, 1), factor=I.LIGHTS + 'radalt-lamps')
    g.xml += I.lit_xml(['lo-light', 'hi-light'], I.HDR + 'radalt-lamps')
    g.xml += text_node('digits', RA + 'digits', '%4.0f', 0.0068, 0.0010, -0.0102, g.dd + 0.0006, width=0.028)
    g.xml += I.select_xml(['digits'], '<property>%sdigits-vis</property>' % RA)
    g.xml += emissive(['digits'], (1.0, 0.45, 0.1), factor=I.LIGHTS + 'radalt-lamps')       # HDR: a text node, unlit
    g.xml += rot('lo-knob', RA + 'lo-ft', factor=0.5, centre=(0, -0.0310, -0.0280))
    g.xml += rot('hi-knob', RA + 'hi-ft', factor=0.5, centre=(0, 0.0330, -0.0280))
    g.xml += I.pick_xml('lo-knob', [(0, 'avionics.radalt_bug(0, "lo", 1)'), (2, 'avionics.radalt_bug(0, "lo", -1)'),
                                    (3, 'avionics.radalt_bug(0, "lo", 1)'), (4, 'avionics.radalt_bug(0, "lo", -1)')],
                        'LO SET: low altitude bug (counterclockwise past 0: OFF)')
    g.xml += I.pick_xml('hi-knob', [(0, 'avionics.radalt_bug(0, "hi", 1)'), (2, 'avionics.radalt_bug(0, "hi", -1)'),
                                    (3, 'avionics.radalt_bug(0, "hi", 1)'), (4, 'avionics.radalt_bug(0, "hi", -1)')],
                        'HI SET: high altitude bug (middle click: PUSH TO TEST)')
    g.xml += I.pick_xml('hi-knob', [(1, 'setprop("%stest", 1);' % RA)], 'HI SET: PUSH TO TEST', release='setprop("%stest", 0);' % RA)
    g.write('radar altimeter AN/APN-209, TM 1-1520-237-10 3.29 and figure 3-34; Nasal/avionics.nas',
            variants=[('apn209-copilot.xml', {'radalt[0]': 'radalt[1]', 'avionics.radalt_bug(0,': 'avionics.radalt_bug(1,'})],
            light='radalt[0]')


# ---------------------------------------------------------------- attitude sphere

def sphere():
    """The VSI's sphere at 4x: the old texture's layout (horizon at v 0.751, 0.0027 of v per degree of pitch), a clean
    ladder: marks every 5 deg, 30 and 60 numbered, CLIMB and DIVE; the rest of the texture upscaled."""
    path = os.path.join(I.INSTR, 'ai', 'ai.png')
    old = Image.open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src', 'ai-512.png')).convert('RGB')   # the original
    # the case's face and the bore round the sphere black, as a VSI's are (they were the panel's grey, and the bore's
    # flat sides cutting the sphere looked like the panel showing through it); its screws stay
    a = np.asarray(old).astype(int); case = a[0:400, 0:250]
    grey = (np.abs(case.mean(2) - 50) < 20) & (case.max(2) - case.min(2) < 12)
    yy, xx = np.mgrid[0:400, 0:250]
    grey[252:326, 0:63] = True                                      # the screw's square, but its head
    grey &= (yy - 288.5) ** 2 + (xx - 29.5) ** 2 > 33 ** 2
    case[grey] = 16
    old = Image.fromarray(a.astype(np.uint8))
    big = old.resize((2048, 2048), Image.LANCZOS)
    N = 1024 * SS
    im = Image.new('RGB', (N, N)); d = ImageDraw.Draw(im)
    v_of = lambda pitch: 0.751 + 0.0027 * pitch                          # texture v of a pitch angle
    row = lambda pitch: (1 - (v_of(pitch) - 0.5) / 0.5) * N              # pixel row in the region (u, v 0.5..1)
    cx = N / 2 + (0.75 - 0.75) * N
    sky, ground = (228, 228, 224), (150, 92, 20)
    d.rectangle([0, 0, N, row(0)], fill=sky); d.rectangle([0, row(0), N, N], fill=ground)
    d.line([(0, row(0)), (N, row(0))], fill=(250, 250, 250), width=6 * SS)
    for p in range(-85, 90, 5):
        if p == 0 or abs(p) in (40, 45):                                  # CLIMB and DIVE there
            continue
        col = (20, 20, 20) if p > 0 else (240, 240, 240)
        half = (0.030 if p % 10 == 0 else 0.016) * N
        if p % 30 == 0:
            half = 0.008 * N
        d.line([(cx - half, row(p)), (cx + half, row(p))], fill=col, width=4 * SS)
        if p % 30 == 0:
            f = I.font(0.030 * N)
            d.text((cx - 0.030 * N, row(p)), str(abs(p))[0], font=f, fill=col, anchor='mm')
            d.text((cx + 0.030 * N, row(p)), str(abs(p))[1], font=f, fill=col, anchor='mm')
    f = I.font(0.026 * N)
    d.text((cx, row(42)), 'CLIMB', font=f, fill=(20, 20, 20), anchor='mm')
    d.text((cx, row(-42)), 'DIVE', font=f, fill=(240, 240, 240), anchor='mm')
    big.paste(im.resize((1024, 1024), Image.LANCZOS), (1024, 0))
    slip_face(big)
    ring_lower(big)
    big.save(path, optimize=True)


RING_U, RING_V = (3.4950, 0.7558), (3.5378, 0.2665)             # the ring round the sphere (ai-ring), mapped flat


def ring_lower(big):
    """The ring's lower half plain black: its bank marks and centre line there (the UH-60A's bank scale and index are
    at the top, TM figure 3-30); no brown smudges (one on its left, one under the sky pointer); its 90 degree marks,
    0 degree line and the glide slope bar stay."""
    a = np.asarray(big).copy()
    yy, xx = np.mgrid[0:2048, 0:2048]
    y = (xx / 2048 - RING_U[1]) / RING_U[0]; z = ((1 - yy / 2048) - RING_V[1]) / RING_V[0]
    m = (xx >= 0.530 * 2048) & (xx <= 0.976 * 2048) & (yy >= (1 - 0.486) * 2048) & (yy <= (1 - 0.042) * 2048)
    ring = m & (np.hypot(y, z) >= 0.0445)
    c = a.astype(int)
    brown = (c[..., 0] - c[..., 2] > 6) & (c[..., 0] > c[..., 1]) & (c[..., 0] > 10)     # smudges (the ring is neutral)
    a[ring & (((z < -0.004) & (y < 0.051)) | brown)] = 0
    big.paste(Image.fromarray(a))


# the slip indicator: the ball's path (an arc about the sphere's axis, the ball's roll travel), the face (object
# ring-slip, mapped flat: u, v of y, z) and its region in the texture
SLIP_R, SLIP_BALL, SLIP_TRAVEL = 0.0562, 0.0029, 13.0
SLIP_U, SLIP_V = (4.0859, -0.0292, 0.2551), (5.0624, 0.3483)
SLIP_BOX = (0, 1700, 1032, 1966)                                  # px in the 2048 texture


def slip_face(big):
    """The face below the sphere: black, the inclinometer's tube (cream glass on the ball's path) and its two
    reference wires; the old one had a smudged tube above the ball's path and a khaki block under it."""
    x0, y0, x1, y1 = SLIP_BOX; q = SS * 2
    im = Image.new('RGB', ((x1 - x0) * q, (y1 - y0) * q), (16, 16, 16)); d = ImageDraw.Draw(im)
    def px(y, z):
        u = SLIP_U[0] * y + SLIP_U[1] * z + SLIP_U[2]; v = SLIP_V[0] * z + SLIP_V[1]
        return ((u * 2048 - x0) * q, ((1 - v) * 2048 - y0) * q)
    arc = lambda r, a: px(r * math.sin(math.radians(a)), -r * math.cos(math.radians(a)))
    half = SLIP_TRAVEL + math.degrees(SLIP_BALL / SLIP_R) + 0.5         # the tube's ends, degrees off centre
    w = SLIP_BALL + 0.0007                                              # its half width
    edge = [arc(SLIP_R - w, a) for a in np.linspace(-half, half, 60)] + [arc(SLIP_R + w, a) for a in np.linspace(half, -half, 60)]
    for a in (-half, half):                                             # round ends
        c = arc(SLIP_R, a); r = (px(0, -w)[1] - px(0, 0)[1])
        d.ellipse([c[0] - abs(r), c[1] - abs(r), c[0] + abs(r), c[1] + abs(r)], fill=(70, 70, 70))
    d.polygon(edge, fill=(70, 70, 70))
    inner = [arc(SLIP_R - w + 0.0003, a) for a in np.linspace(-half, half, 60)] + \
            [arc(SLIP_R + w - 0.0003, a) for a in np.linspace(half, -half, 60)]
    for a in (-half, half):
        c = arc(SLIP_R, a); r = abs(px(0, -w + 0.0003)[1] - px(0, 0)[1])
        d.ellipse([c[0] - r, c[1] - r, c[0] + r, c[1] + r], fill=(226, 222, 204))
    d.polygon(inner, fill=(226, 222, 204))
    wa = math.degrees((SLIP_BALL + 0.0004) / SLIP_R)                     # the wires, just clear of the centred ball
    for a in (-wa, wa):
        d.line([arc(SLIP_R - w, a), arc(SLIP_R + w, a)], fill=(10, 10, 10), width=int(0.0005 * SLIP_U[0] * 2048 * q))
    big.paste(im.resize((x1 - x0, y1 - y0), Image.LANCZOS), (x0, y0))


# the old sphere mesh's centre (d, y, z) and radius in ai.ac, and its texture mapping (u, v per degree of longitude from
# straight ahead and of latitude, both from the middle of the texture's sphere quarter)
BALL_C, BALL_R = (-0.05585, -0.0007, -0.00062), 0.0591
BALL_U, BALL_V = (0.7498, 0.0013497), (0.7501, 0.0026990)


def ball(nlat=64, nlon=96):
    """The VSI's sphere (ai.ac, object AttitudeBall) as a fine mesh: the old one had 16 x 16 facets, its outline a
    polygon with long straight sides and its shading broken up along them. Same place, size and texture mapping, so
    the pitch and roll animations and the texture stay as they were."""
    o = A.Obj('AttitudeBall', 'ai.png')
    cd, cy, cz = BALL_C
    def point(lat, lon):
        a, b = math.radians(lat), math.radians(lon)
        return (cd + BALL_R * math.cos(a) * math.cos(b), cy + BALL_R * math.cos(a) * math.sin(b), cz + BALL_R * math.sin(a))
    uv = lambda lat, lon: (BALL_U[0] + BALL_U[1] * lon, BALL_V[0] + BALL_V[1] * lat)
    for i in range(nlat):
        la0, la1 = -90 + 180 * i / nlat, -90 + 180 * (i + 1) / nlat
        for j in range(nlon):
            lo0, lo1 = -180 + 360 * j / nlon, -180 + 360 * (j + 1) / nlon
            corners = [(la0, lo0), (la0, lo1), (la1, lo1), (la1, lo0)]
            if i == 0:                                                   # the poles: triangles
                corners = [(la0, (lo0 + lo1) / 2), (la1, lo1), (la1, lo0)]
            elif i == nlat - 1:
                corners = [(la0, lo0), (la0, lo1), (la1, (lo0 + lo1) / 2)]
            mid = point((la0 + la1) / 2, (lo0 + lo1) / 2)
            o.face([point(*c) for c in corners], [uv(*c) for c in corners], tuple(m - c for m, c in zip(mid, BALL_C)))
    # the inclinometer's ball: a smooth black sphere on its path below the sphere (texture: black of the slip face)
    s = A.Obj('slip-ball', 'ai.png')
    black = A.const(SLIP_U[0] * -0.050 + SLIP_U[1] * -0.059 + SLIP_U[2], SLIP_V[0] * -0.059 + SLIP_V[1])
    c = (0.0010, 0.0, -SLIP_R)
    for i in range(12):
        la0, la1 = -90 + 15 * i, -75 + 15 * i
        for j in range(24):
            lo0, lo1 = 15 * j, 15 * (j + 1)
            pt = lambda la, lo: tuple(cc + SLIP_BALL * v for cc, v in zip(c, (math.cos(math.radians(la)) * math.cos(math.radians(lo)),
                                      math.cos(math.radians(la)) * math.sin(math.radians(lo)), math.sin(math.radians(la)))))
            corners = [(la0, lo0), (la0, lo1), (la1, lo1), (la1, lo0)]
            if i == 0: corners = [(la0, lo0), (la1, lo1), (la1, lo0)]
            if i == 11: corners = [(la0, lo0), (la0, lo1), (la1, lo0)]
            mid = pt((la0 + la1) / 2, (lo0 + lo1) / 2)
            s.face([pt(*k) for k in corners], [black(0, 0)] * len(corners), tuple(m - cc for m, cc in zip(mid, c)))
    path = os.path.join(I.INSTR, 'ai', 'ai.ac')
    L = open(path, encoding='latin-1').read().split('\n')
    if 'name "bank-index"' in L:                     # the lower bank index: the UH-60A's is at the top (SkyPointer)
        i = L.index('name "bank-index"') - 1
        j = next(k for k in range(i + 1, len(L)) if L[k].startswith('kids'))
        del L[i:j + 1]
        w = next(k for k in range(len(L)) if L[k].startswith('kids'))     # the world's
        L[w] = 'kids %d' % (int(L[w].split()[1]) - 1)
    for obj in (o, s):
        i = L.index('name "%s"' % obj.name) - 1
        j = next(k for k in range(i + 1, len(L)) if L[k].startswith('kids'))
        L[i:j + 1] = obj.lines()
    open(path, 'w', encoding='latin-1').write('\n'.join(L))


# ---------------------------------------------------------------- pilot's display unit

DSEG = os.path.expanduser('~/Library/Application Support/FlightGear/fgdata_2024_1/Fonts/DSEG/DSEG7/Classic/DSEG7Classic-Bold.ttf')
# the original 512 px face's layout (its rows are the tapes' calibration, prop-rpm.xml): value, tick row, label centre row
PDU_RPM = [(130, 103.5, 106), (120, 131.5, 132), (110, 163.5, 164), (105, 195.5, 195), (100, 227.5, 228), (95, 259.5, 260),
           (90, 291.5, 292), (70, 323.5, 324), (50, 355.5, 354), (0, 371.5, 372)]
PDU_RPM_MINOR = [115.5, 147.5, 179.5, 211.5, 243.5, 275.5, 307.5, 339.5]
PDU_TRQ = [(140, 105.5, 106), (120, 136, 136.5), (100, 166, 167), (80, 196, 196), (60, 226, 226), (40, 256, 255), (20, 286, 286),
           (0, 316.5, 317.5)]
PDU_TRQ_MINOR = [122, 152, 182, 212, 242, 272, 302]


def pdu():
    """The PDU's texture at 4x in its original layout: face redrawn, digits in a seven-segment font, the tapes' band
    copied pixel for pixel, the rest upscaled."""
    from PIL import ImageFont
    src = Image.open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src', 'prop-rpm-512.png')).convert('RGB')
    K = 4
    big = src.resize((2048, 2048), Image.LANCZOS)
    big.paste(src.crop((0, 432, 512, 448)).resize((2048, 64), Image.NEAREST), (0, 432 * K))     # the tapes' band
    face = Image.new('RGB', (256 * K * SS, 432 * K * SS), (13, 12, 12)); d = ImageDraw.Draw(face)
    q = K * SS
    W = (238, 238, 238)
    line = lambda x0, y0, x1, y1, w: d.rectangle([x0 * q, y0 * q, x1 * q, y1 * q], fill=W)
    line(144.2, 4, 146.8, 41, 0); line(23, 93.4, 242, 95.6, 0); line(144.2, 93.4, 146.8, 412, 0)
    num = I.font(14.5 * q, narrow=True); title = I.font(22 * q)
    d.text((90 * q, 14 * q), '%RPM', font=title, fill=W, anchor='mm')
    d.text((181 * q, 14 * q), '%TRQ', font=title, fill=W, anchor='mm')
    for v, row, lab in PDU_RPM:
        line(48, row - 1.6, 64.5, row + 1.6, 0); line(107.5, row - 1.6, 122, row + 1.6, 0)
        d.text((46.5 * q, lab * q), str(v), font=num, fill=W, anchor='rm'); d.text((124 * q, lab * q), str(v), font=num, fill=W, anchor='lm')
    for row in PDU_RPM_MINOR:
        line(48, row - 1.5, 64, row + 1.5, 0); line(109.5, row - 1.5, 121.5, row + 1.5, 0)
    for v, row, lab in PDU_TRQ:
        line(166, row - 1.3, 178.5, row + 1.3, 0); line(202.5, row - 1.3, 215.5, row + 1.3, 0)
        d.text((165 * q, lab * q), str(v), font=num, fill=W, anchor='rm'); d.text((217 * q, lab * q), str(v), font=num, fill=W, anchor='lm')
    for row in PDU_TRQ_MINOR:
        line(166, row - 1.1, 178.5, row + 1.1, 0); line(203, row - 1.1, 215.5, row + 1.1, 0)
    small = I.font(12 * q)
    for x, y, t in ((58.5, 385.5, '1'), (86.5, 385.5, 'R'), (112, 384, '2'), (179.5, 331.5, '1'), (202, 331.5, '2')):
        d.text((x * q, y * q), t, font=small, fill=W, anchor='mm')
    big.paste(face.resize((256 * K, 432 * K), Image.LANCZOS), (0, 0))
    # digits: ten cells of a tenth of the width, seven-segment, on black
    cells = Image.new('RGB', (2048 * SS, 64 * K * SS), (0, 0, 0)); dc = ImageDraw.Draw(cells)
    f = ImageFont.truetype(DSEG, int(52 * K * SS)) if os.path.exists(DSEG) else I.font(52 * K * SS)
    for k in range(10):
        cx = (k + 0.5) * 204.8 * SS + 4 * SS
        dc.text((cx, 32 * K * SS), '8', font=f, fill=(34, 14, 4), anchor='mm')
        dc.text((cx, 32 * K * SS), str(k), font=f, fill=(255, 112, 10), anchor='mm')
    big.paste(cells.resize((2048, 64 * K), Image.LANCZOS), (0, 448 * K))
    big.save(os.path.join(I.INSTR, 'prop-rpm', 'prop-rpm.png'), optimize=True)
    vids_lighting('prop-rpm')


def vids_lighting(name):
    """The display unit's scales (object fond) lit by the nonflight instrument lights (TM 2.70.3: NVG information panel
    lighting, integral: the markings light themselves); its lamp columns and digits light themselves, in the HDR
    pipeline at the level of HDR vids, which follows the light outside as the units' photocells do (TM 2.10.3). A marked
    block in the original's XML, replaced each time."""
    path = os.path.join(I.INSTR, name, name + '.xml')
    x = open(path, encoding='latin-1').read()
    x = re.sub(r'\n  <!-- NVG panel lighting.*?<!-- /NVG -->\n', '\n', x, flags=re.S)
    own = re.search(r'<animation>\s*<type>material</type>((?:(?!</animation>).)*?)<factor-prop>controls/lighting/instruments-norm'
                    r'</factor-prop>', x, flags=re.S)
    displays = re.findall(r'<object-name>\s*(\S+?)\s*</object-name>', own.group(1))
    blk = ('\n  <!-- NVG panel lighting: written by fdm/cockpit/gauges.py -->' + I.integral_xml(['fond'], 'non-flt')
           + I.lit_xml(displays, I.HDR + 'vids') + '  <!-- /NVG -->\n')
    open(path, 'w', encoding='latin-1').write(x.replace('\n</PropertyList>', blk + '</PropertyList>', 1))


# ---------------------------------------------------------------- central display unit

# the original 1024 px face's labels: text, box (x0, x1, y0, y1) in its pixels; ticks and lines are kept as they are
CDU_TEXT = [
    ('FUEL', 66, 101, 34, 44), ('QTY', 70, 95, 51, 62), ('LBx100', 60, 105, 65, 74), ('21', 78, 92, 100, 110), ('18', 76, 92, 131, 141),
    ('15', 76, 91, 167, 177), ('12', 76, 91, 198, 208), ('9', 79, 87, 231, 241), ('6', 80, 87, 260, 270), ('3', 80, 86, 286, 296),
    ('0', 79, 87, 311, 321), ('1', 52, 57, 319, 328), ('2', 112, 118, 319, 328), ('TOTAL FUEL', 45, 124, 382, 391),
    ('XMSN', 189, 234, 36, 46), ('TEMP', 162, 199, 55, 64), ('°C', 159, 171, 69, 79), ('x10', 187, 206, 71, 79),
    ('14', 188, 203, 83, 93), ('12', 188, 203, 112, 122), ('10', 188, 204, 146, 156), ('8', 192, 200, 187, 197), ('4', 191, 199, 230, 240),
    ('0', 192, 200, 269, 279), ('-4', 188, 201, 301, 311), ('PRESS', 222, 261, 56, 65), ('PSI', 236, 254, 70, 79),
    ('150', 254, 275, 82, 92), ('110', 253, 274, 108, 118), ('70', 256, 272, 160, 170), ('60', 255, 271, 189, 199), ('50', 256, 272, 221, 231),
    ('40', 255, 272, 251, 261), ('30', 256, 272, 279, 289), ('0', 260, 268, 311, 321),
    ('LAMP TEST', 143, 209, 398, 407), ('BRIGHT', 233, 276, 353, 362), ('DIM', 268, 291, 397, 406),
    ('ENG OIL', 348, 407, 35, 45), ('TEMP', 305, 342, 52, 61), ('°C', 302, 314, 66, 76), ('x10', 330, 349, 68, 76),
    ('14', 315, 331, 83, 93), ('12', 315, 330, 112, 122), ('10', 315, 331, 146, 156), ('8', 319, 327, 187, 197), ('4', 318, 326, 230, 240),
    ('0', 319, 327, 269, 279), ('-4', 315, 328, 301, 311), ('ON', 340, 358, 357, 366), ('OFF', 337, 358, 390, 399),
    ('DIGITS', 325, 372, 403, 413), ('PRESS', 411, 450, 53, 62), ('PSI', 425, 443, 67, 76), ('170', 423, 446, 88, 98),
    ('120', 423, 446, 121, 131), ('100', 422, 445, 142, 152), ('90', 425, 442, 159, 169), ('70', 425, 441, 192, 202), ('50', 426, 442, 229, 239),
    ('30', 426, 442, 272, 282), ('10', 427, 441, 311, 320), ('1', 403, 408, 324, 332), ('2', 457, 462, 323, 331),
    ('1', 409, 414, 372, 381), ('CHAN', 426, 459, 372, 381), ('2', 469, 475, 372, 381),
    ('TGT', 521, 549, 45, 55), ('TEMP', 519, 546, 60, 67), ('9', 528, 536, 91, 101), ('8', 528, 536, 120, 130), ('7', 529, 535, 149, 159),
    ('6', 530, 537, 177, 187), ('5', 529, 535, 206, 216), ('4', 528, 536, 235, 245), ('2', 529, 535, 274, 284), ('0', 528, 536, 311, 321),
    ('1', 504, 509, 324, 333), ('2', 557, 563, 325, 334), ('TGT', 521, 549, 391, 401),
    ('Ng', 618, 636, 44, 57), ('SPEED', 611, 641, 62, 69), ('11', 617, 632, 82, 92), ('10', 616, 632, 114, 124), ('9', 620, 628, 148, 158),
    ('8', 620, 628, 184, 194), ('7', 622, 628, 222, 232), ('4', 622, 630, 265, 275), ('0', 622, 630, 311, 321),
    ('1', 598, 604, 325, 335), ('2', 646, 652, 325, 335), ('Ng', 618, 636, 390, 403),
]

# black smudges on the original face (the lamps' black bezels stay): Ng's shadows, a bar and a notch by XMSN
CDU_SMUDGES = [(630, 644, 46, 61), (630, 644, 392, 406), (139, 147, 270, 342), (280, 296, 342, 345)]


def fit_font(text, w, h):
    """The bold font (narrow if need be) whose rendering of `text` is h pixels high and no wider than about w."""
    from PIL import ImageFont
    for narrow in (False, True):
        lo, hi = 4.0, 400.0
        for _ in range(30):
            m = (lo + hi) / 2; b = I.font(m, narrow).getbbox(text)
            lo, hi = (m, hi) if b[3] - b[1] < h else (lo, m)
        f = I.font(lo, narrow); b = f.getbbox(text)
        if b[2] - b[0] <= w * 1.12 or narrow:
            return f


def cdu():
    """The CDU's texture at 2x in its original layout: labels re-typeset, ticks and lines kept, the tapes' band copied
    pixel for pixel, digits in a seven-segment font."""
    from PIL import ImageFont
    src = Image.open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src', 'combimeter-1024.png')).convert('RGB')
    K = 2
    big = src.resize((2048, 2048), Image.LANCZOS)
    big.paste(src.crop((0, 560, 1024, 640)).resize((2048, 160), Image.NEAREST), (0, 560 * K))     # the tapes' band
    geo = src.crop((0, 0, 690, 432)).copy(); dg = ImageDraw.Draw(geo)
    from scipy import ndimage
    blobs, _ = ndimage.label(np.asarray(geo.convert('L')) > 60, np.ones((3, 3)))
    for x0, x1, y0, y1 in CDU_SMUDGES:
        g = np.asarray(geo).copy(); r = g[y0:y1 + 1, x0:x1 + 1]
        r[r.mean(2) < 8] = 12; geo = Image.fromarray(g); dg = ImageDraw.Draw(geo)
    for t, x0, x1, y0, y1 in CDU_TEXT:
        dg.rectangle([x0, y0, x1, y1], fill=(12, 12, 12))
        inside = np.zeros(blobs.shape, bool); inside[y0:y1 + 1, x0:x1 + 1] = True
        for k in set(np.unique(blobs[inside])) - {0}:              # the glyphs' bits outside the box, not ticks
            out = (blobs == k) & ~inside
            if 0 < out.sum() <= 30:
                for y, x in zip(*np.nonzero(out)):
                    geo.putpixel((int(x), int(y)), (12, 12, 12))
    face = geo.resize((690 * K * SS, 432 * K * SS), Image.NEAREST); d = ImageDraw.Draw(face)
    q = K * SS
    for t, x0, x1, y0, y1 in CDU_TEXT:
        f = fit_font(t, (x1 - x0 + 1) * q, (y1 - y0 + 1) * q)
        d.text(((x0 + x1 + 1) / 2 * q, (y0 + y1 + 1) / 2 * q), t, font=f, fill=(240, 240, 240), anchor='mm')
    big.paste(face.resize((690 * K, 432 * K), Image.LANCZOS), (0, 0))
    cells = Image.new('RGB', (2048 * SS, 167 * K * SS), (0, 0, 0)); dc = ImageDraw.Draw(cells)
    f = ImageFont.truetype(DSEG, int(104 * K * SS)) if os.path.exists(DSEG) else I.font(104 * K * SS)
    for k in range(10):
        cx = (k + 0.5) * 204.8 * SS + 6 * SS
        dc.text((cx, (959.5 - 857) * K * SS), '8', font=f, fill=(34, 14, 4), anchor='mm')
        dc.text((cx, (959.5 - 857) * K * SS), str(k), font=f, fill=(255, 112, 10), anchor='mm')
    big.paste(cells.resize((2048, 167 * K), Image.LANCZOS), (0, 857 * K))
    big.save(os.path.join(I.INSTR, 'combimeter', 'combimeter.png'), optimize=True)
    vids_lighting('combimeter')


if __name__ == '__main__':
    asi(); vsi(); alt(); apn209(); sphere(); ball(); pdu(); cdu()
