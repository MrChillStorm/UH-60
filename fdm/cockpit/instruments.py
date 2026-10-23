"""Builds the UH-60A instruments the old panel lacked; panel_face.py puts them on their TM 1-1520-237-10 figure 2-9
places.

- Vertical speed indicators (the airspeed indicator's case, glass and needle with a vertical speed dial), clocks
  and stabilator position indicators, working.
- The caution/advisory panel with its capsules, which Systems/uh60a-cap.xml lights from what the simulation has
  (table 2-3 of the TM gives each capsule's condition). The failures the simulation doesn't have stay dark. Its
  BRT/DIM-TEST switch tests the capsules and the master warning panel; MASTER CAUTION lights when an amber capsule
  goes on and stays on until pressed (Nasal/caution.nas).
- The mode selectors (VSI/HSI and CIS, TM 3.25.4), integrally lighted panels of lighted push buttons: their markings
  the TM figure's, lettered in bold and lit on the nonflight instrument lights; square caps in raised frames, their
  legends readable by day and lit by Nasal/avionics.nas's switch logic, pressing in while clicked (PushPanel).
- The panels with nothing to drive yet (auxiliary fuel management panel, radar warning indicator, ice rate meter,
  blade deice and IRCM panels, dimming controls): the TM's face art on a raised panel, with 3-D knobs, switches,
  lamps and buttons. The engine ignition switch works (/controls/engines/ignition).

Sizes and places are the TM figure's (panel_layout.py); everything is in metres in the instrument's own frame
(acmesh.py), its origin on the TM bezel's centre and its back on the face plate.

    python3 instruments.py TM.pdf
"""
import math, os, re, subprocess, sys
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
GROUND, LETTER = 38, 215                    # the panels' paint and lettering


def font(size, narrow=False):
    """Arial Bold (or Arial Narrow Bold), found by its file name among the system's fonts, which Pillow searches on
    each OS (the names differ on Windows)."""
    for name in (('Arial Narrow Bold.ttf', 'ARIALNB.TTF') if narrow else ('Arial Bold.ttf', 'arialbd.ttf')):
        try:
            return ImageFont.truetype(name, int(size))
        except OSError:
            pass
    sys.exit('Arial%s Bold is not among the system fonts' % (' Narrow' if narrow else ''))


# ---------------------------------------------------------------- digits
# FlightGear's text nodes neither update nor glow in the HDR pipeline (2024.2), so readouts are digit cells: each a
# quad onto a strip of the digits 0-9 that spans its own texture, which repeats; a textranslate animation steps the
# cell's texture along by a tenth per unit of its digit (the property stepped to the digit's place), and the
# repeat leaves the digit.

SEG7 = {0: 'abcdef', 1: 'bc', 2: 'abdeg', 3: 'abcdg', 4: 'bcfg', 5: 'acdfg', 6: 'acdefg', 7: 'abc', 8: 'abcdefg', 9: 'abcdfg'}


def seven_segment(d, box, digit, on, off=None, slant=0.08):
    """A seven-segment digit filling 80 % of the box's height, half as wide as high, leaning forward by `slant`:
    its segments in `on`, the unlit ones in `off` (None: left out)."""
    x0, y0, x1, y1 = box
    h = 0.80 * (y1 - y0); w = 0.50 * h; t = 0.15 * h; g = 0.012 * h            # height, width, stroke, gap
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    L, R, T, B = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    def P(pts):
        return [(x + slant * (cy - y), y) for x, y in pts]
    def hseg(y):                                                                 # a horizontal segment centred on y
        return P([(L + g, y), (L + g + t / 2, y - t / 2), (R - g - t / 2, y - t / 2), (R - g, y), (R - g - t / 2, y + t / 2),
                  (L + g + t / 2, y + t / 2)])
    def vseg(x, ya, yb):                                                         # a vertical one on x from ya to yb
        return P([(x, ya + g), (x + t / 2, ya + g + t / 2), (x + t / 2, yb - g - t / 2), (x, yb - g), (x - t / 2, yb - g - t / 2),
                  (x - t / 2, ya + g + t / 2)])
    L1, R1, T1, B1, M = L + t / 2, R - t / 2, T + t / 2, B - t / 2, cy
    segs = {'a': hseg(T1), 'g': hseg(M), 'd': hseg(B1), 'b': vseg(R1, T1, M), 'c': vseg(R1, M, B1), 'e': vseg(L1, M, B1),
            'f': vseg(L1, T1, M)}
    for k, poly in segs.items():
        lit = k in SEG7[digit]
        if lit or off is not None:
            d.polygon(poly, fill=on if lit else off)


def digit_strip(path, cw, ch, on, off=None, face=None):
    """Writes the texture `path`: the digits 0-9 in cells of cw x ch pixels side by side across it, on black,
    seven-segment (lit `on`, unlit `off`) or drawn in the font `face` (a counter's drum). Returns its file name."""
    q = 4 if face is None else 1                                   # the segments drawn at 4x, for their edges
    im = Image.new('RGB', (10 * cw * q, ch * q), (0, 0, 0)); d = ImageDraw.Draw(im)
    for k in range(10):
        box = (k * cw * q, 0, (k + 1) * cw * q, ch * q)
        if face is None:
            seven_segment(d, box, k, on, off)
        else:
            d.text(((box[0] + box[2]) / 2, (box[1] + box[3]) / 2), str(k), font=face, fill=on, anchor='mm')
    im.resize((10 * cw, ch), Image.LANCZOS).save(path, optimize=True)
    return os.path.basename(path)


def digit_cells(texture, name, n, y0, y1, z0, z1, d):
    """n cells side by side across y0..y1, z0..z1 at depth d, objects name0 (the units, at the right) to name<n-1>,
    each mapped onto the strip's 0 (the first tenth of the texture)."""
    w = (y1 - y0) / n; objs = []
    for k in range(n):
        ya = y1 - (k + 1) * w
        o = A.Obj('%s%d' % (name, k), texture)
        A.flat(o, [(ya, z0), (ya + w, z0), (ya + w, z1), (ya, z1)], d, A.planar(ya, ya + w, z0, z1, 0.0, 0.1, 0.0, 1.0))
        objs.append(o)
    return objs


def digits_xml(name, n, prop, unit=1.0, rounding=True, blank=False):
    """The cells' animations: cell k shows the digit k (0 the units) of prop / unit, rounded (or truncated); below zero
    it shows 0. Leading zeros blank if `blank`."""
    out = ''
    for k in range(n):
        place = unit * 10 ** k
        out += ('\n  <animation>\n    <type>textranslate</type>\n    <object-name>%s%d</object-name>\n'
                '    <property>%s</property>\n%s    <step> %.9g </step>\n    <factor> %.9g </factor>\n    <min> 0 </min>\n'
                '    <axis>\n      <x> 1 </x>\n      <y> 0 </y>\n      <z> 0 </z>\n    </axis>\n  </animation>\n'
                % (name, k, prop, '    <bias> %.9g </bias>\n' % (0.5 * unit) if rounding else '', place, 0.1 / place))
        if blank and k > 0:
            out += select_xml(['%s%d' % (name, k)], '<greater-than-equals><property>%s</property><value>%.9g</value>'
                              '</greater-than-equals>' % (prop, place - (0.5 * unit if rounding else 0)))
    return out


def cell_names(name, n):
    return ['%s%d' % (name, k) for k in range(n)]


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


# Instrument lighting (TM 1-1520-237-10 2.70.3): NVG blue-green bezel lights on three circuits, the pilot's and the
# copilot's flight instruments and the nonflight instruments, each on its INSTR LT rheostat; Nasal/avionics.nas sets
# their levels (0 without ac power) in LIGHTS: plt-flt, cplt-flt, non-flt, and radalt[n] (RAD ALT DIMMING besides).
# The HDR pipeline leaves out the material animation's emission: there, lighted bezels have lamps (bezel_light_xml),
# and what lights itself emits through Effects/Lit (lit_xml), at a level in HDR (cd/m2, or a circuit's 0-1 level
# times a gain), which Nasal/avionics.nas sets.
LIGHTS = '/instrumentation/uh60/lights/'
HDR = LIGHTS + 'hdr/'
NVG_GREEN = (0.10, 1.00, 0.45)         # NVIS Green A (MIL-L-85762A: u' 0.088, v' 0.543), sRGB about (0, 255, 110)
LIGHT_LUX = 6.0                        # HDR: the mean illuminance of an instrument's face at full BRT, lux
# The HDR pipeline has no photometric units: its light is the sun's spectral irradiance (W m-2 nm-1) put through the
# matrix of fgdata's Shaders/HDR/atmos_spectral.glsl, which gives a flat spectrum of 1 W/(m2 sr nm), 73,000 cd/m2, a
# luminance of 115. Its unit of radiance is so 635 cd/m2 (from the sun, 610-640), and its unit of a light's intensity
# 635 cd: the lights here are written as cd / HDR_CD, and Shaders/HDR/lit.frag takes cd/m2 the same way.
HDR_CD = 635.0
# A lamp's legend or a display's segments sit behind a dead front (smoked or filtered), which is all they show of the
# daylight, lit or not; integrally lit markings are paint (reflectance 1 here: their texture's own).
DEAD_FRONT = 0.05


def objects_xml(objs):
    return ''.join('    <object-name>%s</object-name>\n' % o for o in objs)


def light_xml(objs, light):
    """The NVG instrument lighting of `objs` on circuit `light` (a name in LIGHTS)."""
    return ('\n  <animation>\n    <type>material</type>\n%s    <emission>\n      <factor-prop>%s%s</factor-prop>\n'
            '      <red>   %.2f </red>\n      <green> %.2f </green>\n      <blue>  %.2f </blue>\n    </emission>\n'
            '  </animation>\n' % (objects_xml(objs), LIGHTS, light, *NVG_GREEN))


def lit_xml(objs, level, rgb=(1, 1, 1), gain=1.0, reflect=DEAD_FRONT):
    """HDR pipeline: `objs` emit their texture x rgb x the property `level` x gain (cd/m2), through Effects/Lit, and
    reflect the light on them as their texture x `reflect` (a lamp's dead front by default)."""
    return ('\n  <effect>\n    <inherits-from>../../../../Effects/Lit/lit</inherits-from>\n    <parameters>\n'
            '      <lit-color type="vec3d"> %.2f %.2f %.2f </lit-color>\n      <lit-level type="float"><use>%s</use></lit-level>\n'
            '      <lit-gain type="float"> %.4g </lit-gain>\n      <lit-reflectance type="float"> %.4g </lit-reflectance>\n'
            '    </parameters>\n%s  </effect>\n' % (tuple(rgb) + (level, gain, reflect, objects_xml(objs))))


def integral_xml(objs, light):
    """An integrally lit panel's NVG lighting (its markings light themselves): the material animation, and in the HDR
    pipeline the emission that matches a face lit by LIGHT_LUX."""
    return light_xml(objs, light) + lit_xml(objs, LIGHTS + light, NVG_GREEN, LIGHT_LUX / math.pi, reflect=1.0)


def bezel_lamps(face, half, round=False, glass=None, out=None):
    """The lamps of a lighted bezel: spot lights round the face, a quarter of its half size (the smaller one, m) above
    it but behind its glass (at depth `glass`: 1.5 mm, else the glass mirrors them) and 0.15 m out from its edge, 0.9 m apart (eight round a dial), each aimed inward across the face at the
    nearest point of its centre line (a round face's centre) 0.1 m above it. Their exponent of 3 holds back the lamp's
    light on the face near it, so the face is lit about evenly, a little brighter toward the edge as a lighted bezel
    is (on a dial 1.7 times as bright at 0.9 R as at the centre); the cone takes in the face, and nothing
    outside the bezel: lights cast no shadows in FlightGear. face: (d, y, z) of the face's centre; half: its half
    width and height, or with `round` its radius twice; `out` (m) the lamps' distance out from the face's edge if the
    case leaves less room than 0.15 m (lamps outside it light its walls). Returns [(position, aim point)], the exponent, cutoff (deg),
    range (m), the intensity (cd) that gives the face a mean illuminance of 1 lux, and the face's evenness (its
    illuminance's 98th percentile over its 2nd)."""
    d, y, z = face; hw, hh = half
    m = min(hw, hh)
    h, off, up, gap, e = 0.25 * m, 0.15 * m, 0.10 * m, 0.9 * m, 3.0
    if glass is not None:
        h = min(h, glass - 0.0015 - d)
    if out is not None:
        off = out
    if round:
        R = max(hw, hh)
        path = [((R + off) * math.cos(t), (R + off) * math.sin(t)) for t in np.linspace(0, 2 * math.pi, 721)[:-1]]
    else:                                   # the face's outline grown by off, its corners rounded
        path = []
        for cy, cz, a0 in ((hw, hh, 0), (-hw, hh, 90), (-hw, -hh, 180), (hw, -hh, 270)):
            for a in np.radians(np.arange(a0, a0 + 90, 1.0)):
                path.append((cy + off * math.cos(a), cz + off * math.sin(a)))
        dense = []
        for (y0, z0), (y1, z1) in zip(path, path[1:] + path[:1]):
            k = max(1, int(math.hypot(y1 - y0, z1 - z0) / 0.0002))
            dense += [(y0 + (y1 - y0) * t / k, z0 + (z1 - z0) * t / k) for t in range(k)]
        path = dense
    P = np.array(path); seg = np.hypot(*(np.roll(P, -1, 0) - P).T); s = np.concatenate([[0], np.cumsum(seg)[:-1]])
    n = max(4, int(math.floor(s[-1] / gap + 0.5)))
    if round:
        n = 8
    lamps = []
    for k in range(n):
        q = P[np.searchsorted(s, (k + 0.5) * seg.sum() / n) - 1]
        ay = float(np.clip(q[0], -(hw - m), hw - m)); az = float(np.clip(q[1], -(hh - m), hh - m))
        lamps.append(((d + h, y + q[0], z + q[1]), (d + up, y + ay, z + az)))
    g = np.linspace(-0.97, 0.97, 41); GY, GZ = np.meshgrid(g * hw, g * hh)
    inside = (GY / hw) ** 2 + (GZ / hh) ** 2 <= 0.97 ** 2 if round else np.ones_like(GY, bool)
    F = np.stack([np.full(inside.sum(), d), y + GY[inside], z + GZ[inside]], 1)
    far = max(np.linalg.norm(F - np.array(p), axis=1).max() for p, a in lamps); rng = 1.16 * far
    cut, E = 0.0, np.zeros(len(F))
    for p, a in lamps:
        v = F - np.array(p); dist = np.linalg.norm(v, axis=1); u = v / dist[:, None]
        axis = np.array(a) - np.array(p); axis /= np.linalg.norm(axis)
        cd = np.clip(u @ axis, 1e-6, 1)
        cut = max(cut, math.degrees(math.acos(cd.min())))
        win = np.clip(1 - (dist / rng) ** 4, 0, None) ** 2
        E += cd ** e * (-u[:, 0]) * win / dist ** 2
    q2, q98 = np.percentile(E, [2, 98])
    return lamps, e, min(89.0, cut + 2), rng, 1 / E.mean(), q98 / q2


def lamp_xml(name, p, a, light, cd, e, cut, rng):
    """An NVG blue-green spot light at p aimed at a, on circuit `light` (in LIGHTS): cd its intensity at full BRT."""
    return ('\n  <light>\n    <name>%s</name>\n    <type>spot</type>\n    <position>\n      <x-m> %.4f </x-m>\n'
            '      <y-m> %.4f </y-m>\n      <z-m> %.4f </z-m>\n    </position>\n    <direction>\n'
            '      <lookat-x-m> %.4f </lookat-x-m>\n      <lookat-y-m> %.4f </lookat-y-m>\n      <lookat-z-m> %.4f </lookat-z-m>\n'
            '    </direction>\n    <color>\n      <r> %.2f </r>\n      <g> %.2f </g>\n      <b> %.2f </b>\n    </color>\n'
            '    <intensity>\n      <property>%s%s</property>\n      <scale> %.6g </scale>\n    </intensity>\n'
            '    <spot-exponent> %g </spot-exponent>\n    <spot-cutoff> %.1f </spot-cutoff>\n    <range-m> %.3f </range-m>\n'
            '  </light>\n' % ((name,) + tuple(p) + tuple(a) + NVG_GREEN + (LIGHTS, light, cd / HDR_CD, e, cut, rng)))


def bezel_light_xml(light, face, half, round=False, glass=None, out=None, name='bezel-lamp'):
    """HDR pipeline: the instrument's lighted bezel (bezel_lamps), NVG blue-green, on circuit `light` (in LIGHTS): its
    face's mean illuminance LIGHT_LUX at full BRT."""
    lamps, e, cut, rng, cd, even = bezel_lamps(face, half, round, glass, out)
    print('    bezel lamps: %d, evenness %.1f' % (len(lamps), even))
    return ''.join(lamp_xml('%s-%d' % (name, k), p, a, light, LIGHT_LUX * cd, e, cut, rng) for k, (p, a) in enumerate(lamps))


def window_lamp_xml(light, centre, half, name):
    """HDR pipeline: a lamp over a counter's window in a bezel (d, y, z its centre, half its half width and height):
    4 mm in front of it and 3 mm over its top, aimed at its centre; the window's mean illuminance LIGHT_LUX."""
    d, y, z = centre; hw, hh = half
    p = np.array([d + 0.004, y, z + hh + 0.003])
    g = np.linspace(-1, 1, 21); GY, GZ = np.meshgrid(y + g * hw, z + g * hh)
    F = np.stack([np.full(GY.size, d), GY.ravel(), GZ.ravel()], 1)
    v = F - p; dist = np.linalg.norm(v, axis=1); u = v / dist[:, None]
    axis = np.array([d, y, z]) - p; axis /= np.linalg.norm(axis)
    cut = math.degrees(math.acos(np.clip(u @ axis, -1, 1).min())) + 2
    rng = 1.3 * dist.max()
    E = (-u[:, 0]) * np.clip(1 - (dist / rng) ** 4, 0, None) ** 2 / dist ** 2
    return lamp_xml(name, p, (d, y, z), light, LIGHT_LUX / E.mean(), 0, cut, rng)


def common_xml(objs, lit=(), glass=(), extra='', light='non-flt', bezel=None, integral=False):
    """The animations every instrument has: no shadows, the glass effect, the instrument lighting on `lit` (circuit
    `light`): in the HDR pipeline its lighted bezel (bezel: (face centre, half size[, round[, glass[, out]]]), see
    bezel_lamps), or
    with `integral` the lit objects' own emission (integral_xml)."""
    out = '<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- written by fdm/cockpit/instruments.py -->\n\n'
    out += '<PropertyList>\n\n  <path>%%s</path>\n\n  <animation>\n    <type>noshadow</type>\n%s  </animation>\n' % objects_xml(objs)
    for g in glass:
        out += ('\n  <effect>\n    <inherits-from>../../../../Effects/Glass/instrument-glass</inherits-from>\n'
                '    <object-name>%s</object-name>\n  </effect>\n' % g)
    if lit:
        out += integral_xml(lit, light) if integral else light_xml(lit, light)
    if bezel:
        out += bezel_light_xml(light, *bezel)
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
                     bezel=((0.006, 0.0, 0.0), (r, r), True, 0.013),
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
                     bezel=((0.005, 0.0, 0.0), (ww / 2, wh / 2), False, 0.0115),
                     extra=rotate_xml('needle', '/fdm/jsbsim/stab/pos-deg', 1, centre=(0, py, pz), lo=-12, hi=45))
    write_instrument('stab', [body, face, needle, glass],
                     xml.replace('<PropertyList>', '<!-- stabilator position indicator: /fdm/jsbsim/stab/pos-deg, trailing '
                                 'edge down + -->\n<PropertyList>', 1))
    t.save(os.path.join(INSTR, 'stab', 'stab.png'))


# ---------------------------------------------------------------- panels with the TM's face art

LIGHT_PANELS = False    # the face-art panels stand in until they're modelled in 3-D: no instrument lights till then
                        # (their lamps and lit legends work)


class Panel:
    """A raised panel with the TM figure's art on its face, and 3-D parts on it (positions in figure pt from the
    bezel's centre, y down, as measured on the figure). Parts can be separate objects (to animate or click), lit
    legends are drawn below the art in the texture."""
    def __init__(s, name, art, num, side, depth, texw=None, box=None, room=0, px_mm=14):
        """The model's origin is the TM item's centre; `box` (sheet pt) the panel's outline if not the item's. The
        face is rendered from the figure's vector art at px_mm pixels per mm (texw: its width in pixels)."""
        it = item(num, side)
        s.name, s.sheet = name, it[3]
        s.cx, s.cy = (it[4] + it[5]) / 2, (it[6] + it[7]) / 2
        s.x0, s.x1, s.y0, s.y1 = box or it[4:8]
        s.w, s.h = (s.x1 - s.x0) * PT, (s.y1 - s.y0) * PT
        s.bx, s.bz = ((s.x0 + s.x1) / 2 - s.cx) * PT, -((s.y0 + s.y1) / 2 - s.cy) * PT
        s.depth = depth
        pw = texw or min(2048, int(round(s.w * 1000 * px_mm))); ph = int(round(pw * s.h / s.w))
        W = 1 << max(6, math.ceil(math.log2(pw)))
        H = 1 << max(6, math.ceil(math.log2(ph + 20 + room)))
        s.t = Tex(W, H, (GROUND,) * 3)
        im = tmart.light(art.vector(s.x0, s.x1, s.y0, s.y1, s.sheet, pw, ph), GROUND, LETTER)
        s.t.im.paste(im.convert('RGB'), (0, 0))
        s.pw, s.ph = pw, ph
        s.face_uv = s.t.area(0, 0, pw, ph, s.bx - s.w / 2, s.bx + s.w / 2, s.bz - s.h / 2, s.bz + s.h / 2)
        s.free = [0, ph + 4, 0]                                  # next free x, y and row height in the texture
        s.paint = s.t.swatch((30, 30, 30)); s.knob_side = s.t.swatch((20, 20, 20)); s.knob_top = s.t.swatch((48, 48, 48))
        s.metal = s.t.swatch((170, 170, 172)); s.white = s.t.swatch((230, 230, 230)); s.dark = s.t.swatch((8, 8, 8))
        s.body = A.Obj('panel', name + '.png'); s.parts = {}
        A.prism(s.body, A.rrect(s.w, s.h, min(0.002, s.h / 3), s.bx, s.bz), 0, depth, s.face_uv, s.paint)
        s.lit = ['panel']; s.xml = ''
        s.lamps = 'caut-advsy'          # its lamps' and lit legends' dimming (in LIGHTS and HDR): the cockpit indicator
                                        # lights dim with the caution/advisory panel (TM 2.81.2)

    def at(s, x, y):
        return x * PT, -y * PT

    def obj(s, name):
        if name not in s.parts:
            s.parts[name] = A.Obj(name, s.name + '.png')
        return s.parts[name]

    def alloc(s, w, h):
        x, y, rh = s.free
        if x + w > s.t.im.width:
            x, y, rh = 0, y + rh + 4, 0
        s.free = [x + w + 4, y, max(rh, h)]
        return x, y, x + w, y + h

    def knob(s, x, y, dia, height=0.008, name='knobs', pointer=True):
        """A round knob (dia in mm) with a white index line pointing up."""
        cy, cz = s.at(x, y); o = s.obj(name); r = dia * MM / 2
        A.cylinder(o, r, cy, cz, s.depth, s.depth + height, s.knob_top, s.knob_side)
        if pointer:
            A.box(o, cy - 0.0006, cy + 0.0006, cz + r * 0.25, cz + r * 0.95, s.depth + height, s.depth + height + 0.0003,
                  s.white, s.white)
        return cy, cz

    def toggle(s, x, y, name='toggles', up=True, straight=False):
        """A toggle switch: nut and lever, the lever up, down or straight out (to be animated)."""
        cy, cz = s.at(x, y); o = s.obj(name)
        A.cylinder(o, 0.0035, cy, cz, s.depth, s.depth + 0.003, s.metal, s.metal, n=6)
        tilt = 0 if straight else (0.006 if up else -0.006)
        lo, hi = s.depth + 0.003, s.depth + 0.014
        A.beam(o, (lo, cy, cz), (hi, cy, cz + tilt), 0.0022, s.metal)
        return cy, cz

    def button(s, x, y, bw, bh, height=0.003, name='buttons'):
        """A push button (bw x bh pt) with the face art on its top."""
        cy, cz = s.at(x, y); o = s.obj(name)
        A.prism(o, A.rrect(bw * PT, bh * PT, 0.0008, cy, cz), s.depth, s.depth + height, s.face_uv, s.knob_side)
        if name not in s.lit:
            s.lit.append(name)
        return cy, cz

    def lamp(s, x, y, dia, rgb, name='lamps', lit=None, cond=None):
        """A lamp; with `lit`, a lit top shown on `cond`."""
        cy, cz = s.at(x, y); o = s.obj(name)
        A.cylinder(o, dia * MM / 2, cy, cz, s.depth, s.depth + 0.003, s.t.swatch(rgb), s.knob_side)
        if lit:
            on = s.obj(name + '-on')
            A.flat(on, A.circle(dia * MM / 2 * 0.95, cy, cz, 20), s.depth + 0.0031, s.t.swatch(lit))
            s.xml += select_xml([name + '-on'], cond) + s.lamp_xml([name + '-on'])
        return cy, cz

    def legend(s, name, y0, y1, z0, z1, d, lines, rgb, cond):
        """A lit legend (panel metres), shown on `cond`."""
        pw = int(160 * (y1 - y0) / 0.008); ph = max(24, int(pw * (z1 - z0) / (y1 - y0)))
        x0, py0, x1, py1 = s.alloc(pw, ph)
        dr = s.t.d; dr.rectangle([x0, py0, x1, py1], fill=(0, 0, 0))           # black: in HDR the whole texture emits
        size = ph * (0.42 if len(lines) > 1 else 0.55)
        widest = max(dr.textlength(line, font=font(size)) for line in lines)
        f = font(size * min(1.0, 0.86 * (x1 - x0) / widest))                    # within the cell
        for k, line in enumerate(lines):
            yy = py0 + ph * ((k + 1) / (len(lines) + 1) if len(lines) > 1 else 0.5)
            dr.text(((x0 + x1) / 2, yy), line, font=f, fill=rgb, anchor='mm')
        A.flat(s.obj(name), [(y0, z0), (y1, z0), (y1, z1), (y0, z1)], d, s.t.area(x0, py0, x1, py1, y0, y1, z0, z1))
        s.xml += select_xml([name], cond) + s.lamp_xml([name])

    def lamp_xml(s, objs):
        return emission_xml(objs, LIGHTS + s.lamps) + lit_xml(objs, HDR + s.lamps)

    def glass(s, x, y, dia):
        cy, cz = s.at(x, y); o = s.parts.setdefault('glass', A.Obj('glass', None, A.GLASS))
        A.flat(o, A.circle(dia * PT / 2, cy, cz, 40), s.depth + 0.0015, A.const(0, 0))

    def write(s, comment, extra='', variants=None, light='non-flt'):
        """variants: [(xml name, {text: replacement})] for instances that differ only in their properties."""
        objs = [s.body] + list(s.parts.values())
        glass = ['glass'] if 'glass' in s.parts else []
        integral = getattr(s, 'integral', False)
        lit = s.lit if LIGHT_PANELS or integral else []
        xml = common_xml([o.name for o in objs], lit=lit, glass=glass, extra=s.xml + extra, light=light, integral=integral,
                         bezel=((s.depth, s.bx, s.bz), (s.w / 2, s.h / 2)) if lit and not integral else None)
        xml = xml.replace('<PropertyList>', '<!-- %s -->\n<PropertyList>' % comment, 1)
        write_instrument(s.name, objs, xml)
        for fname, subs in variants or ():
            x = open(os.path.join(INSTR, s.name, s.name + '.xml')).read()
            for a, b in subs.items():
                x = x.replace(a, b)
            open(os.path.join(INSTR, s.name, fname), 'w').write(x)
        s.t.save(os.path.join(INSTR, s.name, s.name + '.png'))


def tm_words(pdf, sheet, x0, x1, y0, y1):
    """The figure's words in a box (sheet pt): [(text, x0, x1, y0, y1)], from the PDF's text layer."""
    out = subprocess.run(['pdftotext', '-q', '-f', str(tmart.PAGES[sheet - 1]), '-l', str(tmart.PAGES[sheet - 1]),
                          '-bbox', pdf, '-'], check=True, capture_output=True, text=True).stdout
    words = []
    for m in re.finditer(r'<word xMin="([\d.]+)" yMin="([\d.]+)" xMax="([\d.]+)" yMax="([\d.]+)">([^<]*)</word>', out):
        a, b, c, d = map(float, m.groups()[:4])
        if x0 <= (a + c) / 2 <= x1 and y0 <= (b + d) / 2 <= y1:
            words.append((m.group(5), a, c, b, d))
    return words


CAP_GROUND, CAP_LEGEND = (14, 14, 15), (175, 175, 175)     # a push button's cap, and its legend unlit, by day


class PushPanel(Panel):
    """An integrally lighted panel of lighted push-button switches (TM 3.25.4): its markings are the TM figure's
    linework, with the words set again in bold and lighted (on the nonflight instrument lights), and the buttons are
    3-D: square caps in raised frames, their legends readable by day under the dead front and lighted by the switch
    logic (lit legends on the LIGHTED SWITCHES dimmer), pressing in while clicked. Positions in figure pt from the
    item's centre, y right and z down as on the figure."""
    FRAME_H, CAP_H, TRAVEL = 0.0025, 0.0018, 0.0012    # m: frames' and caps' height over the face, a press's travel
    GAP = 0.15                                          # pt: a cap's clearance in its frame

    def __init__(s, name, art, num, side, depth, border, frames, lines=(), room=400):
        """border: (y0, y1, z0, z1) inside the panel's edge, the figure's outline outside it erased; frames:
        [(y0, y1, z0, z1)] the raised frames (the figure's art under them erased); lines: [((y0, z0), (y1, z1))]
        markings drawn again across the frames."""
        Panel.__init__(s, name, art, num, side, depth, room=room)
        s.integral = True
        s.frames, s.holes = frames, []
        pw, ph = s.pw, s.ph
        x0, x1, y0, y1 = s.x0, s.x1, s.y0, s.y1
        k = pw / (x1 - x0)                                   # px per pt
        px = lambda y, z: ((y + s.cx - x0) * k, (z + s.cy - y0) * k)
        im = art.vector(x0, x1, y0, y1, s.sheet, pw, ph).convert('L')
        d = ImageDraw.Draw(im)
        words = tm_words(art.pdf, s.sheet, x0, x1, y0, y1)
        for w, a, b, c, e in words:                          # the figure's words, set again below
            m = max(3, 0.2 * k)                               # px: the glyphs reach a little past their boxes
            d.rectangle([(a - x0) * k - m, (c - y0) * k - m, (b - x0) * k + m, (e - y0) * k + m], fill=255)
        by0, by1, bz0, bz1 = border
        q0, q1 = px(by0, bz0), px(by1, bz1)
        mask = Image.new('L', im.size, 255); ImageDraw.Draw(mask).rectangle([q0[0], q0[1], q1[0], q1[1]], fill=0)
        im.paste(255, (0, 0), mask)                          # the outline: the panel's own edge shows it
        for fy0, fy1, fz0, fz1 in frames:
            a, b = px(fy0 - 0.3, fz0 - 0.3), px(fy1 + 0.3, fz1 + 0.3)
            d.rectangle([a[0], a[1], b[0], b[1]], fill=255)
        for (ly0, lz0), (ly1, lz1) in lines:
            d.line([px(ly0, lz0), px(ly1, lz1)], fill=0, width=max(1, int(round(0.28 * k))))
        face = tmart.light(im, GROUND, LETTER).convert('RGB')
        df = ImageDraw.Draw(face)
        inside = lambda wy, wz: any(fy0 <= wy <= fy1 and fz0 <= wz <= fz1 for fy0, fy1, fz0, fz1 in frames)
        for w, a, b, c, e in words:
            wy, wz = (a + b) / 2 - s.cx, (c + e) / 2 - s.cy
            if inside(wy, wz):
                continue                                      # a cap's legend: on the cap
            size = (e - c) * k * 0.92
            f = font(size)
            l, t, r, bot = df.textbbox((0, 0), w, font=f, anchor='mm')
            if r - l > 1.10 * (b - a) * k:
                f = font(size * 1.10 * (b - a) * k / (r - l))
            df.text((((a + b) / 2 - x0) * k, ((c + e) / 2 - y0) * k), w, font=f, fill=(LETTER,) * 3, anchor='mm')
        s.t.im.paste(face, (0, 0))
        s.lit = ['panel', 'frames']
        s.obj('frames')                                      # the frames (write: round the caps), the face's markings on top

    def frame_box(s, o, fy0, fy1, fz0, fz1, holes=None):
        """A raised frame (pt), or its parts between holes."""
        (a0, b0), (a1, b1) = s.at(fy0, fz0), s.at(fy1, fz1)
        holes = sorted(h for h in s.holes if fy0 <= (h[0] + h[1]) / 2 <= fy1 and fz0 <= (h[2] + h[3]) / 2 <= fz1) \
            if holes is None else holes
        if not holes:
            return
        y0, y1, z0, z1 = min(a0, a1), max(a0, a1), min(b0, b1), max(b0, b1)
        hs = [(min(s.at(h[0], 0)[0], s.at(h[1], 0)[0]), max(s.at(h[0], 0)[0], s.at(h[1], 0)[0]),
               min(s.at(0, h[2])[1], s.at(0, h[3])[1]), max(s.at(0, h[2])[1], s.at(0, h[3])[1])) for h in holes]
        hz0, hz1 = min(h[2] for h in hs), max(h[3] for h in hs)
        top = s.depth + s.FRAME_H
        A.box(o, y0, y1, hz1, z1, s.depth, top, s.face_uv, s.paint)          # above the holes
        A.box(o, y0, y1, z0, hz0, s.depth, top, s.face_uv, s.paint)          # below
        edges = [y0] + [c for h in hs for c in (h[0], h[1])] + [y1]
        for a, b in zip(edges[0::2], edges[1::2]):                            # between them
            if b > a:
                A.box(o, a, b, hz0, hz1, s.depth, top, s.face_uv, s.paint)

    def write(s, *args, **kw):
        for f in s.frames:
            s.frame_box(s.parts['frames'], *f)
        Panel.write(s, *args, **kw)

    def cap(s, key, y, z, w, h, lines, split=False):
        """A square cap (pt) at (y, z) with its legend as it reads unlit: one or two lines, or with `split` a line
        each over and under a rule. Returns its centre and half sizes (m)."""
        cy, cz = s.at(y, z); hw, hh = w * PT / 2, h * PT / 2
        pw = 160; ph = int(pw * h / w)
        x0, py0, x1, py1 = s.alloc(pw, ph)
        dr = s.t.d
        dr.rectangle([x0, py0, x1, py1], fill=CAP_GROUND)
        if split:
            dr.line([x0 + 8, (py0 + py1) / 2, x1 - 8, (py0 + py1) / 2], fill=(90, 90, 90), width=3)
            f = font(ph * 0.26)
            dr.text(((x0 + x1) / 2, py0 + ph * 0.27), lines[0], font=f, fill=CAP_LEGEND, anchor='mm')
            dr.text(((x0 + x1) / 2, py0 + ph * 0.74), lines[1], font=f, fill=CAP_LEGEND, anchor='mm')
        else:
            f = font(ph * (0.26 if len(lines) > 1 else 0.30))
            for k, line in enumerate(lines):
                yy = py0 + ph * ((k + 1) / (len(lines) + 1) if len(lines) > 1 else 0.5)
                dr.text(((x0 + x1) / 2, yy), line, font=f, fill=CAP_LEGEND, anchor='mm')
        s.holes.append((y - w / 2 - s.GAP, y + w / 2 + s.GAP, z - h / 2 - s.GAP, z + h / 2 + s.GAP))
        o = s.obj('btn-' + key)
        A.box(o, cy - hw, cy + hw, cz - hh, cz + hh, s.depth, s.depth + s.CAP_H,
              s.t.area(x0, py0, x1, py1, cy - hw, cy + hw, cz - hh, cz + hh), s.knob_side)
        return cy, cz, hw, hh

    def press_xml(s, objs, prop):
        """The cap (and its lit legends) in by TRAVEL while `prop` is 1."""
        return ('\n  <animation>\n    <type>translate</type>\n%s    <property>%s</property>\n    <factor> %g </factor>\n'
                '    <axis>\n      <x> 1 </x>\n      <y> 0 </y>\n      <z> 0 </z>\n    </axis>\n  </animation>\n'
                % (objects_xml(objs), prop, -s.TRAVEL))


def select_xml(objs, cond):
    return ('\n  <animation>\n    <type>select</type>\n%s    <condition>\n      %s\n    </condition>\n  </animation>\n'
            % (''.join('    <object-name>%s</object-name>\n' % o for o in objs), cond))


def emission_xml(objs, factor=None):
    f = '      <factor-prop>%s</factor-prop>\n' % factor if factor else ''
    return ('\n  <animation>\n    <type>material</type>\n%s    <emission>\n%s'
            '      <red> 1 </red>\n      <green> 1 </green>\n      <blue> 1 </blue>\n    </emission>\n  </animation>\n'
            % (''.join('    <object-name>%s</object-name>\n' % o for o in objs), f))


def pick_xml(obj, bindings, tooltip, release=None):
    """bindings: [(mouse button, nasal script)]; release: script on button up."""
    act = ''
    for button, script in bindings:
        act += ('    <action>\n      <button>%d</button>\n      <repeatable>false</repeatable>\n      <binding>\n'
                '        <command>nasal</command>\n        <script>%s</script>\n      </binding>\n' % (button, script))
        if release:
            act += ('      <mod-up>\n        <binding>\n          <command>nasal</command>\n          <script>%s</script>\n'
                    '        </binding>\n      </mod-up>\n' % release)
        act += '    </action>\n'
    return ('\n  <animation>\n    <type>pick</type>\n    <object-name>%s</object-name>\n%s    <hovered>\n      <binding>\n'
            '        <command>set-tooltip</command>\n        <tooltip-id>%s</tooltip-id>\n        <label>%s</label>\n'
            '      </binding>\n    </hovered>\n  </animation>\n' % (obj, act, obj, tooltip))


def rot_table_xml(obj, prop, table, centre, axis=(-1, 0, 0)):
    return ('\n  <animation>\n    <type>rotate</type>\n    <object-name>%s</object-name>\n    <property>%s</property>\n'
            '    <interpolation>\n%s    </interpolation>\n    <center>\n      <x-m> %.4f </x-m>\n      <y-m> %.4f </y-m>\n'
            '      <z-m> %.4f </z-m>\n    </center>\n    <axis>\n      <x> %g </x>\n      <y> %g </y>\n      <z> %g </z>\n'
            '    </axis>\n  </animation>\n' % ((obj, prop, ''.join('      <entry><ind> %g </ind><dep> %g </dep></entry>\n' % e
                                                                for e in table)) + tuple(centre) + tuple(axis)))


PWR = '<property>/controls/electric/battery-switch</property>'
LEGEND = (200, 255, 200)                                       # NVG-green panel legends


def panels(art):
    UH = '/instrumentation/uh60/'
    # VSI/HSI mode selector (TM 3.25.4, figure 2-9): two rows of four lighted push buttons, each row in its frame,
    # three dividers running through them; the top row the navigation source, the bottom row split legends
    p = PushPanel('modesel', art, 11, 'pilot', 0.005, border=(-25.6, 25.6, -26.4, 26.4),
                  frames=[(-24.07, 24.10, -11.98, -0.23), (-24.07, 24.10, 0.67, 12.43)],
                  lines=[((y, -12.1), (y, 12.6)) for y in (-12.50, -0.57, 11.52)], room=600)
    p.lamps = 'switches'                            # TM 2.70.4: the LIGHTED SWITCHES dimmer
    cw = 8.3
    top = (('dplr', ['DPLR']), ('vor-ils', ['VOR', 'ILS']), ('back-crs', ['BACK', 'CRS']), ('fm-home', ['FM', 'HOME']))
    bottom = (('turn-altr', 'NORM', 'ALTR'), ('crs-hdg', 'PLT', 'CPLT'), ('vgyro-altr', 'NORM', 'ALTR'), ('brg2-vor', 'ADF', 'VOR'))
    lift = p.depth + p.CAP_H + 0.0002
    for x, (key, lines) in zip((-17.9, -6.4, 5.8, 17.6), top):
        cy, cz, hw, hh = p.cap(key, x, -5.9, cw, cw, lines)
        p.legend('lit-' + key, cy - 0.84 * hw, cy + 0.84 * hw, cz - 0.84 * hh, cz + 0.84 * hh, lift, lines, LEGEND,
                 '%s<property>%smodesel[0]/%s</property>' % (PWR, UH, key))
        press = '%smodesel[0]/press-%s' % (UH, key)
        p.xml += p.press_xml(['btn-' + key, 'lit-' + key], press)
        p.xml += pick_xml('btn-' + key, [(0, 'setprop("%s", 1); avionics.modesel(0, "%s")' % (press, key))],
                          'Mode select: ' + ' '.join(lines), release='setprop("%s", 0)' % press)
    for x, (key, a, b) in zip((-17.9, -6.4, 5.8, 17.6), bottom):
        cy, cz, hw, hh = p.cap(key, x, 6.8, cw, cw, [a, b], split=True)
        state = '%scrs-hdg-cplt' % UH if key == 'crs-hdg' else '%smodesel[0]/%s' % (UH, key)
        p.legend('lit-%s-a' % key, cy - 0.84 * hw, cy + 0.84 * hw, cz + 0.08 * hh, cz + 0.88 * hh, lift, [a], LEGEND,
                 '%s<not><property>%s</property></not>' % (PWR, state))
        p.legend('lit-%s-b' % key, cy - 0.84 * hw, cy + 0.84 * hw, cz - 0.88 * hh, cz - 0.08 * hh, lift, [b], LEGEND,
                 '%s<property>%s</property>' % (PWR, state))
        press = '%smodesel[0]/press-%s' % (UH, key)
        p.xml += p.press_xml(['btn-' + key, 'lit-%s-a' % key, 'lit-%s-b' % key], press)
        p.xml += pick_xml('btn-' + key, [(0, 'setprop("%s", 1); avionics.modesel(0, "%s")' % (press, key))],
                          'Mode select: %s / %s' % (a, b), release='setprop("%s", 0)' % press)
    p.write('VSI/HSI mode selector, pilot\'s (Nasal/avionics.nas)', variants=[('modesel-copilot.xml', {
        'modesel[0]': 'modesel[1]', 'avionics.modesel(0,': 'avionics.modesel(1,',
        'VSI/HSI mode selector, pilot\'s': 'VSI/HSI mode selector, copilot\'s'})])

    # CIS mode selector: HDG, NAV, ALT, each cap split over its ON legend, in its own frame
    p = PushPanel('cismode', art, 10, 'pilot', 0.005, border=(-27.0, 27.0, -12.2, 12.2),
                  frames=[(-20.53, -8.70, -6.80, 5.65), (-6.20, 5.45, -6.80, 5.65), (8.30, 20.30, -6.80, 5.75)], room=500)
    p.lamps = 'switches'
    lift = p.depth + p.CAP_H + 0.0002
    for x, key in zip((-14.43, -0.18, 14.15), ('hdg', 'nav', 'alt')):
        cy, cz, hw, hh = p.cap(key, x, -0.53, 8.8, 8.7, [key.upper(), 'ON'], split=True)
        p.legend('lit-' + key, cy - 0.84 * hw, cy + 0.84 * hw, cz - 0.88 * hh, cz - 0.08 * hh, lift, ['ON'], LEGEND,
                 '%s<property>%scis/%s-on</property>' % (PWR, UH, key))
        press = '%scis/press-%s' % (UH, key)
        p.xml += p.press_xml(['btn-' + key, 'lit-' + key], press)
        p.xml += pick_xml('btn-' + key, [(0, 'setprop("%s", 1); avionics.press("%s")' % (press, key))],
                          'CIS mode: ' + key.upper(), release='setprop("%s", 0)' % press)
    p.write('CIS mode selector (Nasal/avionics.nas)')

    # auxiliary fuel management panel: no external tanks are fitted, its displays stay blank; the switches move
    p = Panel('afms', art, 21, 'centre', 0.005)
    AF = '/controls/fuel/afms/'
    for x, y, key, label in ((-19.1, 10.4, 'xfer-mode', 'XFER MODE: AUTO / OFF / MAN'),
                             (-5.3, 10.4, 'man-xfer', 'MAN XFER: LEFT / BOTH / RIGHT'),
                             (7.0, 9.8, 'xfer-from', 'XFER FROM: INBD / OUTBD')):
        cy, cz = p.toggle(x, y, name='sw-' + key, straight=True)
        p.xml += rot_table_xml('sw-' + key, AF + key, [(-1, -35), (0, 0), (1, 35)], (p.depth, cy, cz), (0, -1, 0))
        lo = 0 if key == 'xfer-from' else -1
        p.xml += pick_xml('sw-' + key, [(0, 'avionics.step("%s%s", 1, %d, 1)' % (AF, key, lo)),
                                        (2, 'avionics.step("%s%s", -1, %d, 1)' % (AF, key, lo))],
                          label + ' (left click up, right click down; no tanks fitted)')
    cy, cz = p.knob(21.0, 10.3, 9, name='sw-press')
    p.xml += rot_table_xml('sw-press', AF + 'press', [(0, -45), (1, 0), (2, 45), (3, 90)], (p.depth, cy, cz))
    p.xml += pick_xml('sw-press', [(0, 'avionics.step("%spress", 1, 0, 3)' % AF), (2, 'avionics.step("%spress", -1, 0, 3)' % AF)],
                      'PRESS: ALL / OUTBD / INBD / OFF (no tanks fitted)')
    for x in (-19.0, -6.5, 6.5, 19.0):                         # QTY displays, blank
        cy, cz = p.at(x, -6.9)
        A.box(p.obj('displays'), cy - 4.6 * PT, cy + 4.6 * PT, cz - 2.1 * PT, cz + 2.1 * PT, p.depth, p.depth + 0.0004, p.dark, p.dark)
    p.write('auxiliary fuel management panel: no external tanks are fitted, its displays stay blank')

    p = Panel('rwr', art, 20, 'centre', 0.006)
    p.knob(20.2, -17.6, 9); p.lamp(-21.5, -18.1, 5, (70, 20, 15)); p.glass(0, 0, 34.0)
    p.write('radar warning indicator (no threats are simulated)')

    # ice rate meter, blade deice control and test panels, IRCM (TM 2.54, 4.8; Nasal/avionics.nas)
    D = '/controls/anti-ice/blade-deice/'
    p = Panel('deice', art, 0, 'centre', 0.003)
    mcy, mcz = p.at(-0.2, -52.7)
    o = p.obj('lwc-needle')
    A.flat(o, [(mcy - 0.0007, mcz - 0.003), (mcy + 0.0007, mcz - 0.003), (mcy + 0.0004, mcz + 0.020), (mcy - 0.0004, mcz + 0.020)],
           p.depth + 0.0008, p.white)
    A.cylinder(o, 0.0018, mcy, mcz, p.depth + 0.0008, p.depth + 0.0012, p.knob_top, p.knob_top)
    p.xml += rot_table_xml('lwc-needle', D + 'meter-lwc', [(0, -85), (0.25, -71), (0.5, -53), (1.0, -3), (1.5, 52), (2.0, 80)],
                           (p.depth, mcy, mcz))
    p.lit.append('lwc-needle')
    p.glass(-0.2, -52.7, 29.6)
    p.button(-29.6, -52.8, 8.3, 12.0, name='btn-lwc-test')
    p.xml += pick_xml('btn-lwc-test', [(0, 'avionics.ice_meter_test()')], 'Ice rate meter: PRESS TO TEST')
    cy, cz = p.toggle(-19.7, -0.7, name='sw-power', straight=True)
    p.xml += rot_table_xml('sw-power', D + 'power', [(0, 0), (1, 35), (2, -35)], (p.depth, cy, cz), (0, -1, 0))
    p.xml += pick_xml('sw-power', [(0, 'avionics.deice_step(1)'), (2, 'avionics.deice_step(-1)')],
                      'Blade deice POWER: ON / OFF / TEST (left click up, right click down)')
    p.lamp(0.9, -0.6, 10, (25, 45, 25), name='lamp-test', lit=(60, 255, 80),
           cond='<property>%stest-in-progress</property>' % D)
    cy, cz = p.knob(20.0, -1.1, 14, name='sw-mode')
    p.xml += rot_table_xml('sw-mode', D + 'mode', [(0, -28), (1, 31), (2, 77), (3, 120)], (p.depth, cy, cz))
    p.xml += pick_xml('sw-mode', [(0, 'avionics.step("%smode", 1, 0, 3)' % D), (2, 'avionics.step("%smode", -1, 0, 3)' % D)],
                      'Blade deice MODE: AUTO / MANUAL T, L, M')
    cy, cz = p.knob(-15.2, 32.2, 14, name='sw-test-select')
    p.xml += rot_table_xml('sw-test-select', D + 'test-select', [(0, -50), (1, -71), (2, -91), (3, -108), (4, -126)],
                           (p.depth, cy, cz))
    p.xml += pick_xml('sw-test-select', [(0, 'avionics.step("%stest-select", -1, 0, 4)' % D),
                                         (2, 'avionics.step("%stest-select", 1, 0, 4)' % D)],
                      'BLADE DE-ICE TEST: NORM / SYNC 1 / SYNC 2 / OAT / EOT')
    for x, key in ((9.8, 'pwr-main'), (26.3, 'pwr-tail')):
        p.lamp(x, 32.3, 10, (50, 40, 15), name='lamp-' + key, lit=AMBER_LAMP, cond='<property>%s%s</property>' % (D, key))
        p.xml += pick_xml('lamp-' + key, [(0, 'setprop("%spress-pwr", 1);' % D)], 'PWR monitor light: press to test',
                          release='setprop("%spress-pwr", 0);' % D)
    cy, cz = p.toggle(23.2, 61.4, name='sw-ircm', straight=True)
    p.xml += rot_table_xml('sw-ircm', '/controls/ircm/on', [(0, -35), (1, 35)], (p.depth, cy, cz), (0, -1, 0))
    p.xml += pick_xml('sw-ircm', [(0, 'avionics.step("/controls/ircm/on", 1, 0, 1)'), (2, 'avionics.step("/controls/ircm/on", -1, 0, 1)')],
                      'IRCM: ON / OFF (left click on, right click off)')
    p.write('ice rate meter, blade deice control and test panels, IRCM control panel (Nasal/avionics.nas)')

    # dimming controls: master warning and caution/advisory lights (NVG panel), radar altimeter display
    L = '/controls/lighting/'
    def dimmer(p, x, y, prop, label):
        cy, cz = p.knob(x, y, 7, 0.007, name='knob-' + prop)
        p.xml += rot_table_xml('knob-' + prop, L + prop, [(0.1, -135), (1, 135)], (p.depth, cy, cz))
        up, dn = 'avionics.step("%s%s", 0.1, 0.1, 1)' % (L, prop), 'avionics.step("%s%s", -0.1, 0.1, 1)' % (L, prop)
        p.xml += pick_xml('knob-' + prop, [(0, up), (2, dn), (3, up), (4, dn)], label)
    p = Panel('nvgdim', art, 26, 'copilot', 0.0015)
    dimmer(p, -6.7, 0.4, 'ma-wrn-norm', 'MA WRN dimming'); dimmer(p, 6.4, 0.2, 'caut-advsy-norm', 'CAUT/ADVSY dimming')
    p.write('NVG dimming controls: master warning and caution/advisory light intensity')
    p = Panel('radaltdim', art, 27, 'copilot', 0.0015)
    dimmer(p, 0, 0, 'radalt-norm', 'RAD ALT DIMMING')
    p.write('radar altimeter dimming control')

    # engine ignition: a key switch, OFF up and ON to the right, on a plate with its OFF, ON and ENGINE IGNITION labels
    p = Panel('ignition', art, 22, 'centre', 0.004, box=(474.0, 512.0, 218.0, 262.0))
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

    # placards, decals on the face plate (figure 2-9 items 8, 12, 23, 25)
    for name, num, side, comment in (('stabplac', 8, 'pilot', 'stabilator position placard'),
                                     ('radiocall', 12, 'pilot', 'radio call placard'),
                                     ('securepl', 25, 'centre', 'secure warning placard'),
                                     ('radiosel', 23, 'centre', 'radio select placard')):
        p = Panel(name, art, num, side, 0.0006, px_mm=20)
        p.lit = []                                   # placards aren't lit
        p.write(comment)


AMBER_LAMP = (255, 176, 0)


if __name__ == '__main__':
    art = tmart.Vec(sys.argv[1])
    vsi(); clock(); stab(); panels(art)
    import cap
    cap.build(art)
