"""The rotors' models driven blade by blade by the flight model, with a motion blur of the blades' own geometry.

The original model turned all four blades together at the rotor's rpm, with the same coning and collective pitch,
and swapped them for blurred-blade textures above 60 rpm and a disc of translucent wedges above 150. Here:

- the hub turns to the flight model's rotor azimuth (/rotors/main/blade[0]/position-deg), and each blade has its own
  lag, flap and pitch (collective and cyclic) from the flight model: /rotors/main/blade[k]/lag-deg, flap-deg,
  pitch-deg (position-deg and flap-deg are FlightGear's multiplayer rotor properties), plus the static droop along
  the span that the centrifugal force takes out (/rotors/main/cone2-4-deg);
- as the rotor speeds up each blade trails a fan of STRIPS faint wedges (in blade-rotor.ac) over SWEEP deg per rpm, which
  reaches the next blade at flight rpm: a translucent disc. IMAGE of its opacity is a smear at the blade's own
  position, fading over SMEAR wedges behind it, the rest even round the rotor: the blades pass a point 17 times a
  second, near the eye's flicker fusion, and the display samples the smears as a camera does, so the blades show as a
  pattern travelling round the disc and its shadow. Each wedge is twice as wide as their spacing, so that every point
  lies under two of them, the even part of the first and last at half opacity, so that at flight rpm the fans tile;
  along the span, in PIECES steps, the disc is as opaque on average as the blades' time-averaged coverage there (the
  solidity N c / 2 pi r: denser at the root) x GAIN, the tip stripes' colour over the tip; the wedges lag and flap with the blade, but don't pitch (the blur lies in the rotor plane), and narrow
  with the rpm as they close up. Opaque below SOLID_RPM (it casts its shadow), the blade then fades out to nothing at
  FADE_RPM as the fan fades in, both with the rpm and casting partial shadows (Effects/RotorBlur: the HDR pipeline;
  elsewhere they switch, without shadows);
- the original's blur and disc objects are hidden;
- the livery picks the blades' paint: sim/model/livery/blades/main, tail and blur name the textures (the striped
  mainrotor.png, tailblade.png and blur.png, or their -plain copies without the tip stripes), set in every livery file
  since the livery selector only adds properties.

In FlightGear's model frame (x aft, y right, z up) the blade lies along +x with its leading edge toward +y; a rotate
animation turns by the right-hand rule, and the first animation of an object is the outermost (the hub's side).

    python3 rotorvis.py         writes UH-60A/Models/Externals/MainRotor/blade.xml, blade-rotor.ac, blur.png, mainrotor.xml
                                and TailRotor/blade.xml, blade-rotor.ac, blur.png, tailrotor.xml, tailblade.png,
                                and the stripeless mainrotor-plain.png, tailblade-plain.png, blur-plain.png
"""
import os, re

HERE = os.path.dirname(os.path.abspath(__file__))
DIR = os.path.join(HERE, '..', 'UH-60A', 'Models', 'Externals', 'MainRotor')

STRIPS = 24                             # wedges per blade
SWEEP = 90.0 / 258.0                    # deg per rpm: the fan reaches the next blade at 100 % rotor speed (258 rpm)
GAIN = 2.5                              # x the blades' time-averaged coverage
IMAGE, SMEAR = 0.5, 2.0                 # the share of it in the blades' smears; their decay, in wedges (~8 deg)
SOLID_RPM, FADE_RPM = 20, 160          # the opaque blade below; from it to FADE_RPM the blade fades out, the fan in
ROOT, STRIPE, TIP_R, PIECES = 0.834, 7.55, 8.123, 16             # m: blade root, tip stripes from, tip; radial steps
SPAN = [(ROOT * (STRIPE / ROOT) ** (i / PIECES), ROOT * (STRIPE / ROOT) ** ((i + 1) / PIECES)) for i in range(PIECES)] \
    + [(STRIPE, TIP_R)]                 # log-spaced (the opacity goes as 1/r), then the tip stripes
CHORD = 0.505                           # m (blade.ac)
DARK, TIP = (0.10, 0.10, 0.10), (0.85, 0.50, 0.50)              # the blade's paint; its red and white tip stripes, mixed
HINGE = (0.54075, 0.0, 0.0)             # the flap and lag hinge (the original's cone axis 0)
PITCH_AXIS = (0.73449, 0.0, 0.0)        # the pitch bearing
PITCH_OFFSET = -7.5                     # deg: the model's blades are built at 7.5 deg of pitch
BENDS = [((2.95301, -0.03895, -0.00338), 'sim/multiplay/generic/float[21]', 2),   # span bends: centre, droop, from
         ((4.66743, -0.04142, -0.00673), 'sim/multiplay/generic/float[22]', 3),   # segment n out (the multiplayer
         ((6.38185, -0.04390, -0.01008), 'sim/multiplay/generic/float[23]', 4)]   # aliases of /rotors/main/cone2-4-deg)
SEGMENTS = ['blade1', 'blade2', 'blade3', 'blade4']
CUFF = ['attache', 'attache2', 'attache3', 'attache4']
LINKS = ['bielle1', 'bielle2']
OLD_BLUR = ['propblur1', 'propblur2', 'propblur3', 'propblur4', 'propdisc1', 'propdisc2', 'propdisc3', 'propdisc4']


def strip(k):
    return 'blur%d' % k


TBLADE = ['t' + s for s in SEGMENTS]     # the blade's transparent copy, which fades out


def rotor_ac():
    """blade-rotor.ac: blade.ac's objects, the fan and the blade's fading copy (fan_ac)."""
    fan_ac(DIR, SEGMENTS, SPAN, CHORD, 4, GAIN, lambda n: TIP if n == len(SPAN) - 1 else DARK)


def fan_ac(folder, segments, span, chord, nb, gain, colour, retex=None):
    """blade-rotor.ac in folder: blade.ac's objects, the fan's wedges (all at the blade: the blade's XML turns them back)
    and the blade's fading copy (segments, renamed t...), in one model so that the XML's effects reach them. The fan
    spans 360 / nb deg at flight rpm; along the span (pieces in span, colour(n)) the disc is as opaque on average as
    the blades' time-averaged coverage x gain. The wedges' colour and opacity are a swatch of blur.png each: the HDR
    transparent shader takes its alpha from the texture, not from the ac material. retex: (texture, uv(x, y)) maps the
    blade segments onto a texture of their own (x along the span, y across it, m)."""
    import math
    from PIL import Image
    d = math.radians(360.0 / nb / (STRIPS - 1))             # their spacing at flight rpm; each 2 d wide
    tex = Image.new('RGBA', (256, 256), (0, 0, 0, 0)); cells = []
    plain = tex.copy()                                      # blur-plain.png: the tip as the rest of the blade
    g = [math.exp(-k / SMEAR) for k in range(STRIPS)]
    seen = {}
    def swatch(rgb, a):
        key = (rgb, int(round(255 * a)))
        if key in seen:
            return seen[key]
        n = len(cells); x, y = 8 * (n % 32), 8 * (n // 32)
        assert n < 1024, 'blur.png is full'
        tex.paste(tuple(int(round(255 * c)) for c in rgb) + (int(round(255 * a)),), (x, y, x + 8, y + 8))
        plain.paste(tuple(int(round(255 * c)) for c in (DARK if rgb == TIP else rgb)) + (int(round(255 * a)),), (x, y, x + 8, y + 8))
        cells.append(((x + 4) / 256.0, 1 - (y + 4) / 256.0)); seen[key] = cells[-1]
        return cells[-1]
    text = open(os.path.join(folder, 'blade.ac'), encoding='latin-1').read()
    head, *orig = re.split(r'(?m)^(?=OBJECT poly)', text)
    mats = [l for l in head.split('\n') if l.startswith('MATERIAL')]
    mats.append('MATERIAL "blur" rgb 1 1 1  amb 1 1 1  emis 0 0 0  spec 0 0 0  shi 0  trans 0')
    blur_mat = len(mats) - 1
    def mapped(b):
        if not retex or re.search(r'^name "([^"]+)"', b, re.M).group(1) not in segments:
            return b
        tex_name, uvf = retex
        L = b.rstrip('\n').split('\n')
        loc = [float(x) for x in next(l for l in L if l.startswith('loc')).split()[1:]]
        i = next(k for k, l in enumerate(L) if l.startswith('numvert')); n = int(L[i].split()[1])
        V = [[float(c) + o for c, o in zip(L[i + 1 + k].split(), loc)] for k in range(n)]
        for k in range(i + n + 1, len(L)):
            m = re.match(r'^(\d+) \S+ \S+$', L[k])
            if m:
                x, _, ny = V[int(m.group(1))]                 # ac (x, z, -y)
                L[k] = '%s %.5f %.5f' % ((m.group(1),) + uvf(x, -ny))
        L = [('texture "%s"' % tex_name) if l.startswith('texture') else l for l in L]
        return '\n'.join(L)
    orig = [mapped(b) for b in orig]
    objs = [b.rstrip('\n') for b in orig]
    for k in range(STRIPS):
        w = (1 - IMAGE) * (0.5 if k in (0, STRIPS - 1) else 1.0) + IMAGE * STRIPS * g[k] / sum(g)   # mean 1
        faces = []
        for n, (r0, r1) in enumerate(span):
            sigma = nb * chord / (2 * math.pi * math.sqrt(r0 * r1))
            a = min(0.9, gain * sigma / 2 * w)              # two wedges over each point
            uv = swatch(colour(n), a)
            pts = [(r0, 0.0), (r1, 0.0), (r1 * math.cos(-2 * d), r1 * math.sin(-2 * d)), (r0 * math.cos(-2 * d), r0 * math.sin(-2 * d))]
            faces.append((uv, pts))
        V = [pt for _, pts in faces for pt in pts]
        lines = ['OBJECT poly', 'name "%s"' % strip(k), 'loc 0 0 0', 'texture "blur.png"', 'texrep 1 1', 'numvert %d' % len(V)]
        lines += ['%.5f 0 %.5f' % (x, -y) for x, y in V]           # FlightGear (x, y, z = 0) -> ac (x, z, -y)
        lines.append('numsurf %d' % len(faces))
        for f, (uv, _) in enumerate(faces):
            lines += ['SURF 0x30', 'mat %d' % blur_mat, 'refs 4'] + ['%d %.5f %.5f' % ((4 * f + q,) + uv) for q in range(4)]
        objs.append('\n'.join(lines + ['kids 0']))
    for b in orig:
        m = re.search(r'^name "([^"]+)"', b, re.M)
        if m and m.group(1) in segments:
            objs.append(re.sub(r'(?m)^name "[^"]+"', 'name "t%s"' % m.group(1), b).rstrip('\n'))
    tex.save(os.path.join(folder, 'blur.png')); plain.save(os.path.join(folder, 'blur-plain.png'))
    out = ['AC3Db'] + mats + ['OBJECT world', 'kids %d' % len(objs)] + objs
    open(os.path.join(folder, 'blade-rotor.ac'), 'w', encoding='latin-1').write('\n'.join(out) + '\n')


def objects(names, indent='    '):
    return ''.join('%s<object-name>%s</object-name>\n' % (indent, n) for n in names)


def rotate(objs, prop, axis, centre=(0, 0, 0), factor=None, offset=None, alias=False, comment=None):
    out = '\n  <!-- %s -->\n' % comment if comment else '\n'
    out += '  <animation>\n    <type>rotate</type>\n' + objects(objs)
    out += ('    <property alias="%s"/>\n' % prop) if alias else ('    <property>%s</property>\n' % prop)
    if factor is not None:
        out += '    <factor> %g </factor>\n' % factor
    if offset is not None:
        out += '    <offset-deg> %g </offset-deg>\n' % offset
    out += ('    <center>\n      <x-m> %.5f </x-m>\n      <y-m> %.5f </y-m>\n      <z-m> %.5f </z-m>\n    </center>\n'
            '    <axis>\n      <x> %g </x>\n      <y> %g </y>\n      <z> %g </z>\n    </axis>\n  </animation>\n'
            % (tuple(centre) + tuple(axis)))
    return out


def select(objs, cond, comment=None):
    out = '\n  <!-- %s -->\n' % comment if comment else '\n'
    return out + '  <animation>\n    <type>select</type>\n' + objects(objs) + '    <condition>\n%s    </condition>\n  </animation>\n' % cond


def fade(objs, f, t, rough, shadow, comment, rpm=None):
    """The rotor-blur effect on objs, faded from f to t rpm (of /rotors/main/rpm, or rpm)."""
    use = '      <rotor-rpm type="float"><use>%s</use></rotor-rpm>\n' % rpm if rpm else ''
    return ('\n  <!-- %s -->\n  <effect>\n    <inherits-from>../../Effects/RotorBlur/rotor-blur</inherits-from>\n'
            '    <parameters>\n%s      <fade-from type="float"> %g </fade-from>\n      <fade-to type="float"> %g </fade-to>\n'
            '      <pbr-roughness type="float"> %g </pbr-roughness>\n      <shadow-gain type="float"> %g </shadow-gain>\n'
            '    </parameters>\n' % (comment, use, f, t, rough, shadow) + objects(objs) + '  </effect>\n')


def livery_textures(objs, prop, default, comment):
    """objs' texture from the livery, sim/model/livery/blades/prop (else default): the pbr animation for the HDR pipeline,
    the material one for the others; the pbr one has to come first."""
    return ('\n  <!-- %s -->\n  <animation>\n    <type>pbr</type>\n' % comment + objects(objs) +
            '    <property-base>sim/model/livery/blades</property-base>\n'
            '    <base-color-texture-prop>%s</base-color-texture-prop>\n  </animation>\n' % prop +
            '  <animation>\n    <type>material</type>\n' + objects(objs) +
            '    <property-base>sim/model/livery/blades</property-base>\n    <texture-prop>%s</texture-prop>\n'
            '    <texture>%s</texture>\n  </animation>\n' % (prop, default))


def plain_main():
    """mainrotor-plain.png: mainrotor.png with its tip stripes (the red and white block the blades' tip segment maps
    onto) in the blades' black."""
    from PIL import Image
    im = Image.open(os.path.join(DIR, 'mainrotor.png')).convert('RGB'); px = im.load()
    for y in range(im.size[1]):
        for x in range(im.size[0]):
            r, g, b = px[x, y]
            if r > 200 and (g < 60 or g > 240) and abs(g - b) < 40:   # red or white
                px[x, y] = (0, 0, 0)
    im.save(os.path.join(DIR, 'mainrotor-plain.png'))


def scale_xml(objs, prop, nominal, comment):
    return ('\n  <!-- %s -->\n  <animation>\n    <type>scale</type>\n' % comment + objects(objs) +
            '    <property>%s</property>\n    <x-factor> 0 </x-factor>\n    <x-offset> 1 </x-offset>\n'
            '    <y-factor> %.7f </y-factor>\n    <y-offset> 0 </y-offset>\n    <y-min> 0.02 </y-min>\n    <y-max> 1 </y-max>\n'
            '    <z-factor> 0 </z-factor>\n    <z-offset> 1 </z-offset>\n  </animation>\n' % (prop, 1.0 / nominal))


def blade_xml():
    strips = [strip(k) for k in range(STRIPS)]
    x = ('<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- One main rotor blade, written by fdm/rotorvis.py (see there); mainrotor.xml gives each\n'
         '     instance its blade\'s properties in params -->\n\n<PropertyList>\n\n  <path>blade-rotor.ac</path>\n\n'
         '  <params>\n    <lag>rotors/main/blade[0]/lag-deg</lag>\n    <flap>rotors/main/blade[0]/flap-deg</flap>\n'
         '    <pitch>rotors/main/blade[0]/pitch-deg</pitch>\n  </params>\n')
    x += livery_textures(SEGMENTS + TBLADE, 'main', 'mainrotor.png', 'the livery\'s paint: the blade, with or without tip stripes')
    x += livery_textures(strips, 'blur', 'blur.png', 'and its blur')
    x += fade(strips, SOLID_RPM, FADE_RPM, 1.0, 2.0, 'the fan fades in with the rpm; its shadow x 2: two wedges over each point')
    x += fade(TBLADE, FADE_RPM, SOLID_RPM, 0.5, 1.0, 'the blade fades out')
    x += select(OLD_BLUR, '      <property>sim/model/uh60/never</property>\n', 'the original\'s blur and disc: not shown')
    rpm = lambda r: '<property>rotors/main/rpm</property><value> %g </value>' % r
    x += select(SEGMENTS, '      <less-than>%s</less-than>\n' % rpm(SOLID_RPM), 'the opaque blade at low rpm')
    x += select(TBLADE, '      <and>\n        <not><less-than>%s</less-than></not>\n        <less-than>%s</less-than>\n'
                '      </and>\n' % (rpm(SOLID_RPM), rpm(FADE_RPM)), 'then its fading copy')
    x += select(strips, '      <not><less-than>%s</less-than></not>\n' % rpm(SOLID_RPM), 'and the fan')
    for k in range(1, STRIPS):
        x += rotate([strip(k)], 'rotors/main/rpm', (0, 0, 1), factor=-SWEEP * k / (STRIPS - 1),
                    comment='wedge k trails the blade by k / %d of the fan' % (STRIPS - 1) if k == 1 else None)
    blade = SEGMENTS + TBLADE
    x += rotate(blade + strips + CUFF + LINKS, '/params/lag', (0, 0, 1), HINGE, alias=True, comment='lag about the hinge, lead +')
    x += rotate(blade + strips + CUFF, '/params/flap', (0, -1, 0), HINGE, alias=True, comment='flap about the hinge, up +')
    for centre, prop, n in BENDS:
        x += rotate(SEGMENTS[n - 1:] + TBLADE[n - 1:], prop, (0, 1, 0), centre, factor=-1,
                    comment='the static droop along the span (/rotors/main/cone2-4-deg: up +)' if n == 2 else None)
    pitched = blade + ['attache', 'attache2', 'attache4'] + LINKS
    x += rotate(pitched, '/params/pitch', (1, 0, 0), PITCH_AXIS, offset=PITCH_OFFSET, alias=True,
                comment='pitch: collective and cyclic, leading edge up +')
    x += rotate(['bielle2'], '/params/pitch', (-1, 0, 0), (0.58737, 0.20557, 0.0), offset=PITCH_OFFSET, alias=True,
                comment='the pitch link')
    x += scale_xml(strips, 'rotors/main/rpm', 258.0, 'the wedges as wide as twice their spacing at any rpm (blade-rotor.ac: at 258)')
    open(os.path.join(DIR, 'blade.xml'), 'w', encoding='latin-1').write(x + '\n</PropertyList>\n')


def mainrotor_xml():
    hub = ['chapeau', 'assemble', 'plateau', 'pistons', 'axeH', 'axeV']
    x = ('<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- The main rotor, written by fdm/rotorvis.py (see there): the hub at the flight model\'s\n'
         '     rotor azimuth, blade k on FlightGear\'s rotors/main/blade[k] -->\n\n<PropertyList>\n\n  <path>mainrotor.ac</path>\n\n'
         '  <animation>\n' + objects(['chapeau', 'rotor'] + hub[1:]) + '  </animation>\n')
    for k in range(4):
        x += ('\n  <model>\n    <name>Blade%d</name>\n    <path>blade.xml</path>\n    <overlay>\n      <params>\n'
              '        <lag>rotors/main/blade[%d]/lag-deg</lag>\n        <flap>rotors/main/blade[%d]/flap-deg</flap>\n'
              '        <pitch>rotors/main/blade[%d]/pitch-deg</pitch>\n      </params>\n    </overlay>\n'
              % (k + 1, k, k, k))
        if k:
            x += '    <offsets>\n      <heading-deg> %d </heading-deg>\n    </offsets>\n' % (90 * k)
        x += '  </model>\n'
    x += '\n  <animation>\n    <name>rotorcomplet</name>\n' + objects(hub + ['Blade%d' % k for k in range(1, 5)]) + '  </animation>\n'
    x += rotate(['rotorcomplet'], 'rotors/main/blade[0]/position-deg', (0, 0, 1),
                comment='the hub at blade 0\'s azimuth: anticlockwise from above, from the tail')
    open(os.path.join(DIR, 'mainrotor.xml'), 'w', encoding='latin-1').write(x + '\n</PropertyList>\n')


# ---------------------------------------------------------------- the tail rotor

# The same for the tail rotor (Models/Externals/TailRotor): its hub at the flight model's tail rotor azimuth
# (/rotors/tail/blade[0]/position-deg: geared to the main rotor, 4.62 times as fast), the blades at their pitch (pedals;
# the old /rotors/tail/blade[0]/incidence-deg), each blade's flap (/rotors/tail/blade[k]/flap-deg, on the crossbeam's
# centre hinge), no lag, and the same blur, its rpm
# scaled by the gear ratio; the blades painted as the main ones (tail_paint).
TR_DIR = os.path.join(HERE, '..', 'UH-60A', 'Models', 'Externals', 'TailRotor')
TR_RPM = 1190.0                         # 100 %
TR_GAIN = 1.5                           # its solidity is higher than the main rotor's
TR_SOLID, TR_FADE = SOLID_RPM * TR_RPM / 258.0, FADE_RPM * TR_RPM / 258.0
TR_ROOT, TR_TIP = 0.20, 1.50            # m (its blade.ac)
TR_CHORD = 0.31
TR_STRIPE = 1.30                        # m: the tip stripes from here out, as on the main blades (red, white, red, white)
TR_SPAN = [(TR_ROOT * (TR_STRIPE / TR_ROOT) ** (i / 12), TR_ROOT * (TR_STRIPE / TR_ROOT) ** ((i + 1) / 12)) for i in range(12)] \
    + [(TR_STRIPE, TR_TIP)]
TR_CHORD_Y = (-0.156, 0.155)            # m: the blade's chord across y (its blade.ac)


def tail_paint():
    """tailblade.png: the tail blades' paint, the main blades' (mainrotor.png): black, the tip striped red and white; the
    original mapped them onto plain grey (its UVs scrambled, which uniform noise hid); tailblade-plain.png: all black.
    Returns the blades' planar uv."""
    from PIL import Image, ImageDraw
    W, H = 64, 512
    im = Image.new('RGB', (W, H), (12, 12, 12))
    im.save(os.path.join(TR_DIR, 'tailblade-plain.png'))
    d = ImageDraw.Draw(im)
    band = (TR_TIP - TR_STRIPE) / 4
    for k, rgb in enumerate([(236, 0, 0), (255, 255, 255), (236, 0, 0), (255, 255, 255)]):   # from the tip in
        r1 = TR_TIP - k * band; r0 = r1 - band
        y0, y1 = H * (1 - (r1 - TR_ROOT) / (TR_TIP - TR_ROOT)), H * (1 - (r0 - TR_ROOT) / (TR_TIP - TR_ROOT))
        d.rectangle([0, int(round(y0)), W - 1, int(round(y1))], fill=rgb)
    im.save(os.path.join(TR_DIR, 'tailblade.png'))
    c0, c1 = TR_CHORD_Y
    return lambda x, y: (min(1, max(0, (y - c0) / (c1 - c0))), min(1, max(0, (x - TR_ROOT) / (TR_TIP - TR_ROOT))))


def tail_xml():
    strips = [strip(k) for k in range(STRIPS)]
    rpm = lambda r: '<property>rotors/tail/rpm</property><value> %g </value>' % r
    x = ('<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- One tail rotor blade, written by fdm/rotorvis.py (see there); tailrotor.xml gives each\n'
         '     instance its blade\'s flap in params -->\n\n<PropertyList>\n\n  <path>blade-rotor.ac</path>\n\n'
         '  <params>\n    <flap>rotors/tail/blade[0]/flap-deg</flap>\n  </params>\n')
    x += livery_textures(['blade', 'tblade'], 'tail', 'tailblade.png', 'the livery\'s paint: the blade, with or without tip stripes')
    x += livery_textures(strips, 'blur', 'blur.png', 'and its blur')
    x += fade(strips, TR_SOLID, TR_FADE, 1.0, 2.0, 'the fan fades in with the rpm; its shadow x 2: two wedges over each point',
              rpm='/rotors/tail/rpm')
    x += fade(['tblade'], TR_FADE, TR_SOLID, 0.5, 1.0, 'the blade fades out', rpm='/rotors/tail/rpm')
    x += select(['propblur', 'propdisc'], '      <property>sim/model/uh60/never</property>\n', 'the original\'s blur and disc: not shown')
    x += select(['blade'], '      <less-than>%s</less-than>\n' % rpm(TR_SOLID), 'the opaque blade at low rpm')
    x += select(['tblade'], '      <and>\n        <not><less-than>%s</less-than></not>\n        <less-than>%s</less-than>\n'
                '      </and>\n' % (rpm(TR_SOLID), rpm(TR_FADE)), 'then its fading copy')
    x += select(strips, '      <not><less-than>%s</less-than></not>\n' % rpm(TR_SOLID), 'and the fan')
    for k in range(1, STRIPS):
        x += rotate([strip(k)], 'rotors/tail/rpm', (0, 0, 1), factor=-90.0 / TR_RPM * k / (STRIPS - 1),
                    comment='wedge k trails the blade by k / %d of the fan' % (STRIPS - 1) if k == 1 else None)
    x += rotate(['blade', 'tblade'] + strips, '/params/flap', (0, -1, 0), alias=True,
                comment='flap about the centre hinge (the crossbeam), towards the thrust side +')
    x += rotate(['blade', 'tblade'], 'sim/multiplay/generic/float[25]', (1, 0, 0),
                comment='pitch: /rotors/tail/blade[0]/incidence-deg')
    x += scale_xml(strips, 'rotors/tail/rpm', TR_RPM, 'the wedges as wide as twice their spacing at any rpm')
    open(os.path.join(TR_DIR, 'blade.xml'), 'w', encoding='latin-1').write(x + '\n</PropertyList>\n')
    x = ('<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- The tail rotor, written by fdm/rotorvis.py (see there): the hub at the flight model\'s\n'
         '     tail rotor azimuth -->\n\n<PropertyList>\n\n  <path>tailrotor.ac</path>\n\n  <animation>\n    <object-name>rotortail</object-name>\n'
         '  </animation>\n')
    for k in range(4):
        x += ('\n  <model>\n    <name>Blade%d</name>\n    <path>blade.xml</path>\n    <overlay>\n      <params>\n'
              '        <flap>rotors/tail/blade[%d]/flap-deg</flap>\n      </params>\n    </overlay>\n' % (k + 1, k))
        if k:
            x += '    <offsets>\n      <heading-deg> %d </heading-deg>\n    </offsets>\n' % (90 * k)
        x += '  </model>\n'
    x += '\n  <animation>\n    <name>rotorcomplet</name>\n' + objects(['rotortail'] + ['Blade%d' % k for k in range(1, 5)]) + '  </animation>\n'
    x += rotate(['rotorcomplet'], 'rotors/tail/blade[0]/position-deg', (0, 0, 1), comment='the hub at blade 0\'s azimuth')
    open(os.path.join(TR_DIR, 'tailrotor.xml'), 'w', encoding='latin-1').write(x + '\n</PropertyList>\n')


if __name__ == '__main__':
    rotor_ac(); blade_xml(); mainrotor_xml(); plain_main()
    fan_ac(TR_DIR, ['blade'], TR_SPAN, TR_CHORD, 4, TR_GAIN, lambda n: TIP if n == len(TR_SPAN) - 1 else DARK,
           retex=('tailblade.png', tail_paint())); tail_xml()
    print('rotor models written: %d wedges per blade over 90 deg at 100 %% rpm' % STRIPS)
