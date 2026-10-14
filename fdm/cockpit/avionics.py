"""The UH-60A's navigation instruments, after TM 1-1520-237-10 3.25 (figures 3-30, 3-31), driven by
Nasal/avionics.nas (mode selectors and the CIS processor):

- the HSIs, built new (Instruments/hsi60): compass card, heading select marker, course pointer with its deviation bar
  and to-from arrow, No. 1 (Doppler/GPS) and No. 2 (VOR or ADF) bearing pointers, COURSE and KM windows, NAV and HDG
  flags, HDG and CRS knobs;
- the VSIs: the old attitude indicator (Instruments/ai) with the CIS's cyclic roll and pitch command bars and
  collective position indicator driven, and the rest added (Instruments/ai/cis): glide slope and course deviation
  pointers, ATT, CMD, NAV and GS flags, GA, DH and MB lights and the turn rate needle.

Each comes as a pilot's and a copilot's XML (instrumentation/uh60/hsi[n], vsi[n]); panel_face.py places them.

    python3 avionics.py
"""
import math, os, re
from PIL import Image, ImageDraw

import acmesh as A
import instruments as I

U = '/instrumentation/uh60/'
HSI_DIR = os.path.join(I.INSTR, 'hsi60')
AI_DIR = os.path.join(I.INSTR, 'ai')
WHITE, AMBER, RED = (235, 235, 235), (255, 176, 0), (210, 30, 30)
MARK = 'written by fdm/cockpit/avionics.py'


def text_node(name, prop, fmt, size, y, z, d, width=0.03):
    return ('\n  <text>\n    <name>%s</name>\n    <type type="string">number-value</type>\n    <property>%s</property>\n'
            '    <format type="string">%s</format>\n    <truncate type="bool">false</truncate>\n'
            '    <font>Helvetica-Bold.txf</font>\n    <character-size type="double">%g</character-size>\n'
            '    <character-aspect-ratio type="double">0.8</character-aspect-ratio>\n    <axis-alignment>yz-plane</axis-alignment>\n'
            '    <max-height>0.02</max-height>\n    <max-width>%g</max-width>\n    <alignment>center-center</alignment>\n'
            '    <offsets>\n      <x-m> %.4f </x-m>\n      <y-m> %.4f </y-m>\n      <z-m> %.4f </z-m>\n    </offsets>\n  </text>\n'
            % (name, prop, fmt, size, width, d, y, z))


def select(objs, cond):
    return ('\n  <animation>\n    <type>select</type>\n%s    <condition>\n      %s\n    </condition>\n  </animation>\n'
            % (''.join('    <object-name>%s</object-name>\n' % o for o in objs), cond))


def rotate(objs, prop, factor=1, centre=(0, 0, 0)):
    return ('\n  <animation>\n    <type>rotate</type>\n%s    <property>%s</property>\n    <factor> %g </factor>\n'
            '    <center>\n      <x-m> %.4f </x-m>\n      <y-m> %.4f </y-m>\n      <z-m> %.4f </z-m>\n    </center>\n'
            '    <axis>\n      <x> -1 </x>\n      <y> 0 </y>\n      <z> 0 </z>\n    </axis>\n  </animation>\n'
            % ((''.join('    <object-name>%s</object-name>\n' % o for o in objs), prop, factor) + tuple(centre)))


def translate(objs, prop, factor, axis, lo=-1, hi=1):
    return ('\n  <animation>\n    <type>translate</type>\n%s    <property>%s</property>\n    <factor> %g </factor>\n'
            '    <min> %g </min>\n    <max> %g </max>\n    <axis>\n      <x> %g </x>\n      <y> %g </y>\n      <z> %g </z>\n'
            '    </axis>\n  </animation>\n'
            % ((''.join('    <object-name>%s</object-name>\n' % o for o in objs), prop, factor, lo * abs(factor), hi * abs(factor)) + axis))


def emissive(objs, rgb=(1, 1, 1), factor=None):
    f = '      <factor-prop>%s</factor-prop>\n' % factor if factor else ''
    return ('\n  <animation>\n    <type>material</type>\n%s    <emission>\n%s      <red> %g </red>\n      <green> %g </green>\n'
            '      <blue> %g </blue>\n    </emission>\n  </animation>\n'
            % ((''.join('    <object-name>%s</object-name>\n' % o for o in objs), f) + tuple(rgb)))


def knob_pick(obj, prop, label, step=1):
    act = ''
    for button, sign in ((0, 1), (2, -1), (3, 1), (4, -1)):
        for mods, k in (('<not><property>devices/status/keyboard/shift</property></not>', 1),
                        ('<property>devices/status/keyboard/shift</property>', 10)):
            act += ('    <action>\n      <button>%d</button>\n      <repeatable>true</repeatable>\n      <binding>\n'
                    '        <condition>%s</condition>\n        <command>property-adjust</command>\n'
                    '        <property>%s</property>\n        <step>%g</step>\n        <min>0</min>\n        <max>360</max>\n'
                    '        <wrap>true</wrap>\n      </binding>\n    </action>\n' % (button, mods, prop, sign * step * k))
    return ('\n  <animation>\n    <type>pick</type>\n    <object-name>%s</object-name>\n%s    <hovered>\n      <binding>\n'
            '        <command>set-tooltip</command>\n        <tooltip-id>%s</tooltip-id>\n        <label>%s: %%03d (wheel or click; shift: 10)</label>\n'
            '        <property>%s</property>\n      </binding>\n    </hovered>\n  </animation>\n' % (obj, act, obj, label, prop))


def flag_tex(t, x0, y0, w, h, word):
    """A warning flag: red and white stripes, the word in black (none: stripes only, a shutter)."""
    d = t.d
    d.rectangle([x0, y0, x0 + w, y0 + h], fill=(240, 240, 240))
    for k in range(-h, w, 18):
        d.polygon([(x0 + k, y0 + h), (x0 + k + 9, y0 + h), (x0 + k + 9 + h, y0), (x0 + k + h, y0)], fill=RED)
    d.rectangle([x0 - 20, y0 - 20, x0 + w + 20, y0], fill=(0, 0, 0)); d.rectangle([x0 - 20, y0 + h, x0 + w + 20, y0 + h + 20], fill=(0, 0, 0))
    d.rectangle([x0, y0, x0 + w, y0 + h], outline=(20, 20, 20), width=2)
    if word:
        d.rectangle([x0 + w * 0.18, y0 + h * 0.18, x0 + w * 0.82, y0 + h * 0.82], fill=(240, 240, 240))
        d.text((x0 + w / 2, y0 + h / 2), word, font=I.font(h * 0.55), fill=(15, 15, 15), anchor='mm')


def quad(o, y0, y1, z0, z1, d, uv):
    A.flat(o, [(y0, z0), (y1, z0), (y1, z1), (y0, z1)], d, uv)


# ---------------------------------------------------------------- HSI

def hsi():
    w, h = I.tm_size(6, 'pilot')                       # 149 x 125 mm bezel; the card's centre 7.7 mm below its centre
    bz = 0.0077
    t = I.Tex(1024, 1024)
    d = t.d
    # compass card: 5 deg ticks, numbers every 30 deg, drawn in a 600 px circle
    c, R = 300, 296
    d.ellipse([c - R, c - R, c + R, c + R], fill=(10, 10, 10))
    for deg in range(0, 360, 5):
        a = math.radians(90 - deg); ln = 34 if deg % 10 == 0 else 20
        d.line([(c + R * 0.98 * math.cos(a), c - R * 0.98 * math.sin(a)),
                (c + (R * 0.98 - ln) * math.cos(a), c - (R * 0.98 - ln) * math.sin(a))], fill=WHITE, width=5 if deg % 10 == 0 else 3)
    for deg in range(0, 360, 30):
        lab = {0: 'N', 90: 'E', 180: 'S', 270: 'W'}.get(deg, str(deg // 10))
        im = Image.new('RGBA', (120, 90), (0, 0, 0, 0))
        ImageDraw.Draw(im).text((60, 45), lab, font=I.font(70 if len(lab) == 1 else 62), fill=WHITE + (255,), anchor='mm')
        im = im.rotate(-deg, expand=True, resample=Image.BICUBIC)
        a = math.radians(90 - deg); rr = R * 0.70
        t.im.paste(im, (int(c + rr * math.cos(a) - im.width / 2), int(c - rr * math.sin(a) - im.height / 2)), im)
    rc = 0.0445                                                           # card radius
    card_uv = t.area(c - R, c - R, c + R, c + R, -rc, rc, -rc, rc)
    # flags and labels
    flag_tex(t, 620, 20, 180, 70, 'NAV'); flag_tex(t, 620, 110, 180, 70, 'HDG'); flag_tex(t, 820, 20, 180, 70, '')
    NAVF, HDGF = (-0.0215, -0.0065, 0.012, 0.018), (0.018, 0.033, 0.026, 0.032)          # the flags' places
    nav_uv = t.area(620, 20, 800, 90, *NAVF)
    hdgf_uv = t.area(620, 110, 800, 180, *HDGF)
    shut_uv = t.area(820, 20, 1000, 90, -0.0612, -0.0348, 0.0077 + 0.0306, 0.0077 + 0.0394)
    for k, s in enumerate(('COURSE', 'KM', 'HDG', 'CRS')):
        d.rectangle([620, 200 + 60 * k, 1000, 250 + 60 * k], fill=(26, 26, 26))
        d.text((810, 225 + 60 * k), s, font=I.font(40), fill=(215, 215, 215), anchor='mm')
    # labels: COURSE and KM under their windows, HDG and CRS under their knobs, inside the case's bottom edge
    labels = ((0.053, bz + 0.024), (-0.048, bz + 0.026), (-0.054, -0.0505), (0.050, -0.0505))
    lab_uv = [t.area(620, 200 + 60 * k, 1000, 250 + 60 * k, y - 0.0114, y + 0.0114, z - 0.0015, z + 0.0015)
              for k, (y, z) in enumerate(labels)]
    black, white, grey = t.swatch((20, 20, 20)), t.swatch(WHITE), t.swatch((55, 55, 55))
    amber, win = t.swatch(AMBER), t.swatch((6, 6, 6))
    # the instrument; d: case 0..18 mm, card at 6, bearing pointers 7-7.6, course assembly 8.5-9.5, glass 17
    body = A.Obj('case', 'hsi60.png')
    case_outline = A.rrect(w, h, 0.006, 0, bz)
    I.case(body, case_outline, A.circle(0.0465, n=48), 0, 0.018, 0.006, black)
    for k, (y, z) in enumerate(labels):
        quad(body, y - 0.0114, y + 0.0114, z - 0.0015, z + 0.0015, 0.0181, lab_uv[k])
    for y, z, ww in ((0.053, bz + 0.033, 0.020), (-0.048, bz + 0.035, 0.026)):           # COURSE and KM windows
        quad(body, y - ww / 2, y + ww / 2, z - 0.0042, z + 0.0042, 0.0182, win)
    card = A.Obj('card', 'hsi60.png'); A.flat(card, A.circle(rc, n=64), 0.0062, card_uv)
    fixed = A.Obj('lubber', 'hsi60.png')                                 # lubber line and aircraft symbol
    A.flat(fixed, [(-0.0018, 0.0465), (0.0018, 0.0465), (0, 0.0405)], 0.0105, white)
    quad(fixed, -0.0005, 0.0005, -0.0070, 0.0060, 0.0104, white)
    quad(fixed, -0.0060, 0.0060, 0.0010, 0.0020, 0.0104, white)
    quad(fixed, -0.0025, 0.0025, -0.0065, -0.0057, 0.0104, white)
    bug = A.Obj('hdg-bug', 'hsi60.png')
    A.flat(bug, [(-0.0040, 0.0445), (-0.0040, 0.0405), (-0.0012, 0.0405), (0, 0.0425), (0.0012, 0.0405),
                 (0.0040, 0.0405), (0.0040, 0.0445)], 0.0068, amber)
    # bearing pointers: No. 1 single, No. 2 double, heads out at the top
    b1, b2 = A.Obj('brg1', 'hsi60.png'), A.Obj('brg2', 'hsi60.png')
    A.flat(b1, [(-0.0035, 0.033), (0, 0.040), (0.0035, 0.033)], 0.0072, white); quad(b1, -0.0007, 0.0007, -0.040, 0.033, 0.0071, white)
    A.flat(b2, [(-0.0045, 0.030), (0, 0.038), (0.0045, 0.030)], 0.0076, white)
    for y in (-0.0018, 0.0018):
        quad(b2, y - 0.0005, y + 0.0005, 0.010, 0.030, 0.0075, white); quad(b2, y - 0.0005, y + 0.0005, -0.038, -0.010, 0.0075, white)
    # course assembly: pointer head and tail, deviation dots, to-from arrows, NAV flag; the bar moves across it
    crs = A.Obj('course', 'hsi60.png')
    A.flat(crs, [(-0.0035, 0.030), (0, 0.038), (0.0035, 0.030)], 0.0088, amber)
    quad(crs, -0.0010, 0.0010, 0.022, 0.030, 0.0088, amber); quad(crs, -0.0010, 0.0010, -0.038, -0.022, 0.0088, amber)
    for y in (-0.016, -0.008, 0.008, 0.016):
        A.flat(crs, A.circle(0.0012, y, 0, 10), 0.0086, white)
    to, fr = A.Obj('to', 'hsi60.png'), A.Obj('from', 'hsi60.png')
    A.flat(to, [(0.006, 0.010), (0.0115, 0.0035), (0.0005, 0.0035)], 0.0090, white)
    A.flat(fr, [(0.006, -0.010), (0.0115, -0.0035), (0.0005, -0.0035)], 0.0090, white)
    bar = A.Obj('dev-bar', 'hsi60.png'); quad(bar, -0.0010, 0.0010, -0.0215, 0.0215, 0.0094, amber)
    navf = A.Obj('nav-flag', 'hsi60.png'); quad(navf, *NAVF, 0.0095, nav_uv)
    hdgf = A.Obj('hdg-flag', 'hsi60.png'); quad(hdgf, *HDGF, 0.0106, hdgf_uv)
    shutter = A.Obj('km-shutter', 'hsi60.png'); quad(shutter, -0.0612, -0.0348, bz + 0.0306, bz + 0.0394, 0.0186, shut_uv)
    knobs = {}
    for name, y in (('hdg-knob', -0.054), ('crs-knob', 0.050)):
        o = A.Obj(name, 'hsi60.png'); A.cylinder(o, 0.0080, y, -0.0373, 0.018, 0.028, grey, black, n=24)
        A.box(o, y - 0.0008, y + 0.0008, -0.0373 + 0.002, -0.0373 + 0.0075, 0.028, 0.0283, white, white)
        knobs[name] = (o, y)
    glass = A.Obj('glass', None, A.GLASS); A.flat(glass, A.circle(0.0465, n=48), 0.0170, A.const(0, 0))
    objs = [body, card, fixed, bug, b1, b2, crs, to, fr, bar, navf, hdgf, shutter] + [o for o, y in knobs.values()] + [glass]
    os.makedirs(HSI_DIR, exist_ok=True)
    A.write(os.path.join(HSI_DIR, 'hsi60.ac'), objs)
    t.save(os.path.join(HSI_DIR, 'hsi60.png'))
    names = [o.name for o in objs]
    for n, fname in ((0, 'hsi60.xml'), (1, 'hsi60-copilot.xml')):
        H = U + 'hsi[%d]/' % n
        bugp = '/autopilot/settings/heading-bug-deg' if n == 0 else H + 'heading-bug-deg'
        x = I.common_xml(names, lit=['card', 'lubber', 'hdg-bug', 'brg1', 'brg2', 'course', 'to', 'from', 'dev-bar'],
                         glass=['glass'], extra='')
        x = x.replace('%s', 'hsi60.ac', 1).replace('<PropertyList>', '<!-- %s horizontal situation indicator, TM 1-1520-237-10 '
                      'figure 3-31 (%s); data from Nasal/avionics.nas -->\n<PropertyList>' % (('pilot\'s', 'copilot\'s')[n], MARK), 1)
        a = ''
        a += translate(['dev-bar'], H + 'dev-norm', 0.016, (0, 1, 0))
        a += rotate(['course', 'to', 'from', 'nav-flag', 'dev-bar'], H + 'course-deg')
        a += rotate(['hdg-bug'], bugp)
        a += rotate(['card', 'hdg-bug', 'course', 'to', 'from', 'nav-flag', 'dev-bar'], '/orientation/heading-magnetic-deg', -1)
        a += rotate(['brg1'], H + 'brg1-rel') + rotate(['brg2'], H + 'brg2-rel')
        a += select(['to'], '<property>%sto</property>' % H) + select(['from'], '<property>%sfrom</property>' % H)
        a += select(['nav-flag'], '<property>%snav-flag</property>' % H) + select(['hdg-flag'], '<property>%shdg-flag</property>' % H)
        a += select(['km-shutter'], '<not><property>%skm-vis</property></not>' % H)
        a += rotate(['hdg-knob'], bugp, 5, (0, -0.054, -0.0373)) + rotate(['crs-knob'], H + 'course-deg', 5, (0, 0.050, -0.0373))
        a += knob_pick('hdg-knob', bugp, 'Heading select') + knob_pick('crs-knob', H + 'course-deg', 'Course')
        a += text_node('course-digits', H + 'course-deg', '%03.0f', 0.0075, 0.053, bz + 0.033, 0.0187)
        a += text_node('km-digits', H + 'km', '%04.0f', 0.0075, -0.048, bz + 0.035, 0.0184, 0.026)
        a += emissive(['course-digits', 'km-digits'], (0.9, 0.9, 0.9), 'controls/lighting/instruments-norm')
        a += select(['course-digits', 'km-digits'], '<property>controls/electric/battery-switch</property>')
        x = x.replace('\n</PropertyList>', a + '\n</PropertyList>')
        open(os.path.join(HSI_DIR, fname), 'w').write(x)


# ---------------------------------------------------------------- VSI

def vsi():
    """Additions to the old attitude indicator (its frame: origin at the sphere's centre, glass at 6.4 mm)."""
    t = I.Tex(512, 512)
    for k, word in enumerate(('ATT', 'CMD', 'NAV', 'GS')):
        flag_tex(t, 10, 10 + 70 * k, 150, 56, word)
    flags = (('att-flag', -0.044, 0.026), ('cmd-flag', -0.036, 0.042), ('nav-flag', -0.036, -0.042), ('gs-flag', 0.044, -0.012))
    flag_uv = [t.area(10, 10 + 70 * k, 160, 66 + 70 * k, y - 0.0055, y + 0.0055, z - 0.0022, z + 0.0022)
               for k, (name, y, z) in enumerate(flags)]
    lights = (('ga', 'GA', (60, 255, 80), -0.034), ('dh', 'DH', AMBER, -0.017), ('mb', 'MB', (170, 200, 255), 0.017))
    lit_uv, dark_uv = [], []
    for k, (key, word, rgb, y) in enumerate(lights):
        for j, col in enumerate(((105, 105, 105), rgb)):
            x0, y0 = 200 + 100 * j, 10 + 60 * k
            t.d.rectangle([x0, y0, x0 + 90, y0 + 46], fill=(14, 14, 14))
            t.d.text((x0 + 45, y0 + 23), word, font=I.font(30), fill=col, anchor='mm')
            (dark_uv if j == 0 else lit_uv).append(t.area(x0, y0, x0 + 90, y0 + 46, y - 0.005, y + 0.005, 0.0650, 0.0700))
    white = t.swatch(WHITE)
    objs = []
    def obj(name):
        o = A.Obj(name, 'cis.png'); objs.append(o); return o
    s = obj('gs-scale-r')
    for z in (-0.020, -0.010, 0.010, 0.020):
        A.flat(s, A.circle(0.0011, 0.0555, z, 10), 0.0046, white)
    quad(s, 0.0535, 0.0575, -0.0004, 0.0004, 0.0046, white)
    A.flat(obj('gs-ptr'), [(0.0515, -0.0022), (0.0545, 0), (0.0515, 0.0022)], 0.0050, white)
    s = obj('dev-scale')
    for y in (-0.016, -0.008, 0.008, 0.016):
        A.flat(s, A.circle(0.0011, y, -0.0285, 10), 0.0046, white)
    quad(s, -0.0004, 0.0004, -0.0305, -0.0265, 0.0046, white)
    A.flat(obj('dev-ptr'), [(-0.0022, -0.0245), (0.0022, -0.0245), (0, -0.0272)], 0.0050, white)
    for k, (name, y, z) in enumerate(flags):
        quad(obj(name), y - 0.0055, y + 0.0055, z - 0.0022, z + 0.0022, 0.0055, flag_uv[k])
    for k, (key, word, rgb, y) in enumerate(lights):
        quad(obj(key + '-off'), y - 0.005, y + 0.005, 0.0650, 0.0700, 0.0245, dark_uv[k])
        quad(obj(key + '-on'), y - 0.005, y + 0.005, 0.0650, 0.0700, 0.0247, lit_uv[k])
    quad(obj('turn'), -0.0008, 0.0008, -0.0470, -0.0425, 0.0050, white)
    s = obj('turn-marks')                    # 4-minute and 2-minute turns: the needle one and two widths off centre
    for y in (0.0024, 0.0040):
        for sy in (-1, 1):
            quad(s, sy * y - 0.0002, sy * y + 0.0002, -0.0490, -0.0474, 0.0048, white)
    A.write(os.path.join(AI_DIR, 'cis.ac'), objs)
    t.save(os.path.join(AI_DIR, 'cis.png'))
    names = [o.name for o in objs]
    for n in (0, 1):
        V = U + 'vsi[%d]/' % n
        x = I.common_xml(names, lit=['gs-scale-r', 'gs-ptr', 'dev-scale', 'dev-ptr', 'turn', 'turn-marks'])
        x = x.replace('%s', 'cis.ac', 1).replace('<PropertyList>', '<!-- %s VSI: what the old attitude indicator lacked '
                      '(TM 1-1520-237-10 figure 3-30); %s -->\n<PropertyList>' % (('pilot\'s', 'copilot\'s')[n], MARK), 1)
        a = translate(['gs-ptr'], V + 'gs-norm', 0.020, (0, 0, 1)) + translate(['dev-ptr'], V + 'dev-norm', 0.016, (0, 1, 0))
        a += translate(['turn'], V + 'turn-widths', 0.0016, (0, 1, 0), -3, 3)
        for f in ('att', 'cmd', 'nav', 'gs'):
            a += select([f + '-flag'], '<property>%s%s-flag</property>' % (V, f))
        for key, word, rgb, y in lights:
            a += select([key + '-on'], '<property>%s%s</property>' % (V, key))
            a += emissive([key + '-on'])
        x = x.replace('\n</PropertyList>', a + '\n</PropertyList>')
        open(os.path.join(AI_DIR, ('cis.xml', 'cis-copilot.xml')[n]), 'w').write(x)
    # the attitude indicator itself: the CIS's command bars and collective pointer, and the additions
    base = open(os.path.join(AI_DIR, 'ai.xml'), encoding='latin-1').read()
    base = re.sub(r'\n*  <!-- CIS:.*?<!-- /CIS -->\n', '\n', base, flags=re.S)
    base = base.replace('\n    <object-name>bank-index</object-name>', '')     # gone from ai.ac (gauges.ball)
    base = re.sub(r'\n  <animation>\s*<type>rotate</type>\s*<property>orientation/roll-deg</property>\s*<min-deg>-120</min-deg>.*?</animation>\n',
                  '\n', base, flags=re.S)
    C = U + 'cis/'
    for n, fname in ((0, 'ai.xml'), (1, 'ai-copilot.xml')):
        a = '\n  <!-- CIS: %s -->\n' % MARK
        a += '  <model>\n    <path>%s</path>\n  </model>\n' % ('cis.xml', 'cis-copilot.xml')[n]
        a += translate(['nav-pitch-steering-ba'], C + 'roll-cmd', 0.025, (0, 1, 0))
        a += select(['nav-pitch-steering-ba'], '<property>%sroll-vis</property>' % C)
        a += translate(['nav-yaw-steering-bar'], C + 'pitch-cmd', 0.025, (0, 0, 1))
        a += select(['nav-yaw-steering-bar'], '<property>%spitch-vis</property>' % C)
        a += translate(['ils-flag'], C + 'coll-cmd', 0.020, (0, 0, 1))
        a += select(['ils-flag'], '<property>%scoll-vis</property>' % C)
        a += '  <!-- /CIS -->\n'
        open(os.path.join(AI_DIR, fname), 'w', encoding='latin-1').write(base.replace('\n</PropertyList>', a + '\n</PropertyList>', 1))


def radalt():
    """The radar altimeter's LO light: on below the LO bug (the CISP's DH), dimmed by RAD ALT DIMMING."""
    path = os.path.join(I.INSTR, 'RadarAlt', 'RadarAlt.xml')
    x = open(path, encoding='latin-1').read()
    x = re.sub(r'\n  <!-- LO light:.*?<!-- /LO light -->\n', '\n', x, flags=re.S)
    blk = ('\n  <!-- LO light: %s -->\n  <animation>\n    <type>material</type>\n    <object-name>Alarm</object-name>\n'
           '    <condition>\n      <property>%svsi[0]/dh</property>\n    </condition>\n    <emission>\n'
           '      <factor-prop>/controls/lighting/radalt-norm</factor-prop>\n      <red> 1 </red>\n      <green> 0.15 </green>\n'
           '      <blue> 0.1 </blue>\n    </emission>\n  </animation>\n  <!-- /LO light -->\n' % (MARK, U))
    open(path, 'w', encoding='latin-1').write(x.replace('\n</PropertyList>', blk + '\n</PropertyList>', 1))


if __name__ == '__main__':
    hsi(); vsi()
