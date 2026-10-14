"""The caution/advisory panel (TM 1-1520-237-10 2.81.1, figure 2-24, table 2-3) and what lights it.

Writes Instruments/cap (the panel: the TM's face art, a raised capsule per legend and its lit version),
Systems/uh60a-cap.xml (property rules: one flag per legend that the simulation can drive, /sim/model/uh60/cap/<id>),
Nasal/caution.nas (MASTER CAUTION: on when an amber capsule lights, off when pressed) and the master
warning panel's conditions in Instruments/caution/caution.xml (MASTER CAUTION and the lamp test).

Capsules light with dc power (the battery: the model has no APU), from their flag, or all of them while the BRT/DIM-TEST
switch is held at TEST; the fuel-low capsules flash. Legends for failures the simulation doesn't have get no flag.
"""
import os, re
from PIL import Image, ImageDraw

import acmesh as A
import instruments as I
import tmart

P = '/sim/model/uh60/cap/'
WOW = '<or><property>/gear/gear[0]/wow</property><property>/gear/gear[1]/wow</property>' \
      '<property>/gear/gear[2]/wow</property></or>'
NR = '/rotors/main/rpm-pct'


def lt(prop, v):
    return '<less-than><property>%s</property><value>%g</value></less-than>' % (prop, v)


def gt(prop, v):
    return '<greater-than><property>%s</property><value>%g</value></greater-than>' % (prop, v)


def off(prop):
    return lt(prop, 0.5)


def eng(n, what):
    return '/engines/engine[%d]/%s' % (n, what)


# Conditions, from table 2-3 and the systems the simulation has:
#   generators: off line below 375 Hz (GCU underfrequency, ground only) = 93.75 % rotor speed; in flight they stay on
#   the converters and the ac essential bus follow them (no APU or external power)
#   hydraulic pumps (on the transmission's accessory modules): pressure assumed low below half rotor speed, the
#   estimate the cosmetic transmission oil pressure (55 psi at 100 %) also makes (MAIN XMSN OIL PRESS < 14 psi)
#   FUEL PRESS: below 9 psi; the engine-driven pump assumed to give it above light-off speed (20 % Ng), the prime/boost
#   pump always
COND = {
    'fuel-low-1': lt('/consumables/fuel/tank[0]/level-lbs', 172),
    'fuel-low-2': lt('/consumables/fuel/tank[1]/level-lbs', 172),
    'fuel-press-1': lt(eng(0, 'n1-pct'), 20) + off('/controls/engines/engine[0]/fuel-pump'),
    'fuel-press-2': lt(eng(1, 'n1-pct'), 20) + off('/controls/engines/engine[1]/fuel-pump'),
    'oil-press-1': lt(eng(0, 'oil-pressure-psi'), 20),
    'oil-press-2': lt(eng(1, 'oil-pressure-psi'), 20),
    'oil-temp-1': gt(eng(0, 'oil-temperature-degc'), 150),
    'oil-temp-2': gt(eng(1, 'oil-temperature-degc'), 150),
    'gen-1': WOW + lt(NR, 93.75), 'gen-2': WOW + lt(NR, 93.75),
    'conv-1': WOW + lt(NR, 93.75), 'conv-2': WOW + lt(NR, 93.75), 'ac-ess': WOW + lt(NR, 93.75),
    'starter-1': gt('/fdm/jsbsim/eng[0]/starter-on', 0.5),
    'starter-2': gt('/fdm/jsbsim/eng[1]/starter-on', 0.5),
    'pri-servo-1': lt(NR, 50), 'pri-servo-2': lt(NR, 50), 'hyd-pump-1': lt(NR, 50), 'hyd-pump-2': lt(NR, 50),
    'boost-servo': lt(NR, 50),
    'xmsn-oil-temp': gt('/sim/model/uh60/xmsn-oil-temp-degc', 120),
    'xmsn-oil-press': lt('/sim/model/uh60/xmsn-oil-press-psi', 14),
    'prime-boost': '<or><property>/controls/engines/engine[0]/fuel-pump</property>'
                   '<property>/controls/engines/engine[1]/fuel-pump</property></or>',
    'ldg-lt': '<property>/controls/lighting/landing-lights</property>',
    'parking-brake': '<property>/controls/gear/brake-parking</property>',
}
FLASH = {'fuel-low-1', 'fuel-low-2'}
# flags Nasal sets: avionics.nas the ice detector's test, the blade deice test panel's failures, the IRCM's cooldown;
# caution.nas the AFCS's (a property rule would create JSBSim's switches before JSBSim starts, and JSBSim then leaves
# them off instead of at their default, on)
NASAL = {'ice', 'mr-deice-fail', 'tr-deice-fail', 'ircm-inop', 'stabilator', 'sas-off', 'flt-path-stab'}

# figure 2-24 (UH), rows top down, columns left to right: (id, legend lines); rows 1-16 amber, 17-21 green;
# None: the BRT/DIM-TEST switch or no capsule
GRID = [
    [('fuel-low-1', ['#1 FUEL LOW']), ('gen-1', ['#1 GEN']), ('gen-2', ['#2 GEN']), ('fuel-low-2', ['#2 FUEL LOW'])],
    [('fuel-press-1', ['#1 FUEL', 'PRESS']), ('gen-brg-1', ['#1 GEN BRG']), ('gen-brg-2', ['#2 GEN BRG']),
     ('fuel-press-2', ['#2 FUEL', 'PRESS'])],
    [('oil-press-1', ['#1 ENGINE', 'OIL PRESS']), ('conv-1', ['#1 CONV']), ('conv-2', ['#2 CONV']),
     ('oil-press-2', ['#2 ENGINE', 'OIL PRESS'])],
    [('oil-temp-1', ['#1 ENGINE', 'OIL TEMP']), ('ac-ess', ['AC ESS', 'BUS OFF']), ('dc-ess', ['DC ESS', 'BUS OFF']),
     ('oil-temp-2', ['#2 ENGINE', 'OIL TEMP'])],
    [('chip-1', ['CHIP', '#1 ENGINE']), ('batt-low', ['BATT LOW', 'CHARGE']), ('batt-fault', ['BATTERY', 'FAULT']),
     ('chip-2', ['CHIP', '#2 ENGINE'])],
    [('fuel-fltr-1', ['#1 FUEL', 'FLTR BYPASS']), ('gust-lock', ['GUST', 'LOCK']), ('pitch-bias', ['PITCH BIAS', 'FAIL']),
     ('fuel-fltr-2', ['#2 FUEL', 'FLTR BYPASS'])],
    [('starter-1', ['#1 ENGINE', 'STARTER']), ('oil-fltr-1', ['#1 OIL', 'FLTR BYPASS']),
     ('oil-fltr-2', ['#2 OIL', 'FLTR BYPASS']), ('starter-2', ['#2 ENGINE', 'STARTER'])],
    [('pri-servo-1', ['#1 PRI', 'SERVO PRESS']), ('hyd-pump-1', ['#1 HYD', 'PUMP']), ('hyd-pump-2', ['#2 HYD', 'PUMP']),
     ('pri-servo-2', ['#2 PRI', 'SERVO PRESS'])],
    [('tr-quadrant', ['TAIL ROTOR', 'QUADRANT']), ('ircm-inop', ['IRCM', 'INOP']), ('aux-fuel', ['AUX FUEL']),
     ('tr-servo-1', ['#1 TAIL RTR', 'SERVO'])],
    [('xmsn-oil-temp', ['MAIN XMSN', 'OIL TEMP']), ('int-xmsn-temp', ['INT XMSN', 'OIL TEMP']),
     ('tail-xmsn-temp', ['TAIL XMSN', 'OIL TEMP']), ('apu-oil-temp', ['APU OIL', 'TEMP HI'])],
    [('boost-servo', ['BOOST SERVO', 'OFF']), ('stabilator', ['STABILATOR']), ('sas-off', ['SAS OFF']),
     ('trim-fail', ['TRIM FAIL'])],
    [('pitot-l', ['LFT PITOT', 'HEAT']), ('flt-path-stab', ['FLT PATH', 'STAB']), ('iff', ['IFF']),
     ('pitot-r', ['RT PITOT', 'HEAT'])],
    [('chip-input-l', ['CHIP INPUT', 'MDL-LH']), ('chip-int', ['CHIP', 'INT XMSN']), ('chip-tail', ['CHIP', 'TAIL XMSN']),
     ('chip-input-r', ['CHIP INPUT', 'MDL-RH'])],
    [('chip-access-l', ['CHIP ACCESS', 'MDL-LH']), ('chip-main', ['CHIP MAIN', 'MDL SUMP']), ('apu-fail', ['APU', 'FAIL']),
     ('chip-access-r', ['CHIP ACCESS', 'MDL-RH'])],
    [('mr-deice-fail', ['MR DE-ICE', 'FAIL']), ('mr-deice-fault', ['MR DE-ICE', 'FAULT']),
     ('tr-deice-fail', ['TR DE-ICE', 'FAIL']), ('ice', ['ICE', 'DETECTED'])],
    [('xmsn-oil-press', ['MAIN XMSN', 'OIL PRESS']), ('rsvr-1', ['#1 RSVR', 'LOW']), ('rsvr-2', ['#2 RSVR', 'LOW']),
     ('rsvr-backup', ['BACK-UP', 'RSVR LOW'])],
    [('eng-ai-1', ['#1 ENG', 'ANTI-ICE ON']), ('inlet-ai-1', ['#1 ENG INLET', 'ANTI-ICE ON']),
     ('inlet-ai-2', ['#2 ENG INLET', 'ANTI-ICE ON']), ('eng-ai-2', ['#2 ENG', 'ANTI-ICE ON'])],
    [('apu-on', ['APU ON']), ('apu-gen', ['APU GEN ON']), ('prime-boost', ['PRIME BOOST', 'PUMP ON']),
     ('backup-pump', ['BACK-UP', 'PUMP ON'])],
    [('apu-accum', ['APU ACCUM', 'LOW']), ('search-lt', ['SEARCH LT', 'ON']), ('ldg-lt', ['LDG LT ON']),
     ('tr-servo-2', ['#2 TAIL RTR', 'SERVO ON'])],
    [None, ('cargo-hook', ['CARGO', 'HOOK OPEN']), ('hook-armed', ['HOOK ARMED']), ('blank-1', [])],
    [None, ('parking-brake', ['PARKING', 'BRAKE ON']), ('ext-pwr', ['EXT PWR', 'CONNECTED']), ('blank-2', [])],
]
CAUTION_ROWS = 16
# the capsules on figure 2-9 (sheet 1 pt): columns' left edges, the first row's top, row pitch, capsule size
COLS, ROW0, PITCH, CW, CH = (369.8, 388.4, 407.0, 425.6), 97.3, 6.38, 17.32, 4.88
TEST_SWITCH = (376.0, 225.5)                   # BRT/DIM-TEST switch, sheet 1 pt
AMBER, GREEN = (255, 176, 0), (60, 255, 80)


def legend(d, box, lines, rgb):
    x0, y0, x1, y1 = box
    d.rectangle(box, fill=(14, 14, 14))
    if not lines:
        d.line([(x0 + (x1 - x0) * 0.3, (y0 + y1) / 2), (x0 + (x1 - x0) * 0.7, (y0 + y1) / 2)], fill=rgb, width=3)
        return
    f = I.font((y1 - y0) * (0.36 if len(lines) > 1 else 0.44), narrow=True)
    for k, s in enumerate(lines):
        y = y0 + (y1 - y0) * ((k + 1) / (len(lines) + 1) if len(lines) > 1 else 0.5)
        d.text(((x0 + x1) / 2, y), s, font=f, fill=rgb, anchor='mm')


def build(art):
    num, side = 24, 'centre'
    it = I.item(num, side); bx0, bx1, by0, by1 = it[4:8]
    cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
    w, h = (bx1 - bx0) * I.PT, (by1 - by0) * I.PT
    depth = 0.006
    t = I.Tex(2048, 2048, (I.GROUND,) * 3)
    pw = 1024; ph = int(round(pw * h / w))
    t.im.paste(tmart.light(art.vector(bx0, bx1, by0, by1, 1, pw, ph), I.GROUND, I.LETTER).convert('RGB'), (0, 0))
    face_uv = t.area(0, 0, pw, ph, -w / 2, w / 2, -h / 2, h / 2)
    paint, side_uv, metal = t.swatch((30, 30, 30)), t.swatch((16, 16, 16)), t.swatch((170, 170, 172))
    body = A.Obj('panel', 'cap.png'); A.prism(body, A.rrect(w, h, 0.002), 0, depth, face_uv, paint)
    caps = A.Obj('capsules', 'cap.png'); lit = []
    cell = [1040 + 192 * (k % 5) for k in range(5)]
    n = 0; legends = []
    for r, row in enumerate(GRID):
        for c, entry in enumerate(row):
            if entry is None:
                continue
            cid, lines = entry
            colour = AMBER if r < CAUTION_ROWS else GREEN
            # atlas: unlit and lit versions of the legend, 192 x 54 px each
            ux, uy = cell[n % 5], 4 + 54 * (n // 5) * 2; n += 1
            legend(t.d, (ux, uy, ux + 186, uy + 50), lines, (58, 58, 58))
            legend(t.d, (ux, uy + 54, ux + 186, uy + 104), lines, colour)
            ccy, ccz = (COLS[c] + CW / 2 - cx) * I.PT, -(ROW0 + PITCH * r + CH / 2 - cy) * I.PT
            cw, ch = CW * I.PT, CH * I.PT
            y0, y1, z0, z1 = ccy - cw / 2, ccy + cw / 2, ccz - ch / 2, ccz + ch / 2
            A.prism(caps, [(y0, z0), (y1, z0), (y1, z1), (y0, z1)], depth, depth + 0.002,
                    t.area(ux, uy, ux + 186, uy + 50, y0, y1, z0, z1), side_uv)
            o = A.Obj('lit-' + cid, 'cap.png')
            A.flat(o, [(y0, z0), (y1, z0), (y1, z1), (y0, z1)], depth + 0.0022, t.area(ux, uy + 54, ux + 186, uy + 104, y0, y1, z0, z1))
            lit.append(o); legends.append((cid, r < CAUTION_ROWS))
    # BRT/DIM-TEST switch: a lever straight out, held down at TEST
    sw = A.Obj('test-switch', 'cap.png')
    sy, sz = (TEST_SWITCH[0] - cx) * I.PT, -(TEST_SWITCH[1] - cy) * I.PT
    A.cylinder(sw, 0.0035, sy, sz, depth, depth + 0.003, metal, metal, n=6)
    A.beam(sw, (depth + 0.003, sy, sz), (depth + 0.014, sy, sz), 0.0022, metal)
    objs = [body, caps, sw] + lit
    xml = ''
    for o, (cid, amber) in zip(lit, legends):
        cond = '<property>%s%s</property>' % (P, cid) if cid in COND or cid in NASAL else ''
        cond = '<or>%s<property>/sim/model/uh60/cap-test</property></or>' % cond if cond else \
               '<property>/sim/model/uh60/cap-test</property>'
        if cid in FLASH:
            cond += '<property>/sim/model/uh60/warning-flash</property>'
        xml += ('\n  <animation>\n    <type>select</type>\n    <object-name>%s</object-name>\n    <condition>\n'
                '      <property>/controls/electric/battery-switch</property>\n      %s\n    </condition>\n  </animation>\n'
                % (o.name, cond))
    xml += ('\n  <animation>\n    <type>material</type>\n' + ''.join('    <object-name>%s</object-name>\n' % o.name for o in lit) +
            '    <emission>\n      <factor-prop>/controls/lighting/caut-advsy-norm</factor-prop>\n      <red> 1 </red>\n'
            '      <green> 1 </green>\n      <blue> 1 </blue>\n    </emission>\n  </animation>\n')
    xml += ('\n  <animation>\n    <type>rotate</type>\n    <object-name>test-switch</object-name>\n'
            '    <property>/sim/model/uh60/cap-test</property>\n    <factor> 30 </factor>\n'
            '    <center>\n      <x-m> %.4f </x-m>\n      <y-m> %.4f </y-m>\n      <z-m> %.4f </z-m>\n    </center>\n'
            '    <axis>\n      <x> 0 </x>\n      <y> 1 </y>\n      <z> 0 </z>\n    </axis>\n  </animation>\n' % (depth, sy, sz))
    xml += ('\n  <animation>\n    <type>pick</type>\n    <object-name>test-switch</object-name>\n    <action>\n'
            '      <button>0</button>\n      <repeatable>false</repeatable>\n      <binding>\n'
            '        <command>property-assign</command>\n        <property>/sim/model/uh60/cap-test</property>\n'
            '        <value>1</value>\n      </binding>\n      <mod-up>\n        <binding>\n'
            '          <command>property-assign</command>\n          <property>/sim/model/uh60/cap-test</property>\n'
            '          <value>0</value>\n        </binding>\n      </mod-up>\n    </action>\n    <hovered>\n'
            '      <binding>\n        <command>set-tooltip</command>\n        <tooltip-id>uh60-cap-test</tooltip-id>\n'
            '        <label>Caution/advisory lights: hold to test</label>\n      </binding>\n    </hovered>\n  </animation>\n')
    head = I.common_xml([o.name for o in objs], lit=['panel'], extra=xml)
    I.write_instrument('cap', objs, head.replace('<PropertyList>', '<!-- caution/advisory panel: capsules lit by '
                       'Systems/uh60a-cap.xml (fdm/cockpit/cap.py) -->\n<PropertyList>', 1))
    t.save(os.path.join(I.INSTR, 'cap', 'cap.png'))
    write_rules(legends)
    write_nasal([cid for cid, amber in legends if amber and (cid in COND or cid in NASAL)])
    write_master_warning()


def write_rules(legends):
    out = ['<?xml version="1.0" encoding="ISO-8859-1"?>', '',
           '<!-- Caution/advisory panel: a flag per capsule the simulation can drive, /sim/model/uh60/cap/<id>, from',
           '     TM 1-1520-237-10 table 2-3 (fdm/cockpit/cap.py writes this file; see it for the assumptions) -->', '',
           '<PropertyList>']
    for cid, amber in legends:
        if cid not in COND:
            continue
        out += ['', '  <logic>', '    <name>%s</name>' % cid, '    <input>', '      <and>%s</and>' % COND[cid], '    </input>',
                '    <output>%s%s</output>' % (P, cid), '  </logic>']
    out += ['', '</PropertyList>', '']
    open(os.path.join(I.SYSTEMS, 'uh60a-cap.xml'), 'w').write('\n'.join(out))


def write_nasal(cautions):
    s = '''# MASTER CAUTION (TM 1-1520-237-10 2.81): lights whenever an amber capsule of the caution/advisory panel goes on
# and stays on until pressed, so that a second caution lights it again. The capsules' flags come from
# Systems/uh60a-cap.xml, the AFCS's from here. Written by fdm/cockpit/cap.py.

var CAUTIONS = [%s];
var lit = {};
var master = props.globals.getNode("/sim/model/uh60/master-caution", 1);
master.setBoolValue(0);

# STABILATOR, SAS OFF (both SAS off, or no hydraulics below half rotor speed) and FLT PATH STAB. getprop, unlike a
# property rule or a listener, doesn't create JSBSim's switches before JSBSim has set them on.
var off = func(p) { var v = getprop(p); return v != nil and v < 0.5; };
var afcs = func {
    var C = "/sim/model/uh60/cap/";
    setprop(C ~ "stabilator", off("/fdm/jsbsim/stab/auto"));
    setprop(C ~ "sas-off", (off("/fdm/jsbsim/ctl/sas1") and off("/fdm/jsbsim/ctl/sas2"))
                           or (getprop("/rotors/main/rpm-pct") or 0) < 50);
    setprop(C ~ "flt-path-stab", off("/fdm/jsbsim/ctl/fps"));
};

var update = func {
    afcs();
    foreach (var c; CAUTIONS) {
        if (getprop("/sim/model/uh60/cap/" ~ c)) {
            if (!contains(lit, c)) {
                lit[c] = 1;
                master.setBoolValue(1);
            }
        } elsif (contains(lit, c)) {
            delete(lit, c);
        }
    }
};

var reset = func { master.setBoolValue(0); };

var timer = maketimer(0.25, update);
timer.start();
''' % ', '.join('"%s"' % c for c in cautions)
    open(os.path.join(I.NASAL, 'caution.nas'), 'w').write(s)


# master warning panel, figure 2-23 (sheet 1 pt): capsules' left edges, rows' tops, capsule size; the legends
MW_COLS, MW_ROWS, MW_W, MW_H = (154.7, 175.3, 195.9), (86.8, 94.3), 19.5, 5.85
MW = [('eng-out-1', ['#1 ENG', 'OUT']), (None, []), ('eng-out-2', ['#2 ENG', 'OUT']),
      ('fire', ['FIRE']), ('master', ['MASTER CAUTION', 'PRESS TO RESET']), ('low-rpm', ['LOW ROTOR', 'RPM'])]


def write_master_warning():
    """The master warning panel (TM 2.81, figure 2-23): #1 and #2 ENG OUT, FIRE, MASTER CAUTION (Nasal/caution.nas,
    pressed to reset) and LOW ROTOR RPM, amber; all on in the lamp test, LOW ROTOR RPM flashing; dimmed by the NVG
    panel's MA WRN control."""
    it = I.item(4, 'copilot')                      # sheet 1, where the capsules were measured
    bx0, bx1, by0, by1 = it[4:8]
    cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
    w, h = (bx1 - bx0) * I.PT, (by1 - by0) * I.PT
    t = I.Tex(2048, 1024, (20, 20, 20))
    dark, side = t.swatch((24, 24, 24)), t.swatch((14, 14, 14))
    body = A.Obj('panel', 'caution.png'); A.prism(body, A.rrect(w, h, 0.0015), 0, 0.004, dark, side)
    caps = A.Obj('capsules', 'caution.png'); objs = [body, caps]; lit = []
    for k, (cid, lines) in enumerate(MW):
        r, c = divmod(k, 3)
        ux, uy = 16 + 672 * c, 16 + 464 * r
        legend(t.d, (ux, uy, ux + 640, uy + 208), lines, (60, 52, 40))
        legend(t.d, (ux, uy + 224, ux + 640, uy + 432), lines, AMBER)
        y0 = (MW_COLS[c] - cx) * I.PT; y1 = y0 + MW_W * I.PT
        z1 = -(MW_ROWS[r] - cy) * I.PT; z0 = z1 - MW_H * I.PT
        o = A.Obj('btn-master', 'caution.png') if cid == 'master' else caps
        A.prism(o, [(y0, z0), (y1, z0), (y1, z1), (y0, z1)], 0.004, 0.0055, t.area(ux, uy, ux + 640, uy + 208, y0, y1, z0, z1), side)
        if o is not caps:
            objs.append(o)
        if cid:
            l = A.Obj('lit-' + cid, 'caution.png')
            A.flat(l, [(y0, z0), (y1, z0), (y1, z1), (y0, z1)], 0.0057, t.area(ux, uy + 224, ux + 640, uy + 432, y0, y1, z0, z1))
            objs.append(l); lit.append(l.name)
    A.write(os.path.join(I.INSTR, 'caution', 'caution.ac'), objs)
    t.save(os.path.join(I.INSTR, 'caution', 'caution.png'))
    test = '<property>sim/model/uh60/cap-test</property>'
    def light(obj, comment, cond, flash=False):
        return """
  <!-- %s -->
  <animation>
    <type>select</type>
    <object-name>%s</object-name>
    <condition>
      <property>controls/electric/battery-switch</property>%s
      <or>
        %s
        %s
      </or>
    </condition>
  </animation>
""" % (comment, obj, '\n      <property>sim/model/uh60/warning-flash</property>' if flash else '', test, cond)
    names = ''.join('    <object-name>%s</object-name>\n' % o.name for o in objs)
    x = """<?xml version="1.0" encoding="ISO-8859-1"?>

<!-- Master warning panel, TM 1-1520-237-10 2.81 and figure 2-23, with the flight model's values. The lights need dc
     power (battery switch) and all light in the caution/advisory panel's lamp test. Written by fdm/cockpit/cap.py. -->

<PropertyList>

  <path>caution.ac</path>

  <animation>
    <type>noshadow</type>
%s  </animation>

  <animation>
    <type>material</type>
%s    <emission>
      <factor-prop>/controls/lighting/ma-wrn-norm</factor-prop>
      <red> 1 </red>
      <green> 1 </green>
      <blue> 1 </blue>
    </emission>
  </animation>
""" % (names, ''.join('    <object-name>%s</object-name>\n' % o for o in lit))
    for n in (0, 1):
        x += light('lit-eng-out-%d' % (n + 1), '#%d ENG OUT: gas generator speed Ng below 55 %%' % (n + 1),
                   '<less-than><property>engines/engine[%d]/n1-pct</property><value>55</value></less-than>' % n)
    x += light('lit-fire', 'FIRE: no fire is simulated; the lamp test only', test)
    x += light('lit-master', 'MASTER CAUTION: an amber capsule of the caution/advisory panel has lit (Nasal/caution.nas)',
               '<property>sim/model/uh60/master-caution</property>')
    x += light('lit-low-rpm', 'LOW ROTOR RPM: rotor speed below 96 %, flashing',
               '<less-than><property>rotors/main/rpm-pct</property><value>96</value></less-than>', flash=True)
    x += """
  <animation>
    <type>pick</type>
    <object-name>btn-master</object-name>
    <object-name>lit-master</object-name>
    <action>
      <button>0</button>
      <repeatable>false</repeatable>
      <binding>
        <command>nasal</command>
        <script>caution.reset()</script>
      </binding>
    </action>
    <hovered>
      <binding>
        <command>set-tooltip</command>
        <tooltip-id>uh60-master-caution</tooltip-id>
        <label>Master caution: press to reset</label>
      </binding>
    </hovered>
  </animation>

</PropertyList>
"""
    open(os.path.join(I.INSTR, 'caution', 'caution.xml'), 'w', encoding='latin-1').write(x)
