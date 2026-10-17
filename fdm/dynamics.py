"""SAS-off transient responses vs the USAAEFA flight test and GenHel (Ballin TM-88360 appendix D, digitized into
data/ballin_appD.json): the model trimmed at each case's weight, CG, test-day atmosphere (pressure and density altitude) and speed
(Ballin table C1, his inertias) with
SAS, FPS and PBA off and the stabilator fixed at its trim value, then
the recorded increments of all four controls applied (the pilot's off-axis inputs and recovery included).
Responses are increments from the first 0.5 s, as the test aircraft had drifted from trim at the input.
python3 dynamics.py [--pba] [file ...]   (PBA off by default: no stability augmentation in the tests)"""
import sys, json, math, os
import numpy as np
from multiprocessing import Pool
import harness as H

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, 'data', 'ballin_appD.json')))
SPEED = {'hover': 0.0, '60 kt': 60.0, '100 kt': 100.0}
# Ballin appendix C, table C1: gross weight (lb), CG station and waterline (in), density altitude (ft), equivalent
# airspeed (kt), pressure altitude (ft); all runs: SAS, FPS and PBA off, stabilator fixed at its trim value, Ballin's inertias
COND = {'102': (15990, 348.8, 231.7, 6809, 93.0, 5500), '103': (16150, 349.5, 231.6, 6565, 91.9, 5256),
        '201': (15870, 359.3, 232.6, 6658, 0, 5582), '202': (15760, 359.2, 232.7, 6673, 0, 5493), '204': (15950, 359.4, 232.5, 6510, 0, 5370),
        '209': (15750, 359.3, 232.7, 6659, 0, 5742), '212': (16030, 359.4, 232.5, 6296, 0, 5411), '401': (16530, 359.6, 232.3, 5084, 93.3, 4484),
        '404': (16540, 359.5, 232.3, 5001, 92.8, 4455), '405': (16490, 359.5, 232.3, 4872, 55.8, 4401), '502': (16110, 351.1, 232.4, 6268, 56.9, 4969),
        '503': (16050, 351.1, 232.4, 6313, 54.6, 5020), '504': (15770, 348.3, 231.7, 5718, 54.4, 4514), '509': (15510, 352.0, 232.0, 6969, 53.1, 5953),
        '510': (15720, 351.4, 231.8, 5930, 53.96, 5515), '512': (15600, 351.4, 231.9, 5631, 53.1, 5948), '706': (15410, 351.9, 232.1, 7427, 92.8, 6450)}
INERTIA = (4659, 38512, 36796, 1882)        # slug ft2, Ixx Iyy Izz Ixz


def case_model(fid, root=None):
    """the Ballin model file with the case's weight, CG and Ballin's inertias (all mass at the CG, as the test
    configuration of genuh60): aircraft/uh60a-b<file> next to uh60a-ballin"""
    import re
    root = root or os.environ.get('UH60_ROOT') or os.path.join(HERE, 'root')      # UH60_ROOT: a variant (report/mkresults.py --tr)
    src = os.path.join(root, 'aircraft', 'uh60a', 'uh60a-ballin.xml')
    name = 'uh60a-b' + fid
    d = os.path.join(root, 'aircraft', name)
    dst = os.path.join(d, name + '.xml')
    if not os.path.exists(dst) or os.path.getmtime(dst) < os.path.getmtime(src):
        os.makedirs(d, exist_ok=True)
        if not os.path.lexists(os.path.join(d, 'Systems')):
            os.symlink('../uh60a/Systems', os.path.join(d, 'Systems'))
        w, x, z = COND[fid][:3]
        t = open(src).read()
        mb = t[t.index('<mass_balance>'):t.index('</mass_balance>')]
        tanks = sum(float(v) for v in re.findall(r'<pointmass name="Fuel tank \d">\s*<weight unit="LBS">([0-9.]+)', mb))
        m2 = re.sub(r'<ixx unit="SLUG\*FT2">[0-9.]+</ixx><iyy unit="SLUG\*FT2">[0-9.]+</iyy><izz unit="SLUG\*FT2">[0-9.]+</izz><ixz unit="SLUG\*FT2">[0-9.]+</ixz>',
                    '<ixx unit="SLUG*FT2">%d</ixx><iyy unit="SLUG*FT2">%d</iyy><izz unit="SLUG*FT2">%d</izz><ixz unit="SLUG*FT2">%d</ixz>' % INERTIA, mb)
        m2 = re.sub(r'<emptywt unit="LBS">[0-9.]+</emptywt>', '<emptywt unit="LBS">%.1f</emptywt>' % (w - tanks), m2)
        m2 = re.sub(r'<x>[0-9.]+</x><y>0.0</y><z>[0-9.]+</z>', '<x>%.1f</x><y>0.0</y><z>%.1f</z>' % (x, z), m2)
        open(dst, 'w').write(t.replace(mb, m2).replace('uh60a-ballin', name))
    return name


def test_day(f, pa, da, n=4):
    """the test day's atmosphere: geometric altitude and temperature offset (atmosphere/delta-T, which stays set)
    giving the case's pressure altitude and density altitude; JSBSim's pressure follows the warmer or colder air"""
    Tt = 518.67 * (1 - 6.8756e-6 * pa) ** 5.2559 / (1 - 6.8756e-6 * da) ** 4.2559
    h = float(pa)
    for _ in range(n):
        f['ic/h-sl-ft'] = h; f['ic/vt-kts'] = 100.0; f['ic/terrain-elevation-ft'] = H.TERRAIN_FT
        f.run_ic()
        f['atmosphere/delta-T'] = f['atmosphere/delta-T'] + Tt - f['atmosphere/T-R']
        h += pa - f['atmosphere/pressure-altitude']
    return h


def tas(eas, da):
    """true airspeed at a density altitude (standard atmosphere)"""
    return eas / math.sqrt((1 - 6.8756e-6 * da) ** 4.2559)
# Ballin's control signs: lateral + right, longitudinal + aft, pedal + right, collective + up (inches)
GEAR = {'lat': 1 / 5.0, 'lon': -1 / 5.0, 'ped': 1 / 2.69, 'coll': 1 / 10.0}     # -> model norms (+ right, + fwd, + right, + up)
AXIS = {'lat': 'p', 'lon': 'q', 'ped': 'r', 'coll': 'r'}                       # on-axis rate (collective: torque yaw)
OUT = os.path.join(HERE, 'out', 'dynamics')


def series(y):
    """digitized list -> array with spikes (scan specks) removed by a 7-sample median, gaps interpolated"""
    a = np.array([np.nan if v is None else v for v in y], float)
    m = a.copy()
    for i in range(len(a)):
        w = a[max(0, i - 3):i + 4]; w = w[~np.isnan(w)]
        m[i] = np.median(w) if len(w) else np.nan
    return m


def curve(case, v, who):
    c = case.get(v) or {}
    y = c.get(who)
    if not y: return None
    y = series(y)
    if who == 'test':
        # the dashed line hides under the solid one where they agree (before the input, mostly): fill from GenHel
        # there; and drop dashed pieces that jump away from both neighbours (scan specks, crossing lines)
        g = c.get('genhel')
        if g:
            g = series(g)
            t = np.arange(len(y))
            ok = ~np.isnan(y)
            jump = np.zeros(len(y), bool)
            yi = np.interp(t, t[ok], y[ok]) if ok.sum() > 2 else y
            d1 = np.abs(np.diff(yi))
            jump[1:] |= d1 > 3 * np.nanmedian(d1[d1 > 0]) + 2.0
            y[jump] = np.nan
            pre = np.isnan(y) & ~np.isnan(g)
            first = np.argmax(~np.isnan(y)) if (~np.isnan(y)).any() else len(y)
            y[:first][pre[:first]] = g[:first][pre[:first]]
    return y


def inputs(case):
    """control increments from the trace that drove both (GenHel's, which has the test's increments on its own trim)"""
    t = np.array(case['t'])
    inc = {}
    for v in GEAR:
        y = curve(case, v, 'genhel')
        if y is None or np.isnan(y).all(): y = curve(case, v, 'test')
        ok = ~np.isnan(y)
        y = np.interp(t, t[ok], y[ok])
        inc[v] = y - np.median(y[t < 0.5])
    return t, inc


def primary(case):
    """the test input's control (from the case title) and the time it starts: 30 % of the nominal amplitude"""
    d = case['input']
    k = 'ped' if 'pedal' in d else 'coll' if 'collective' in d else 'lon' if ('aft' in d or 'forward' in d) else 'lat'
    amp = 0.5 if d.startswith('0.5') else 1.0
    t, inc = inputs(case)
    big = np.abs(inc[k]) > 0.3 * amp
    return k, t[np.argmax(big)] if big.any() else 1.0


def hover_trim(f, pba, alt=5250.0, dT=0.0):
    """the hover as in the test: the test autopilot (genap) flies it steady with the SAS on, out of ground effect;
    then hands off, and the SAS off at the start of the record (1-2.5 s before the input)"""
    import genap
    f['atmosphere/delta-T'] = dT
    f['ic/terrain-elevation-ft'] = min(0.0, alt - 1000.0); f['ic/h-sl-ft'] = alt; f['ic/vt-kts'] = 0.0; f['ic/psi-true-deg'] = 0.0
    f['ic/theta-deg'] = 4.5; f['ic/phi-deg'] = -2.5
    f['ic/p-rad_sec'] = 0; f['ic/q-rad_sec'] = 0; f['ic/r-rad_sec'] = 0
    f['eng/init-running'] = 1
    f.run_ic()
    f['ctl/sas1'] = 1.0; f['ctl/sas2'] = 1.0; f['ctl/pba'] = 1.0 if pba else 0.0
    H.controls(f, 0.6, 0.0, 0.08, -0.5)
    s = genap.Sim(f, 3)
    s.step(); s.step(); s['engage'] = 1
    s['target/u-fps'] = 0; s['target/v-fps'] = 0
    for _ in range(int(50 / f['simulation/dt'])): s.step()
    # hands off with the SAS on for a few seconds (the autopilot's soft loops leave a small 1.2 Hz roll-yaw motion,
    # which the SAS damps), then the SAS off
    for ax in ('coll', 'lon', 'lat', 'yaw'): s['axis-' + ax] = 0
    for _ in range(int(6 / f['simulation/dt'])): s.step()
    f['ctl/sas1'] = 0.0; f['ctl/sas2'] = 0.0
    coll = 1 - f[genap.CTL['thr']]
    return [coll, f['fcs/elevator-cmd-norm'], f['fcs/aileron-cmd-norm'], -f['fcs/rudder-cmd-norm']], abs(f['velocities/h-dot-fps']) + abs(s['internal/u-fps']) + abs(s['internal/v-fps'])


def residual_hover(f, x, V_kt, alt_ft=1000.0, gamma=0.0, settle_s=1.5, revs=2, wings_level=False, beta=0.0):
    """harness.residual for the hover: x[4] is the pitch attitude (angle of attack means nothing at rest)"""
    coll, lon, lat, ped, th, ph = x
    H.controls(f, coll, lon, lat, ped)
    # JSBSim sets the pitch through alpha + gamma, which do not exist at rest: zero body velocities instead
    f['ic/vt-kts'] = 0.0; f['ic/u-fps'] = 0.0; f['ic/v-fps'] = 0.0; f['ic/w-fps'] = 0.0; f['ic/psi-true-deg'] = 0.0
    f['ic/theta-deg'] = th; f['ic/phi-deg'] = ph
    f['ic/h-sl-ft'] = alt_ft; f['ic/terrain-elevation-ft'] = H.TERRAIN_FT
    f['ic/p-rad_sec'] = 0; f['ic/q-rad_sec'] = 0; f['ic/r-rad_sec'] = 0
    f.run_ic()
    for _ in range(int(settle_s / f['simulation/dt'])):
        f.run()
    n = int(round(revs * 2 * math.pi / f['rotor/omega-rad_sec'] / f['simulation/dt']))
    acc = np.zeros(6); pw = 0.0
    for _ in range(n):
        f.run()
        acc += [f[k] for k in H.ACC]; pw += f['aero/rotor/power-hp']
    return acc / n, pw / n


def trim_hover(f, pba, model='uh60a-ballin', root=None, alt=5250.0):
    """Newton trim of the hover from the autopilot's hover (hover_trim) as the first guess"""
    g = H.load(root=root, model=model)
    x0, _ = hover_trim(g, pba, alt, f['atmosphere/delta-T'])
    th, ph = g['attitude/theta-deg'], g['attitude/phi-deg']
    f['ctl/sas1'] = 0.0; f['ctl/sas2'] = 0.0; f['ctl/pba'] = 1.0 if pba else 0.0
    keep = H.residual
    H.residual = residual_hover
    try:
        x, res, hp = H.trim(f, 0.0, alt_ft=alt, iters=20, x0=list(x0) + [th, ph])
    finally:
        H.residual = keep
    H.freeze(f, False); f['ctl/init'] = 0
    return x, res


def simulate(args):
    """args: (file, pba[, model root[, {property: value} set after the trim (e.g. a frozen stabilator)]])"""
    fid, pba = args[:2]
    root = args[2] if len(args) > 2 else None
    after = args[3] if len(args) > 3 else {}
    case = D[fid]
    w, cgx, cgz, da, eas, pa = COND[fid]
    model = case_model(fid, root)
    f = H.load(root=root, model=model)
    h = test_day(f, pa, da)
    if eas == 0:
        x, res = trim_hover(f, pba, model, root, h)
    else:
        f['ctl/sas1'] = 0.0; f['ctl/sas2'] = 0.0; f['ctl/pba'] = 1.0 if pba else 0.0
        x, res, hp = H.trim(f, tas(eas, da), alt_ft=h, iters=20)
        H.freeze(f, False); f['ctl/init'] = 0
    f['stab/auto'] = 0          # fixed at the trim value set by the control system [Ballin app. C]
    coll, lon, lat, ped = x[:4]
    for k, val in after.items():
        f[k] = f['stab/pos-deg'] if val == 'trim' else val
    t, inc = inputs(case)
    dt = f['simulation/dt']
    rec = []
    for i in range(int(round(6.5 / dt)) + 1):
        tt = i * dt
        d = {v: float(np.interp(tt, t, inc[v])) * GEAR[v] for v in GEAR}
        H.controls(f, coll + d['coll'], lon + d['lon'], lat + d['lat'], ped + d['ped'])
        f.run()
        rec.append((tt, math.degrees(f['velocities/p-rad_sec']), math.degrees(f['velocities/q-rad_sec']), math.degrees(f['velocities/r-rad_sec']),
                    f['attitude/phi-deg'], f['attitude/theta-deg'], f['attitude/psi-deg']))
    r = np.array(rec)
    m = {}
    r[:, 6] = np.degrees(np.unwrap(np.radians(r[:, 6])))     # heading through 0/360 before interpolating
    for j, v in enumerate(('p', 'q', 'r', 'phi', 'theta', 'psi'), 1):
        m[v] = np.interp(t, r[:, 0], r[:, j])
    m['trim'] = dict(coll=float(coll), lon=float(lon), lat=float(lat), ped=float(ped))
    return fid, pba, m, float(np.abs(res).max())


def rel(y, t, t0):
    """increment from the value just before the input"""
    w = (t >= t0 - 0.6) & (t <= t0 - 0.1)
    b = y[w][~np.isnan(y[w])]
    return y - (np.median(b) if len(b) else np.nan)


if __name__ == '__main__':
    pba = '--pba' in sys.argv
    files = [a for a in sys.argv[1:] if not a.startswith('--')] or list(D)
    with Pool(min(len(files), os.cpu_count() or 4)) as pool:
        runs = pool.map(simulate, [(fid, pba) for fid in files])
    os.makedirs(OUT, exist_ok=True)
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    print('SAS off, PBA %s.  On-axis rate in the 2.5 s after the input (increments from trim, deg/s): peak, and RMS error vs test'
          % ('on' if pba else 'off'))
    print('fig   file  cond    input                          axis   test peak  GenHel  model   | RMS err GenHel  model   | off-axis peak test/GenHel/model')
    rows = []
    for fid, _, m, res in runs:
        case = D[fid]; t = np.array(case['t'])
        k, t0 = primary(case)
        ax = AXIS[k]
        off = {'p': 'q', 'q': 'p', 'r': 'p'}[ax] if k != 'coll' else 'q'
        w = (t >= t0) & (t <= t0 + 2.5)
        def pk(y):
            if y is None: return np.nan
            y = rel(y, t, t0)[w]
            return y[np.nanargmax(np.abs(y))] if np.isfinite(y).any() else np.nan
        test, gh = curve(case, ax, 'test'), curve(case, ax, 'genhel')
        to, go = curve(case, off, 'test'), curve(case, off, 'genhel')
        rm = lambda a, b: float(np.sqrt(np.nanmean((rel(a, t, t0)[w] - rel(b, t, t0)[w]) ** 2))) if a is not None and b is not None else np.nan
        line = '%-4s  %4s  %-6s  %-30s %s   %7.1f  %7.1f  %6.1f   |  %6.1f      %6.1f   | %5.1f / %5.1f / %5.1f' % (
            case['figure'], fid, case['condition'], case['input'][:30], ax, pk(test), pk(gh), pk(m[ax]), rm(gh, test), rm(m[ax], test),
            pk(to), pk(go), pk(m[off]))
        print(line + ('   (trim residual %.2f)' % res if res > 0.05 else ''))
        rows.append((case, m, t, t0))
    # plots: one page per case, increments of p, q, r, phi, theta, psi
    for case, m, t, t0 in rows:
        fig, axs = plt.subplots(3, 2, figsize=(11, 8), sharex=True)
        for a, v, lab in zip(axs.T.flat, ('p', 'q', 'r', 'phi', 'theta', 'psi'),
                             ('roll rate', 'pitch rate', 'yaw rate', 'roll', 'pitch', 'heading')):
            for who, st, name in (('test', 'k--', 'USAAEFA flight test'), ('genhel', 'b-', 'GenHel (Ames)')):
                y = curve(case, v, who)
                if y is not None: a.plot(t, rel(y, t, t0), st, lw=1.2, label=name)
            a.plot(t, rel(m[v], t, t0), 'r-', lw=1.5, label='this model')
            a.axvline(t0, color='0.8', lw=0.8)
            a.set_ylabel(lab + (', deg/s' if v in 'pqr' else ', deg')); a.grid(alpha=0.3)
        axs[0, 0].legend(fontsize=8)
        for a in axs[-1]: a.set_xlabel('time, s')
        fig.suptitle('%s (file %s): %s, %s - SAS off, PBA %s' % (case['figure'], case['file'], case['input'], case['condition'], 'on' if pba else 'off'))
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, '%s_%s%s.png' % (case['figure'], case['file'], '_pba' if pba else '')), dpi=80)
        plt.close(fig)
    print('plots in', OUT)
