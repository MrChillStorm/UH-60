"""Hover yaw derivatives by component, quasi-static: the airframe held in a trimmed hover while the rotors, wake, engines
and governor settle at an imposed yaw rate, side velocity or pedal. N_r, N_v and N_ped from central differences of the
yaw moment, split into the tail rotor (thrust, in-plane force and hub moments), the fin, the fuselage and the rest (the
main rotor's drive torque and hub moments). Conditions: TM-88360 case 209 (the hover pedal step) and TM-85890's 1-kt
condition (16,400 lb, FS 360.4, sea level), whose model gives N_r -0.334, N_v 0.0098 and N_ped 0.604 [HB table 10].
For comparison, flight identification (frequency domain, RASCAL and Airloads UH-60A in hover, 15,350 lb, with rotor
speed and engine states of their own) gives N_r -0.445 1/s [Fletcher, NASA TM 110362, 1995].
python3 yawdamp.py [root ...]"""
import sys, os, math
import numpy as np
import harness as H, dynamics as DY

FIN_X = None        # fin strip stations, read from trwake when the fin is in strips


def _fin_x():
    import trwake
    return [st['fs'] for st in trwake.load()['strips']]


def parts(f, cgx):
    """yaw moment about the CG (ft lb): total, tail rotor, fin, fuselage, rest (main rotor)"""
    try:
        tr = (cgx - 732) / 12 * f['aero/rotor/tr/FY-lbs'] - 14 / 12 * f['aero/rotor/tr/FX-lbs'] \
            + 0.93969262 * f['aero/rotor/tr/Mb-ftlb'] - 0.34202014 * f['aero/rotor/tr/Mt-ftlb']
    except KeyError:                       # Bailey's rotor: thrust only
        tr = (cgx - 732) / 12 * 0.93969262 * f['aero/rotor/tr/thrust-lbs']
    try:
        fin = sum((cgx - x) / 12 * f['aero/emp/v%d/Fy-lbs' % k] for k, x in enumerate(_fin_x()))
    except KeyError:                       # one fin (GenHel's blockage)
        fin = (cgx - 695) / 12 * f['aero/emp/v/Fy-lbs']
    fus = f['aero/fus/N-ftlb'] + (cgx - 358) / 12 * f['aero/fus/FY-lbs']
    tot = f['moments/n-aero-lbsft']
    return np.array([tot, tr, fin, fus, tot - tr - fin - fus])


def held(f, x, h, th, ph, r=0.0, vy=0.0, settle=4.0, revs=4):
    H.controls(f, *x[:4])
    f['ic/vt-kts'] = 0.0; f['ic/u-fps'] = 0.0; f['ic/v-fps'] = vy; f['ic/w-fps'] = 0.0; f['ic/psi-true-deg'] = 0.0
    f['ic/theta-deg'] = th; f['ic/phi-deg'] = ph; f['ic/h-sl-ft'] = h; f['ic/terrain-elevation-ft'] = H.TERRAIN_FT
    f['ic/p-rad_sec'] = 0; f['ic/q-rad_sec'] = 0; f['ic/r-rad_sec'] = r
    f.run_ic()
    for _ in range(int(settle / f['simulation/dt'])):
        f.run()
    n = int(round(revs * 2 * math.pi / f['rotor/omega-rad_sec'] / f['simulation/dt']))
    w = np.sin(np.pi * (np.arange(n) + 0.5) / n) ** 2
    w /= w.sum()
    cgx = f['inertia/cg-x-in']
    acc = np.zeros(5)
    for i in range(n):
        f.run()
        acc += w[i] * parts(f, cgx)
    return acc


CONDS = {'209': None, 'hb': ((16400, 360.4, 247.2, 0, 0, 0), (5629, 40000, 37200, 1670))}


def run(args):
    root, cond = args
    if CONDS[cond]:
        DY.COND['hb1'], inertia = CONDS[cond]
        keep = DY.INERTIA
        DY.INERTIA = inertia
        try:
            model = DY.case_model('hb1', root)
        finally:
            DY.INERTIA = keep
        key = 'hb1'
    else:
        model, key = DY.case_model(cond, root), cond
    pa, da = DY.COND[key][5], DY.COND[key][3]
    f = H.load(root=root, model=model)
    h = DY.test_day(f, pa, da)
    x, res = DY.trim_hover(f, False, model, root, h)
    th, ph = f['attitude/theta-deg'], f['attitude/phi-deg']
    f['ctl/init'] = 1
    H.freeze(f)
    Izz = f['inertia/izz-slugs_ft2']
    dr, dv, dp = math.radians(5.0), 3.0, 0.1 / 2.69
    Nr = (held(f, x, h, th, ph, r=dr) - held(f, x, h, th, ph, r=-dr)) / (2 * dr) / Izz
    Nv = (held(f, x, h, th, ph, vy=dv) - held(f, x, h, th, ph, vy=-dv)) / (2 * dv) / Izz
    xl, xr = list(x), list(x)
    xl[3] -= dp; xr[3] += dp
    Np = (held(f, xl, h, th, ph) - held(f, xr, h, th, ph)) / 0.2 / Izz          # per inch of left pedal
    names = ('total', 'tr', 'fin', 'fus', 'rest')
    return dict(cond=cond, resid=float(np.abs(res).max()), Izz=Izz, Nr=dict(zip(names, Nr.tolist())),
                Nv=dict(zip(names, Nv.tolist())), Nped=dict(zip(names, Np.tolist())))


def stage(root=None):
    from multiprocessing import Pool
    with Pool(len(CONDS)) as p:
        return {d['cond']: d for d in p.map(run, [(root, c) for c in CONDS])}


if __name__ == '__main__':
    for root in [a for a in sys.argv[1:] if os.path.isdir(a)] or [None]:
        for c, d in stage(root).items():
            print('== %s %s (trim residual %.3f)' % (root or 'root', c, d['resid']))
            for k in ('Nr', 'Nv', 'Nped'):
                print('   %-5s ' % k + '  '.join('%s %+.4f' % (n, v) for n, v in d[k].items()))
