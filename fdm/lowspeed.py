"""Low-speed flight in every wind direction against USAAEFA 77-17 [data/usaaefa_lowspeed.py]: trims at 10-40 KTAS
and relative wind azimuths 0-360 deg, in ground effect at the test's 25 ft wheel height, at each aircraft's weight,
CG and density altitude (Ballin's inertias, the CG on the centreline). Pedal (in. from full left) and tail rotor
shaft power. python3 lowspeed.py [root ...] [--json=file]   (several roots: one run each, e.g. tail rotor variants)"""
import sys, os, math, json
import numpy as np
from multiprocessing import Pool
import harness as H, dynamics as DY
from data import usaaefa_lowspeed as L

SPEEDS = (10, 20, 30, 35, 40)
CG_WL = 232.0           # in, not given: Ballin's test CGs sit at WL 231.6-232.7
WHEEL_WL = 176.18       # in, main wheel contact (the FDM's gear)


def pressure_alt(da, oat):
    """pressure altitude (ft) at which the air at this temperature (deg C) has this density altitude's density"""
    dens = 0.0023769 * (1 - 6.8756e-6 * da) ** 4.2559
    rho = lambda pa: 2116.22 * (1 - 6.8756e-6 * pa) ** 5.2559 / (1716.5 * (oat + 273.15) * 1.8)
    lo, hi = da - 5000.0, da + 5000.0
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if rho(mid) > dens else (lo, mid)
    return (lo + hi) / 2


def residual_wind(f, x, V_kt, alt_ft=1000.0, gamma=0.0, settle_s=1.5, revs=2, wings_level=False, beta=0.0):
    """harness.residual for level flight with the relative wind from any azimuth (beta here, deg from the nose,
    clockwise): x[4], x[5] are the pitch and roll attitudes and the body velocities follow from the earth-frame
    velocity (angle of attack and sideslip are no use at 90 deg off the nose)"""
    coll, lon, lat, ped, th, ph = x
    H.controls(f, coll, lon, lat, ped)
    V = V_kt * 1.68781
    vn, ve = V * math.cos(math.radians(beta)), V * math.sin(math.radians(beta))
    t, r = math.radians(th), math.radians(ph)
    f['ic/psi-true-deg'] = 0.0; f['ic/theta-deg'] = th; f['ic/phi-deg'] = ph
    f['ic/u-fps'] = math.cos(t) * vn
    f['ic/v-fps'] = math.sin(r) * math.sin(t) * vn + math.cos(r) * ve
    f['ic/w-fps'] = math.cos(r) * math.sin(t) * vn - math.sin(r) * ve
    f['ic/h-sl-ft'] = alt_ft; f['ic/terrain-elevation-ft'] = H.TERRAIN_FT
    f['ic/p-rad_sec'] = 0; f['ic/q-rad_sec'] = 0; f['ic/r-rad_sec'] = 0
    f.run_ic()
    for _ in range(int(settle_s / f['simulation/dt'])):
        f.run()
    n = int(round(revs * 2 * math.pi / f['rotor/omega-rad_sec'] / f['simulation/dt']))
    w = np.sin(np.pi * (np.arange(n) + 0.5) / n) ** 2
    w /= w.sum()
    acc = np.zeros(6); pw = 0.0
    for i in range(n):
        f.run()
        acc += w[i] * np.array([f[k] for k in H.ACC]); pw += w[i] * f['aero/rotor/power-hp']
    return acc, pw


def trim_wind(f, V, h, az, x0):
    keep = H.residual
    H.residual = residual_wind
    try:
        return H.trim(f, V, alt_ft=h, x0=x0, iters=25, beta=az)
    finally:
        H.residual = keep


def run(args):
    root, ac, V = args
    c = L.COND[ac]
    key = 'ls' + ac
    pa = pressure_alt(c['da'], c['oat'])
    DY.COND[key] = (c['w'], c['fs'], CG_WL, c['da'], 0, pa)
    model = DY.case_model(key, root)
    f = H.load(root=root, model=model)
    h = DY.test_day(f, pa, c['da'])
    H.TERRAIN_FT = h - (L.WHEEL_FT + (CG_WL - WHEEL_WL) / 12)
    tr = 'aero/rotor/tr/'
    # start from a conventional trim into wind, then walk round the azimuths in 15 deg steps
    x, r, pw = H.trim(f, V, alt_ft=h, iters=25)
    x0 = list(x[:4]) + [f['attitude/theta-deg'], f['attitude/phi-deg']]
    out = []
    for az in range(0, 361, 15):
        try:
            x, r, pw = trim_wind(f, V, h, az, x0)
            if not np.all(np.isfinite(r)):
                raise FloatingPointError('the model blew up')
        except (np.linalg.LinAlgError, FloatingPointError) as e:
            if az in L.AZ:
                out.append(dict(az=az, error=str(e)))
            f = H.load(root=root, model=model); h = DY.test_day(f, pa, c['da'])
            continue
        if np.abs(r).max() < 0.05:
            x0 = x
        if az not in L.AZ:
            continue
        H.freeze(f)
        n = int(round(2 * 2 * math.pi / f['rotor/omega-rad_sec'] / f['simulation/dt']))
        keys = (tr + 'power-hp', tr + 'thrust-lbs', 'aero/rotor/power-hp', 'aero/rotor/hub-agl-ft', 'aero/emp/v/Fy-lbs')
        acc = dict.fromkeys(keys, 0.0)
        for _ in range(n):
            f.run()
            for k in keys:
                acc[k] += f[k] / n
        H.freeze(f, False)
        out.append(dict(az=az, ped=3 * (1 + x[3]), coll=float(x[0]), lon=5 * (1 - x[1]), lat=5 * (1 + x[2]), resid=float(np.abs(r).max()), tr_hp=acc[tr + 'power-hp'],
                        tr_lb=acc[tr + 'thrust-lbs'], mr_hp=acc['aero/rotor/power-hp'], hub_agl=acc['aero/rotor/hub-agl-ft'],
                        fin_lb=acc['aero/emp/v/Fy-lbs'], theta=float(x[4]), phi=float(x[5])))
    return root, ac, V, out


def sweep(roots, log=None):
    jobs = [(r, ac, V) for r in roots for ac in L.COND for V in SPEEDS]
    res = []
    with Pool(min(len(jobs), os.cpu_count() or 4)) as p:
        for item in p.imap_unordered(run, jobs):
            res.append(item)
            if log:
                print('done %s %s %d kt (%d/%d)' % (item[0], item[1], item[2], len(res), len(jobs)), file=log, flush=True)
    return {r: {'%s_%d' % (ac, V): o for rr, ac, V, o in res if rr == r} for r in roots}


if __name__ == '__main__':
    roots = [a for a in sys.argv[1:] if os.path.isdir(a)] or [None]
    res = sweep(roots, sys.stderr)
    out = next((a.split('=', 1)[1] for a in sys.argv[1:] if a.startswith('--json=')), None)
    if out:
        json.dump(res, open(out, 'w'), indent=1)
    for r, d in res.items():
        print('== %s' % (r or 'root'))
        for k, rows in d.items():
            ac, V = k.split('_'); V = int(V)
            tp, th = L.PED[(ac, V)], L.TR_HP.get(V) if ac == '716' else None
            print('%s %2d kt  ped model/test: %s' % (ac, V, ' '.join('%4.2f/%s' % (q.get('ped', float('nan')), '%4.2f' % t if t is not None else ' -- ')
                                                                      for q, t in zip(rows, tp))))
            if th:
                print('          TR hp model/test: %s' % ' '.join('%3.0f/%s' % (q.get('tr_hp', float('nan')), '%3.0f' % t if t is not None else '--')
                                                              for q, t in zip(rows, th)))
            bad = [q['az'] for q in rows if q.get('resid', 1) > 0.05]
            if bad:
                print('          not trimmed at', bad)
