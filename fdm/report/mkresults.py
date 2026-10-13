"""Run the validation programme for the report and cache the results in report/results/<stage>.json.

  python3 report/mkresults.py [stage ...]      (default: all stages; a stage's cache is rewritten when run)

Stages: b1 (level-flight trims, Ballin fig. B1), latdir (lateral-directional trims, USAAEFA), dyn (Ballin app. D
control responses), hover (OGE hover power), level (level-flight power), fm (rotor figure of merit and a blade
element momentum reference), ge (ground effect, hover flown by the test autopilot), derivs (stability derivatives
at Hilbert's loading), split (power budget in the hover and in cruise)."""
import sys, os, math, json, time
HERE = os.path.dirname(os.path.abspath(__file__))
FDM = os.path.dirname(HERE)
sys.path.insert(0, FDM); os.chdir(FDM)
import numpy as np
from multiprocessing import Pool
import harness as H, dynamics as DY, uh60data as U
OUT = os.path.join(HERE, 'results')
A = math.pi * U.R ** 2
SIGMA = U.NB * U.CHORD / (math.pi * U.R)


def rho_std(h):
    return 0.0023769 * (1 - 6.8756e-6 * h) ** 4.2559


# ---------------------------------------------------------------- level-flight trims [Ballin fig. B1]
B1_SPEEDS = (1, 10, 22, 30, 40, 50, 60, 72, 85, 95, 105, 117, 128, 139, 150, 162)


def b1(_=None):
    f = H.load(model='uh60a-ballin')
    rows, x0 = [], None
    for V in B1_SPEEDS:
        x, r, pw = H.trim(f, V, alt_ft=5250.0, x0=x0, iters=25)
        x0 = x
        lon, lat, coll, ped = H.pct(x)
        rows.append(dict(V=V, lon=lon, lat=lat, coll=coll, ped=ped, theta=f['attitude/theta-deg'], phi=f['attitude/phi-deg'],
                         hp=pw, stab=f['stab/pos-deg'], beta=f['aero/beta-deg'], resid=float(np.abs(r).max())))
    keys = ('V', 'lon', 'lat', 'coll', 'ped', 'theta', 'phi', 'hp', 'stab')
    return dict(model=rows, test=[dict(zip(keys, t)) for t in H.BALLIN_B1])


# ---------------------------------------------------------------- lateral-directional trims [USAAEFA 79-24, ADA180404]
def _latdir(c):
    import latdir
    latdir.ROOT = None
    return latdir.case(c)


def latdir_stage(_=None):
    import latdir
    from data import usaaefa_latdir as LD
    C = latdir.CLIMB
    cases = [(60, 0, False, 0), (100, 0, False, 0), (140, 0, False, 0), (60, 0, True, 0), (100, 0, True, 0), (140, 0, True, 0),
             (100, C, False, 0), (100, -C, False, 0), (100, C, True, 0), (100, -C, True, 0), (100, 0, False, 2.1), (100, 0, False, -4.1)]
    with Pool(len(cases)) as p:
        res = p.map(_latdir, cases)
    m = {('%d_%s_%s_%g' % (V, 'climb' if g > 0 else 'descent' if g < 0 else 'level', 'wl' if wl else 'b0', bl)): r
         for (V, g, wl, bl), r in zip(cases, res)}
    S, at0 = LD.SWEEPS, latdir.at0
    test = {}
    for V, fig in ((60, 20), (100, 25), (140, 22)):
        test['%d_level' % V] = dict(roll=at0(S[fig]['roll']), lat=at0(S[fig]['lat']), ped=100 * at0(S[fig]['ped']) / 6)
    for name in ('descent', 'climb'):
        test['100_' + name] = dict(roll=S['24_' + name]['roll'][4][1], ped=100 * S['24_trim']['ped'] / 6)
    for side, bl in (('right', 2.1), ('left', -4.1)):
        t = S['23_' + side]
        test['100_bl%g' % bl] = dict(roll=at0(t['roll'], t['trim']), lat=at0(t['lat'], t['trim']))
    return dict(model=m, test=test, climb_deg=C)


# ---------------------------------------------------------------- control responses [Ballin app. D]
def dyn(_=None):
    with Pool(4) as p:
        runs = p.map(DY.simulate, [(fid, False) for fid in DY.D])
    return {fid: dict(resid=res, **{k: [float(x) for x in v] for k, v in m.items()}) for fid, _, m, res in runs}


# ---------------------------------------------------------------- hover and level-flight power [USAAEFA 77-17, 83-24]
def hover(_=None):
    import performance as P
    with Pool(4) as p:
        res = p.map(P.hover, P.HOVER_W)
    return [dict(w=w, resid=r, ct=ct, cp=cp, ct_tr=ctt, cp_tr=cpt, mr_hp=d['aero/rotor/power-hp'], tr_hp=d['aero/rotor/tr/power-hp'],
                 loss_hp=d['drive/loss-ftlb'] * d['rotor/omega-rad_sec'] / 550) for w, r, ct, cp, ctt, cpt, d in res]


def level(_=None):
    import performance as P
    with Pool(4) as p:
        res = p.map(P.level, P.LEVEL_CT)
    return {str(ct): [dict(mu=mu, cp=cp, resid=r) for mu, cp, r in rows] for ct, rows in res}


# ---------------------------------------------------------------- rotor figure of merit (sea level standard)
FM_W = [7000, 9000, 11000, 13000, 15500, 18000, 20500, 22500]


def _fm(w):
    key = 'fm%d' % w
    DY.COND[key] = (w, 355.0, 245.0, 0, 0, 0)
    model = DY.case_model(key)
    f = H.load(model=model)
    x, res = DY.trim_hover(f, False, model, None, DY.test_day(f, 0, 0))
    H.freeze(f)
    n = int(round(8 * 2 * math.pi / f['rotor/omega-rad_sec'] / f['simulation/dt']))
    T = P = 0.0
    for _ in range(n):
        f.run(); T += f['aero/rotor/thrust-lbs'] / n; P += f['aero/rotor/power-hp'] / n
    rho = f['atmosphere/rho-slugs_ft3']; Vt = f['rotor/omega-rad_sec'] * U.R
    ct = T / (rho * A * Vt ** 2); cp = P * 550 / (rho * A * Vt ** 3)
    return dict(w=w, resid=float(np.abs(res).max()), ct=ct, cp=cp, fm=ct ** 1.5 / math.sqrt(2) / cp, mtip=Vt / f['atmosphere/a-fps'])


def bemt(cts, inflow, tip, n=120):
    """hover blade element momentum reference with the model's blade (SC1095 tables + DCD, twist, swept tip), sea
    level, Mtip 0.649: inflow 'uniform' (one momentum balance for the disc) or 'annulus' (per annulus); tip 'B' (lift
    cut at B R, as GenHel) or 'F' (Prandtl's tip loss factor). Returns the figure of merit at C_T/sigma."""
    from data import genhel_sc1095 as AF
    rho, a = 0.0023769, 1116.4
    R, OM, NB, c = U.R, U.OMEGA, U.NB, U.CHORD

    def clcd(al, M):
        al = max(-32, min(32, al)); M = max(0, min(1, M))
        i = min(len(AF.MACH) - 2, int(M / 0.1)); fm = (M - AF.MACH[i]) / 0.1
        f = lambda tab: (lambda lo, hi: lo + fm * (hi - lo))(np.interp(al, AF.ALPHA_LO, tab[AF.MACH[i]]), np.interp(al, AF.ALPHA_LO, tab[AF.MACH[i + 1]]))
        return f(AF.CL_LO), f(AF.CD_LO) + U.DCD

    def sect(r, d, v, th0, fl):
        UT = OM * r; phi = math.atan2(v, UT); al = th0 + U.twist(r) - math.degrees(phi)
        M = math.hypot(UT, v) / a * (math.cos(U.TIP_SWEEP) if r > U.TIP_SWEEP_R * R else 1)
        cl, cd = clcd(al, M); q = 0.5 * rho * (UT * UT + v * v) * c * d
        return NB * q * (cl * fl * math.cos(phi) - cd * math.sin(phi)), NB * r * q * (cl * fl * math.sin(phi) + cd * math.cos(phi))

    def rotor(th0):
        rs = np.linspace(U.R_ROOT, R, n + 1); rm = 0.5 * (rs[1:] + rs[:-1]); dr = np.diff(rs)
        cut = lambda r: 1.0 if (tip == 'F' or r < U.TIP_LOSS * R) else 0.0
        T = Q = 0.0
        if inflow == 'uniform':
            v = 30.0
            for _ in range(80):
                T = Q = 0.0
                for r, d in zip(rm, dr):
                    dT, dQ = sect(r, d, v, th0, cut(r)); T += dT; Q += dQ
                v = 0.5 * v + 0.5 * math.sqrt(max(T, 1) / (2 * rho * A))
            return T, Q
        for r, d in zip(rm, dr):
            v = 30.0
            for _ in range(300):
                phi = math.atan2(v, OM * r)
                F = 2 / math.pi * math.acos(min(1, math.exp(-NB / 2 * (1 - r / R) / max(1e-4, math.sin(phi))))) if tip == 'F' else 1.0
                dT, dQ = sect(r, d, v, th0, cut(r))
                v = 0.7 * v + 0.3 * math.sqrt(max(dT, 0) / (4 * math.pi * rho * r * d * F))
            T += dT; Q += dQ
        return T, Q

    out = []
    for cts in cts:
        T0 = cts * SIGMA * rho * A * (OM * R) ** 2; lo, hi = 0.0, 25.0
        for _ in range(30):
            th = 0.5 * (lo + hi)
            lo, hi = (th, hi) if rotor(th)[0] < T0 else (lo, th)
        T, Q = rotor(th)
        out.append(T ** 1.5 / math.sqrt(2 * rho * A) / (Q * OM))
    return out


def _bemt(args):
    return bemt(*args)


def fm(_=None):
    with Pool(4) as p:
        model = p.map(_fm, FM_W)
        cts = [0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10]
        ref = p.map(_bemt, [(cts, 'uniform', 'B'), (cts, 'annulus', 'F')])
    # Norman (AHS Forum 2002) fig. 22, UH-60 rotor figure of merit from flight and wind tunnel tests, mean of the data
    # read off the figure (+-0.01)
    norman = [(0.045, 0.60), (0.056, 0.65), (0.067, 0.69), (0.077, 0.72), (0.088, 0.74), (0.0965, 0.755)]
    return dict(model=model, bemt_cts=cts, bemt_uniform_B=ref[0], bemt_annulus_F=ref[1], norman=norman)


# ---------------------------------------------------------------- ground effect: the test autopilot flies the hover
GE_HEIGHTS = ((5, 50), (10, 20), (20, 20), (40, 25), (100, 45))


def ge(_=None):
    import genap as AP
    f = H.load(model='uh60a')
    f['eng/init-running'] = 1
    f['ic/terrain-elevation-ft'] = 0.0; f['ic/h-agl-ft'] = 5.9; f['ic/vt-kts'] = 0; f['ic/psi-true-deg'] = 0
    f.run_ic()
    H.controls(f, 0.0, 0, 0, 0)
    s = AP.Sim(f, 3)
    dt = f['simulation/dt']

    def run(sec, rec=None):
        for i in range(int(sec / dt)):
            brk = 1.0 if s['internal/h-wheel-ft'] < 1.0 and f['simulation/sim-time-sec'] < 6 else 0.0
            f['fcs/left-brake-cmd-norm'] = brk; f['fcs/right-brake-cmd-norm'] = brk
            s.step()
            if rec is not None and i * dt > sec - 8:
                rec.append((s['internal/h-wheel-ft'], f['eng[0]/torque-pct'] + f['eng[1]/torque-pct'], f['inertia/weight-lbs'],
                            f['atmosphere/density-altitude']))
    run(3); s['engage'] = 1; run(1)
    out = []
    for h, sec in GE_HEIGHTS:
        s['target/height-ft'] = h
        rec = []; run(sec, rec)
        a = np.array(rec)
        out.append(dict(height=h, wheel_ft=float(a[:, 0].mean()), wheel_sd=float(a[:, 0].std()), torque=float(a[:, 1].mean() / 2),
                        weight=float(a[:, 2].mean()), da=float(a[:, 3].mean())))
    return out


# ---------------------------------------------------------------- stability derivatives [Hilbert TM-85890 tables 5-10]
GENHEL_DERIVS = dict(Xu=-0.03238, Xw=0.06427, Xq=2.788, Zu=-0.008851, Zw=-0.7897, Zq=4.981, Mu=0.002507, Mw=0.009212, Mq=-1.606,
                     Xe=-1.083, Ze=-6.138, Me=0.4594, Xc=0.6461, Zc=-9.630, Mc=0.09507)


def _derivs(cfg):
    pba, stab_auto = cfg
    DY.COND['hilbert'] = (16400, 360.4, 247.2, 0, 100, 0)
    keep = DY.INERTIA
    DY.INERTIA = (5629, 40000, 37200, 1670)                # Hilbert table 1
    try:
        model = DY.case_model('hilbert')
    finally:
        DY.INERTIA = keep
    f = H.load(model=model)
    f['ctl/sas1'] = 0.0; f['ctl/sas2'] = 0.0; f['ctl/pba'] = pba
    x, r, hp = H.trim(f, 100.0, alt_ft=1000, iters=20)
    f['ctl/init'] = 0
    if not stab_auto:
        f['stab/auto'] = 0
    u0, v0, w0 = f['velocities/u-fps'], f['velocities/v-fps'], f['velocities/w-fps']
    th, ph = f['attitude/theta-deg'], f['attitude/phi-deg']
    m = f['inertia/weight-lbs'] / 32.174; Iyy = f['inertia/iyy-slugs_ft2']

    def at(du=0.0, dw=0.0, q=0.0, de=0.0, dc=0.0):
        H.controls(f, x[0] + dc, x[1] + de, x[2], x[3])
        f['ic/u-fps'] = u0 + du; f['ic/v-fps'] = v0; f['ic/w-fps'] = w0 + dw
        f['ic/theta-deg'] = th; f['ic/phi-deg'] = ph; f['ic/psi-true-deg'] = 0.0
        f['ic/h-sl-ft'] = 1000.0; f['ic/terrain-elevation-ft'] = H.TERRAIN_FT
        f['ic/p-rad_sec'] = 0; f['ic/q-rad_sec'] = q; f['ic/r-rad_sec'] = 0
        f.run_ic()
        for _ in range(int(2.0 / f['simulation/dt'])):
            f.run()
        n = int(round(4 * 2 * math.pi / f['rotor/omega-rad_sec'] / f['simulation/dt']))
        acc = np.zeros(3)
        for _ in range(n):
            f.run(); acc += [f['forces/fbx-aero-lbs'] / m, f['forces/fbz-aero-lbs'] / m, f['moments/m-aero-lbsft'] / Iyy]
        return acc / n
    H.freeze(f)
    d = {}
    for nm, kw, h in (('u', 'du', 1.0), ('w', 'dw', 1.0), ('q', 'q', math.radians(5.0))):
        dd = (at(**{kw: h}) - at(**{kw: -h})) / (2 * h)
        d['X' + nm], d['Z' + nm], d['M' + nm] = [float(v) for v in dd]
    de = (at(de=-0.02) - at(de=0.02)) / 0.2              # per inch aft (Hilbert: delta_e + aft)
    dc = (at(dc=0.01) - at(dc=-0.01)) / 0.2              # per inch up
    d['Xe'], d['Ze'], d['Me'] = [float(v) for v in de]
    d['Xc'], d['Zc'], d['Mc'] = [float(v) for v in dc]
    return dict(pba=pba, stab_auto=stab_auto, resid=float(np.abs(r).max()), d=d)


def derivs(_=None):
    with Pool(2) as p:
        res = p.map(_derivs, [(1, 1), (0, 0)])
    return dict(genhel=GENHEL_DERIVS, augmented=res[0], aero=res[1])


# ---------------------------------------------------------------- power budget
def _split(case):
    kind, w_or_ct, mu = case
    segs, _ = U.segments()
    if kind == 'hover':
        DY.COND['sph'] = (w_or_ct, 355.0, 245.0, 3700, 0, 2300)
        model = DY.case_model('sph')
        f = H.load(model=model)
        x, res = DY.trim_hover(f, False, model, None, DY.test_day(f, 2300, 3700))
        res = float(np.abs(res).max())
        rho = f['atmosphere/rho-slugs_ft3']
    else:
        rho = rho_std(5250)
        DY.COND['spc'] = (w_or_ct * 1e-4 * rho * A * (U.OMEGA * U.R) ** 2, 351.0, 231.5, 5250, 0, 5250)
        f = H.load(model=DY.case_model('spc'))
        x, r, pw = H.trim(f, mu * U.OMEGA * U.R / 1.68781, alt_ft=5250.0, iters=25)
        res = float(np.abs(r).max())
    H.freeze(f)
    n = int(round(8 * 2 * math.pi / f['rotor/omega-rad_sec'] / f['simulation/dt']))
    acc = np.zeros(11)
    for _ in range(n):
        f.run()
        P0 = sum(f['aero/rotor/b%d/s%d/Dk' % (k, j)] * (f['aero/rotor/b%d/s%d/V2' % (k, j)] + f['aero/rotor/b%d/s%d/UR' % (k, j)] ** 2)
                 for k in range(U.NB) for j in range(len(segs)))
        u, v, w = f['velocities/u-aero-fps'], f['velocities/v-aero-fps'], f['velocities/w-aero-fps']
        V = math.sqrt(u * u + v * v + w * w)
        Pp = f['aero/rotor/FX-lbs'] * u + f['aero/rotor/FY-lbs'] * v + f['aero/rotor/FZ-lbs'] * w
        drag = -(f['aero/fus/FX-lbs'] * u + f['aero/fus/FY-lbs'] * v + f['aero/fus/FZ-lbs'] * w + f['aero/emp/h/Fx-lbs'] * u
                 + f['aero/emp/h/Fz-lbs'] * w + f['aero/emp/v/Fx-lbs'] * u + f['aero/emp/v/Fy-lbs'] * v) / max(V, 1e-3)
        acc += [f['aero/rotor/power-hp'], P0 / 550, Pp / 550, f['aero/rotor/thrust-lbs'], V, drag, f['aero/rotor/tr/power-hp'],
                f['drive/loss-ftlb'] * f['rotor/omega-rad_sec'] / 550, f['aero/fus/FZ-lbs'] + f['aero/emp/h/Fz-lbs'], f['inertia/weight-lbs'],
                f['rotor/omega-rad_sec']]
    Pmr, P0, Pp, T, V, Dr, Ptr, Pl, dl, W, Om = acc / n
    Pi = Pmr - P0 - Pp
    Pid = (T * T / (2 * rho * A * V) if kind != 'hover' else T ** 1.5 / math.sqrt(2 * rho * A)) / 550
    Vt = Om * U.R
    return dict(kind=kind, resid=res, weight=W, mu=V / Vt, ct=W / (rho * A * Vt ** 2), mr_hp=Pmr, profile_hp=P0, propulsive_hp=Pp,
                induced_hp=Pi, ideal_hp=Pid, kappa=Pi / Pid, tr_hp=Ptr, loss_hp=Pl, download_lb=dl, thrust_lb=T,
                fe_ft2=(Dr / (0.5 * rho * V * V) if kind != 'hover' else float('nan')), shp=Pmr + Ptr + Pl,
                cp=(Pmr + Ptr + Pl) * 550 / (rho * A * Vt ** 3))


def split(_=None):
    with Pool(3) as p:
        return p.map(_split, [('hover', 17000, 0), ('cruise', 70, 0.22), ('cruise', 70, 0.30)])


STAGES = dict(b1=b1, latdir=latdir_stage, dyn=dyn, hover=hover, level=level, fm=fm, ge=ge, derivs=derivs, split=split)

if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    for name in [a for a in sys.argv[1:] if a in STAGES] or list(STAGES):
        t0 = time.time()
        res = STAGES[name]()
        json.dump(res, open(os.path.join(OUT, name + '.json'), 'w'), indent=1, default=float)
        print('%-7s %5.0f s' % (name, time.time() - t0), flush=True)
