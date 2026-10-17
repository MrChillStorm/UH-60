"""Tail rotor power in level flight against the UH-60A Airloads flights [data/yeo_trpower.py]: trims at each weight
coefficient over the advance ratios flown, 16,500 lb at the density giving that C_W, mid CG (FS 352, WL 232),
standard day, wings level as the flights were flown (sideslip free; --beta=deg for a fixed sideslip instead, + nose
right; --beta0 = --beta=0). The flights' sideslip was not published: the pilots held the wings level, which takes some
sideslip [Yeo], and the aircraft carried the ESSS fairings, which add 2-3 deg of nose-left inherent sideslip (ADA181947).
python3 trpower.py [root ...] [--beta=deg | --beta0] [--json=file]"""
import sys, os, math, json
import numpy as np
from multiprocessing import Pool
import harness as H, dynamics as DY, uh60data as U
from data import yeo_trpower as Y

W = 16500.0
BETA = next((float(a.split('=', 1)[1]) for a in sys.argv[1:] if a.startswith('--beta=')), 0.0 if '--beta0' in sys.argv else None)
WINGS_LEVEL = BETA is None
MUS = (0.08, 0.10, 0.13, 0.16, 0.20, 0.24, 0.28, 0.31, 0.34, 0.37)


def density_alt(rho):
    return (1 - (rho / 0.0023769) ** (1 / 4.2559)) / 6.8756e-6


def run(args):
    """args: (root, C_W[, sideslip deg: fixed; None: wings level, the module's default from the command line])"""
    root, cw = args[:2]
    beta = args[2] if len(args) > 2 else BETA
    VT = U.OMEGA * U.R
    rho = W / (cw * math.pi * U.R ** 2 * VT ** 2)
    da = density_alt(rho)
    key = 'yeo%d' % round(cw * 1e4)
    DY.COND[key] = (W, 352.0, 232.0, da, 0, da)
    model = DY.case_model(key, root)
    f = H.load(root=root, model=model)
    h = DY.test_day(f, da, da)
    mmax = max(m for m, c in Y.TR_POWER[cw]) + 0.015
    out, x0 = [], None
    for mu in MUS:
        if mu > mmax:
            break
        V = mu * VT / 1.68781
        x, r, pw = H.trim(f, V, alt_ft=h, x0=x0, iters=25, wings_level=beta is None, beta=beta or 0.0)
        if np.abs(r).max() < 0.05:
            x0 = x
        H.freeze(f)
        n = int(round(4 * 2 * math.pi / f['rotor/omega-rad_sec'] / f['simulation/dt']))
        P = T = 0.0
        for _ in range(n):
            f.run(); P += f['aero/rotor/tr/power-hp'] / n; T += f['aero/rotor/tr/thrust-lbs'] / n
        H.freeze(f, False)
        rho_m = f['atmosphere/rho-slugs_ft3']; Vt = f['rotor/omega-rad_sec'] * U.OMEGA_TR / U.OMEGA * U.R_TR
        cp = P * 550 / (rho_m * math.pi * U.R_TR ** 2 * Vt ** 3)
        out.append(dict(mu=mu, V=V, cp_tr=cp, tr_hp=P, tr_lb=T, ped=H.pct(x)[3], mr_hp=pw, resid=float(np.abs(r).max()),
                        beta=f['aero/beta-deg'], phi=f['attitude/phi-deg']))
    return root, cw, da, out


if __name__ == '__main__':
    roots = [a for a in sys.argv[1:] if os.path.isdir(a)] or [None]
    jobs = [(r, cw) for r in roots for cw in Y.TR_POWER]
    with Pool(min(len(jobs), os.cpu_count() or 4)) as p:
        res = p.map(run, jobs)
    outp = next((a.split('=', 1)[1] for a in sys.argv[1:] if a.startswith('--json=')), None)
    if outp:
        json.dump([dict(root=r, cw=cw, da=da, rows=o) for r, cw, da, o in res], open(outp, 'w'), indent=1)
    for r, cw, da, o in res:
        test = Y.TR_POWER[cw]
        def tint(mu):
            ms = [m for m, c in test]; cs = [c for m, c in test]
            return float(np.interp(mu, ms, cs)) if ms[0] <= mu <= ms[-1] else float('nan')
        print('%-60s C_W %.4f DA %5.0f: ' % (r or 'root', cw, da) + ' '.join('%.2f:%.2f/%.2f%s' % (q['mu'], q['cp_tr'] * 1e3, tint(q['mu']) * 1e3, '*' if q['resid'] > 0.05 else '')
                                                                              for q in o))
