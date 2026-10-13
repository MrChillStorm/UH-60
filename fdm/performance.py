"""Hover and level-flight power against the USAAEFA flight tests [data/usaaefa_performance.py]: total engine shaft
power (main and tail rotor, drive losses, accessories) as C_P against C_T, and the tail rotor alone in the hover.
python3 performance.py [hover] [level] [root]"""
import sys, os, math
import numpy as np
from multiprocessing import Pool
import harness as H, dynamics as DY, uh60data as U
from data import usaaefa_performance as P

ROOT = next((a for a in sys.argv[1:] if os.path.isdir(a)), None)
A = math.pi * U.R ** 2
A_TR, R_TR, ARM_TR = 95.033, 5.5, 32.567          # the test's tail rotor reduction [77 app. D]
HOVER_W = [11000, 13000, 15000, 17000, 19000, 21000, 22000]
HOVER_PA, HOVER_DA = 2300, 3700                   # the 2302 ft test site, ~22 C
LEVEL_CT = [65, 70, 80, 90]
LEVEL_ALT = 5250.0                                # standard day: referred rotor speed 262.7 rpm


def average(f, keys, revs=8):
    H.freeze(f)
    n = int(round(revs * 2 * math.pi / f['rotor/omega-rad_sec'] / f['simulation/dt']))
    a = np.zeros(len(keys))
    for _ in range(n):
        f.run(); a += [f[k] for k in keys]
    H.freeze(f, False)
    return dict(zip(keys, a / n))


def shaft_hp(d):
    return d['aero/rotor/power-hp'] + d['aero/rotor/tr/power-hp'] + d['drive/loss-ftlb'] * d['rotor/omega-rad_sec'] / 550


KEYS = ['aero/rotor/power-hp', 'aero/rotor/tr/power-hp', 'drive/loss-ftlb', 'rotor/omega-rad_sec', 'atmosphere/rho-slugs_ft3',
        'aero/rotor/torque-ftlb']


def hover(w):
    key = 'oge%d' % w
    DY.COND[key] = (w, 355.0, 245.0, HOVER_DA, 0, HOVER_PA)
    model = DY.case_model(key, ROOT)
    f = H.load(root=ROOT, model=model)
    x, res = DY.trim_hover(f, False, model, ROOT, DY.test_day(f, HOVER_PA, HOVER_DA))
    d = average(f, KEYS)
    Om, rho = d['rotor/omega-rad_sec'], d['atmosphere/rho-slugs_ft3']
    q, Vt = rho * A * (Om * U.R) ** 2, Om * U.R
    ct, cp = w / q, shaft_hp(d) * 550 / (q * Vt)
    Vtr = U.OMEGA_TR / U.OMEGA * Om * R_TR; qtr = rho * A_TR * Vtr ** 2
    ct_tr, cp_tr = d['aero/rotor/torque-ftlb'] / ARM_TR / qtr, d['aero/rotor/tr/power-hp'] * 550 / (qtr * Vtr)
    return w, float(np.abs(res).max()), ct, cp, ct_tr, cp_tr, d


def level(ct):
    rho = 0.0023769 * (1 - 6.8756e-6 * LEVEL_ALT) ** 4.2559; Vt = U.OMEGA * U.R
    key = 'lf%d' % ct
    DY.COND[key] = (ct * 1e-4 * rho * A * Vt ** 2, 351.0, 231.5, LEVEL_ALT, 0, LEVEL_ALT)
    f = H.load(root=ROOT, model=DY.case_model(key, ROOT))
    out, x0 = [], None
    for mu in sorted(P.LEVEL):
        x, r, pw = H.trim(f, mu * Vt / 1.68781, alt_ft=LEVEL_ALT, x0=x0, iters=25); x0 = x
        d = average(f, KEYS)
        q = d['atmosphere/rho-slugs_ft3'] * A * (d['rotor/omega-rad_sec'] * U.R) ** 3
        out.append((mu, shaft_hp(d) * 550 / q, float(np.abs(r).max())))
    return ct, out


if __name__ == '__main__':
    do = [a for a in sys.argv[1:] if a in ('hover', 'level')] or ['hover', 'level']
    with Pool(4) as pool:
        if 'hover' in do:
            print('OGE hover, %d ft PA, %d ft DA: total engine C_P x 1e5, model / test [77 reanalysed in 6Y; ESSS = 6Y]' % (HOVER_PA, HOVER_DA))
            print('weight   C_T e4   model  test   ESSS   ratio  |  tail rotor: C_T e4 (net)  C_P e5  test  ratio  |  MR hp  TR hp  losses hp')
            for w, res, ct, cp, ct_tr, cp_tr, d in pool.map(hover, HOVER_W):
                t, te = P.hover_oge(ct), P.tail_rotor_hover(ct_tr)
                loss = d['drive/loss-ftlb'] * d['rotor/omega-rad_sec'] / 550
                print('%6d   %5.1f   %6.2f %6.2f %6.2f  %.3f  |  %5.1f%s  %6.1f %6.1f  %.3f  |  %5.0f  %4.0f  %4.0f%s' % (
                    w, ct * 1e4, cp * 1e5, t * 1e5, P.hover_oge(ct, True) * 1e5, cp / t, ct_tr * 1e4, ' ' if ct_tr >= 0.0102 else '*',
                    cp_tr * 1e5, te * 1e5, cp_tr / te, d['aero/rotor/power-hp'], d['aero/rotor/tr/power-hp'], loss,
                    '  (trim residual %.2f)' % res if res > 0.05 else ''))
            print('(* below the tail rotor data, line extrapolated)')
        if 'level' in do:
            res = dict(pool.map(level, LEVEL_CT))
            print('level flight, %.0f ft standard day: total engine C_P x 1e5, model / test [77 figs. 15-26]' % LEVEL_ALT)
            print('  mu ' + ''.join('     C_T %d: model test ratio' % c for c in LEVEL_CT))
            for i, mu in enumerate(sorted(P.LEVEL)):
                row = '%.2f ' % mu
                for c in LEVEL_CT:
                    m, r = res[c][i][1], res[c][i][2]; t = P.LEVEL[mu].get(c, float('nan'))
                    row += '          %5.1f %5.1f %.3f%s' % (m * 1e5, t, m * 1e5 / t, '*' if r > 0.05 else ' ')
                print(row)
            print('(* trim not converged: retreating blade stall)')
