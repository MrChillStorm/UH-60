"""The main rotor wake's induced velocity at the tail rotor, from the same skewed-cylinder ring wake as trwake.py
(uniform loading, Heyson & Katzoff NACA TR 1319): rings parallel to the tip-path plane, leaving the hub along the
skew line at chi from the plane's normal, towards where the air carries them (against the aircraft's in-plane
motion). Averaged over the tail rotor's disc and normalised by the velocity at the main rotor's centre (the
model's mean inflow).

GenHel computes the tail rotor's air "in the same manner as the empennage" [GH 3]: with the stabilator's and fin's
wash factors (EKX, EKZ against the wake skew and a1), which put the far wake's 2 v0 on it in any wind direction.
The tail rotor hub is 1.21 R aft of the main rotor's and 0.8 ft above it, at the edge of the wake when that is
blown aft and well clear of it when it is blown sideways or forward.

Tip-path plane: the shaft's 3 deg forward tilt less the longitudinal flapping a1 (+ = disc aft, as GenHel's
tables); lateral flapping is left out. The ground's image is left out (the tests are at 1.4 R)."""
import math, os, json
import numpy as np
from scipy.integrate import quad_vec
import uh60data as U
from trwake import ring

CHI = [0, 10, 20, 30, 40, 50, 60, 70, 80, 85, 90]      # deg, wake skew from the plane's normal
PSI = list(range(0, 361, 30))                          # deg, direction of the aircraft's in-plane motion: 0 forward, 90 right
A1 = [-6.0, 0.0, 6.0]                                  # deg, longitudinal flapping, + = disc aft
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'mrwake_tr.json')


def wake_velocity(P, n, s_hat, chi):
    """induced air velocity at P (body axes, rotor radii from the hub) of a semi-infinite cylinder of unit rings
    parallel to the disc, axis n into the wake, skewed by chi towards s_hat"""
    w = math.cos(chi) * n + math.sin(chi) * s_hat

    def vel(s):
        d = P - s * w
        z = float(d @ n)
        rv = d - z * n
        r = float(np.linalg.norm(rv))
        uz, ur = ring(r, z)
        return uz * n + (ur * rv / r if r > 1e-9 else 0.0)

    pts = [x for x in (float(P @ n) / math.cos(chi) if abs(math.cos(chi)) > 1e-6 else None, float(P @ w)) if x is not None and 0 < x < 60]
    return quad_vec(vel, 0, 60, limit=500, points=pts or None)[0] + quad_vec(vel, 60, np.inf, limit=200)[0]


def mr_frame(a1_deg):
    """wake axis (into the wake, down), in-plane forward and right unit vectors of the tip-path plane (body axes)"""
    tilt = U.SHAFT_TILT - math.radians(a1_deg)
    up = np.array([math.sin(tilt), 0.0, -math.cos(tilt)])
    a = np.array([math.cos(tilt), 0.0, math.sin(tilt)])
    return -up, a, np.array([0.0, 1.0, 0.0])


def tr_disc_points(nr=5, na=12):
    """points over the tail rotor's disc (body axes, main rotor radii from the main rotor hub) with area weights"""
    from trwake import frame
    t, a0, b0 = frame()
    hub = np.array([-(U.FS_TR - U.FS_MR), U.BL_TR - U.BL_MR, -(U.WL_TR - U.WL_MR)]) / 12.0
    out = []
    for i in range(nr):
        r0, r1 = U.R_TR * math.sqrt(i / nr), U.R_TR * math.sqrt((i + 1) / nr)       # equal-area annuli
        rm = math.sqrt((r0 * r0 + r1 * r1) / 2)
        for j in range(na):
            ph = 2 * math.pi * (j + 0.5) / na
            out.append(((hub + rm * (math.cos(ph) * a0 + math.sin(ph) * b0)) / U.R, 1.0 / (nr * na)))
    return out


def fin_points():
    """points over the fin's planform (trwake's strips; body axes, main rotor radii from its hub) with area weights"""
    from trwake import fin_points as fp
    off = np.array([-(U.FS_TR - U.FS_MR), U.BL_TR - U.BL_MR, -(U.WL_TR - U.WL_MR)]) / 12.0
    pts = [(P + off, A) for strip in fp(nc=4, ns=3) for P, A in strip[0]]
    tot = sum(A for P, A in pts)
    return [(P / U.R, A / tot) for P, A in pts]


def stab_points(ns=12, nc=4):
    """points over the stabilator's planform [79-24 para 22: span 172.6 in, chords 44 / 30.5 in, unswept quarter
    chord] at GenHel's station (FS 700.1, WL 244), level (its incidence left out), with area weights"""
    from data import genhel_empennage as E
    half, cr, ct = 172.6 / 2, 44.0, 30.5
    pts, tot = [], 0.0
    for i in range(ns):
        y = -half + (i + 0.5) * 2 * half / ns
        c = cr + (ct - cr) * abs(y) / half
        for j in range(nc):
            fs = E.FS_H - c / 4 + (j + 0.5) * c / nc
            A = c / nc * 2 * half / ns
            pts.append((np.array([-(fs - U.FS_MR), y - U.BL_MR, -(E.WL_H - U.WL_MR)]) / 12.0 / U.R, A)); tot += A
    return [(P, A / tot) for P, A in pts]


TARGETS = {'tr': tr_disc_points, 'fin': fin_points, 'stab': stab_points}


def _case(args):
    c, p, a1, target = args
    chi = math.radians(min(c, 89.5))
    n, a, b = mr_frame(a1)
    s_hat = -(math.cos(math.radians(p)) * a + math.sin(math.radians(p)) * b)
    w0 = float(wake_velocity(np.zeros(3), n, s_hat, chi) @ n)
    k = sum(wake_velocity(P, n, s_hat, chi) * wt for P, wt in TARGETS[target]()) / w0
    return k.tolist()


def tables(target='tr'):
    """k[a1][chi][psi] = (kx, ky, kz): the main rotor's induced air velocity, mean over the tail rotor's disc (or the
    fin's planform), over the main rotor's mean inflow, body axes"""
    from multiprocessing import Pool
    cases = [(c, p, a1, target) for a1 in A1 for c in CHI for p in PSI]
    with Pool() as pool:
        res = dict(zip(cases, pool.map(_case, cases)))
    return {'chi': CHI, 'psi': PSI, 'a1': A1,
            'k': [[[res[(c, p, a1, target)] for p in PSI] for c in CHI] for a1 in A1]}


def path(target):
    return DATA if target == 'tr' else DATA.replace('_tr.json', '_%s.json' % target)


def load(target='tr'):
    """the tables, from data/mrwake_<target>.json (python3 mrwake.py --write [fin] recomputes them)"""
    return json.load(open(path(target)))


if __name__ == '__main__':
    import sys
    target = 'fin' if 'fin' in sys.argv else 'stab' if 'stab' in sys.argv else 'tr'
    comp = 1 if target == 'fin' else 2
    if '--write' in sys.argv:
        json.dump(tables(target), open(path(target), 'w'), indent=0)
        print('wrote', path(target))
    d = load(target)
    for ia, a1 in enumerate(d['a1']):
        print('a1 %+.0f: k%s over the %s; rows chi, columns psi (aircraft motion: 0 fwd, 90 right)' % (a1, 'xyz'[comp], target))
        print('      ' + ' '.join('%6d' % p for p in d['psi']))
        for ic, c in enumerate(d['chi']):
            print('%5d ' % c + ' '.join('%6.2f' % d['k'][ia][ic][ip][comp] for ip in range(len(d['psi']))))
