"""The tail rotor's induced velocity at the fin. Wake = skewed cylinder of ring vortices (uniform loading, as
heyson.py, after Heyson & Katzoff NACA TR 1319): rings parallel to the disc, their centres leaving the hub along
the wake's skew line, at chi from the disc's axis towards the in-plane flow. Evaluated over the fin's planform and
normalised by the velocity at the disc's centre (the model's uniform inflow), it gives the fin's sidewash from the
tail rotor at any skew: the hover's fin blockage and its forward-flight remainder from one wake.

Hub frame as the model's: t the thrust axis (right and up, cant 20 deg), a0 aft, b0 = t x a0; the wake leaves
along -t. Fin: the trapezoid of the principal dimensions [79-24 para 22; GH table 4.1]: span 8 ft 2 in, 32.3 ft2,
taper ratio 1.623, quarter chord swept 41 deg, vertical in the plane BL 0, its mean aerodynamic chord's quarter
point at GenHel's fin station (FS 695, WL 273) [GH table 5.3]; cut into spanwise strips."""
import math, os, json
import numpy as np
from scipy.special import ellipk, ellipe
from scipy.integrate import quad_vec
import uh60data as U

FIN_SPAN, FIN_TAPER, FIN_SWEEP = 98.0, 1.623, math.radians(41.0)     # in, root / tip chord, quarter chord sweep


def planform():
    """(WL, leading edge FS, trailing edge FS) at the root and the tip, inches"""
    from data import genhel_empennage as E
    c_mean = E.S_V * 144.0 / FIN_SPAN
    ct = 2 * c_mean / (1 + FIN_TAPER); cr = FIN_TAPER * ct; lam = ct / cr
    y_mac = FIN_SPAN / 3 * (1 + 2 * lam) / (1 + lam)                      # height of the mean aerodynamic chord
    wl0 = E.WL_V - y_mac
    qc0 = E.FS_V - y_mac * math.tan(FIN_SWEEP)
    qc1 = qc0 + FIN_SPAN * math.tan(FIN_SWEEP)
    return (wl0, qc0 - cr / 4, qc0 + 3 * cr / 4), (wl0 + FIN_SPAN, qc1 - ct / 4, qc1 + 3 * ct / 4)
NSTRIP = 4
CHI = list(range(0, 181, 10))           # deg, wake skew from -t (0: straight down the axis, 90: edgewise, 180: up the axis)
PSI = list(range(0, 361, 30))           # deg, in-plane flow direction from a0 (aft) towards b0 (down)


def ring(r, z, R=1.0):
    """axial and radial velocity of a unit-circulation ring (radius R) at radial distance r, axial distance z"""
    a = (R + r) ** 2 + z * z
    m = min(4 * R * r / a, 1 - 1e-12)
    K, E = ellipk(m), ellipe(m)
    d = (R - r) ** 2 + z * z
    uz = (K + (R * R - r * r - z * z) / d * E) / (2 * math.pi * math.sqrt(a))
    ur = z / (2 * math.pi * r * math.sqrt(a)) * (-K + (R * R + r * r + z * z) / d * E) if r > 1e-9 else 0.0
    return uz, ur


def frame():
    cK, sK = math.cos(U.CANT_TR), math.sin(U.CANT_TR)
    t = np.array([0.0, cK, -sK]); a0 = np.array([-1.0, 0.0, 0.0]); b0 = np.cross(t, a0)
    return t, a0, b0


def induced(P, chi, psi, R=1.0):
    """induced air velocity (body axes, per unit centre velocity) at P (body axes, from the hub, in rotor radii)"""
    t, a0, b0 = frame()
    n = -t
    s_hat = math.cos(psi) * a0 + math.sin(psi) * b0
    w = math.cos(chi) * n + math.sin(chi) * s_hat

    def vel(s):
        d = P - s * w
        z = float(d @ n)
        rv = d - z * n
        r = float(np.linalg.norm(rv))
        uz, ur = ring(r, z, R)
        return uz * n + (ur * rv / r if r > 1e-9 else 0.0)

    # break points: where the rings' plane passes P, and where the cylinder's wall comes nearest
    pts = [x for x in (float(P @ n) / math.cos(chi) if abs(math.cos(chi)) > 1e-6 else None, float(P @ w)) if x is not None and 0 < x < 40]
    return quad_vec(vel, 0, 40, limit=400, points=pts or None)[0] + quad_vec(vel, 40, np.inf, limit=200)[0]


def centre(chi, psi):
    t = frame()[0]
    return float(induced(np.zeros(3), chi, psi) @ -t)


def fin_points(nc=6, ns=4):
    """per strip: [(P body ft from the hub, area ft2)] and the strip's (FS, WL, area)"""
    (w0, le0, te0), (w1, le1, te1) = planform()
    hub = np.array([U.FS_TR, U.BL_TR, U.WL_TR])
    strips = []
    for k in range(NSTRIP):
        pts, area, fs_a, wl_a = [], 0.0, 0.0, 0.0
        for j in range(ns):
            f = (k + (j + 0.5) / ns) / NSTRIP
            wl = w0 + f * (w1 - w0); le = le0 + f * (le1 - le0); te = te0 + f * (te1 - te0)
            dh = (w1 - w0) / NSTRIP / ns
            for i in range(nc):
                fs = le + (i + 0.5) / nc * (te - le)
                A = (te - le) / nc * dh / 144.0
                P = np.array([-(fs - hub[0]), 0.0 - hub[1], -(wl - hub[2])]) / 12.0
                pts.append((P, A)); area += A
            fs_a += (le + 0.25 * (te - le)) * (te - le) * dh / 144.0; wl_a += wl * (te - le) * dh / 144.0
        strips.append((pts, fs_a / area, wl_a / area, area))
    return strips


def _case(args):
    c, p = args
    ch, ps = math.radians(min(c, 179.9) if c else 0.0), math.radians(p)
    ch = math.radians(89.9) if c == 90 else ch
    w0 = centre(ch, ps)
    return [(sum(induced(P / U.R_TR, ch, ps) * A for P, A in pts) / area / w0)[:2].tolist() for pts, fs, wl, area in fin_points()]


def tables():
    """per strip: (FS of the quarter chord, WL, area ft2) and k[x|y][chi][psi], the area-mean induced air velocity
    over the strip (body x and y) over the disc centre's"""
    from multiprocessing import Pool
    cases = [(c, p) for c in CHI for p in PSI]
    uniq = [(c, p if c not in (0, 180) else 0) for c, p in cases]
    with Pool() as pool:
        res = dict(zip(sorted(set(uniq)), pool.map(_case, sorted(set(uniq)))))
    out = []
    for k, (pts, fs, wl, area) in enumerate(fin_points()):
        kx = [[res[(c, p if c not in (0, 180) else 0)][k][0] for p in PSI] for c in CHI]
        ky = [[res[(c, p if c not in (0, 180) else 0)][k][1] for p in PSI] for c in CHI]
        out.append(dict(fs=fs, wl=wl, area=area, kx=kx, ky=ky))
    return out


DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'trwake_fin.json')


def load():
    """the tables, from data/trwake_fin.json (python3 trwake.py --write recomputes it)"""
    return json.load(open(DATA))


if __name__ == '__main__':
    import sys
    t, a0, b0 = frame()
    print('fin root WL %.1f FS %.1f-%.1f, tip WL %.1f FS %.1f-%.1f' % (*planform()[0], *planform()[1]))
    for pts, fs, wl, area in fin_points():
        P = sum(P * A for P, A in pts) / area
        print('strip FS %.0f WL %.0f %.1f ft2: downstream %.2f R, in-plane %.2f R' % (fs, wl, area, -P @ t / U.R_TR,
              math.hypot(P @ a0, P @ b0) / U.R_TR))
    if '--write' in sys.argv:
        json.dump(dict(chi=CHI, psi=PSI, strips=tables()), open(DATA, 'w'), indent=0)
        print('wrote', DATA)
