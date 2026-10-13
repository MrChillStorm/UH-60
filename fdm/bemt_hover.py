"""Hover check outside JSBSim: uniform inflow vs blade element momentum with Prandtl tip loss, GenHel SC1095 data."""
import math, numpy as np
import uh60data as U
from data import genhel_sc1095 as AF

rho = 0.002048; a_snd = 1097.0
W, R, OM, NB, c = U.OMEGA, U.R, U.OMEGA, U.NB, U.CHORD
def clcd(al, M):
    al = max(-32, min(32, al)); M = max(0, min(1, M))
    i = min(len(AF.MACH) - 2, int(M / 0.1)); fm = (M - AF.MACH[i]) / 0.1
    def interp(tab):
        xs = AF.ALPHA_LO
        lo = np.interp(al, xs, tab[AF.MACH[i]]); hi = np.interp(al, xs, tab[AF.MACH[i + 1]])
        return lo + fm * (hi - lo)
    return interp(AF.CL_LO), interp(AF.CD_LO) + U.DCD

def rotor(th0, mode, n=60):
    rs = np.linspace(U.R_ROOT, R, n + 1); rm = 0.5 * (rs[1:] + rs[:-1]); dr = np.diff(rs)
    A = math.pi * R * R
    T = 0; Q = 0
    if mode == 'uniform':
        v = 30.0
        for _ in range(60):
            T, Q = 0, 0
            for r, d in zip(rm, dr):
                UT = OM * r; phi = math.atan2(v, UT); al = th0 + U.twist(r) - math.degrees(phi)
                M = math.hypot(UT, v) / a_snd * (math.cos(U.TIP_SWEEP) if r > U.TIP_SWEEP_R * R else 1)
                cl, cd = clcd(al, M); q = 0.5 * rho * (UT * UT + v * v) * c * d
                fl = 1.0 if r < U.TIP_LOSS * R else 0.0
                T += NB * q * (cl * fl * math.cos(phi) - cd * math.sin(phi)); Q += NB * r * q * (cl * fl * math.sin(phi) + cd * math.cos(phi))
            v = 0.5 * v + 0.5 * math.sqrt(max(T, 1) / (2 * rho * A))
    else:  # annulus momentum with Prandtl tip loss F
        for r, d in zip(rm, dr):
            UT = OM * r; v = 30.0
            for _ in range(200):
                phi = math.atan2(v, UT)
                F = 2 / math.pi * math.acos(min(1, math.exp(-NB / 2 * (1 - r / R) / max(1e-4, math.sin(phi)) * 1)))
                al = th0 + U.twist(r) - math.degrees(phi)
                M = math.hypot(UT, v) / a_snd * (math.cos(U.TIP_SWEEP) if r > U.TIP_SWEEP_R * R else 1)
                cl, cd = clcd(al, M); q = 0.5 * rho * (UT * UT + v * v) * c * d
                dT = NB * q * (cl * math.cos(phi) - cd * math.sin(phi))
                vn = math.sqrt(max(dT, 0) / (4 * math.pi * rho * r * d * F))
                v = 0.7 * v + 0.3 * vn
            T += dT; Q += NB * r * q * (cl * math.sin(phi) + cd * math.cos(phi))
    return T, Q * OM / 550

for mode in ('uniform', 'bemt'):
    lo, hi = 10.0, 25.0
    for _ in range(40):
        th = 0.5 * (lo + hi); T, P = rotor(th, mode)
        if T < 16000 * 1.025: lo = th
        else: hi = th
    T, P = rotor(th, mode)
    print('%-8s theta0 %.2f deg (coll %.1f %%)  T %.0f  P %.0f hp  FM %.3f' % (mode, th, (th - 9.9) / 16 * 100, T, P, T ** 1.5 / math.sqrt(2 * rho * math.pi * R * R) / 550 / P))
