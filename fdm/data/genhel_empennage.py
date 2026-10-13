"""UH-60A empennage aerodynamics, from Sikorsky GenHel: NASA CR-166309 (Howlett 1981), section 5.3,
Tables 3.5.1 to 3.5.10. Horizontal tail = the stabilator (incidence from the control system, trailing edge
down +); vertical tail = the fin (incidence 0).

Interference velocities, added to the local air velocity (body axes, x fwd, z down) [GH 5.3-5 to 5.3-10]:
  main rotor:  VX = EKX v0,  VZ = -EKZ v0          (v0 = main rotor uniform downwash; same maps for the fin)
  fuselage:    VZ(stab) = -Vx EPS / 57.3 * KQH,    VY(fin) = -Vx SIG / 57.3 * KQV
  free stream at the surface scaled by KQ = sqrt(QRATIO)
"""

FS_H, WL_H, BL_H, S_H, AR_H = 700.1, 244.0, 0.0, 45.0, 4.6      # NACA 0014
FS_V, WL_V, BL_V, S_V, AR_V = 695.0, 273.0, 0.0, 32.3, 1.92     # NACA 0021 mod, incidence 0

# main rotor wash at the tail [3.5.1, 3.5.2] against the wake skew angle (deg) and a1 (deg, + = disc aft)
WASH_CHI = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
WASH_A1 = [-6, 0, 6]
EKX = [[0.0, -0.2, 0.05, 0.3, 0.54, 0.8, 1.04, 1.3, 1.55, 0.8, 0.0],
       [-0.4, -0.6, -0.2, 0.12, 0.36, 0.6, 0.83, 1.06, 1.3, 0.66, 0.0],
       [-0.56, -0.8, -0.74, -0.32, 0.04, 0.32, 0.6, 0.86, 1.12, 0.54, 0.0]]
EKZ = [[-0.13, 0.8, 1.8, 1.82, 1.86, 1.88, 1.91, 1.94, 1.69, 1.42, 1.14],
       [0.4, 0.94, 1.84, 1.91, 1.98, 2.04, 2.08, 2.14, 1.89, 1.62, 1.35],
       [0.78, 1.36, 1.91, 1.98, 2.06, 2.14, 2.21, 2.28, 2.16, 1.96, 1.56]]

LO = list(range(-30, 31, 5))
HI = list(range(-90, 91, 10))
ABS_LO = list(range(0, 31, 5))
ABS_HI = list(range(30, 91, 10))

# dynamic pressure ratio at the stabilator vs fuselage alpha [3.5.3] (1 outside the map)
QH_LO = [1.0, 1.0, 0.95, 0.76, 0.76, 0.76, 0.76, 0.76, 0.76, 0.82, 0.91, 1.0, 1.0]
# fuselage downwash at the stabilator, deg, vs fuselage alpha [3.5.4]
EPS_LO = [1.8, 1.4, 1.1, 0.8, 0.55, 0.5, 0.45, 0.4, 0.38, 0.33, 0.19, -0.12, -0.4]
EPS_HI = [0.0, 0.25, 0.7, 1.2, 1.6, 1.9, 1.8, 1.1, 0.55, 0.45, 0.38, 0.19, -0.4, -0.7, -0.75, -0.65,
          -0.45, -0.15, 0.0]
# stabilator lift vs |alpha| (odd) [3.5.5], drag vs |alpha| (even) [3.5.6]
CLH_LO = [0.0, 0.356, 0.71, 1.03, 0.95, 0.795, 0.745]
CLH_HI = [0.745, 0.847, 0.847, 0.745, 0.558, 0.294, 0.0]
CDH_LO = [0.01, 0.022, 0.04, 0.19, 0.36, 0.37, 0.43]
CDH_HI = [0.43, 0.531, 0.702, 0.888, 1.05, 1.161, 1.2]
# dynamic pressure ratio at the fin vs |psi| [3.5.7] (1 beyond 30 deg)
QV_LO = [0.62, 0.64, 0.66, 0.72, 0.79, 0.88, 1.0]
# fuselage sidewash at the fin, deg, vs psi (odd) [3.5.8]
SIG_LO = [0.0, -0.4, -0.6, 0.8, 1.4, 0.6, 0.2]
SIG_HI_X = [0, 30, 60, 90]
SIG_HI = [0.0, 0.2, 0.0, 0.0]
# fin lift and drag vs beta [3.5.9, 3.5.10]
CLV_LO = [-1.0, -1.0, -0.93, -0.73, -0.5, -0.28, -0.06, 0.16, 0.38, 0.61, 0.82, 0.89, 0.89]
CLV_HI = [0.0, -0.12, -0.28, -0.46, -0.66, -0.88, -1.0, -0.93, -0.5, -0.06, 0.38, 0.82, 0.89, 0.8, 0.63,
          0.48, 0.32, 0.17, 0.0]
CDV_LO = [0.36, 0.265, 0.174, 0.118, 0.066, 0.033, 0.018, 0.021, 0.044, 0.092, 0.162, 0.248, 0.355]
CDV_HI = [1.1, 1.025, 0.965, 0.875, 0.745, 0.575, 0.36, 0.174, 0.066, 0.018, 0.044, 0.162, 0.355, 0.58,
          0.75, 0.875, 0.965, 1.02, 1.08]


def check():
    for name, lo_x, lo, hi_x, hi in (('EPS', LO, EPS_LO, HI, EPS_HI), ('CLV', LO, CLV_LO, HI, CLV_HI),
                                     ('CDV', LO, CDV_LO, HI, CDV_HI), ('CLH', ABS_LO, CLH_LO, ABS_HI, CLH_HI),
                                     ('CDH', ABS_LO, CDH_LO, ABS_HI, CDH_HI)):
        assert len(lo) == len(lo_x) and len(hi) == len(hi_x), name
        for x, y in zip(lo_x, lo):
            if x in hi_x:
                assert hi[hi_x.index(x)] == y, (name, x, y, hi[hi_x.index(x)])
    assert len(QH_LO) == len(LO) and len(QV_LO) == len(ABS_LO) and len(SIG_LO) == len(ABS_LO)
    return True


if __name__ == '__main__':
    print('overlaps agree:', check())
