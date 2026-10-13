"""UH-60A fuselage aerodynamics (tail off, IR suppressors off), from Sikorsky GenHel:
NASA CR-166309 (Howlett 1981), section 5.2, Tables 2.5.1 to 2.5.11 (wind tunnel data).

Forces are D/q, Y/q, L/q in ft^2, moments M/q in ft^3, in wind-tunnel wind axes at the mounting point
FS 345.5, BL 0, WL 234.0 (incidence 0). Angles are the tunnel's, not Euler angles [GH 5.2-6]:
  alpha (ALFWF) = atan(Vz / |Vx|), body velocity relative to the air (down +)
  psi   (PSIWF) = yaw table angle = -beta, beta = atan(Vy / sqrt(Vx^2 + Vz^2))
Each map has a low-angle part (5 deg steps) and a high-angle part (10 deg steps) that agree where they overlap.
"""

FSWF, BLWF, WLWF = 345.5, 0.0, 234.0

# ---------------------------------------------------------------- rotor wash on the fuselage [Tables 2.5.1, 2.5.2]
# VXIWF = EKXWF v0, VZIWF = -EKZWF v0 (v0 = main rotor uniform downwash), against the wake skew angle (deg)
# and the longitudinal flapping a1 (deg, + = disc tilted aft)
WASH_CHI = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
WASH_A1 = [-6, 0, 6]
EKX = [[0.08, 0.18, 0.30, 0.43, 0.55, 0.66, 0.79, 0.90, 1.03, 0.55, 0.0],
       [0.00, 0.10, 0.21, 0.32, 0.42, 0.54, 0.66, 0.80, 0.94, 0.50, 0.0],
       [-0.12, 0.02, 0.08, 0.18, 0.28, 0.40, 0.53, 0.67, 0.82, 0.40, 0.0]]
EKZ = [[1.11, 1.09, 1.08, 1.065, 1.05, 1.04, 1.02, 1.01, 1.00, 0.88, 0.6],
       [1.12, 1.12, 1.12, 1.12, 1.12, 1.12, 1.12, 1.12, 1.11, 0.96, 0.6],
       [1.15, 1.15, 1.15, 1.15, 1.16, 1.17, 1.18, 1.22, 1.16, 0.98, 0.6]]

LO = list(range(-30, 31, 5))       # low-angle maps
HI = list(range(-90, 91, 10))      # high-angle maps
ABS_LO = list(range(0, 31, 5))     # maps of |psi|
ABS_HI = list(range(30, 91, 10))

# drag vs alpha [2.5.3]
DQ_LO = [45.08, 37.38, 31.68, 27.48, 25.06, 23.58, 23.58, 25.08, 27.58, 31.28, 36.58, 43.08, 51.08]
DQ_HI = [150.0, 145.0, 133.0, 114.0, 88.0, 61.0, 45.08, 31.68, 25.06, 23.58, 27.58, 36.58, 51.08,
         66.0, 84.0, 110.0, 132.0, 145.0, 150.0]
# incremental drag vs |psi| [2.5.4]
DDQ_LO = [0.0, 1.0, 4.0, 9.0, 16.3, 28.0, 38.5]
DDQ_HI = [38.5, 76.5, 113.5, 141.5, 164.5, 169.5, 170.5]
# side force vs |psi|, Y(psi) = -Y(-psi) [2.5.5]
YQ_LO = [0.0, 11.0, 23.0, 35.0, 50.0, 65.0, 72.0]
YQ_HI = [72.0, 92.0, 103.0, 100.0, 84.0, 64.0, 37.0]
# lift vs alpha [2.5.6]
LQ_LO = [-70.0, -52.0, -35.0, -25.0, -13.0, -5.0, 1.0, 10.0, 20.0, 25.0, 30.0, 34.0, 37.0]
LQ_HI = [-24.0, -54.0, -72.0, -81.0, -85.0, -83.0, -70.0, -35.0, -13.0, 1.0, 20.0, 30.0, 37.0,
         43.0, 48.0, 50.0, 48.0, 39.0, 22.0]
# incremental lift vs psi (low-angle map only) [2.5.7]
DLQ_LO = [30.0, 20.0, 12.0, 7.0, 3.0, 2.0, 0.0, 2.0, 5.0, 10.0, 15.0, 22.0, 30.0]
# rolling moment vs |psi|, R(psi) = -R(-psi) [2.5.8]
RQ_LO = [0.0, 0.0, 0.0, -30.0, -75.0, -120.0, -110.0]
RQ_HI = [-110.0, -106.0, -103.0, -101.0, -100.0, -100.0, -100.0]
# pitching moment vs alpha [2.5.9]
MQ_LO = [-740.0, -700.0, -630.0, -520.0, -380.0, -230.0, -90.0, 10.0, 100.0, 290.0, 450.0, 600.0, 750.0]
MQ_HI = [-200.0, -470.0, -645.0, -730.0, -760.0, -760.0, -740.0, -630.0, -380.0, -90.0, 100.0, 450.0, 750.0,
         810.0, 825.0, 730.0, 650.0, 470.0, 200.0]
# incremental pitching moment vs |psi| (low-angle map only) [2.5.10]
DMQ_LO = [0.0, 10.0, 20.0, 50.0, 90.0, 130.0, 180.0]
# yawing moment vs psi [2.5.11]
NQ_LO = [-140.0, -190.0, -240.0, -220.0, -180.0, -100.0, 0.0, 100.0, 180.0, 220.0, 240.0, 190.0, 140.0]
NQ_HI = [440.0, 392.0, 332.0, 259.0, 160.0, 40.0, -140.0, -240.0, -180.0, 0.0, 180.0, 240.0, 140.0,
         59.0, -30.0, -125.0, -220.0, -320.0, -420.0]


def merge(lo_x, lo_y, hi_x, hi_y):
    """one breakpoint list: the low-angle map inside its range, the high-angle map outside it"""
    a, b = lo_x[0], lo_x[-1]
    pts = dict((x, y) for x, y in zip(hi_x, hi_y) if x < a or x > b)
    pts.update(zip(lo_x, lo_y))
    xs = sorted(pts)
    return xs, [pts[x] for x in xs]


def check():
    for name, lo_x, lo, hi_x, hi in (('DQ', LO, DQ_LO, HI, DQ_HI), ('LQ', LO, LQ_LO, HI, LQ_HI),
                                     ('MQ', LO, MQ_LO, HI, MQ_HI), ('NQ', LO, NQ_LO, HI, NQ_HI),
                                     ('DDQ', ABS_LO, DDQ_LO, ABS_HI, DDQ_HI), ('YQ', ABS_LO, YQ_LO, ABS_HI, YQ_HI),
                                     ('RQ', ABS_LO, RQ_LO, ABS_HI, RQ_HI)):
        assert len(lo) == len(lo_x) and len(hi) == len(hi_x), name
        for x, y in zip(lo_x, lo):
            if x in hi_x and x % 10 == 0:
                assert hi[hi_x.index(x)] == y, (name, x, y, hi[hi_x.index(x)])
    for t in (EKX, EKZ):
        assert all(len(r) == len(WASH_CHI) for r in t)
    return True


if __name__ == '__main__':
    print('overlaps agree:', check())
