"""Lateral-directional trim vs the USAAEFA flight test (data/usaaefa_latdir.py): roll, lateral cyclic and pedal at
zero sideslip, and the wings-level (ball-centered) sideslip, in level flight, 1500 fpm climb/descent and with a
lateral cg offset.  python3 latdir.py [root]"""
import sys, math, json
from multiprocessing import Pool
import harness as H
from data import usaaefa_latdir as D

ROOT = sys.argv[1] if len(sys.argv) > 1 else None
CLIMB = math.degrees(math.asin(25.0 / (100 * 1.68781)))     # 1500 fpm at 100 KTAS


def case(args):
    V, gam, wl, bl = args
    f = H.load(root=ROOT, model='uh60a-ballin')
    if bl:   # move the heaviest point mass sideways to put the cg at BL bl
        k = max(range(6), key=lambda i: f['inertia/pointmass-weight-lbs[%d]' % i])
        f['inertia/pointmass-location-Y-inches[%d]' % k] += bl * f['inertia/weight-lbs'] / f['inertia/pointmass-weight-lbs[%d]' % k]
    x, r, pw = H.trim(f, V, alt_ft=5000, gamma=gam, iters=25, wings_level=wl)
    return dict(beta=f['aero/beta-deg'], roll=f['attitude/phi-deg'], lat=5 * (1 + x[2]), ped=50 * (1 + x[3]), hp=pw)


def at0(pts, b0=0.0):
    """value at beta b0 from the two points around it"""
    pts = sorted(pts)
    for (b1, v1), (b2, v2) in zip(pts, pts[1:]):
        if b1 <= b0 <= b2:
            return v1 + (v2 - v1) * (b0 - b1) / (b2 - b1)


if __name__ == '__main__':
    cases = [(60, 0, False, 0), (100, 0, False, 0), (140, 0, False, 0), (60, 0, True, 0), (100, 0, True, 0),
             (140, 0, True, 0), (100, CLIMB, False, 0), (100, -CLIMB, False, 0), (100, CLIMB, True, 0),
             (100, -CLIMB, True, 0), (100, 0, False, 2.1), (100, 0, False, -4.1)]
    with Pool(len(cases)) as p:
        res = dict(zip(cases, p.map(case, cases)))
    S = D.SWEEPS
    print('zero sideslip          roll deg (test / model)   lat in from left   pedal % from left')
    for V, fig in ((60, 20), (100, 25), (140, 22)):
        m = res[(V, 0, False, 0)]
        print('  %3d KTAS level      %+5.1f / %+5.1f            %4.2f / %4.2f        %4.1f / %4.1f' % (
            V, at0(S[fig]['roll']), m['roll'], at0(S[fig]['lat']), m['lat'], 100 * at0(S[fig]['ped']) / 6, m['ped']))
    for name, g in (('descent', -CLIMB), ('climb', CLIMB)):
        m = res[(100, g, False, 0)]
        print('  100 KTAS 1500 fpm %-7s %+5.1f / %+5.1f    (trim)   %4.2f     %4.1f / %4.1f' % (
            name, S['24_' + name]['roll'][4][1], m['roll'], m['lat'], 100 * S['24_trim']['ped'] / 6, m['ped']))
    for side, bl in (('right', 2.1), ('left', -4.1)):
        m = res[(100, 0, False, bl)]
        print('  100 KTAS cg BL %+4.1f  %+5.1f / %+5.1f            %4.2f / %4.2f' % (
            bl, at0(S['23_' + side]['roll'], S['23_' + side]['trim']), m['roll'], at0(S['23_' + side]['lat'], S['23_' + side]['trim']), m['lat']))
    print('wings-level sideslip (model)  60 / 100 / 140 KTAS: %+4.1f %+4.1f %+4.1f;  100 KTAS climb %+4.1f, descent %+4.1f' % (
        res[(60, 0, True, 0)]['beta'], res[(100, 0, True, 0)]['beta'], res[(140, 0, True, 0)]['beta'],
        res[(100, CLIMB, True, 0)]['beta'], res[(100, -CLIMB, True, 0)]['beta']))
    print('  test: -4 @ 67 KCAS to -1 @ 150 KCAS [79-24]; -1.5..+2 above 95 kt by C_T [85]; +5 IRP climb, -5 1000 fpm descent [85]')
