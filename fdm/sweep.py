"""Level-flight trim sweep against Ballin TM-88360 figure B1 (USAAEFA flight test and Sikorsky GenHel)."""
import sys, numpy as np
import harness as H

def run(model='uh60a-ballin', speeds=(1, 22, 30, 50, 72, 95, 117, 139, 162), alt=5250.0, wings_level=False):
    f = H.load(model=model)
    test = {int(r[0]): r for r in H.BALLIN_B1}
    x0 = None; out = []
    print(' kt |  lon%  test |  lat%  test | coll%  test |  ped%  test | pitch  test | roll  test |   hp  test | stab test | resid')
    for V in speeds:
        x, r, pw = H.trim(f, V, alt_ft=alt, x0=x0, iters=25, wings_level=wings_level)
        x0 = x
        lon, lat, coll, ped = H.pct(x)
        th, ph, st = f['attitude/theta-deg'], f['aero/beta-deg'] if wings_level else f['attitude/phi-deg'], f['stab/pos-deg']
        t = min(H.BALLIN_B1, key=lambda q: abs(q[0] - V))
        t = t if abs(t[0] - V) <= 3 else (V,) + (float('nan'),) * 8
        print('%3d | %5.1f %5.1f | %5.1f %5.1f | %5.1f %5.1f | %5.1f %5.1f | %5.1f %5.1f | %4.1f %5.1f | %4.0f %5.0f | %4.1f %4.1f | %.2f' % (
            V, lon, t[1], lat, t[2], coll, t[3], ped, t[4], th, t[5], ph, t[6], pw, t[7], st, t[8], np.abs(r).max()))
        out.append((V, lon, lat, coll, ped, th, ph, pw, st))
    return out

if __name__ == '__main__':
    run(wings_level='level' in sys.argv[1:])
