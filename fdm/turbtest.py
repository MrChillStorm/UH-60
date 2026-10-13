import sys, math, numpy as np
import harness as H
def run(V, agl, scale, T=30.0):
    f = H.load(model='uh60a')
    x, r, pw = H.trim(f, V, alt_ft=1000 if agl is None else agl + 10, iters=25)
    if agl is not None:
        pass
    H.freeze(f, False); f['ctl/init'] = 0
    f['rotor/turb/enable'] = 1; f['rotor/turb/scale'] = scale
    coll, lon, lat, ped = x[:4]
    dt = f['simulation/dt']; rec = []
    for i in range(int(T / dt)):
        H.controls(f, coll, lon, lat, ped); f.run()
        if i > int(2 / dt):
            rec.append([math.degrees(f['velocities/p-rad_sec']), math.degrees(f['velocities/q-rad_sec']), math.degrees(f['velocities/r-rad_sec']),
                        f['accelerations/Nz'], f['rotor/turb/sigma'], f['rotor/turb/mu-star'], f['position/h-agl-ft']])
    a = np.array(rec)
    sd = a[:, :4].std(axis=0)
    print('V %3d agl %s scale %.1f: sigma %.3f mu* %.2f | sd p %.2f q %.2f r %.2f deg/s  Nz %.3f g | final agl %.0f' % (V, agl, scale, a[:, 4].mean(), a[:, 5].mean(), *sd, a[-1, 6]))
for V, agl in ((1, None), (15, None), (25, None), (60, None), (120, None)):
    run(V, agl, 1.0, T=12)
