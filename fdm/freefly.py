"""Release the aircraft from a trim and fly it with fixed controls (plus optional steps); print a time history."""
import sys, math, numpy as np
import harness as H

def run(V=100.0, alt=5250.0, model='uh60a-ballin', T=20.0, steps=(), keys=None, every=1.0, x0=None):
    f = H.load(model=model)
    x, r, pw = H.trim(f, V, alt_ft=alt, iters=25, x0=x0)
    coll, lon, lat, ped = x[:4]
    H.freeze(f, False)
    f['ctl/init'] = 0
    keys = keys or ['velocities/vc-kts', 'attitude/theta-deg', 'attitude/phi-deg', 'attitude/psi-deg', 'velocities/q-rad_sec',
                    'velocities/p-rad_sec', 'velocities/r-rad_sec', 'velocities/h-dot-fps', 'drive/nr-pct', 'eng[0]/torque-pct',
                    'eng[0]/ng-pct', 'eng[0]/tgt-degc', 'stab/pos-deg']
    short = [k.split('/')[-1][:10] for k in keys]
    print('   t ' + ' '.join('%10s' % s for s in short))
    dt = f['simulation/dt']; n = int(T / dt); out = []
    for i in range(n + 1):
        t = i * dt
        c = [coll, lon, lat, ped]
        for (t0, idx, d) in steps:
            if t >= t0: c[idx] += d
        H.controls(f, *c)
        if i % int(every / dt) == 0:
            row = [f[k] for k in keys]
            out.append([t] + row)
            print('%5.1f ' % t + ' '.join('%10.3f' % (math.degrees(v) if 'rad_sec' in k else v) for k, v in zip(keys, row)))
        f.run()
    return f, out

if __name__ == '__main__':
    V = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
    steps = []
    if len(sys.argv) > 2:   # e.g. "5:0:0.1" = at 5 s, collective +0.1
        for s in sys.argv[2:]:
            t0, idx, d = s.split(':'); steps.append((float(t0), int(idx), float(d)))
    run(V, steps=steps)
