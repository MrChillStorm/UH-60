import harness as H, numpy as np, math, uh60data as U, sys
f = H.load(); H.freeze(f); f['ctl/init'] = 1
x = [float(a) for a in sys.argv[1:7]]; V = float(sys.argv[7]); alt = float(sys.argv[8])
r, pw = H.residual(f, x, V, alt)
print('resid', r.round(3), 'hp', round(pw), 'theta', f['attitude/theta-deg'])
segs, edges = U.segments()
n = int(round(2*math.pi/27/f['simulation/dt']))
Qp = np.zeros(8); Qi = np.zeros(8)
keys = ['aero/rotor/FX-lbs','aero/rotor/FZ-lbs','aero/fus/FX-lbs','aero/fus/FZ-lbs','aero/emp/h/Fx-lbs','aero/emp/h/Fz-lbs','aero/emp/v/Fx-lbs',
        'forces/fbx-aero-lbs','forces/fbz-aero-lbs','aero/rotor/thrust-lbs','aero/rotor/Fx-s','aero/rotor/Fz-s','aero/rotor/a1-deg','aero/rotor/b1-deg',
        'rotor/v0-fps','aero/rotor/tr/thrust-lbs','aero/rotor/tr/power-hp','moments/m-aero-lbsft','aero/rotor/MM-ftlb','aero/fus/M-ftlb','stab/pos-deg']
acc = {k:0 for k in keys}
for i in range(n):
    f.run()
    for k in keys: acc[k] += f[k]/n
    for kb in range(4):
        beta = f['rotor/b%d/beta-rad'%kb]
        for j,(rr,dr) in enumerate(segs):
            e='aero/rotor/b%d/s%d/'%(kb,j)
            qd=f[e+'qcdr']; cl=f[e+'cl']; cd=f[e+'cd']; UT=f[e+'UT']; UP=f[e+'UP']; Vv=f[e+'V']
            fl = max(0.0, min(1.0, (U.TIP_LOSS*U.R - edges[j])/(edges[j+1]-edges[j])))
            a = U.E_FLAP + (rr-U.E_FLAP)*math.cos(beta)
            Qp[j] += a*qd*cd*UT/Vv/n
            Qi[j] += a*qd*cl*fl*UP/Vv/n
for k in keys: print(k, round(acc[k],1))
print('profile hp', (Qp*27/550).round(1), 'total', round(Qp.sum()*27/550))
print('induced+parasite hp', (Qi*27/550).round(1), 'total', round(Qi.sum()*27/550))
