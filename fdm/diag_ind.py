import harness as H, numpy as np, math, uh60data as U, sys
f = H.load(); H.freeze(f); f['ctl/init'] = 1
x = [float(a) for a in sys.argv[1:7]]; V = float(sys.argv[7]); alt = float(sys.argv[8])
r, pw = H.residual(f, x, V, alt)
segs, edges = U.segments()
n = int(round(2*math.pi/27/f['simulation/dt']))
P = dict(ind=0, hubflow=0, flap=0, other=0, total=0, prof=0)
W = 27.0
for i in range(n):
    f.run()
    v0 = f['rotor/v0-fps']; Vhz = f['aero/rotor/Vhz-fps']
    for kb in range(4):
        b='aero/rotor/b%d/'%kb
        beta = f['rotor/b%d/beta-rad'%kb]; bd = f['rotor/b%d/betadot-rad_sec'%kb]
        gx = f[b+'gradx']
        for j,(rr,dr) in enumerate(segs):
            e=b+'s%d/'%j
            qd=f[e+'qcdr']; cl=f[e+'cl']; cd=f[e+'cd']; UT=f[e+'UT']; UP=f[e+'UP']; Vv=f[e+'V']
            fl = max(0.0, min(1.0, (U.TIP_LOSS*U.R - edges[j])/(edges[j+1]-edges[j])))
            L = qd*cl*fl
            a = U.E_FLAP + (rr-U.E_FLAP)*math.cos(beta)
            vi = v0*(1 + (gx-1)*rr)
            rho_h = rr - U.E_FLAP
            # UP = Vrel . n ; contributions: hub velocity (-Vhz cb + inplane sb terms), flap (-rho bd), induced (+vi cb)
            up_ind = vi*math.cos(beta)
            up_flap = rho_h*bd
            w = W*a*L/Vv/n          # power per unit UP
            P['ind'] += w*up_ind; P['flap'] += w*up_flap; P['total'] += w*UP
            P['prof'] += W*a*qd*cd*UT/Vv/n
for k in P: print(k, round(P[k]/550))
print('other (hub inflow, in-plane x sin beta, rates) =', round((P['total']-P['ind']-P['flap'])/550))
print('v0', f['rotor/v0-fps'], 'Vhz', f['aero/rotor/Vhz-fps'], 'Vhx', f['aero/rotor/Vhx-fps'], 'kappa', f['aero/rotor/kappa-x'])
