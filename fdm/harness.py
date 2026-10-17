"""JSBSim harness for the UH-60A model: load, freeze the airframe, run the rotor, average over revolutions."""
import os, math, shutil, tempfile
import jsbsim
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
FG_PROPS = ['/controls/engines/engine[0]/throttle']
TERRAIN_FT = -10000.0      # trims: terrain elevation (far below = out of ground effect)


def load(root=None, dt=1 / 480.0, model='uh60a', fps=False):        # the rate FlightGear runs (uh60a-set.xml)
    root = root or os.environ.get('UH60_ROOT') or os.path.join(HERE, 'root')      # UH60_ROOT: a variant (report/mkresults.py --tr)
    f = jsbsim.FGFDMExec(root)
    f.set_debug_level(0)
    for prop in FG_PROPS:
        f[prop] = 1.0
    f['/sim/freeze/fuel'] = 1.0       # constant weight in tests
    f['rotor/turb/enable'] = 0.0      # deterministic (no wake turbulence)
    f['ctl/fps'] = 1.0 if fps else 0.0   # FPS off: the tests fly the SAS-only aircraft
    f.load_model(model)
    f.set_dt(dt)
    return f


def freeze(f, on=True):
    for k in ('simulation/integrator/rate/rotational', 'simulation/integrator/rate/translational',
              'simulation/integrator/position/rotational', 'simulation/integrator/position/translational'):
        f[k] = 0 if on else 1      # 0 = none


def set_state(f, V_kt=0.0, alt_ft=1000.0, theta=0.0, phi=0.0, gamma=0.0, beta=0.0, agl=None):
    f['ic/vt-kts'] = V_kt; f['ic/theta-deg'] = theta; f['ic/phi-deg'] = phi; f['ic/beta-deg'] = beta
    f['ic/gamma-deg'] = gamma
    if agl is None:
        f['ic/terrain-elevation-ft'] = 0; f['ic/h-sl-ft'] = alt_ft
    else:
        f['ic/terrain-elevation-ft'] = 0; f['ic/h-agl-ft'] = agl
    f['ic/p-rad_sec'] = 0; f['ic/q-rad_sec'] = 0; f['ic/r-rad_sec'] = 0
    f.run_ic()


def controls(f, coll=0.5, lon=0.0, lat=0.0, ped=0.0):
    """coll 0..1 (up), lon -1..1 (+ forward), lat -1..1 (+ right), ped -1..1 (+ right)"""
    f['/controls/engines/engine[0]/throttle'] = 1.0 - coll
    f['fcs/elevator-cmd-norm'] = lon; f['fcs/aileron-cmd-norm'] = lat; f['fcs/rudder-cmd-norm'] = -ped   # JSBSim: + = left


def settle(f, seconds=4.0, avg_revs=2):
    """run frozen, then average the rotor outputs over whole revolutions"""
    n = int(seconds / f['simulation/dt'])
    for _ in range(n):
        f.run()
    nav = int(round(avg_revs * 2 * math.pi / 27.0 / f['simulation/dt']))
    keys = ['aero/rotor/thrust-lbs', 'aero/rotor/power-hp', 'aero/rotor/tr/thrust-lbs', 'aero/rotor/tr/power-hp',
            'forces/fbx-aero-lbs', 'forces/fby-aero-lbs', 'forces/fbz-aero-lbs',
            'moments/l-aero-lbsft', 'moments/m-aero-lbsft', 'moments/n-aero-lbsft', 'rotor/v0-fps']
    acc = {k: 0.0 for k in keys}
    beta = np.zeros((nav, 4)); psi = np.zeros(nav)
    for i in range(nav):
        f.run()
        for k in keys: acc[k] += f[k] / nav
        psi[i] = f['rotor/psi-rad']
        beta[i] = [f['rotor/b%d/beta-rad' % k] for k in range(4)]
    # flapping harmonics from blade 0: beta = a0 - a1 cos psi - b1 sin psi
    ps = psi.copy(); b = beta[:, 0]
    A = np.c_[np.ones_like(ps), -np.cos(ps), -np.sin(ps)]
    a0, a1, b1 = np.degrees(np.linalg.lstsq(A, b, rcond=None)[0])
    acc.update(a0=a0, a1=a1, b1=b1)
    return acc


ACC = ['accelerations/udot-ft_sec2', 'accelerations/vdot-ft_sec2', 'accelerations/wdot-ft_sec2',
       'accelerations/pdot-rad_sec2', 'accelerations/qdot-rad_sec2', 'accelerations/rdot-rad_sec2']
# Ballin TM-88360 figure B1: USAAEFA level-flight trim, 16000 lb, CG FS 351.0 / WL 231.5, 5250 ft density altitude
# (kt, lon % from full forward, lat % from full left, coll %, pedal % from full left, pitch deg, roll deg, MR hp, stab deg TED)
BALLIN_B1 = [(22, 53, 51.5, 62, 28.5, 2.5, -0.5, 1630, 44.5), (30, 56, 57.5, 63, 28.5, 1.4, -2.6, 1680, 44.5),
             (32, 57, 57, 65, 26.5, 0.8, -0.6, 1720, 44.5), (50, 51, 48.5, 42.5, 40.5, -0.6, -0.1, 950, 40.5),
             (72, 48, 49, 41.5, 49, -0.1, 0.2, 930, 19.6), (95, 41, 52, 45.5, 51, 0.1, 0.9, 1000, 8.0),
             (117, 36, 55, 55, 52.5, -0.4, 1.7, 1250, 5.6), (139, 36, 56.5, 72, 52, -3.6, 1.5, 1700, 6.6),
             (162, 32, 56, 93, 59.5, -5.9, -0.6, 2510, 4.5)]
# GenHel (Sikorsky) curve read from the same figure: kt -> (lon, lat, coll, ped, pitch, roll, hp, stab)
GENHEL_B1 = {0: (57, 49, 57, 31, 2.5, -2.3, 1610, 39), 50: (55, 47, 43, 50, -0.7, 0.0, 950, 30),
             100: (41, 54, 47, 61, 0.0, 0.0, 1010, 4), 150: (37, 56, 80, 68, -3.5, 0.0, 1950, 2)}


def residual(f, x, V_kt, alt_ft=1000.0, gamma=0.0, settle_s=1.5, revs=2, wings_level=False, beta=0.0):
    """x = coll, lon, lat, ped, alpha, and roll (or sideslip when wings_level); beta: fixed sideslip otherwise"""
    coll, lon, lat, ped, th, ph = x
    controls(f, coll, lon, lat, ped)
    # x[4] is the angle of attack: JSBSim keeps alpha when theta is set, which would trim a climb instead
    f['ic/vt-kts'] = V_kt; f['ic/psi-true-deg'] = 0.0
    f['ic/phi-deg'] = 0.0 if wings_level else ph
    f['ic/beta-deg'] = ph if wings_level else beta
    f['ic/gamma-deg'] = gamma; f['ic/alpha-deg'] = th
    f['ic/h-sl-ft'] = alt_ft; f['ic/terrain-elevation-ft'] = TERRAIN_FT
    f['ic/p-rad_sec'] = 0; f['ic/q-rad_sec'] = 0; f['ic/r-rad_sec'] = 0
    f.run_ic()
    for _ in range(int(settle_s / f['simulation/dt'])):
        f.run()
    n = int(round(revs * 2 * math.pi / f['rotor/omega-rad_sec'] / f['simulation/dt']))
    # Hann-weighted over whole main rotor revolutions: the main rotor's harmonics still cancel, and the tail
    # rotor's (4.6 of its revolutions per main rotor one) no longer leave a phase-dependent remainder
    w = np.sin(np.pi * (np.arange(n) + 0.5) / n) ** 2
    w /= w.sum()
    acc = np.zeros(6); pw = 0.0
    for i in range(n):
        f.run()
        acc += w[i] * np.array([f[k] for k in ACC]); pw += w[i] * f['aero/rotor/power-hp']
    return acc, pw


def trim(f, V_kt, alt_ft=1000.0, gamma=0.0, x0=None, tol=None, iters=12, verbose=False, wings_level=False, beta=0.0):
    """Newton trim of collective, cyclic, pedals, angle of attack and roll for steady straight flight (airframe
    frozen, rotor running), with bounds and a backtracking line search. Returns x, residuals, main rotor hp; the
    pitch attitude is attitude/theta-deg."""
    f['ctl/init'] = 1
    freeze(f)
    lo = np.array([0.0, -1, -1, -1, -25, -25]); hi = np.array([1.0, 1, 1, 1, 25, 25])
    x = np.array(x0 if x0 is not None else [0.5, 0.0, 0.0, 0.0, 2.0, 0.0], float)
    h = np.array([0.01, 0.01, 0.01, 0.01, 0.3, 0.3])
    scale = np.array([1, 1, 1, 10, 10, 10.0])     # ft/s2 vs rad/s2
    cost = lambda r: float(np.abs(r * scale).max())
    r0, pw = residual(f, x, V_kt, alt_ft, gamma, wings_level=wings_level, beta=beta)
    for it in range(iters):
        if verbose:
            print(it, np.round(x, 4), np.round(r0, 4), round(pw))
        if cost(r0) < (tol or 0.02):
            break
        J = np.zeros((6, 6))
        for j in range(6):
            xp = x.copy(); xp[j] += h[j]
            J[:, j] = (residual(f, xp, V_kt, alt_ft, gamma, wings_level=wings_level, beta=beta)[0] - r0) / h[j]
        if not (np.all(np.isfinite(J)) and np.all(np.isfinite(r0))):
            break           # the model blew up (returned with the residual as it is)
        dx = np.linalg.lstsq(J * scale[:, None], -r0 * scale, rcond=None)[0]
        lim = np.array([0.05, 0.1, 0.1, 0.1, 3, 3])
        k = np.max(np.abs(dx) / lim)
        if k > 1:
            dx /= k
        step = 1.0
        for _ in range(6):
            xn = np.clip(x + step * dx, lo, hi)
            rn, pn = residual(f, xn, V_kt, alt_ft, gamma, wings_level=wings_level, beta=beta)
            if cost(rn) < cost(r0):
                break
            step *= 0.5
        x, r0, pw = xn, rn, pn
    return x, r0, pw


def pct(x):
    """model controls -> Ballin's percent (lon 0 = full forward, lat/ped 0 = full left)"""
    coll, lon, lat, ped = x[:4]
    return 50 * (1 - lon), 50 * (1 + lat), 100 * coll, 50 * (1 + ped)
