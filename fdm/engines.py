"""UH-60A drive system: two T700-GE-700 engines, freewheel clutches, rotor brake and the rotor speed degree of
freedom, as a JSBSim system (FCS components, no Nasal).

Sources:
  [GH]  NASA CR-166309 section 5.6: rotor/drivetrain inertias, steady-state torque vs gas generator speed (table
        6.5.1), fuel flow vs Ng (6.5.4), load demand spindle schedule (6.5.2, 6.5.3), gas generator time constants
        from dQgg/dNgg (6.5.6) with KGF/JGG = 0.479 %/s per ft lb
  [TM]  TM 1-1520-237-10: start sequence (starter dropout 52-65 % Ng, idle 63-69 % Ng), limits (TGT, torque)

The engine is a nonlinear model built around those data: the HMU meters fuel to hold a gas generator speed
reference (idle, the power available spindle from the PCL, or the load demand spindle from the collective plus the
ECU's Np governor trim), within acceleration and deceleration fuel schedules; the gas generator follows the fuel
with its speed-dependent lag; the power turbine torque depends on Ng and on its own speed. Corrected parameters
(delta, theta) carry the data to other altitudes and temperatures.

States are properties written back by fcs_function <output>s, integrated once per frame like the rotor.
"""
import math
from xmlfun import *

G0 = 32.174
OMEGA0 = 27.0
GEAR_TR = 124.62 / 27.0
J_BLADE = 1512.6 + 2 * 1.25 * 86.7 + 256.9 / G0 * 1.25 ** 2   # one blade about the shaft, slug ft^2
J_ROTOR = 943.9 + 4 * J_BLADE        # [GH] JREST (hub, shafts, gearbox downstream of the clutches) + blades
J_ENG = 690.6 / 2                    # [GH] JTOTE, per engine, referred to the rotor
# the part of the rotor's inertia that spins about the shaft: the blades (the hub's share is small; the gears and
# shafts in JREST and the power turbines spin about other axes, faster). The rotor keeps its own angular momentum:
# when the airframe yaws, the rotor's speed relative to it changes by the yaw rate (Fletcher NASA TM 110362 eq. 13)
J_SPIN = 4 * J_BLADE
SHAFT_TILT = math.radians(3.0)       # forward (uh60data.SHAFT_TILT)
ACCESSORY_FTLB = 65.8 * 550 / OMEGA0  # accessory drive torque at 100 % rpm: 65.8 hp [Sikorsky, in Yeo JAHS 2004]
Q100 = 1414.0 * 550 / OMEGA0         # 100 % TRQ: 1414 shp per engine at 100 % rpm (2828 shp transmission)

# torque at the rotor per engine, ft lb, at 100 % Np, sea level standard, against corrected Ng % [GH 6.5.1];
# below 70 % extrapolated along the same slope
NG_Q = [0, 40, 60, 70, 84.6, 89.9, 94.0, 97.2, 100.1, 102.6, 106]
Q_NG = [-28770, -12330, -4110, 0, 6000, 12000, 18000, 24000, 30000, 36000, 44000]
# fuel flow per engine, lb/hr, corrected [GH 6.5.4], extended to zero and above 100 %
NG_WF = [0, 20, 40, 50, 60, 65, 70, 75, 80, 85, 90, 95, 100, 103, 106]
WF_NG = [0, 25, 45, 60, 95, 120, 140, 166, 210, 280, 390, 575, 790, 930, 1080]
# gas generator lag, s: 1/(0.479 |dQgg/dNgg|) above 70 % [GH 6.5.6]; slower below (start and spool-down)
NG_TAU = [0, 20, 40, 55, 65, 70, 75, 80, 85, 90, 95, 100, 106]
TAU_NG = [6.0, 6.0, 5.0, 3.5, 2.5, 1.9, 0.99, 0.67, 0.49, 0.37, 0.29, 0.45, 0.45]
# TGT / ambient temperature (both absolute) at steady state, against corrected Ng (T700-GE-700 marks: normal to
# 775 C, 30 min 850 C, 10 s 886 C)
NG_TGT = [0, 20, 50, 65, 80, 90, 95, 100, 102.6, 106]
TGT_NG = [1.0, 2.15, 2.45, 2.61, 2.89, 3.20, 3.41, 3.69, 3.90, 4.10]
# load demand spindle: Ng reference % against collective % [GH 6.5.2, 6.5.3: LDS = 0.85 deg per % collective]
LDS_COLL = [0, 23.5, 47.1, 70.6, 78.8, 94.1, 100]
LDS_NG = [78.6, 85.0, 91.3, 97.8, 100.0, 100.0, 100.0]

NG_IDLE = 65.0       # corrected; [TM] idle 63-69 %
NG_MAX = 103.0       # corrected HMU maximum
NG_STARTER = 24.0    # motoring speed on the starter alone
NG_DROPOUT = 56.0    # [TM] starter cuts out at 52-65 %
NP_OVERSPEED = 120.0
TGT_LIMIT = 850.0    # ECU TGT limiter, deg C
PCL_IDLE, PCL_FLY = 0.5, 1.0


def build(fcs_name='UH-60A drive'):
    f = FCS(fcs_name)
    for n in range(2):
        e = '/controls/engines/engine[%d]' % n
        f.declare(e + '/mixture', 1.0)        # ENG POWER CONT lever: 0 OFF, 0.5 IDLE, 1 FLY (FlightGear's mixture)
        f.declare(e + '/lockout', 0)
        f.declare(e + '/starter', 0)
        f.declare(e + '/np-trim', 0.0)        # ENG RPM trim, % (-4 .. +1)
        f.declare('eng[%d]/ng-pct' % n, 0.0)
        f.declare('eng[%d]/np-rad_sec' % n, 0.0)
        f.declare('eng[%d]/tgt-degc' % n, 15.0)
        f.declare('eng[%d]/lit' % n, 0)
        f.declare('eng[%d]/engaged' % n, 0)
        f.declare('eng[%d]/starter-on' % n, 0)
        f.declare('eng[%d]/ecu-int' % n, 0.0)
    f.declare('/controls/engines/ignition', 1)
    f.declare('/controls/fuel/crossfeed', 0)
    for n in range(2):
        f.declare('/consumables/fuel/tank[%d]/level-lbs' % n, 1206.0)   # FlightGear: tied to its tank list
    f.declare('/controls/rotor/brake', 0.0)
    f.declare('eng/init-running', 1)          # 0: cold and dark at start (FlightGear sets it)
    f.declare('/sim/freeze/fuel', 0)

    dt = p('simulation/dt')
    init = p('ctl/init-fast')
    f.channel('start state')
    # engines running at the start if asked to, or when starting in the air
    run0 = f.fn('eng/running-at-start', MAX(p('eng/init-running'), GT(p('position/h-agl-ft'), v(50))))
    for n in range(2):
        e = '/controls/engines/engine[%d]' % n
        # cold and dark: the power levers start at OFF (afterwards this passes the pilot's lever through)
        f.fn('eng[%d]/pcl-init' % n, IFTHEN(GT(init, v(0.5)), IFTHEN(run0, p(e + '/mixture'), v(0)), p(e + '/mixture')), out=e + '/mixture')
    # parked: rotor brake on; running: off
    f.fn('drive/brake-init', IFTHEN(GT(init, v(0.5)), IFTHEN(run0, v(0), v(1)), p('/controls/rotor/brake')), out='/controls/rotor/brake')
    W = p('rotor/omega-rad_sec')
    clamp = lambda x, lo, hi: MAX(v(lo), MIN(v(hi), x))

    f.channel('ambient')
    th = f.fn('eng/theta', Q(p('atmosphere/T-R'), v(518.67)))
    dl = f.fn('eng/delta', Q(p('atmosphere/P-psf'), v(2116.22)))
    sth = f.fn('eng/sqrt-theta', SQ(th))
    dsth = f.fn('eng/delta-sqrt-theta', M(dl, sth))
    Tamb = f.fn('eng/tamb-k', Q(p('atmosphere/T-R'), v(1.8)))
    coll = f.fn('eng/coll-pct', M(v(10.0), p('ctl/coll-in')))

    # load on the drive train (previous frame's aerodynamics)
    f.channel('rotor load')
    Qmr = p('aero/rotor/torque-ftlb')
    Qtr = M(v(GEAR_TR), p('aero/rotor/tr/torque-ftlb'))
    Wn = Q(W, v(OMEGA0))
    f.fn('drive/loss-ftlb', S(M(v(ACCESSORY_FTLB), SQR(Wn)), M(v(0.03), ABS(Qmr))), desc='accessories and gearbox losses')
    f.fn('drive/brake-ftlb', M(p('/controls/rotor/brake'), v(25000), Q(W, S(ABS(W), v(0.3)))), desc='rotor brake, smooth at standstill')
    Qload = f.fn('drive/load-ftlb', S(Qmr, Qtr, p('drive/loss-ftlb'), p('drive/brake-ftlb')))

    for n in range(2):
        e = '/controls/engines/engine[%d]' % n
        g = 'eng[%d]' % n
        f.channel('engine %d' % (n + 1))
        ng, np_, tgt = p(g + '/ng-pct'), p(g + '/np-rad_sec'), p(g + '/tgt-degc')
        lit, eng, st = p(g + '/lit'), p(g + '/engaged'), p(g + '/starter-on')
        pcl = p(e + '/mixture')
        ngc = f.fn(g + '/ngc-pct', Q(ng, sth), desc='corrected gas generator speed')
        npp = f.fn(g + '/np-pct', M(v(100 / OMEGA0), np_))
        # starter: a press latches it until dropout speed (or the PCL is pulled to OFF after light-off)
        f.fn(g + '/starter-next', IFTHEN(GT(S(p(e + '/starter'), st), v(0.5)),
                                         IFTHEN(GT(ng, v(NG_DROPOUT)), v(0), v(1)), v(0)), out=g + '/starter-on')
        # light-off: ignition on, PCL at IDLE or above, enough airflow; flame out with the PCL at OFF, on an
        # overspeed trip or below 8 % Ng
        # fuel: engine n feeds from tank n, or from the other tank with crossfeed
        tank, tank_o = p('/consumables/fuel/tank[%d]/level-lbs' % n), p('/consumables/fuel/tank[%d]/level-lbs' % (1 - n))
        fuel_ok = f.fn(g + '/fuel-ok', MAX(GT(tank, v(0.5)), M(GT(p('/controls/fuel/crossfeed'), v(0.5)), GT(tank_o, v(0.5)))))
        lightable = M(GT(p('/controls/engines/ignition'), v(0.5)), GT(pcl, v(0.4)), GT(ng, v(15)), fuel_ok)
        keep = M(GT(pcl, v(0.25)), GT(ng, v(8)), LT(npp, v(NP_OVERSPEED)), fuel_ok)
        f.fn(g + '/lit-next', IFTHEN(GT(init, v(0.5)), run0, M(keep, MAX(lit, lightable))), out=g + '/lit')
        # HMU: gas generator speed reference
        ng_pas = f.fn(g + '/ng-pas', M(sth, S(v(NG_IDLE), M(v((NG_MAX - NG_IDLE) / (PCL_FLY - PCL_IDLE)), clamp(D(pcl, v(PCL_IDLE)), 0, PCL_FLY - PCL_IDLE)))),
                      desc='power available spindle: idle to maximum over IDLE..FLY')
        lds = f.fn(g + '/ng-lds', M(sth, T1('eng/coll-pct', LDS_COLL, LDS_NG)), desc='load demand spindle (collective)')
        ng_idle = M(v(NG_IDLE), sth)
        f.fn(g + '/q-share-c', Q(M(v(0.5), Qload), dsth))
        steady_ng = f.fn(g + '/ng-for-load', M(sth, T1(g + '/q-share-c', Q_NG, NG_Q)), desc='Ng that carries half the load at 100 % Np')
        # ECU: Np governor (PI on Np error, % Ng per % Np) with torque matching up to the other engine
        npref = S(v(100), p(e + '/np-trim'))
        err = f.fn(g + '/np-err', D(npref, npp))
        other = 'eng[%d]' % (1 - n)
        match = M(v(0.0004), MAX(v(0), D(p(other + '/torque-ftlb'), p(g + '/torque-ftlb'))))
        ecu_on = M(LT(p(e + '/lockout'), v(0.5)), GT(pcl, v(0.6)), lit)
        f.fn(g + '/ecu-int-next', IFTHEN(GT(init, v(0.5)), clamp(D(MAX(ng_idle, steady_ng), lds), -40, 10),
                                         IFTHEN(ecu_on, clamp(S(p(g + '/ecu-int'), M(dt, S(M(v(1.2), err), match))), -40, 10), v(0))),
             out=g + '/ecu-int')
        ecu = f.fn(g + '/ecu-trim', M(ecu_on, S(M(v(2.0), err), p(g + '/ecu-int'))), desc='ECU Ng trim, %')
        tgtlim = M(ecu_on, v(0.5), MAX(v(0), D(tgt, v(TGT_LIMIT))))
        ngref = f.fn(g + '/ng-ref', IFTHEN(GT(p(e + '/lockout'), v(0.5)), ng_pas,
                                           MAX(ng_idle, D(MIN(ng_pas, S(lds, ecu)), tgtlim))))
        # fuel: steady flow for the reference plus a proportional term, inside the acceleration/deceleration
        # schedules (corrected flow x delta sqrt(theta)); none with the PCL at OFF
        wfss = lambda x: T1(x, NG_WF, WF_NG)
        f.fn(g + '/ngref-c', Q(ngref, sth))
        wfc = f.fn(g + '/wf-cmd-c', S(T1(g + '/ngref-c', NG_WF, WF_NG), M(v(40), D(p(g + '/ngref-c'), ngc))))
        wf_acc = S(M(v(1.35), wfss(g + '/ngc-pct')), v(60))
        wf_dec = M(v(0.55), wfss(g + '/ngc-pct'))
        wf = f.fn(g + '/fuel-flow-pph', M(lit, GT(pcl, v(0.25)), dsth, MAX(wf_dec, MIN(wf_acc, wfc))), desc='lb/hr')
        # gas generator speed
        f.fn(g + '/wf-c', Q(wf, dsth))
        tau = T1(g + '/ngc-pct', NG_TAU, TAU_NG)
        ng_lit = M(sth, T1(g + '/wf-c', WF_NG, NG_WF))
        ng_unlit = M(st, v(NG_STARTER))
        target = IFTHEN(lit, ng_lit, ng_unlit)
        tau_eff = IFTHEN(lit, tau, IFTHEN(st, v(3.0), v(8.0)))
        assist = M(st, lit, v(1.5), MAX(v(0), D(v(1), Q(ng, v(60)))))
        f.fn(g + '/ng-next', IFTHEN(GT(init, v(0.5)), M(run0, MAX(ng_idle, steady_ng)),
                                    MAX(v(0), S(ng, M(dt, S(Q(D(target, ng), tau_eff), assist))))), out=g + '/ng-pct')
        # power turbine torque at the rotor (positive drives): table torque at 100 % Np plus the free turbine's
        # rise as it slows; nothing unlit
        qtab = T1(g + '/ngc-pct', NG_Q, Q_NG)
        kq = M(v(0.012), MAX(v(0), S(qtab, v(12000))))
        q = f.fn(g + '/torque-ftlb', M(lit, dsth, S(qtab, M(kq, D(v(100), npp)))), desc='at the rotor shaft')
        f.fn(g + '/torque-pct', Q(M(v(100), MAX(v(0), M(q, eng))), v(Q100)))
        # TGT: steady value for Ng, hotter when fuel exceeds the steady flow (acceleration), 1.5 s lag
        ratio = clamp(Q(S(p(g + '/wf-c'), v(5)), S(wfss(g + '/ngc-pct'), v(5))), 0.3, 2.5)
        tgt_ss = IFTHEN(lit, D(M(Tamb, T1(g + '/ngc-pct', NG_TGT, TGT_NG), SQ(ratio)), v(273.15)), D(Tamb, v(273.15)))
        f.fn(g + '/tgt-ss', tgt_ss)
        f.fn(g + '/tgt-next', IFTHEN(GT(init, v(0.5)), p(g + '/tgt-ss'), S(tgt, M(dt, Q(D(p(g + '/tgt-ss'), tgt), IFTHEN(lit, v(1.5), v(20)))))),
             out=g + '/tgt-degc')

    # fuel burn: each engine from its own tank, or the other one when crossfeed is on and its own is dry
    f.channel('fuel')
    frozen = GT(p('/sim/freeze/fuel'), v(0.5))
    for n in range(2):
        t = '/consumables/fuel/tank[%d]/level-lbs' % n
        own = M(p('eng[%d]/fuel-flow-pph' % n), GT(p(t), v(0.5)))
        other = M(p('eng[%d]/fuel-flow-pph' % (1 - n)), LT(p('/consumables/fuel/tank[%d]/level-lbs' % (1 - n)), v(0.5)), GT(p('/controls/fuel/crossfeed'), v(0.5)))
        f.fn('fuel/burn-tank%d' % n, IFTHEN(frozen, p(t), MAX(v(0), D(p(t), M(S(own, other), dt, v(1 / 3600.0))))), out=t)
        # the tank's fuel is JSBSim point mass 4 + n
        f.fn('fuel/mass-tank%d' % n, p(t), out='inertia/pointmass-weight-lbs[%d]' % (4 + n))

    # drive train: freewheel clutches, rotor speed, and the engines' own power turbine speeds when disengaged
    f.channel('drive train')
    q0, q1 = p('eng[0]/torque-ftlb'), p('eng[1]/torque-ftlb')
    e0, e1 = p('eng[0]/engaged'), p('eng[1]/engaged')
    J = S(v(J_ROTOR), M(v(J_ENG), S(e0, e1)))
    # rotor speed relative to the airframe: J dW/dt = Q_engines - Q_load + J_spin dr_s/dt (the rotor turns
    # anticlockwise seen from above; the airframe's yaw acceleration about the shaft, last frame's, is the rotor's own
    # with the opposite sign relative to the airframe), so a nose-left yaw droops the rotor and the governor's
    # answer is felt by the airframe as the drive torque's reaction
    rds = f.fn('drive/r-dot-shaft', S(M(v(math.cos(SHAFT_TILT)), p('accelerations/rdot-rad_sec2')), M(v(-math.sin(SHAFT_TILT)), p('accelerations/pdot-rad_sec2'))),
               desc='airframe angular acceleration about the main rotor shaft (down +)')
    wdot = f.fn('drive/omega-dot', Q(S(D(S(M(e0, q0), M(e1, q1)), Qload), M(v(J_SPIN), IFTHEN(GT(init, v(0.5)), v(0), rds))), J))
    w_new = f.fn('drive/omega-new', MAX(v(0), S(W, M(dt, wdot))))
    w_init = M(run0, M(v(OMEGA0 / 100), S(v(100), p('/controls/engines/engine[0]/np-trim'))))
    for n, (q, en) in enumerate(((q0, e0), (q1, e1))):
        g = 'eng[%d]' % n
        np_ = p(g + '/np-rad_sec')
        free = MAX(v(0), S(np_, M(dt, Q(D(q, M(v(400), SQR(Q(np_, v(OMEGA0))))), v(J_ENG)))))
        f.fn(g + '/np-free', free)
        # engaged: follows the rotor while it drives; free: engages when it catches the rotor
        f.fn(g + '/engaged-next', IFTHEN(GT(init, v(0.5)), run0, IFTHEN(en, GT(q, v(0)), GT(p(g + '/np-free'), w_new))), out=g + '/engaged')
        f.fn(g + '/np-next', IFTHEN(GT(init, v(0.5)), w_init, IFTHEN(p(g + '/engaged'), w_new, p(g + '/np-free'))),
             out=g + '/np-rad_sec')
    f.fn('drive/omega-next', IFTHEN(GT(init, v(0.5)), w_init, w_new), out='rotor/omega-rad_sec')
    f.fn('drive/mr-torque-ftlb', IFTHEN(GT(init, v(0.5)), Qmr, D(S(M(e0, q0), M(e1, q1)), S(Qtr, p('drive/loss-ftlb'), p('drive/brake-ftlb')))),
         desc='torque into the main rotor shaft; its reaction acts on the airframe')
    f.fn('drive/nr-pct', M(v(100 / OMEGA0), W))
    return f
