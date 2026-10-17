#!/usr/bin/env python3
"""Generate the UH-60A JSBSim model: individual-blade main rotor, tail rotor, airframe.

  python3 genuh60.py OUTDIR      writes OUTDIR/uh60a-jsb.xml and OUTDIR/Systems/uh60a-*.xml

Main rotor: the 4 blades are simulated individually, as in Sikorsky's GenHel (NASA CR-166309): each blade has
its own flapping (rigid blade on an offset hinge), integrated in the FCS at the FDM rate, and 8 segments of
equal annulus area with the SC1095 lift and drag tables of GenHel (angle of attack and Mach number, +-180 deg).
The segment velocities come from the full kinematics in shaft axes: the aircraft's air velocity and rotation,
the rotor rotation, the blade's flapping motion and the induced inflow. Each blade's aerodynamic and inertial
loads act on the hub at its flap hinge; the shaft torque goes to the rotor speed, and the airframe gets the
drive torque's reaction.

Shaft axes: x forward, y right, z down along the shaft (3 deg forward tilt); the rotor turns anticlockwise
seen from above; azimuth psi from the tail. Body axes: JSBSim (x forward, y right, z down).
"""
import math, os, sys
import uh60data as U
import engines
from xmlfun import *
from data import genhel_sc1095 as AF
from data import genhel_fuselage as FUS
from data import genhel_empennage as EMP
from data import heyson_ground as HG
import peters_he as PH
import mrwake
import numpy as np

FT = 1.0 / 12.0
G0 = 32.174


# ---------------------------------------------------------------- airfoil tables over -180..180 deg
def section_tables():
    a_rows = [a for a in AF.ALPHA_HI if a < -32] + AF.ALPHA_LO + [a for a in AF.ALPHA_HI if a > 32]
    hi_cl = dict(zip(AF.ALPHA_HI, AF.CL_HI)); hi_cd = dict(zip(AF.ALPHA_HI, AF.CD_HI))
    # GenHel's high-angle drag falls to zero at exactly +-180 deg; a sharp trailing edge leading the flow
    # still separates: reverse flow drag 0.06 at 180, rising into the table by 172 deg
    for a, cd in ((180, 0.06), (178, 0.20), (176, 0.45), (174, 0.75)):
        hi_cd[a] = cd; hi_cd[-a] = cd
    cl, cd = [], []
    for a in a_rows:
        if -32 <= a <= 32:
            i = AF.ALPHA_LO.index(a)
            cl.append([AF.CL_LO[m][i] for m in AF.MACH]); cd.append([AF.CD_LO[m][i] for m in AF.MACH])
        else:
            cl.append([hi_cl[a]] * len(AF.MACH)); cd.append([hi_cd[a]] * len(AF.MACH))
    return a_rows, cl, cd


# SC1095 angle of maximum lift against the Mach numbers of its tables (deg, from the tables, held at 16 above Mach 0.6)
STALL_ALPHA = [12, 12, 12, 12, 10, 14, 16, 16, 16, 16, 16]
# and its lift curve slope, per deg (-4 to 4 deg)
LIFT_SLOPE = [0.1138, 0.1138, 0.1138, 0.1138, 0.1162, 0.1212, 0.1354, 0.1638, 0.1288, 0.1163, 0.1094]


def _odd(xs, ys):
    """a table over |x| made odd: y(-x) = -y(x)"""
    return [-x for x in reversed(xs[1:])] + list(xs), [-y for y in reversed(ys[1:])] + list(ys)


def _even(xs, ys):
    return [-x for x in reversed(xs[1:])] + list(xs), list(reversed(ys[1:])) + list(ys)


def _mirror180(xs, ys, negate):
    """extend a table over -90..90 deg to +-180: reverse flow sees the section mirrored front to back,
    y(180 - x) = -y(x) for lift (negate), +y(x) for drag"""
    k = -1 if negate else 1
    pts = dict(zip(xs, ys))
    for x, y in zip(xs, ys):
        if 0 <= x < 90:
            pts[180 - x] = k * y
        if -90 < x <= 0:
            pts[-180 - x] = k * y
    out = sorted(pts)
    return out, [pts[x] for x in out]


class Model:
    def __init__(s):
        s.g = Graph()               # aerodynamics functions
        s.ctl = FCS('UH-60A controls')
        s.rot = FCS('UH-60A rotor')
        s.X, s.Y, s.Z, s.L, s.Mm, s.N = [], [], [], [], [], []

    # ------------------------------------------------------------ controls (rigging and mixing)
    def controls(s):
        c = s.ctl
        c.declare('/controls/engines/engine[0]/throttle', 1.0)
        c.channel('pilot')
        coll = c.fn('ctl/coll-in', M(D(v(1), p('/controls/engines/engine[0]/throttle')), v(U.COLL_IN)), clip=('0', '%g' % U.COLL_IN),
                    desc='collective lever, inches up from the bottom (throttle axis: 1 = down)')
        # Stick and pedals, -1..1 from their centre (FlightGear trim included). The centre is the trim point, which the
        # FPS trim actuators move: on the real aircraft they move the stick, which FlightGear cannot, so here the trim
        # point moves under a centred stick, and a full deflection still reaches the stops.
        # JSBSim's rudder convention is positive left (FlightGear hands it the negated rudder), so negate it back.
        dlon = c.fn('ctl/lon-norm', S(p('fcs/elevator-cmd-norm'), p('fcs/pitch-trim-cmd-norm')), clip=('-1', '1'))
        dlat = c.fn('ctl/lat-norm', S(p('fcs/aileron-cmd-norm'), p('fcs/roll-trim-cmd-norm')), clip=('-1', '1'))
        dped = c.fn('ctl/ped-norm', NEG(S(p('fcs/rudder-cmd-norm'), p('fcs/yaw-trim-cmd-norm'))), clip=('-1', '1'))
        for prop in ('fps/pitch-pct', 'fps/roll-pct', 'fps/yaw-pct'):
            c.declare(prop, 0)
        def trimmed(name, d, x, L, desc):
            sgn = IFTHEN(LT(d, v(0)), v(-1), v(1))
            return c.fn(name, S(x, M(d, D(v(L), M(x, sgn)))), clip=('-%g' % L, '%g' % L), desc=desc)
        lon = trimmed('ctl/lon-fwd-in', dlon, M(v(-0.1), p('fps/pitch-pct')), U.LON_IN, 'cyclic, inches forward of centre')
        lat = trimmed('ctl/lat-in', dlat, M(v(0.1), p('fps/roll-pct')), U.LAT_IN, 'cyclic, inches right of centre')
        ped = trimmed('ctl/ped-in', dped, M(v(0.0538), p('fps/yaw-pct')), U.PED_IN, 'pedals, inches right')
        # tail wheel lock (TAIL WHEEL LOCK switch): locked, the wheel stays straight and resists side loads
        c.declare('/controls/gear/tailwheel-lock', 1)
        c.channel('tail wheel')
        c.fn('ctl/tailwheel-castered', IFTHEN(GT(p('/controls/gear/tailwheel-lock'), v(0.5)), v(0), v(1)), out='gear/unit[2]/castered')
        c.fn('ctl/tailwheel-steer', v(0), out='fcs/steer-pos-deg[2]')
        # JSBSim measures a wheel's compression, and divides its force, along the strut: tipped on its side the
        # strut lies along the ground and both run away. Past 60 deg of tilt the wheels are out of the game
        # ("retracted") and the structure contacts carry the aircraft.
        c.fn('ctl/wheels-active', IFTHEN(GT(M(COS(p('attitude/phi-rad')), COS(p('attitude/theta-rad'))), v(0.5)), v(1), v(0)),
             out='gear/gear-pos-norm')
        s.sensors()
        s.stabilator()
        s.sas()
        s.pitch_bias()
        s.fps()
        c.channel('mixer')
        # augmentation in series with the pilot: SAS and pitch bias outputs are % of control travel [GH 5.5-16]
        lon = c.fn('ctl/lon-total-in', D(lon, M(v(0.1), S(p('sas/pitch-pct'), p('pba/pct')))), desc='+ = forward')
        lat = c.fn('ctl/lat-total-in', S(lat, M(v(0.1), p('sas/roll-pct'))))
        ped = c.fn('ctl/ped-total-in', S(ped, M(v(0.0538), p('sas/yaw-pct'))))
        # mechanical mixing [HB table 2]: collective -> forward and left cyclic, left pedal; right pedal -> forward cyclic
        c.fn('ctl/B1s-cmd-deg', S(M(v(U.LON_DEG_IN), S(lon, M(v(-U.MIX_COLL_LON), coll), M(v(-U.MIX_PED_LON), ped))), v(-0.6)),
             clip=('%g' % U.B1S_LIM[0], '%g' % U.B1S_LIM[1]), desc='longitudinal cyclic at the head, + = forward (disc tilts forward)')
        c.fn('ctl/A1s-cmd-deg', M(v(U.LAT_DEG_IN), S(lat, M(v(U.MIX_COLL_LAT), coll))),
             clip=('%g' % U.A1S_LIM[0], '%g' % U.A1S_LIM[1]), desc='lateral cyclic at the head, + = right')
        c.fn('ctl/theta0-cmd-deg', S(v(U.COLL_CUFF[0]), M(v((U.COLL_CUFF[1] - U.COLL_CUFF[0]) / U.COLL_IN), coll)),
             desc='main rotor blade pitch at the cuff (twist zero), deg')
        c.fn('ctl/thetaTR-cmd-deg', S(v(U.TR_CUFF_CENTER), M(v(U.TR_DEG_IN), S(ped, v(U.PED_BIAS_IN), M(v(U.MIX_COLL_PED), D(coll, v(U.COLL_IN / 2)))))),
             clip=('%g' % U.TR_CUFF_LIM[0], '%g' % U.TR_CUFF_LIM[1]), desc='tail rotor blade pitch at the cuff, deg')
        c.channel('primary servos')
        # 1/(0.0002081 s^2 + 0.020 s + 1): Ames' no-load acceptance test value [Ballin TM-88360 app. A]
        for n in ('B1s', 'A1s', 'theta0', 'thetaTR'):
            c.second_order('ctl/%s-deg' % n, 'ctl/%s-cmd-deg' % n, (0, 0, 1), (0.0002081, 0.020, 1))

    # ------------------------------------------------------------ sensors [GH fig. 5.1.2]
    def sensors(s):
        c = s.ctl
        c.declare('ctl/init', 0)
        c.channel('sensors')
        # rate-limited actuators start where their commands are: in the first second, or while ctl/init is set
        c.fn('ctl/init-fast', IFTHEN(LT(p('simulation/sim-time-sec'), v(1.0)), v(1), MIN(v(1), MAX(v(0), p('ctl/init')))))
        R2D = 180 / math.pi
        for a in 'pqr':
            c.fn('sens/%s-raw' % a, M(v(R2D), p('velocities/%s-rad_sec' % a)))
            c.second_order('sens/%s-dps' % a, 'sens/%s-raw' % a, (0, 0, 1), (0.00014, 0.016, 1))
        c.fn('sens/ay-raw', M(v(G0), p('accelerations/Ny')), desc='lateral specific force, ft/s2')
        c.second_order('sens/ay-fps2', 'sens/ay-raw', (0, 0, 1), (0.0254, 0.233, 1))
        c.fn('sens/vx-kt-raw', M(v(1 / 1.68781), p('velocities/u-aero-fps'), SQ(Q(p('atmosphere/rho-slugs_ft3'), v(0.0023769)))),
             desc='indicated (equivalent) airspeed along the body axis, kt')
        c.lag('sens/vx-kt', 'sens/vx-kt-raw', 0.398)
        c.fn('sens/coll-pct', M(v(100.0 / U.COLL_IN), p('ctl/coll-in')))

    # ------------------------------------------------------------ stabilator [GH fig. 5.1.7]
    def stabilator(s):
        c = s.ctl
        c.channel('stabilator')
        V = p('sens/vx-kt')
        clamp = lambda x, lo, hi: MAX(v(lo), MIN(v(hi), x))
        pc = p('sens/coll-pct')
        kvx1 = NEG(S(v(0.67), M(v(0.003286), clamp(D(v(50), pc), 0, 50)), M(v(0.003286), clamp(D(v(70), pc), 0, 20))))
        c.fn('stab/cmd-deg', S(v(42), M(v(0.16), p('sens/q-dps')),
                              M(p('sens/ay-fps2'), v(-0.0433), clamp(D(V, v(30)), 0, 30)),
                              M(v(-0.1119), clamp(D(V, v(80)), 0, 67)),
                              M(clamp(D(V, v(30)), 0, 50), kvx1)),
             desc='scheduled on airspeed and collective, pitch rate and lateral acceleration feedback; TED +')
        # STAB AUTO off: the stabilator stays where it is until slewed (stab/manual-deg)
        c.declare('stab/auto', 1)
        c.declare('stab/manual-deg', 0)
        auto = GT(p('stab/auto'), v(0.5))
        c.fn('stab/manual-deg-next', IFTHEN(auto, p('stab/pos-deg'), p('stab/manual-deg')), out='stab/manual-deg')
        c.fn('stab/sel-deg', IFTHEN(auto, p('stab/cmd-deg'), p('stab/manual-deg')))
        c.second_order('stab/servo', 'stab/sel-deg', (0, 0, 1), (0.0254, 0.223, 1))
        c.fn('stab/rate-dps', IFTHEN(p('ctl/init-fast'), v(1000), v(7)))
        c.actuator('stab/pos-deg', 'stab/servo', rate='stab/rate-dps', clip=('%g' % U.STAB_LIM[0], '%g' % U.STAB_LIM[1]))

    # ------------------------------------------------------------ stability augmentation [GH fig. 5.1.3]
    def sas(s):
        """digital (SAS 2) and analog (SAS 1) channels; each has twice the gain when the other is off"""
        c = s.ctl
        c.declare('ctl/sas1', 1)
        c.declare('ctl/sas2', 1)
        c.channel('SAS')
        gD = c.fn('sas/gain-d', M(p('ctl/sas2'), D(v(2), p('ctl/sas1'))))
        gA = c.fn('sas/gain-a', M(p('ctl/sas1'), D(v(2), p('ctl/sas2'))))
        lim = lambda x, a: MAX(v(-a), MIN(v(a), x))
        hi = c.fn('sas/hispeed', IFTHEN(GT(p('sens/vx-kt'), v(60)), v(1), v(0)))
        c.actuator('sas/hispeed-fade', 'sas/hispeed', rate=0.5, clip=('0', '1'))
        fade = p('sas/hispeed-fade')          # the 60 kt switch, faded over 2 s in every term
        # pitch
        c.lag('sas/q1', 'sens/q-dps', 0.026)
        c.fn('sas/pd-in', M(v(-0.5125), p('sas/q1'))); c.lag('sas/pd-a', 'sas/pd-in', 0.023)
        c.fn('sas/pd-in2', M(v(-0.541), p('sas/q1'))); c.lag('sas/pd-b', 'sas/pd-in2', 2.0)
        c.fn('sas/pd-sum', S(p('sas/pd-a'), p('sas/pd-b'))); c.washout('sas/pd-w', 'sas/pd-sum', 7.0)
        c.lag('sas/q4', 'sens/q-dps', 0.022); c.washout('sas/q4w', 'sas/q4', 7.0)
        c.fn('sas/pa-in', M(v(0.1785 * -6.194), p('sas/q4w'))); c.lag('sas/pa-a', 'sas/pa-in', 0.024)
        c.fn('sas/pa-in2', M(v(0.1785 * -6.047), p('sas/q4w'))); c.lag('sas/pa-b', 'sas/pa-in2', 1.993)
        c.fn('sas/pitch-pct', S(lim(M(gD, p('sas/pd-w')), 5.0), lim(M(gA, v(0.5), S(p('sas/pa-a'), p('sas/pa-b'))), 5.5)))
        # roll (rate, lagged rate and attitude)
        c.lag('sas/p1', 'sens/p-dps', 0.026)
        c.fn('sas/rd-in', M(v(-0.1035), p('sas/p1'))); c.lag('sas/rd-b', 'sas/rd-in', 1.02)
        phi = p('attitude/phi-deg')
        c.fn('sas/ra-in', M(v(-0.847 * 0.1163), p('sens/p-dps'))); c.lag('sas/ra-b', 'sas/ra-in', 0.996)
        c.fn('sas/roll-pct', S(lim(M(gD, S(M(v(-0.2085), p('sas/p1')), p('sas/rd-b'), M(v(-0.139), lim(phi, 7)))), 5.0),
                                lim(M(gA, S(M(v(-1.738 * 0.1163), p('sens/p-dps')), p('sas/ra-b'), M(v(-0.703 * 0.0885 * 2.413), phi))), 5.5)))
        # yaw: rate and lagged rate below 60 kt; above, lateral acceleration and roll rate for turn coordination
        c.lag('sas/r1', 'sens/r-dps', 0.026)
        c.fn('sas/yd-in', M(v(-0.557), p('sas/r1'))); c.lag('sas/yd-a', 'sas/yd-in', 0.047)
        c.fn('sas/yd-in2', M(v(-0.901), p('sas/r1'), D(v(1), fade))); c.lag('sas/yd-b', 'sas/yd-in2', 1.0)
        c.fn('sas/yd-sum', S(p('sas/yd-a'), p('sas/yd-b'))); c.washout('sas/yd-w', 'sas/yd-sum', 2.0)
        c.fn('sas/yd-ay', M(v(-1.1), p('sens/ay-fps2'))); c.lag('sas/yd-c', 'sas/yd-ay', 1.0)
        c.lag('sas/r4', 'sens/r-dps', 0.027); c.washout('sas/r4w', 'sas/r4', 2.0)
        c.fn('sas/ya-in', M(v(0.072 * -16.95), p('sas/r4w'))); c.lag('sas/ya-a', 'sas/ya-in', 0.047)
        c.fn('sas/ya-in2', M(v(-28.11), S(M(v(0.072), p('sas/r4w'), D(v(1), fade)), M(v(0.0876), p('sens/ay-fps2'), fade))))
        c.lag('sas/ya-b', 'sas/ya-in2', 1.045)
        c.fn('sas/yaw-pct', S(lim(M(gD, S(p('sas/yd-w'), M(fade, S(M(v(0.1378), p('sas/p1')), p('sas/yd-c'))))), 5.0),
                               lim(M(gA, S(M(v(0.5), S(p('sas/ya-a'), p('sas/ya-b'))), M(fade, v(0.296 * 1.738 * 0.1163), p('sens/p-dps')))), 5.5)))

    # ------------------------------------------------------------ pitch bias actuator [GH fig. 5.1.4]
    def pitch_bias(s):
        c = s.ctl
        c.declare('ctl/pba', 1)
        c.channel('pitch bias actuator')
        lim = lambda x, lo, hi: MAX(v(lo), MIN(v(hi), x))
        c.fn('pba/cmd-pct', M(p('ctl/pba'), S(M(v(-0.375), lim(p('attitude/theta-deg'), -20, 20)), M(v(-0.13125), lim(p('sas/q1'), -34.5, 34.5)),
                                              M(v(0.0525), lim(D(p('sens/vx-kt'), v(80)), 0, 100)), v(-4.9))),
             desc='pitch attitude, pitch rate and airspeed; actuator 15 % authority, 3 %/s; off: it centres')
        c.fn('pba/rate-pct_s', IFTHEN(p('ctl/init-fast'), v(1000), v(3)))
        c.actuator('pba/pct', 'pba/cmd-pct', lag=10.0, rate='pba/rate-pct_s', clip=('-15', '15'))

    # ------------------------------------------------------------ flight path stabilization [GH fig. 5.1.5]
    def fps(s):
        """Attitude, airspeed and heading hold and turn coordination through the trim actuators (24.1/s, 10 %/s).
        The pilot's TRIM RELEASE (cyclic) and pedal switches are automatic, since the stick cannot stay where it was
        trimmed: the pilot is flying while a control (stick plus FlightGear trim) moves, or rests away from its trim
        point, the position where the FPS last took over. Then the channel synchronizes (the references follow, the
        actuators stay). Back at the trim point, the FPS takes over half a second after the control stops; held still
        somewhere else for 4 s, that becomes the trim point. The integrators take over the actuator positions at every
        synchronization, so taking over never bumps the controls."""
        c = s.ctl
        c.declare('ctl/fps', 1)
        c.declare('fps/detent', 0.03)          # away from the trim point by more than this (fraction of travel)
        c.declare('fps/detent-ped', 0.04)
        c.declare('fps/rate-thr', 0.1)         # moving faster than this (travel per second)
        c.declare('fps/accept-s', 4.0)         # held still elsewhere this long: new trim point
        c.declare('fps/holdoff-s', 0.5)
        for prop in ('fps/cyc-hold', 'fps/ped-hold', 'fps/air-s', 'fps/turn', 'fps/theta-ref', 'fps/vx-ref', 'fps/phi-ref',
                     'fps/psi-ref', 'fps/coll-ref', 'fps/pitch-int', 'fps/roll-int', 'fps/yaw-int',
                     'fps/lon-rest', 'fps/lat-rest', 'fps/ped-rest', 'fps/cyc-still-s', 'fps/ped-still-s'):
            c.declare(prop, 0)
        clamp = lambda x, lo, hi: MAX(v(lo), MIN(v(hi), x))
        dt = p('simulation/dt')
        c.channel('FPS pilot')
        for a in ('lon', 'lat', 'ped'):
            c.washout('fps/%s-move' % a, 'ctl/%s-norm' % a, 0.15)         # ~ 0.15 x the rate of the control
        moving = lambda a: GT(ABS(p('fps/%s-move' % a)), M(v(0.15), p('fps/rate-thr')))
        away = lambda a, det: GT(ABS(D(p('ctl/%s-norm' % a), p('fps/%s-rest' % a))), p(det))
        for grp, axes, det in (('cyc', ('lon', 'lat'), 'fps/detent'), ('ped', ('ped',), 'fps/detent-ped')):
            mov = MAX(*[moving(a) for a in axes]) if len(axes) > 1 else moving(axes[0])
            c.fn('fps/%s-still-next' % grp, IFTHEN(mov, v(0), MIN(v(100), S(p('fps/%s-still-s' % grp), dt))), out='fps/%s-still-s' % grp)
            accept = GT(p('fps/%s-still-s' % grp), p('fps/accept-s'))
            for a in axes:
                # the trim point follows the control within the detent, and jumps to it once accepted
                c.fn('fps/%s-rest-next' % a, IFTHEN(MAX(D(v(1), away(a, det)), accept), p('ctl/%s-norm' % a), p('fps/%s-rest' % a)),
                     out='fps/%s-rest' % a)
            off = MAX(*[away(a, det) for a in axes]) if len(axes) > 1 else away(axes[0], det)
            c.fn('fps/%s-pilot' % grp, MAX(mov, off))
        cyc, ped = p('fps/cyc-pilot'), p('fps/ped-pilot')
        decay = Q(dt, p('fps/holdoff-s'))
        c.fn('fps/cyc-hold-next', MAX(cyc, D(p('fps/cyc-hold'), decay)), out='fps/cyc-hold')
        c.fn('fps/ped-hold-next', MAX(ped, D(p('fps/ped-hold'), decay)), out='fps/ped-hold')
        cyc_on, ped_on = GT(p('fps/cyc-hold'), v(0)), GT(p('fps/ped-hold'), v(0))
        wow = MAX(*[p('gear/unit[%d]/WOW' % i) for i in range(3)])
        c.fn('fps/air-s-next', IFTHEN(GT(wow, v(0.5)), v(0), MIN(v(10), S(p('fps/air-s'), dt))), out='fps/air-s')
        # running: on, and not standing with the collective down (then the actuators centre at 10 %/s)
        run = c.fn('fps/run', M(p('ctl/fps'), D(v(1), M(GT(wow, v(0.5)), LT(p('sens/coll-pct'), v(25))))))
        on = c.fn('fps/on', M(run, GT(p('fps/air-s'), v(1.0)), D(v(1), p('ctl/init-fast'))), desc='holding')
        sync_cyc = c.fn('fps/sync-cyc', MAX(D(v(1), on), cyc_on))
        V = p('sens/vx-kt')
        phi = p('attitude/phi-deg')
        # turn coordination: entered by rolling with the cyclic above 60 kt, left with the feet on the pedals [GH], or
        # by rolling back towards level with the cyclic, or slowing down. GenHel enters beyond 2 deg of bank; 5 here,
        # as this model trims with up to 3 deg of left bank at speed
        engage = M(on, GT(V, v(60)), cyc_on, GT(ABS(phi), v(5)), D(v(1), ped_on))
        stay = M(on, GT(V, v(55)), D(v(1), ped_on), D(v(1), M(cyc_on, LT(ABS(phi), v(4)))))
        tnew = c.fn('fps/turn-new', IFTHEN(GT(p('fps/turn'), v(0.5)), stay, engage))
        chg = c.fn('fps/turn-chg', ABS(D(tnew, p('fps/turn'))))
        c.fn('fps/turn-next', tnew, out='fps/turn', desc='turn coordination (above 60 kt, banked)')
        turn = p('fps/turn')
        sync_yaw = c.fn('fps/sync-yaw', MAX(D(v(1), on), ped_on, chg))

        def channel(axis, sync, P, Ki_e, act_name):
            """bumpless: while synchronized the integrator holds the actuator position less the proportional terms"""
            Pp = c.fn('fps/%s-p' % axis, P)
            X = p('fps/%s-pct' % axis)
            c.fn('fps/%s-int-next' % axis, IFTHEN(sync, D(X, Pp), clamp(S(p('fps/%s-int' % axis), M(dt, Ki_e)), -60, 60)),
                 out='fps/%s-int' % axis)
            c.fn('fps/%s-u' % axis, M(run, S(p('fps/%s-int' % axis), Pp)), clip=('-50', '50'))
            c.actuator(act_name, 'fps/%s-u' % axis, rate=10.0, lag=24.1, clip=('-50', '50'), out='fps/%s-pct' % axis)

        c.channel('FPS pitch')
        th = p('attitude/theta-deg')
        c.fn('fps/theta-ref-next', IFTHEN(sync_cyc, th, p('fps/theta-ref')), out='fps/theta-ref')
        c.fn('fps/vx-ref-next', IFTHEN(MAX(sync_cyc, LT(V, v(30))), V, p('fps/vx-ref')), out='fps/vx-ref')
        dth, dv = D(th, p('fps/theta-ref')), D(V, p('fps/vx-ref'))
        channel('pitch', sync_cyc, S(M(v(-0.336), p('sas/q1')), M(v(-1.42), dth), clamp(M(v(0.22), dv), -1.6, 1.6)),
                M(v(0.0188), dv), 'fps/pitch-act')

        c.channel('FPS roll')
        c.fn('fps/phi-ref-next', IFTHEN(sync_cyc, phi, p('fps/phi-ref')), out='fps/phi-ref')
        dphi = D(phi, p('fps/phi-ref'))
        lin = clamp(dphi, -7.9, 7.9)
        channel('roll', sync_cyc, S(M(v(-0.189), p('sas/p1')), M(v(-0.75), lin), M(v(-0.375), D(dphi, lin))),
                M(v(-0.0868), dphi, D(v(1), turn)), 'fps/roll-act')

        c.channel('FPS yaw')
        psi = p('attitude/psi-deg')
        c.fn('fps/psi-ref-next', IFTHEN(MAX(sync_yaw, turn), psi, p('fps/psi-ref')), out='fps/psi-ref')
        c.fn('fps/coll-ref-next', IFTHEN(sync_yaw, p('sens/coll-pct'), p('fps/coll-ref')), out='fps/coll-ref')
        dpsi = c.fn('fps/dpsi', D(MOD(S(D(psi, p('fps/psi-ref')), v(540)), v(360)), v(180)))
        c.lag('fps/ay-lag', 'sens/ay-fps2', 1.0)
        c.actuator('fps/vx-rl', 'sens/vx-kt', rate=10.0)
        mix = clamp(Q(D(v(100), p('fps/vx-rl')), v(60)), 0, 1)
        ay = p('fps/ay-lag')
        heading = D(v(1), turn)
        channel('yaw', sync_yaw,
                S(M(heading, v(-2.2), dpsi), M(turn, S(M(v(0.11), p('sas/p1')), M(v(-2.38), ay))),
                  M(v(-0.625), D(p('sens/coll-pct'), p('fps/coll-ref')), mix)),
                S(M(heading, v(-0.348), D(dpsi, clamp(dpsi, -1, 1))), M(turn, v(-0.656), ay)), 'fps/yaw-act')

    # ------------------------------------------------------------ main rotor states (FCS)
    def rotor_states(s):
        r = s.rot
        ci, si = math.cos(U.SHAFT_TILT), math.sin(U.SHAFT_TILT)
        r.declare('rotor/omega-rad_sec', U.OMEGA)
        r.declare('rotor/psi-rad', 0.0)
        r.declare('rotor/tr/psi-rad', 0.0)
        for k in range(U.NB):
            r.declare('rotor/b%d/beta-rad' % k, 0.0)
            r.declare('rotor/b%d/betadot-rad_sec' % k, 0.0)
            r.declare('rotor/b%d/delta-rad' % k, -0.07 if U.LAG_DOF else 0.0)    # lead +
            r.declare('rotor/b%d/deltadot-rad_sec' % k, 0.0)
            r.declare('rotor/b%d/ldm-len-prev' % k, 16.0)
        r.declare('rotor/v0-fps', 0.0)
        if U.INFLOW_MODEL == 'peters-he':
            for st, tg in ((U.PH_COS, 'c'), (U.PH_SIN, 's')):
                for i in range(len(st)):
                    r.declare('rotor/ph/%s%d' % (tg, i), 0.0)
        r.declare('rotor/tr/dw', 0.0)
        if U.TR_MODEL == 'blades':
            for k in range(U.NB_TR):
                r.declare('rotor/tr/b%d/beta-rad' % k, U.PRECONE_TR)
                r.declare('rotor/tr/b%d/betadot-rad_sec' % k, 0.0)
            r.declare('rotor/tr/v0-fps', 0.0)
        if U.TR_MODEL == 'blades' and getattr(U, 'TR_DYN_STALL', None):
            # the tail rotor's section angles of attack of the frame before (the aerodynamic functions keep their last
            # values until they run again), for their rates in the stall delay
            # and whether each has stalled: its delayed angle past the static stall angle (SC1095's angle of maximum lift),
            # until the angle falls 4 deg below that again; Gormont's reattachment delay applies only then
            r.channel('tail rotor section angles')
            for k in range(U.NB_TR):
                for j in range(U.NSEG_TR):
                    e, q = 'aero/rotor/tr/b%d/s%d' % (k, j), 'rotor/tr/b%d/s%d' % (k, j)
                    r.declare(q + '/stalled', 0)
                    r.fn(q + '/alpha-prev', p(e + '/alpha-deg'))
                    ass = T1(e + '/mach', AF.MACH, STALL_ALPHA)
                    r.fn(q + '/stalled-next', IFTHEN(GT(ABS(p(e + '/alpha-lift-deg')), ass), v(1),
                                                     IFTHEN(LT(ABS(p(e + '/alpha-deg')), D(ass, v(4))), v(0), p(q + '/stalled'))), out=q + '/stalled')
        r.channel('shaft kinematics')
        # body rates (inertial) in shaft axes
        r.fn('rotor/ps', S(M(v(ci), p('velocities/p-rad_sec')), M(v(si), p('velocities/r-rad_sec'))))
        r.fn('rotor/qs', p('velocities/q-rad_sec'))
        r.fn('rotor/pds', S(M(v(ci), p('accelerations/pdot-rad_sec2')), M(v(si), p('accelerations/rdot-rad_sec2'))))
        r.fn('rotor/qds', p('accelerations/qdot-rad_sec2'))
        r.fn('rotor/Az', M(v(-G0), p('accelerations/Nz')), desc='hub specific force along the shaft (down +), ft/s2')
        r.channel('rotor azimuth')
        # (JSBSim's <mod> is an integer modulus, so the wrap is explicit)
        psn = S(p('rotor/psi-rad'), M(p('rotor/omega-rad_sec'), p('simulation/dt')))
        r.fn('rotor/psi-next', IFTHEN(GT(psn, v(2 * math.pi)), D(psn, v(2 * math.pi)), psn), out='rotor/psi-rad')
        # the tail rotor's, geared to it (for the model: fdm/rotorvis.py)
        psn = S(p('rotor/tr/psi-rad'), M(v(U.OMEGA_TR / U.OMEGA), p('rotor/omega-rad_sec'), p('simulation/dt')))
        r.fn('rotor/tr/psi-next', IFTHEN(GT(psn, v(2 * math.pi)), D(psn, v(2 * math.pi)), psn), out='rotor/tr/psi-rad')
        dt = p('simulation/dt')
        init_fast = GT(p('ctl/init-fast'), v(0.5))
        if U.LAG_DOF:
            # lag damper [GH 5.1-30]: position of the hub pickup relative to the blade (cuff) pickup in damper axes
            # (in); its moments on the blade follow from virtual work, -F dL/d(delta) and -F dL/d(beta)
            r.channel('lag damper')
            A_, B_, C_, D_, R_ = U.LDM_A, U.LDM_B, U.LDM_C, U.LDM_D, U.LDM_R
            for k in range(U.NB):
                b = 'rotor/b%d' % k
                be, dl = p(b + '/beta-rad'), p(b + '/delta-rad')
                cb, sb = COS(be), SIN(be)
                dd = S(dl, v(U.LDM_DELTA0))
                cd, sd = COS(dd), SIN(dd)
                th = r.fn(b + '/ldm-theta-rad', M(v(math.pi / 180), D(p('aero/rotor/b%d/theta-deg' % k), v(math.degrees(U.LDM_THETA_GEO)))),
                          desc='damper pickup angle: blade pitch at the cuff less the geometric angle')
                X = r.fn(b + '/ldm-x-in', S(M(v(A_), sb), M(v(B_), cd, cb), v(C_), M(v(D_), sd, cb)))
                Y = r.fn(b + '/ldm-y-in', S(NEG(M(v(R_), COS(th))), NEG(M(v(B_), sd)), M(v(D_), cd)))
                Z = r.fn(b + '/ldm-z-in', D(D(D(M(v(A_), cb), M(v(R_), SIN(th))), M(v(B_), sb, cd)), M(v(D_), sb, sd)))
                L = r.fn(b + '/ldm-len-in', SQ(S(SQR(X), SQR(Y), SQR(Z))))
                bcds = S(M(v(B_), cd), M(v(D_), sd))
                dLd = r.fn(b + '/ldm-dl-ddelta', Q(S(M(X, D(M(v(D_), cd), M(v(B_), sd)), cb), NEG(M(Y, bcds)), M(Z, D(M(v(B_), sd), M(v(D_), cd)), sb)), L),
                           desc='in per rad')
                dLb = r.fn(b + '/ldm-dl-dbeta', Q(S(M(X, D(M(v(A_), cb), M(bcds, sb))), NEG(M(Z, S(M(v(A_), sb), M(bcds, cb))))), L))
                # (not across a reset: the first frames after an initial condition have no previous length)
                Ld = r.fn(b + '/ldm-speed-ins', IFTHEN(LT(p('simulation/sim-time-sec'), v(0.05)), v(0), Q(D(L, p(b + '/ldm-len-prev')), dt)),
                          desc='stroke speed, in/s (+ extending)')
                r.fn(b + '/ldm-speed-abs', ABS(Ld))
                F = r.fn(b + '/ldm-force-lbs', M(IFTHEN(LT(Ld, v(0)), v(-1), v(1)), T1(b + '/ldm-speed-abs', U.LDM_V, U.LDM_F)),
                         desc='axial force, + resisting extension [GH table 1.5.4]')
                r.fn(b + '/ldm-mlag', M(v(-1 / 12.0), F, dLd), desc='damper moment about the lag hinge, ft lb (lead +)')
                r.fn(b + '/ldm-mflap', M(v(-1 / 12.0), F, dLb), desc='damper moment about the flap hinge, ft lb (up +)')
                r.fn(b + '/ldm-len-next', L, out=b + '/ldm-len-prev')
        r.channel('flapping')
        Ie = U.I_B + U.E_FLAP * U.S_B
        for k in range(U.NB):
            b = 'rotor/b%d' % k
            psi = S(p('rotor/psi-rad'), v(k * math.pi / 2))
            c, sn = COS(psi), SIN(psi)
            W = p('rotor/omega-rad_sec')
            be = p(b + '/beta-rad')
            if U.LAG_DOF:
                # centrifugal stiffness with lag: the lead rate adds to the spin, the hinge offset part acts at cos(delta)
                cf = NEG(M(S(M(v(U.I_B), COS(be), S(SQR(W), M(v(2), W, p(b + '/deltadot-rad_sec')))),
                             M(v(U.E_FLAP * U.S_B), COS(p(b + '/delta-rad')), W, W)), SIN(be)))
                extra = [p(b + '/ldm-mflap')]
            else:
                cf = NEG(M(S(M(v(U.I_B), COS(be)), v(U.E_FLAP * U.S_B)), W, W, SIN(be)))
                extra = []
            bdd = r.fn(b + '/betaddot', Q(S(
                p('aero/rotor/b%d/Mflap' % k),
                M(v(U.S_B), p('rotor/Az')),
                M(v(2 * Ie), W, D(M(p('rotor/ps'), c), M(p('rotor/qs'), sn))),
                M(v(Ie), S(M(p('rotor/pds'), sn), M(p('rotor/qds'), c))),
                cf, *extra), v(U.I_B)),
                desc='flap acceleration: aerodynamic moment, blade weight and g, gyroscopic terms, centrifugal stiffness, damper')
            bd_new = S(p(b + '/betadot-rad_sec'), M(bdd, p('simulation/dt')))
            b_new = S(p(b + '/beta-rad'), M(bd_new, p('simulation/dt')))
            lo, hi = U.FLAP_STOP
            # flap stops: the blade stays on the stop and loses the velocity into it
            r.fn(b + '/betadot-next', IFTHEN(LT(b_new, v(lo)), MAX(bd_new, v(0)), IFTHEN(GT(b_new, v(hi)), MIN(bd_new, v(0)), bd_new)),
                 out=b + '/betadot-rad_sec')
            # (a stopped rotor starts with its blades resting on the droop stops, rather than falling onto them)
            r.fn(b + '/beta-next', IFTHEN(M(init_fast, LT(W, v(5))), v(lo),
                                          MAX(v(lo), MIN(v(hi), S(p(b + '/beta-rad'), M(p(b + '/betadot-rad_sec'), p('simulation/dt')))))),
                 out=b + '/beta-rad')
            if U.LAG_DOF:
                # lead-lag [GH 5.1-31, derived in these axes]: aerodynamic and damper moments, centrifugal stiffness
                # through the hinge offset, Coriolis from flapping (a blade flapping up speeds up), rotor acceleration
                cb, sb = COS(be), SIN(be)
                dl, dld = p(b + '/delta-rad'), p(b + '/deltadot-rad_sec')
                # the hub's own angular acceleration: the rotor speed's relative to the airframe less the airframe's yaw
                # acceleration about the shaft (the rotor turns anticlockwise seen from above)
                Wd = IFTHEN(init_fast, v(0), D(p('drive/omega-dot'), p('drive/r-dot-shaft')))
                ddd = r.fn(b + '/deltaddot', S(
                    Q(S(p('aero/rotor/b%d/Mlag' % k), p(b + '/ldm-mlag')), M(v(U.I_B), SQR(cb))),
                    NEG(M(Q(v(U.E_FLAP * U.S_B / U.I_B), cb), S(M(SQR(W), SIN(dl)), M(Wd, COS(dl))))),
                    M(v(2), Q(sb, cb), p(b + '/betadot-rad_sec'), S(W, dld)),
                    NEG(Wd)), desc='lag acceleration (lead +)')
                dn = S(dld, M(ddd, dt))
                lo, hi = U.LAG_STOP
                dln = S(dl, M(dn, dt))
                r.fn(b + '/deltadot-next', IFTHEN(LT(dln, v(lo)), MAX(dn, v(0)), IFTHEN(GT(dln, v(hi)), MIN(dn, v(0)), dn)), out=b + '/deltadot-rad_sec')
                r.fn(b + '/delta-next', MAX(v(lo), MIN(v(hi), S(dl, M(p(b + '/deltadot-rad_sec'), dt)))), out=b + '/delta-rad')
        s.wake_turbulence()
        r.channel('elastic twist')
        # in the FCS so that it uses the previous frame's blade loads (an aerodynamic function may not depend on itself)
        vkt = Q(p('velocities/vt-fps'), v(1.68781))
        kfp = MAX(v(-0.00052), MIN(v(-0.0003), D(v(-0.0003), M(v(0.0000044), D(vkt, v(100))))))
        kw = getattr(U, 'WINDUP', 1.0)       # experiments: scale the wind-up
        r.fn('rotor/theta-dy-tip-deg', M(kfp, *([v(kw)] if kw != 1.0 else []), p('aero/rotor/blade-load-lbs')), desc='GenHel torsional wind-up at the tip, deg')
        r.channel('inflow')
        if U.INFLOW_MODEL == 'peters-he':
            # Peters-He finite-state wake [peters_he.py]: M a' + L^-1 V a = tau / 2 per state set (time Omega t), in the
            # wake frame. The states live in the hub frame: each harmonic pair (cos, sin) is turned by r x delta into
            # the wake frame, stepped semi-implicitly (the state's own stiffness implicit, the coupling to the other
            # states explicit), and turned back.
            h = M(MAX(p('rotor/omega-rad_sec'), v(1.0)), p('simulation/dt'))
            cd, sd = p('aero/rotor/ph/cd'), p('aero/rotor/ph/sd')
            rot = {1: (cd, sd), 2: (r.fn('rotor/ph/cd2', D(SQR(cd), SQR(sd))), r.fn('rotor/ph/sd2', M(v(2), cd, sd)))}
            sets = {'c': U.PH_COS, 's': U.PH_SIN}
            pair = {}                                    # (tg, i) -> index of the partner in the other set
            for i, st in enumerate(U.PH_COS):
                if st[1] > 0 and st in U.PH_SIN:
                    pair[('c', i)] = U.PH_SIN.index(st); pair[('s', U.PH_SIN.index(st))] = i
            def wake(tg, i, prop):
                # value of state / forcing (tg, i) in the wake frame from the hub-frame pair
                rh = sets[tg][i][1]
                if rh == 0 or (tg, i) not in pair:
                    return p(prop % (tg, i))
                cr, sr = rot[rh]; o = 's' if tg == 'c' else 'c'; po = p(prop % (o, pair[(tg, i)]))
                me = p(prop % (tg, i))
                return S(M(me, cr), M(po, sr)) if tg == 'c' else D(M(me, cr), M(po, sr))
            for tg, st in sets.items():
                for i in range(len(st)):
                    r.fn('rotor/ph/w%s%d' % (tg, i), wake(tg, i, 'rotor/ph/%s%d'))
                    r.fn('rotor/ph/wtau-%s%d' % (tg, i), wake(tg, i, 'aero/rotor/ph/tau-%s%d'))
            for tg, st in sets.items():
                for i, (j, rh) in enumerate(st):
                    k_ = Q(h, v(2 / math.pi * PH.H(j, rh)))
                    a = lambda i_, kk: p('aero/rotor/ph/A%s%d-%d' % (tg, i_, kk))
                    off = [M(a(i, kk), p('rotor/ph/w%s%d' % (tg, kk))) for kk in range(len(st)) if kk != i]
                    num = S(p('rotor/ph/w%s%d' % (tg, i)), M(k_, D(M(v(0.5), p('rotor/ph/wtau-%s%d' % (tg, i))), S(*off) if off else v(0))))
                    r.fn('rotor/ph/n%s%d' % (tg, i), Q(num, S(v(1), M(k_, a(i, i)))))
            for tg, st in sets.items():
                for i, (j, rh) in enumerate(st):
                    if rh == 0 or (tg, i) not in pair:
                        r.fn('rotor/ph/%s%d-next' % (tg, i), p('rotor/ph/n%s%d' % (tg, i)), out='rotor/ph/%s%d' % (tg, i))
                    else:
                        cr, sr = rot[rh]; o = 's' if tg == 'c' else 'c'; po = p('rotor/ph/n%s%d' % (o, pair[(tg, i)])); me = p('rotor/ph/n%s%d' % (tg, i))
                        # back to the hub frame: cos = a cr - b sr, sin = a sr + b cr
                        r.fn('rotor/ph/%s%d-next' % (tg, i), D(M(me, cr), M(po, sr)) if tg == 'c' else S(M(po, sr), M(me, cr)), out='rotor/ph/%s%d' % (tg, i))
            r.fn('rotor/v0-out', MAX(v(0), M(v(math.sqrt(3) * U.R), p('rotor/ph/c0'), p('rotor/omega-rad_sec'))), out='rotor/v0-fps',
                 desc='mean inflow, sqrt(3) x the first state, ft/s')
        else:
            # Pitt-Peters uniform inflow out of ground effect: (8/3pi) R dv/dt + 2 V' v = T / (rho A)
            A = math.pi * U.R ** 2
            r.fn('rotor/v0-next', MAX(v(0), S(p('rotor/v0-fps'), M(p('simulation/dt'), Q(D(
                Q(p('aero/rotor/thrust-lbs'), M(p('atmosphere/rho-slugs_ft3'), v(A * (U.TIP_LOSS ** 2 if getattr(U, 'TIP_LOSS_INFLOW', False) else 1.0)))),
                M(v(2), p('aero/rotor/Vprime-fps'), p('rotor/v0-fps'))), v(8 / (3 * math.pi) * U.R))))), out='rotor/v0-fps')
        if U.TR_MODEL == 'blades':
            s.tail_rotor_states()
            return
        # tail rotor: GenHel's closed-form Bailey solution [GH 5.4-7]. The inflow equation is solved by fixed-point
        # iteration, one (relaxed) step per frame, from the previous frame's thrust and advance ratios.
        r.channel('tail rotor')
        B = U.TIP_LOSS_TR
        mu, muz = p('aero/rotor/tr/mu'), p('aero/rotor/tr/muz')
        th = r.fn('rotor/tr/theta-rad', Q(S(p('ctl/thetaTR-deg'), M(v(-U.DA0_DT_TR * U.K1_TR), p('aero/rotor/tr/thrust-lbs')), v(U.BIAS_TR)), v(180 / math.pi)),
                  desc='blade pitch: rigging + 6 deg bias - delta-3 x coning')
        G = U.A_TR * U.SIGMA_TR / 2
        mu2 = SQR(mu)
        t31 = S(v(B ** 2 / 2), M(v(0.25), mu2)); t32 = S(v(B ** 3 / 3), M(v(B / 2), mu2)); t33 = S(v(B ** 4 / 4), M(v(B ** 2 / 4), mu2))
        dw = p('rotor/tr/dw')
        B2 = B ** 2 if getattr(U, 'TIP_LOSS_INFLOW', False) else 1.0     # momentum over the effective disc B^2 A
        dwn = Q(M(v(G), S(M(muz, t31), M(th, t32), M(v(U.TWIST_TR), t33))), S(M(v(2 * B2), SQ(S(mu2, SQR(D(muz, dw)), v(1e-6)))), M(v(G), t31)))
        r.fn('rotor/tr/dw-next', S(dw, M(v(0.5), D(dwn, dw))), out='rotor/tr/dw')

    def tail_rotor_states(s):
        """The blade-element tail rotor's states (aerodynamics in tail_rotor_blades): each blade's flapping on its centre
        hinge, as the main rotor's (semi-implicit, at the FDM rate: 15 deg of azimuth a step at 1190 rpm, its flap
        frequency 0.29 rad a step), and the uniform inflow (Pitt-Peters' momentum, over the effective disc B^2 A):
        (8 / 3 pi) R dv/dt + 2 V' v = T / (rho A), V' the mass flow through the disc. Its axial part goes to nearly
        nothing when the rotor moves into its own wake (a left crosswind or a right yaw, the wake blowing left): then
        the induced velocity grows and the thrust falls away, vortex ring state as momentum theory can't have it."""
        r = s.rot
        cK, sK = math.cos(U.CANT_TR), math.sin(U.CANT_TR)
        dt = p('simulation/dt')
        W = M(v(U.OMEGA_TR / U.OMEGA), p('rotor/omega-rad_sec'))
        init_fast = GT(p('ctl/init-fast'), v(0.5))
        r.channel('tail rotor flapping')
        # body rates in the hub frame (a0 aft, b0 = t x a0), inertial
        Pi, Qi, Ri = p('velocities/p-rad_sec'), p('velocities/q-rad_sec'), p('velocities/r-rad_sec')
        wa, wb = r.fn('rotor/tr/wa', NEG(Pi)), r.fn('rotor/tr/wb', S(M(v(sK), Qi), M(v(cK), Ri)))
        Pd, Qd, Rd = p('accelerations/pdot-rad_sec2'), p('accelerations/qdot-rad_sec2'), p('accelerations/rdot-rad_sec2')
        wad, wbd = r.fn('rotor/tr/wad', NEG(Pd)), r.fn('rotor/tr/wbd', S(M(v(sK), Qd), M(v(cK), Rd)))
        I = U.I_B_TR
        for k in range(U.NB_TR):
            b = 'rotor/tr/b%d' % k
            psi = S(p('rotor/tr/psi-rad'), v(k * math.pi / 2))
            c, sn = COS(psi), SIN(psi)
            be = p(b + '/beta-rad')
            # gyroscopic (Coriolis of the blade's spin in the turning hub frame) and hub angular acceleration terms
            wr = S(M(c, wa), M(sn, wb)); wed = S(NEG(M(sn, wad)), M(c, wbd))
            bdd = r.fn(b + '/betaddot', S(Q(S(
                p('aero/rotor/tr/b%d/Mflap' % k),
                NEG(M(v(U.K_FLAP_TR), D(be, v(U.PRECONE_TR)))),
                NEG(M(v(2 * I), W, wr))), v(I)),
                NEG(M(SQR(W), COS(be), SIN(be))), wed),
                desc='flap acceleration: aerodynamic and spring moments, gyroscopic, centrifugal stiffness, hub angular acceleration')
            bd_new = S(p(b + '/betadot-rad_sec'), M(bdd, dt))
            b_new = S(be, M(bd_new, dt))
            lo, hi = U.FLAP_STOP_TR
            r.fn(b + '/betadot-next', IFTHEN(LT(b_new, v(lo)), MAX(bd_new, v(0)), IFTHEN(GT(b_new, v(hi)), MIN(bd_new, v(0)), bd_new)),
                 out=b + '/betadot-rad_sec')
            r.fn(b + '/beta-next', IFTHEN(M(init_fast, LT(W, v(20))), v(U.PRECONE_TR),
                                          MAX(v(lo), MIN(v(hi), S(be, M(p(b + '/betadot-rad_sec'), dt))))), out=b + '/beta-rad')
        r.channel('tail rotor inflow')
        A = math.pi * U.R_TR ** 2 * (U.TIP_LOSS_TR ** 2 if getattr(U, 'TIP_LOSS_INFLOW', False) else 1.0)
        v0 = p('rotor/tr/v0-fps')
        Vm = r.fn('rotor/tr/Vprime-fps', SQ(S(SQR(p('aero/rotor/tr/Vinplane-fps')), SQR(S(p('aero/rotor/tr/Vaxial-fps'), v0)), v(25.0))),
                  desc='mass flow velocity through the disc (with a 5 ft/s floor, as the main rotor)')
        # (it reverses with the thrust: full right pedal)
        r.fn('rotor/tr/v0-next', S(v0, M(dt, Q(D(Q(p('aero/rotor/tr/rotor-thrust-lbs'), M(p('atmosphere/rho-slugs_ft3'), v(A))),
                                                M(v(2), Vm, v0)), v(8 / (3 * math.pi) * U.R_TR)))), out='rotor/tr/v0-fps')

    # ------------------------------------------------------------ unsteady wake (FCS)
    def wake_turbulence(s):
        """The uniform-plus-gradient inflow is steady; a real rotor's is not, most of all at low speed: the wake
        recirculates in ground effect, rolls up through translational lift (and washes the tail), and breaks down
        in a vortex ring descent. Modelled as coloured noise on the blade inflow (uniform: thrust; fore-aft and
        lateral gradients: pitch and roll) and on the tail rotor inflow (yaw), as fractions of the induced velocity.
        The intensities are estimates (not UH-60 data); rotor/turb/scale tunes them and rotor/turb/enable = 0 makes
        the model deterministic (trims, validation)."""
        r = s.rot
        r.declare('rotor/turb/enable', 1)
        r.declare('rotor/turb/scale', 1)
        r.channel('wake turbulence')
        A = math.pi * U.R ** 2
        clamp = lambda x, lo, hi: MAX(v(lo), MIN(v(hi), x))
        vh = r.fn('rotor/turb/vh-fps', SQ(MAX(v(1), Q(p('aero/rotor/thrust-lbs'), M(v(2 * A), p('atmosphere/rho-slugs_ft3'))))),
                  desc='hover induced velocity for the current thrust')
        mu = r.fn('rotor/turb/mu-star', Q(p('aero/rotor/Vinplane-fps'), MAX(vh, v(5))))
        ds = r.fn('rotor/turb/descent-star', Q(p('aero/rotor/Vhz-fps'), MAX(vh, v(5))))
        ige = r.fn('rotor/turb/ige', clamp(Q(D(v(3 * U.R), p('aero/rotor/hub-agl-ft')), v(2.2 * U.R)), 0, 1),
                   desc='1 near the ground, 0 above 1.5 diameters')
        etl = r.fn('rotor/turb/etl', tag('exp', NEG(SQR(Q(D(mu, v(0.9)), v(0.5))))), desc='translational lift band')
        vrs = r.fn('rotor/turb/vrs', M(tag('exp', NEG(SQR(Q(D(ds, v(0.8)), v(0.4))))), clamp(D(v(1), Q(mu, v(0.8))), 0, 1)))
        gain = M(p('rotor/turb/enable'), p('rotor/turb/scale'), GT(p('rotor/omega-rad_sec'), v(5)))
        sig = r.fn('rotor/turb/sigma', M(gain, S(M(v(0.05), tag('exp', M(v(-0.5), SQR(mu)))),
                                               M(v(0.10), ige, tag('exp', M(v(-0.35), SQR(mu)))),
                                               M(v(0.10), etl), M(v(0.25), vrs))))
        sig_tr = r.fn('rotor/turb/sigma-tr', M(gain, S(M(v(0.10), etl), M(v(0.05), ige))))
        # unit-variance coloured noise: white noise through a lag (variance dt/2tau)
        for name, tau in (('n0', 0.25), ('nx', 0.4), ('ny', 0.4), ('ntr', 0.3)):
            r.fn('rotor/turb/%s-in' % name, M(tag('random'), SQ(Q(v(2 * tau), MAX(p('simulation/dt'), v(1e-4))))))
            r.lag('rotor/turb/%s' % name, 'rotor/turb/%s-in' % name, tau)
        r.fn('rotor/turb/u0', M(sig, p('rotor/turb/n0')), desc='uniform inflow perturbation / v0')
        r.fn('rotor/turb/gx', M(sig, p('rotor/turb/nx')), desc='fore-aft gradient / v0 at the tip (+: more downwash in front)')
        r.fn('rotor/turb/gy', M(sig, p('rotor/turb/ny')), desc='lateral gradient / v0 at the tip (+: more on the right)')
        r.fn('rotor/turb/tr', M(sig_tr, p('rotor/turb/ntr')), desc='tail rotor axial velocity perturbation / v0')

    # ------------------------------------------------------------ geometry helpers
    def arm_rp(s, fs, bl, wl):
        """body-axis vector (ft) from the aerodynamic reference point to a station"""
        g = s.g
        return (Q(D(p('metrics/aero-rp-x-in'), v(fs)), v(12)), Q(D(v(bl), p('metrics/aero-rp-y-in')), v(12)), Q(D(p('metrics/aero-rp-z-in'), v(wl)), v(12)))

    def arm_cg(s, fs, bl, wl):
        return (Q(D(p('inertia/cg-x-in'), v(fs)), v(12)), Q(D(v(bl), p('inertia/cg-y-in')), v(12)), Q(D(p('inertia/cg-z-in'), v(wl)), v(12)))

    def add(s, F, Mo):
        for lst, x in zip((s.X, s.Y, s.Z, s.L, s.Mm, s.N), tuple(F) + tuple(Mo)):
            if x is not None: lst.append(x)

    # ------------------------------------------------------------ main rotor aerodynamics
    def main_rotor(s):
        g = s.g
        ci, si = math.cos(U.SHAFT_TILT), math.sin(U.SHAFT_TILT)
        to_s = lambda bx, by, bz: (S(M(v(ci), bx), M(v(si), bz)), by, S(M(v(-si), bx), M(v(ci), bz)))
        # hub air velocity (body), then shaft axes
        rx, ry, rz = s.arm_cg(U.FS_MR, U.BL_MR, U.WL_MR)
        rx, ry, rz = [g.fn('aero/rotor/hub-r%s-ft' % a, e) for a, e in zip('xyz', (rx, ry, rz))]
        Pa, Qa, Ra = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
        vb = (S(p('velocities/u-aero-fps'), D(M(Qa, rz), M(Ra, ry))),
              S(p('velocities/v-aero-fps'), D(M(Ra, rx), M(Pa, rz))),
              S(p('velocities/w-aero-fps'), D(M(Pa, ry), M(Qa, rx))))
        vs = to_s(*vb)
        Vhx, Vhy, Vhz = [g.fn('aero/rotor/Vh%s-fps' % a, e) for a, e in zip('xyz', vs)]
        ps_a = g.fn('aero/rotor/ps-aero', S(M(v(ci), Pa), M(v(si), Ra)))
        qs_a = Qa
        rs_a = g.fn('aero/rotor/rs-aero', S(M(v(-si), Pa), M(v(ci), Ra)))
        Vip = g.fn('aero/rotor/Vinplane-fps', SQ(S(SQR(Vhx), SQR(Vhy), v(1e-6))))
        # direction of the in-plane flow, along which the wake skews and the wash on the airframe has its in-plane
        # part. GenHel's wash tables put that part along the body x axis (forward flight); here it follows the flow,
        # which has no direction in the hover: below 5 ft/s it fades back to x, so a drift of a fraction of a foot
        # per second does not swing the full wash sideways onto the fin and tail rotor
        wdir = MIN(v(1), Q(Vip, v(5.0)))
        ux = g.fn('aero/rotor/uhx', S(M(wdir, Q(Vhx, Vip)), D(v(1), wdir)))
        uy = g.fn('aero/rotor/uhy', M(wdir, Q(Vhy, Vip)))
        v0 = p('rotor/v0-fps')
        # Glauert / Pitt-Peters mass flow and wake skew; inflow gradient towards the downstream side (Coleman)
        Vn = g.fn('aero/rotor/Vnormal-fps', D(v0, Vhz), 'flow through the disc (down +)')
        g.fn('aero/rotor/Vprime-fps', SQ(S(SQR(Vip), SQR(Vn), v(25.0))), 'mass flow velocity (with a 5 ft/s floor for vortex ring)')
        chi = g.fn('aero/rotor/chi-rad', ATAN2(Vip, MAX(Vn, v(0.5))), 'wake skew angle')
        if getattr(U, 'INFLOW_GRADIENT', 'genhel') == 'genhel':
            kx = g.fn('aero/rotor/kappa-x', SQR(SIN(chi)), 'Glauert factor sin^2 chi [GH 5.1-20]')
        elif U.INFLOW_GRADIENT == 'none':
            kx = g.fn('aero/rotor/kappa-x', v(0), 'uniform inflow (experiments)')
        else:
            kx = g.fn('aero/rotor/kappa-x', M(v(15 * math.pi / 32), Q(SIN(chi), S(v(1), COS(chi)))), 'tan(chi/2) * 15 pi/32')
        if getattr(U, 'LATERAL_INFLOW', 'none') == 'drees':
            g.fn('aero/rotor/kappa-y', M(v(-2.0 * getattr(U, 'KY_SCALE', 1.0)), Q(Vip, M(p('rotor/omega-rad_sec'), v(U.R)))), 'Drees lateral gradient -2 mu')
        # ground effect on the induced velocity (as GenHel [GH 5.8]): the hover value from the operator's manual hover
        # chart [uh60data], carried into forward flight with Heyson's ratio of forward-flight to hover interference
        # [NASA TN D-234], against the hub height and the wake's angle from the ground normal (skew minus disc tilt)
        hh = g.fn('aero/rotor/hub-agl-ft', S(p('position/h-agl-ft'), Q(D(v(U.WL_MR), p('inertia/cg-z-in')), v(12))))
        zr = g.fn('aero/rotor/ge-z-r', MAX(v(U.GE_ZR_MIN), Q(hh, v(U.R))))
        th_, ph_ = p('attitude/theta-rad'), p('attitude/phi-rad')
        dn = to_s(NEG(SIN(th_)), M(SIN(ph_), COS(th_)), M(COS(ph_), COS(th_)))     # earth down, shaft axes
        wd = g.fn('aero/rotor/ge-wake-down', S(M(NEG(Vhx), dn[0]), M(NEG(Vhy), dn[1]), M(Vn, dn[2])))
        w2 = S(SQR(Vip), SQR(Vn))
        g.fn('aero/rotor/ge-chi-deg', M(v(180 / math.pi), ATAN2(SQ(MAX(v(0), D(w2, SQR(wd)))), wd)),
             'wake angle from the ground normal')
        kh = g.fn('aero/rotor/ge-k-hover', tag('pow', S(v(1), Q(v(U.GE_K), SQR(zr))), v(U.GE_EXP)))
        g.fn('aero/rotor/kge', S(v(1), M(D(kh, v(1)), T2('aero/rotor/ge-z-r', 'aero/rotor/ge-chi-deg', HG.ZR, HG.CHI_DEG, HG.RATIO))),
             'induced velocity in / out of ground effect')
        g.fn('aero/rotor/vi-fps', M(p('aero/rotor/kge'), v0), 'uniform induced velocity with ground effect')
        pheon = U.INFLOW_MODEL == 'peters-he'
        if pheon:
            # Peters-He: flow parameters, wake skew X = tan(chi/2), gain matrices L^-1(X) x V; the azimuth is taken from
            # the downstream side of the in-plane flow (body axes in hover)
            WR = g.fn('aero/rotor/ph/omega-r-fps', MAX(M(p('rotor/omega-rad_sec'), v(U.R)), v(30)))
            X = g.fn('aero/rotor/ph/X', Q(SIN(chi), S(v(1), COS(chi))))
            VT = g.fn('aero/rotor/ph/VT', Q(p('aero/rotor/Vprime-fps'), WR))
            mu, lm = Q(Vip, WR), Q(v0, WR)
            la = Q(Vn, WR)
            Vm = g.fn('aero/rotor/ph/V', MAX(Q(v(5), WR), Q(S(SQR(mu), M(S(la, lm), la)), VT)))
            Xs = [0.05 * i for i in range(21)]
            for st, tg, kind in ((U.PH_COS, 'c', 'c'), (U.PH_SIN, 's', 's')):
                Li = [np.linalg.inv(PH.Lmat(st, x, kind)) for x in Xs]
                for i in range(len(st)):
                    for kk in range(len(st)):
                        Vk = VT if st[kk] == (1, 0) else Vm
                        g.fn('aero/rotor/ph/A%s%d-%d' % (tg, i, kk), M(T1('aero/rotor/ph/X', Xs, [float(L_[i, kk]) for L_ in Li]), Vk))
            # wake frame: azimuth measured from the downstream side of the in-plane flow, delta from the tail in the hub
            # frame (cos delta = ux, sin delta = -uy); any direction will do in hover, where X = 0 makes the equations
            # the same in every frame
            nw = g.fn('aero/rotor/ph/vip-fps', SQ(S(SQR(Vhx), SQR(Vhy))))
            slow = LT(nw, v(0.5))
            g.fn('aero/rotor/ph/cd', IFTHEN(slow, v(1), Q(Vhx, MAX(nw, v(0.5)))))
            g.fn('aero/rotor/ph/sd', IFTHEN(slow, v(0), NEG(Q(Vhy, MAX(nw, v(0.5))))))
            ph_tau = {('c', i): [] for i in range(len(U.PH_COS))}; ph_tau.update({('s', i): [] for i in range(len(U.PH_SIN))})
        rho = p('atmosphere/rho-slugs_ft3')
        a_snd = p('atmosphere/a-fps')
        W = p('rotor/omega-rad_sec')
        segs, edges = U.segments()
        a_rows, cl_tab, cd_tab = section_tables()
        Ft_k, Fn_k = [], []
        Fsum = {'x': [], 'y': [], 'z': []}; Msum = {'x': [], 'y': [], 'z': []}
        hrp = s.arm_rp(U.FS_MR, U.BL_MR, U.WL_MR)
        hrp_s = [g.fn('aero/rotor/hubrp-%s-ft' % a, e) for a, e in zip('xyz', to_s(*hrp))]
        Ie = U.I_B + U.E_FLAP * U.S_B
        mb = U.W_BLADE / G0; xcg = U.S_B / mb
        T_terms, Q_terms, torque_hinge = [], [], []
        for k in range(U.NB):
            b = 'aero/rotor/b%d' % k
            psi = g.fn(b + '/psi', S(p('rotor/psi-rad'), v(k * math.pi / 2)), 'azimuth of the hinge')
            ch = g.fn(b + '/cpsi', COS(psi)); sh = g.fn(b + '/spsi', SIN(psi))
            # the blade itself is lagged by delta (lead +) about the hinge: its aerodynamics and load directions
            # use the blade azimuth, the hinge position and the inertial loads the hinge azimuth
            dl = p('rotor/b%d/delta-rad' % k); dld = p('rotor/b%d/deltadot-rad_sec' % k)
            psib = g.fn(b + '/psib', S(psi, dl), 'azimuth of the (lagged) blade')
            c = g.fn(b + '/cpsib', COS(psib)); sn = g.fn(b + '/spsib', SIN(psib))
            beta = p('rotor/b%d/beta-rad' % k); bd = p('rotor/b%d/betadot-rad_sec' % k)
            cb = g.fn(b + '/cbeta', COS(beta)); sb = g.fn(b + '/sbeta', SIN(beta))
            # blade pitch: collective and cyclic through the swashplate (phase angle), twist added per segment
            ph = math.radians(U.SWASH_PHASE)
            th0 = g.fn(b + '/theta-deg', D(D(p('ctl/theta0-deg'), M(p('ctl/A1s-deg'), COS(S(psib, v(ph))))),
                                         M(p('ctl/B1s-deg'), SIN(S(psib, v(ph))))))
            if pheon:
                # states and forcing in the hub frame (azimuth from the tail); the FCS turns them into the wake frame
                trig = {0: v(1), 1: (c, sn), 2: (g.fn(b + '/ph-c2', D(SQR(c), SQR(sn))), g.fn(b + '/ph-s2', M(v(2), c, sn)))}
                tf = lambda tg, rh: v(1) if rh == 0 else trig[rh][0 if tg == 'c' else 1]
                proj = [(tg, i, jj, rh, g.fn(b + '/ph-%s%d' % (tg, i), M(p('rotor/ph/%s%d' % (tg, i)), tf(tg, rh))) if rh else p('rotor/ph/%s%d' % (tg, i)))
                        for st, tg in ((U.PH_COS, 'c'), (U.PH_SIN, 's')) for i, (jj, rh) in enumerate(st)]
            lat_terms = []
            if getattr(U, 'LATERAL_INFLOW', 'none') == 'drees':
                # Drees: k_y = -2 mu, less inflow on the advancing side (perpendicular to the in-plane wind)
                lat_terms = [M(p('aero/rotor/kappa-y'), S(M(sn, ux), M(c, uy)))]
            kx_terms = [] if pheon else [M(p('aero/rotor/kappa-x'), D(M(c, ux), M(sn, uy)))]
            gx = g.fn(b + '/gradx', S(v(1), M(Q(v(1), v(U.R)), S(*kx_terms, *lat_terms,
                                                                 NEG(M(c, p('rotor/turb/gx'))), M(sn, p('rotor/turb/gy'))))),
                      'inflow gradient factor per ft of radius (1 + kappa r/R cos(angle to the downstream side))')
            ft_j, fn_j, mf_j, q_j, fr_j, ml_j = [], [], [], [], [], []
            for j, (rr, dr) in enumerate(segs):
                e = 'aero/rotor/b%d/s%d' % (k, j)
                rho_h = rr - U.E_FLAP
                a_ = S(v(U.E_FLAP), M(v(rho_h), cb))
                dx, dy, dz = NEG(M(a_, c)), M(a_, sn), NEG(M(v(rho_h), sb))
                if pheon:
                    lam = S(*[M(v(PH.phi(jj, rh, rr / U.R)), P) for (tg, i, jj, rh, P) in proj])
                    vi = M(p('aero/rotor/kge'), S(M(WR, lam, S(v(1), p('rotor/turb/u0'))), M(v0, D(p(b + '/gradx'), v(1)), v(rr))))
                else:
                    vi = M(p('aero/rotor/vi-fps'), S(v(1), p('rotor/turb/u0'), M(D(p(b + '/gradx'), v(1)), v(rr))))
                Vx = g.fn(e + '/Vx', S(Vhx, D(M(qs_a, dz), M(rs_a, dy)), M(W, a_, sn), M(v(rho_h), bd, sb, c), M(v(rho_h), dld, cb, sn)))
                Vy = g.fn(e + '/Vy', S(Vhy, D(M(rs_a, dx), M(ps_a, dz)), M(W, a_, c), NEG(M(v(rho_h), bd, sb, sn)), M(v(rho_h), dld, cb, c)))
                Vz = g.fn(e + '/Vz', S(Vhz, D(M(ps_a, dy), M(qs_a, dx)), NEG(M(v(rho_h), bd, cb)), NEG(vi)))
                UT = g.fn(e + '/UT', S(M(Vx, sn), M(Vy, c)))
                UP = g.fn(e + '/UP', D(M(D(M(Vx, c), M(Vy, sn)), sb), M(Vz, cb)))
                V2 = g.fn(e + '/V2', S(SQR(UT), SQR(UP), v(1e-4)))
                Vv = g.fn(e + '/V', SQ(V2))
                th = S(th0, v(U.twist(rr)), M(p('rotor/theta-dy-tip-deg'), v(0.28 + 0.72 * math.sin(math.pi / 2 * rr / U.R))))
                ar = g.fn(e + '/alpha-raw', D(th, M(v(180 / math.pi), ATAN2(UP, UT))))
                al = g.fn(e + '/alpha-deg', IFTHEN(GT(ar, v(180)), D(ar, v(360)), IFTHEN(LT(ar, v(-180)), S(ar, v(360)), ar)))
                mach = Q(Vv, a_snd)
                if rr > U.TIP_SWEEP_R * U.R:
                    mach = M(mach, v(math.cos(U.TIP_SWEEP)))
                mach = g.fn(e + '/mach', MIN(mach, v(1.0)))
                cl = g.fn(e + '/cl', T2(e + '/alpha-deg', e + '/mach', a_rows, AF.MACH, cl_tab))
                cd = g.fn(e + '/cd', S(T2(e + '/alpha-deg', e + '/mach', a_rows, AF.MACH, cd_tab), v(U.DCD)))
                # tip loss: no lift outboard of B*R
                r_in, r_out = edges[j], edges[j + 1]
                fl = max(0.0, min(1.0, (U.TIP_LOSS * U.R - r_in) / (r_out - r_in)))
                # lift from the components normal to the span; drag along the whole flow, spanwise flow included
                UR = g.fn(e + '/UR', D(M(Vy, sn), M(Vx, c)), 'spanwise flow (outward), ft/s')
                Lk = g.fn(e + '/Lk', M(v(0.5 * U.CHORD * dr * fl), rho, Vv, cl), 'lift / V')
                span_drag = getattr(U, 'SPANWISE_DRAG', 1.0) > 0.5
                Dk = g.fn(e + '/Dk', M(v(0.5 * U.CHORD * dr), rho, cd, SQ(S(V2, SQR(UR))) if span_drag else Vv), 'drag / V')
                Ft = g.fn(e + '/Ft', NEG(S(M(Lk, UP), M(Dk, UT))), 'force along the blade motion, lbf')
                Fn = g.fn(e + '/Fn', D(M(Lk, UT), M(Dk, UP)), 'force normal to the blade (up), lbf')
                ft_j.append(Ft); fn_j.append(Fn); mf_j.append(M(v(rho_h), Fn)); q_j.append(NEG(M(a_, Ft))); ml_j.append(M(v(rho_h), cb, Ft))
                fr_j.append(NEG(M(Dk, UR)) if span_drag else v(0))
            Ft = g.fn(b + '/Ft', S(*ft_j)); Fn = g.fn(b + '/Fn', S(*fn_j))
            if pheon:
                for (tg, i, jj, rh, P) in proj:
                    sb_ = S(*[M(v(PH.phi(jj, rh, rr / U.R)), p('aero/rotor/b%d/s%d/Fn' % (k, j))) for j, (rr, dr) in enumerate(segs)])
                    ph_tau[(tg, i)].append(M(tf(tg, rh), cb, sb_))
            g.fn(b + '/Mflap', S(*mf_j), 'aerodynamic flap moment about the hinge, ft lbf')
            g.fn(b + '/Mlag', S(*ml_j), 'aerodynamic moment about the lag hinge, ft lbf (lead +)')
            Qb = g.fn(b + '/Q', S(*q_j), 'aerodynamic torque about the shaft (opposing rotation), ft lbf')
            Q_terms.append(Qb)
            T_terms.append(M(Fn, cb))
            # blade load on the hub at the hinge (shaft axes): aerodynamic + inertial reaction
            bdd = p('rotor/b%d/betaddot' % k)
            Fra = g.fn(b + '/Fr-aero', S(*fr_j), 'spanwise drag, outward, lbf')
            # inertial reaction of the blade on the hinge, in the hinge's radial and tangential directions: the blade
            # centre of mass is at e + x cos(beta) cos(delta) outward, x cos(beta) sin(delta) ahead (rotor speed
            # changes are left to the rotor degree of freedom)
            ddd = p('rotor/b%d/deltaddot' % k) if U.LAG_DOF else v(0)
            cdl, sdl = COS(dl), SIN(dl)
            Fr = g.fn(b + '/Finertial-r', S(M(v(mb * U.E_FLAP), W, W), M(v(U.S_B), S(
                M(SQR(W), cb, cdl), M(v(2), W, D(M(cb, cdl, dld), M(sb, sdl, bd))), M(cb, cdl, S(M(bd, bd), M(dld, dld))),
                M(sb, cdl, bdd), NEG(M(v(2), sb, sdl, bd, dld)), M(cb, sdl, ddd)))))
            Fc = g.fn(b + '/Finertial-t', M(v(U.S_B), S(
                M(SQR(W), cb, sdl), M(v(2), W, S(M(sb, cdl, bd), M(cb, sdl, dld))), M(cb, sdl, S(M(bd, bd), M(dld, dld))),
                M(sb, sdl, bdd), M(v(2), sb, cdl, bd, dld), NEG(M(cb, cdl, ddd)))))
            Fu = g.fn(b + '/Finertial-up', NEG(M(v(U.S_B), D(M(bdd, cb), M(bd, bd, sb)))))
            # aerodynamic loads along the lagged blade: e_t = (s, c, 0), n = (sb c, -sb s, -cb), e_r = (-c, s, 0);
            # inertial loads along the hinge radius (-ch, sh) and tangent (sh, ch); u = (0, 0, -1)
            Fx = g.fn(b + '/Fx', S(M(Ft, sn), M(Fn, sb, c), NEG(M(Fra, c)), NEG(M(Fr, ch)), M(Fc, sh)))
            Fy = g.fn(b + '/Fy', S(M(Ft, c), NEG(M(Fn, sb, sn)), M(Fra, sn), M(Fr, sh), M(Fc, ch)))
            Fz = g.fn(b + '/Fz', S(NEG(M(Fn, cb)), NEG(Fu)))
            # moment about the reference point: (hub + e e_r) x F, at the hinge
            hx = S(hrp_s[0], NEG(M(v(U.E_FLAP), ch))); hy = S(hrp_s[1], M(v(U.E_FLAP), sh)); hz = hrp_s[2]
            Fsum['x'].append(Fx); Fsum['y'].append(Fy); Fsum['z'].append(Fz)
            Msum['x'].append(D(M(hy, Fz), M(hz, Fy))); Msum['y'].append(D(M(hz, Fx), M(hx, Fz))); Msum['z'].append(D(M(hx, Fy), M(hy, Fx)))
            # torque about the shaft from forces at the hinge: (e e_r x F) . z = e (-c Fy - s Fx)
            torque_hinge.append(M(v(U.E_FLAP), NEG(S(M(ch, Fy), M(sh, Fx)))))
        T = g.fn('aero/rotor/thrust-lbs', S(*T_terms), 'aerodynamic thrust along the shaft (up)')
        if pheon:
            # pressure forcing tau = (1/2pi or 1/pi) sum over blades of the radial integral of lift / (rho Omega^2 R^3) x phi
            qn = MAX(M(rho, SQR(MAX(W, v(5))), v(U.R ** 4)), v(1e-6))
            for (tg, i), terms in ph_tau.items():
                rh = (U.PH_COS if tg == 'c' else U.PH_SIN)[i][1]
                # tip loss: the wake takes the loading over the effective disc B^2 A
                kb = 1 / U.TIP_LOSS ** 2 if getattr(U, 'TIP_LOSS_INFLOW', False) else 1.0
                g.fn('aero/rotor/ph/tau-%s%d' % (tg, i), Q(M(v(kb / (2 * math.pi) if rh == 0 else kb / math.pi), S(*terms)), qn))
        # GenHel's empirical torsional wind-up [GH 5.1-24, 5.1-49]: the tip twists nose down in proportion to the
        # mean blade aerodynamic load, more at high speed; the mode shape is 0.28 + 0.72 sin(90 deg r/R)
        Fb = g.fn('aero/rotor/blade-load-lbs', Q(S(*[SQ(S(SQR(p('aero/rotor/b%d/Fn' % k)), SQR(p('aero/rotor/b%d/Ft' % k)))) for k in range(U.NB)]), v(U.NB)))
        Qa = g.fn('aero/rotor/torque-ftlb', S(*Q_terms))
        g.fn('aero/rotor/power-hp', Q(M(Qa, W), v(550)))
        Qd = g.fn('aero/rotor/drive-torque-ftlb', p('drive/mr-torque-ftlb'), 'torque the drive train puts into the rotor shaft')
        Fs = [g.fn('aero/rotor/F%s-s' % a, S(*Fsum[a])) for a in 'xyz']
        Ms = [g.fn('aero/rotor/M%s-s' % a, S(*Msum[a])) for a in 'xyz']
        # the airframe gets the drive torque reaction (z down along the shaft: +Q yaws the nose right),
        # not the in-plane forces' torque about the shaft
        Mzs = g.fn('aero/rotor/Mz-airframe-s', S(D(Ms[2], S(*torque_hinge)), Qd))
        to_b = lambda x, y, z: (D(M(v(ci), x), M(v(si), z)), y, S(M(v(si), x), M(v(ci), z)))
        Fb = [g.fn('aero/rotor/F%s-lbs' % a, e) for a, e in zip('XYZ', to_b(*Fs))]
        Mb = [g.fn('aero/rotor/M%s-ftlb' % a, e) for a, e in zip('LMN', to_b(Ms[0], Ms[1], Mzs))]
        s.add(Fb, Mb)

    # ------------------------------------------------------------ tail rotor [GH 5.4]
    def psi_flow(s):
        """direction of the in-plane flight path at the main rotor hub (deg from the nose, to the right), for the
        ring-wake tables [mrwake.py]; defined once"""
        if not s.g.has('aero/rotor/psi-flow-deg'):
            ps = ATAN2(p('aero/rotor/uhy'), p('aero/rotor/uhx'))
            s.g.fn('aero/rotor/psi-flow-deg', M(v(180 / math.pi), IFTHEN(LT(ps, v(0)), S(ps, v(2 * math.pi)), ps)),
                   'direction of the in-plane flight path at the hub, from the nose to the right')

    def tr_hub_velocity(s):
        """The hub's velocity relative to the air (body axes): flight, rotation, the main rotor's wake (the ring-wake model
        over the disc, TR_WASH 'vortex', or GenHel's empennage factors), the fuselage sidewash and the fin maps' dynamic
        pressure loss [GH 5.4-5, 5.4-6]."""
        g = s.g
        rx, ry, rz = s.arm_cg(U.FS_TR, U.BL_TR, U.WL_TR)
        rx, ry, rz = [g.fn('aero/rotor/tr/r%s-ft' % a, e) for a, e in zip('xyz', (rx, ry, rz))]
        Pa, Qa, Ra = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
        kq, v0 = p('aero/emp/kqv'), p('aero/rotor/vi-fps')
        wash = getattr(U, 'TR_WASH', 'genhel')
        if wash == 'vortex':
            # the main rotor's wake from the ring-wake model [mrwake.py] against the skew, the direction it is blown
            # and the disc's tilt, mean over the tail rotor's disc: the hub's velocity relative to the air is less it
            mw = mrwake.load()
            s.psi_flow()
            k = [g.fn('aero/rotor/tr/k-mr-%s' % a, T3('aero/rotor/chi-deg', 'aero/rotor/psi-flow-deg', 'aero/rotor/a1-deg', mw['chi'], mw['psi'],
                                                       mw['a1'], [[[c[i] for c in row] for row in tab] for tab in mw['k']]),
                      'main rotor induced velocity over the tail rotor disc, over its mean inflow') for i, a in enumerate('xyz')]
            wx, wy, wz = [NEG(M(ki, v0)) for ki in k]
            if getattr(U, 'TR_WASH_BLEND', None):
                # near the hover the ideal ring wake (no contraction, no tip-vortex roll-up) leaves the tail rotor outside
                # the wake; GenHel's measured empennage factors put 0.4 of the inflow on it there. Blend from GenHel's at
                # a skew of TR_WASH_BLEND[0] to the ring wake at TR_WASH_BLEND[1]
                c0, c1 = U.TR_WASH_BLEND
                bw = g.fn('aero/rotor/tr/wash-blend', MAX(v(0), MIN(v(1), Q(D(p('aero/rotor/chi-deg'), v(c0)), v(c1 - c0)))),
                          '0: GenHel\'s wash factors, 1: the ring wake')
                ekx, ekz = p('aero/emp/ekx'), p('aero/emp/ekz')
                ux, uy = p('aero/rotor/uhx'), p('aero/rotor/uhy')
                gx, gy, gz = M(ekx, v0, ux), M(ekx, v0, uy), NEG(M(ekz, v0))
                wx, wy, wz = [S(M(bw, a), M(D(v(1), bw), b)) for a, b in ((wx, gx), (wy, gy), (wz, gz))]
        elif wash == 'none':
            wx = wy = wz = v(0)
        else:
            ekx, ekz = p('aero/emp/ekx'), p('aero/emp/ekz')
            ux, uy = p('aero/rotor/uhx'), p('aero/rotor/uhy')
            wx, wy, wz = M(ekx, v0, ux), M(ekx, v0, uy), NEG(M(ekz, v0))
        ufwd = MAX(v(0), p('velocities/u-aero-fps'))
        return [g.fn('aero/rotor/tr/V%s-fps' % a, e) for a, e in zip('xyz', (
            S(M(kq, p('velocities/u-aero-fps')), D(M(Qa, rz), M(Ra, ry)), wx),
            S(M(kq, p('velocities/v-aero-fps')), D(M(Ra, rx), M(Pa, rz)), wy, NEG(M(ufwd, p('aero/emp/sigma-deg'), kq, v(math.pi / 180)))),
            S(M(kq, p('velocities/w-aero-fps')), D(M(Pa, ry), M(Qa, rx)), wz)))]

    def tr_blockage(s):
        vkt = Q(p('velocities/vt-fps'), v(1.68781))
        return s.g.fn('aero/rotor/tr/k-blockage', IFTHEN(LT(vkt, v(U.VBLK_TR)), S(v(U.KBLK_TR), M(v((1 - U.KBLK_TR) / U.VBLK_TR ** 2), SQR(vkt))), v(1)),
                      'fin blockage [GH 5.4-8]')

    def tr_wake_on_fin(s, Vt, Va, Vb, vi):
        """the tail rotor wake's skew and direction and its induced velocity, for the fin's sidewash (empennage)"""
        g = s.g
        g.fn('aero/rotor/tr/vi-fps', vi, 'induced velocity at the disc')
        g.fn('aero/rotor/tr/chi-deg', M(v(180 / math.pi), ATAN2(p('aero/rotor/tr/Vinplane-fps'), S(Vt, vi))),
             'wake skew from the axis (0: straight along -t; over 90: blown back through the disc)')
        ps = ATAN2(NEG(Vb), NEG(Va))
        g.fn('aero/rotor/tr/psi-wake-deg', M(v(180 / math.pi), IFTHEN(LT(ps, v(0)), S(ps, v(2 * math.pi)), ps)),
             'direction the wake leaves in the disc plane, from aft towards down')

    def tail_rotor(s):
        if U.TR_MODEL == 'blades':
            return s.tail_rotor_blades()
        s.tail_rotor_bailey()

    def tail_rotor_blades(s):
        """Blade-element tail rotor, canted 20 deg, top blade aft: four blades flapping on a centre hinge (states in
        tail_rotor_states), with delta-3, each of NSEG_TR segments at its own angle of attack and Mach number in the
        SC1095 tables, in the air at the hub (tr_hub_velocity) plus its own motion and the induced inflow. Thrust,
        in-plane force and the blades' moments on the hub (the torque among them: the drive's reaction on the
        airframe) all come from the blades; the fin blockage is the fin's own force in the rotor's wake (TR_BLOCKAGE 'fin',
        empennage) or GenHel's factor on the thrust.

        Hub frame: t the thrust axis (right and up), a0 aft (a blade at zero azimuth, as in the model's tail rotor),
        b0 = t x a0 (down); a blade at azimuth psi lies along r = cos psi a0 + sin psi b0 and moves along
        e = -sin psi a0 + cos psi b0; flapped by beta (towards t) it lies along d = cos b r + sin b t, normal
        n = -sin b r + cos b t."""
        g = s.g
        cK, sK = math.cos(U.CANT_TR), math.sin(U.CANT_TR)
        t, a0, b0 = (0.0, cK, -sK), (-1.0, 0.0, 0.0), (0.0, sK, cK)
        dot = lambda u, x: S(*[M(v(cu), xe) for cu, xe in zip(u, x) if cu != 0])
        vb = s.tr_hub_velocity()
        Va = g.fn('aero/rotor/tr/Va-fps', dot(a0, vb)); Vb = g.fn('aero/rotor/tr/Vb-fps', dot(b0, vb))
        Vt = g.fn('aero/rotor/tr/Vaxial-fps', S(dot(t, vb), M(p('aero/rotor/vi-fps'), p('rotor/turb/tr'))),
                  'hub velocity along the thrust axis, relative to the air')
        g.fn('aero/rotor/tr/Vinplane-fps', SQ(S(SQR(Va), SQR(Vb), v(1e-6))))
        rates = (p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec'))
        wa, wb, wt = [g.fn('aero/rotor/tr/w%s' % n, dot(u, rates)) for n, u in (('a', a0), ('b', b0), ('t', t))]
        W = g.fn('aero/rotor/tr/omega-rad_sec', M(v(U.OMEGA_TR / U.OMEGA), p('rotor/omega-rad_sec')))
        v0 = p('rotor/tr/v0-fps')
        rho, a_snd = p('atmosphere/rho-slugs_ft3'), p('atmosphere/a-fps')
        segs, edges = U.segments(U.NSEG_TR, U.R_ROOT_TR, U.R_TR)
        a_rows, cl_tab, cd_tab = section_tables()
        Fa, Fb, Ft_, Ma, Mb, Mt = [], [], [], [], [], []
        ds = U.TR_DYN_STALL == 'gormont'
        for k in range(U.NB_TR):
            b = 'aero/rotor/tr/b%d' % k
            psi = g.fn(b + '/psi', S(p('rotor/tr/psi-rad'), v(k * math.pi / 2)))
            c, sn = g.fn(b + '/cpsi', COS(psi)), g.fn(b + '/spsi', SIN(psi))
            beta, bd = p('rotor/tr/b%d/beta-rad' % k), p('rotor/tr/b%d/betadot-rad_sec' % k)
            cb, sb = g.fn(b + '/cbeta', COS(beta)), g.fn(b + '/sbeta', SIN(beta))
            vr = g.fn(b + '/Vr', S(M(c, Va), M(sn, Vb)))
            ve = g.fn(b + '/Ve', S(NEG(M(sn, Va)), M(c, Vb)))
            wr, we = S(M(c, wa), M(sn, wb)), g.fn(b + '/we', S(NEG(M(sn, wa)), M(c, wb)))
            vn = g.fn(b + '/Vn', S(NEG(M(sb, vr)), M(cb, S(Vt, v0))), 'hub velocity and inflow along the blade normal')
            vd = g.fn(b + '/Vd', S(M(cb, vr), M(sb, S(Vt, v0))), 'spanwise')
            rote = g.fn(b + '/rot-e', D(M(wt, cb), M(wr, sb)), 'body rotation: velocity along e per ft of radius')
            th0 = g.fn(b + '/theta-deg', D(S(p('ctl/thetaTR-deg'), v(U.BIAS_TR)), M(v(U.K1_TR * 180 / math.pi), beta)),
                       'pitch at the centre: cuff + bias - tan(delta-3) x flap [GH 5.4-7]')
            fn_j, ft_j, fr_j, mf_j, mt_j = [], [], [], [], []
            for j, (rr, dr) in enumerate(segs):
                e = b + '/s%d' % j
                UT = g.fn(e + '/UT', S(ve, M(v(rr), rote), M(W, v(rr), cb)))
                UP = g.fn(e + '/UP', S(vn, NEG(M(v(rr), we)), M(v(rr), bd)), 'flow through the blade, from the thrust side +')
                V2 = g.fn(e + '/V2', S(SQR(UT), SQR(UP), v(1e-4)))
                Vv = g.fn(e + '/V', SQ(V2))
                th = S(th0, v(math.degrees(U.TWIST_TR) * rr / U.R_TR))
                ar = g.fn(e + '/alpha-raw', D(th, M(v(180 / math.pi), ATAN2(UP, UT))))
                g.fn(e + '/alpha-deg', IFTHEN(GT(ar, v(180)), D(ar, v(360)), IFTHEN(LT(ar, v(-180)), S(ar, v(360)), ar)))
                g.fn(e + '/mach', MIN(Q(Vv, a_snd), v(1.0)))
                if ds:
                    cl, cd = s.stall_delay(e, 'rotor/tr/b%d/s%d/alpha-prev' % (k, j), Vv, a_rows, cl_tab, cd_tab)
                else:
                    cl = g.fn(e + '/cl', T2(e + '/alpha-deg', e + '/mach', a_rows, AF.MACH, cl_tab))
                    cd = g.fn(e + '/cd', S(T2(e + '/alpha-deg', e + '/mach', a_rows, AF.MACH, cd_tab), v(U.DCD)))
                r_in, r_out = edges[j], edges[j + 1]
                fl = max(0.0, min(1.0, (U.TIP_LOSS_TR * U.R_TR - r_in) / (r_out - r_in)))
                Lk = g.fn(e + '/Lk', M(v(0.5 * U.CHORD_TR * dr * fl), rho, Vv, cl), 'lift / V')
                Dk = g.fn(e + '/Dk', M(v(0.5 * U.CHORD_TR * dr), rho, cd, SQ(S(V2, SQR(vd)))), 'drag / V')
                Fn = g.fn(e + '/Fn', D(M(Lk, UT), M(Dk, UP)), 'force along the blade normal (thrust side), lbf')
                Ft = g.fn(e + '/Ft', NEG(S(M(Lk, UP), M(Dk, UT))), 'force along the blade motion, lbf')
                fn_j.append(Fn); ft_j.append(Ft); fr_j.append(NEG(M(Dk, vd))); mf_j.append(M(v(rr), Fn)); mt_j.append(M(v(rr), Ft))
            Fn_k, Ft_k, Fr_k = g.fn(b + '/Fn', S(*fn_j)), g.fn(b + '/Ft', S(*ft_j)), g.fn(b + '/Fr', S(*fr_j))
            g.fn(b + '/Mflap', S(*mf_j), 'aerodynamic flap moment about the hinge, ft lbf (towards the thrust side +)')
            Mte = g.fn(b + '/Me', S(*mt_j), 'moment of the forces along the blade motion, ft lbf')
            # on the hub (hinge at the centre: no flap moment): forces, and r Ft (cos b t - sin b r)
            rad = g.fn(b + '/F-radial', D(M(Fr_k, cb), M(Fn_k, sb)))
            Fa.append(D(M(rad, c), M(Ft_k, sn))); Fb.append(S(M(rad, sn), M(Ft_k, c))); Ft_.append(S(M(Fn_k, cb), M(Fr_k, sb)))
            Ma.append(NEG(M(Mte, sb, c))); Mb.append(NEG(M(Mte, sb, sn))); Mt.append(M(Mte, cb))
        Fha, Fhb = g.fn('aero/rotor/tr/Fa-lbs', S(*Fa), 'in-plane force, aft'), g.fn('aero/rotor/tr/Fb-lbs', S(*Fb))
        T = g.fn('aero/rotor/tr/rotor-thrust-lbs', S(*Ft_), 'the blades\' thrust')
        Mha, Mhb, Mht = [g.fn('aero/rotor/tr/M%s-ftlb' % n, S(*x)) for n, x in (('a', Ma), ('b', Mb), ('t', Mt))]
        Qtr = g.fn('aero/rotor/tr/torque-ftlb', NEG(Mht), 'aerodynamic torque (against the rotation)')
        g.fn('aero/rotor/tr/power-hp', Q(M(Qtr, W), v(550)))
        # fin blockage: the fin's own force in the tail rotor's wake (empennage), or GenHel's factor on the thrust
        if U.TR_BLOCKAGE == 'fin':
            s.tr_wake_on_fin(Vt, Va, Vb, v0)
            kb = g.fn('aero/rotor/tr/k-blockage', v(1), 'none on the thrust: the fin\'s own force in the wake')
        else:
            kb = s.tr_blockage()
        Tn = g.fn('aero/rotor/tr/thrust-lbs', M(T, kb), 'net of the fin blockage')
        # body axes: x = -a, y = sK b + cK t, z = cK b - sK t
        Fx, Fy, Fz = NEG(Fha), S(M(v(sK), Fhb), M(v(cK), Tn)), D(M(v(cK), Fhb), M(v(sK), Tn))
        Mx, My, Mz = NEG(Mha), S(M(v(sK), Mhb), M(v(cK), Mht)), D(M(v(cK), Mhb), M(v(sK), Mht))
        Fx, Fy, Fz = [g.fn('aero/rotor/tr/F%s-lbs' % n, e) for n, e in zip('XYZ', (Fx, Fy, Fz))]
        ax, ay, az = s.arm_rp(U.FS_TR, U.BL_TR, U.WL_TR)
        Mo = (S(D(M(ay, Fz), M(az, Fy)), Mx), S(D(M(az, Fx), M(ax, Fz)), My), S(D(M(ax, Fy), M(ay, Fx)), Mz))
        s.add((Fx, Fy, Fz), Mo)

    def stall_delay(s, e, prev, Vv, a_rows, cl_tab, cd_tab):
        """Gormont's dynamic stall delay [USAAMRDL TR 72-67, eqs. 39-45] on a section e: the static tables are entered
        at alpha - K1 gamma sqrt|c alpha' / 2V| sign(alpha') (K1 1 pitching up, 0.5 down; gamma from the thickness
        ratio, falling to nothing at high Mach, one for lift and one for drag and moment), the lift carried back to the
        section's own angle along the lift curve slope: the linear lift less the stall's shortfall at the delayed angle
        (Gormont scales by (alpha - alpha0) / (alpha_ref - alpha0), the same in the linear range but singular near zero
        lift; held within +-2.2). alpha' from the frame before's angle (prev). Faded out in reverse flow (|alpha| > 35 deg)
        and below 60-120 ft/s, where the angle swings through the blade's own wake rather than pitching. The reattachment delay (K1 0.5 with the angle falling) only once the section has
        stalled (the flag kept in the FCS): a section that never separated has nothing to reattach."""
        g = s.g
        d = 0.06 - U.TC_SC1095
        gl, m1l, m0l = 1.4 - 6.0 * d, 0.4 + 5.0 * d, 0.9 + 2.5 * d
        gd, m1d, m0d = 1.0 - 2.5 * d, 0.2, 0.7 + 2.5 * d
        a = p(e + '/alpha-deg')
        da = D(a, p(prev))
        da = IFTHEN(GT(da, v(180)), D(da, v(360)), IFTHEN(LT(da, v(-180)), S(da, v(360)), da))
        ad = g.fn(e + '/alpha-dot', Q(M(v(math.pi / 180), da), p('simulation/dt')), 'rad/s')
        rt = SQ(MIN(v(0.05), Q(M(v(0.5 * U.CHORD_TR), ABS(ad)), MAX(Vv, v(60)))))
        fade = M(MAX(v(0), MIN(v(1), Q(D(Vv, v(60)), v(60)))), LT(ABS(a), v(35)))
        k1 = IFTHEN(LT(ad, v(0)), M(v(-U.DS_K1_DOWN), p(prev.replace('alpha-prev', 'stalled'))), v(1.0))
        base = M(v(180 / math.pi), k1, rt, fade)
        dl = g.fn(e + '/dalpha-lift-deg', M(base, T1(e + '/mach', [0, m1l, m0l, 1.0], [gl, gl, 0, 0])), 'stall delay')
        dd = M(base, T1(e + '/mach', [0, m1d, m0d, 1.0], [gd, gd, 0, 0]))
        g.fn(e + '/alpha-lift-deg', D(a, dl))
        g.fn(e + '/alpha-drag-deg', D(a, dd))
        slope = T1(e + '/mach', AF.MACH, LIFT_SLOPE)
        cl = g.fn(e + '/cl', MAX(v(-2.2), MIN(v(2.2), S(T2(e + '/alpha-lift-deg', e + '/mach', a_rows, AF.MACH, cl_tab), M(slope, dl)))))
        cd = g.fn(e + '/cd', S(T2(e + '/alpha-drag-deg', e + '/mach', a_rows, AF.MACH, cd_tab), v(U.DCD)))
        return cl, cd

    def tail_rotor_bailey(s):
        """Bailey's closed-form rotor (inflow solved in the FCS), canted 20 deg, in the air of tr_hub_velocity, with the
        fin blockage as the blade-element rotor's."""
        g = s.g
        cn, sn_ = math.cos(U.CANT_TR), math.sin(U.CANT_TR)
        t = (0.0, cn, -sn_)          # thrust axis: right and up
        vb = s.tr_hub_velocity()
        v0 = p('aero/rotor/vi-fps')
        Vax = g.fn('aero/rotor/tr/Vaxial-fps', S(M(v(t[1]), vb[1]), M(v(t[2]), vb[2]), M(v0, p('rotor/turb/tr'))), 'along the thrust axis')
        V2 = S(SQR(vb[0]), SQR(vb[1]), SQR(vb[2]))
        Vip = g.fn('aero/rotor/tr/Vinplane-fps', SQ(MAX(v(1e-6), D(V2, SQR(Vax)))))
        Wr = Q(p('rotor/omega-rad_sec'), v(U.OMEGA))          # tail rotor speed follows the main rotor
        ORv = g.fn('aero/rotor/tr/tipspeed-fps', M(v(U.OMEGA_TR * U.R_TR), Wr))
        ORs = MAX(ORv, v(1))
        mu = g.fn('aero/rotor/tr/mu', Q(Vip, ORs))
        muz = g.fn('aero/rotor/tr/muz', Q(NEG(Vax), ORs), 'flow through the disc towards the thrust side')
        dw = p('rotor/tr/dw')
        lam = g.fn('aero/rotor/tr/lambda', D(muz, dw))
        vkt = Q(p('velocities/vt-fps'), v(1.68781))
        if U.TR_BLOCKAGE == 'fin':
            cK_, sK_ = math.cos(U.CANT_TR), math.sin(U.CANT_TR)
            s.tr_wake_on_fin(Vax, NEG(vb[0]), S(M(v(sK_), vb[1]), M(v(cK_), vb[2])), M(dw, ORv))
            kb = g.fn('aero/rotor/tr/k-blockage', v(1), 'none on the thrust: the fin\'s own force in the wake')
        else:
            kb = g.fn('aero/rotor/tr/k-blockage', IFTHEN(LT(vkt, v(U.VBLK_TR)), S(v(U.KBLK_TR), M(v((1 - U.KBLK_TR) / U.VBLK_TR ** 2), SQR(vkt))), v(1)),
                      'fin blockage [GH 5.4-8]')
        A = math.pi * U.R_TR ** 2
        rho = p('atmosphere/rho-slugs_ft3')
        ct = g.fn('aero/rotor/tr/ct', M(v(2 * (U.TIP_LOSS_TR ** 2 if getattr(U, 'TIP_LOSS_INFLOW', False) else 1.0)), dw, SQ(S(SQR(mu), SQR(lam)))))
        T = g.fn('aero/rotor/tr/thrust-lbs', M(v(A), rho, SQR(ORv), ct, kb))
        cq = S(NEG(M(lam, ct)), M(v(U.SIGMA_TR * U.CD0_TR / 8), S(v(1), M(v(4.65), SQR(mu)))))
        Qtr = g.fn('aero/rotor/tr/torque-ftlb', M(v(A * U.R_TR), rho, SQR(ORv), cq))
        g.fn('aero/rotor/tr/power-hp', Q(M(Qtr, p('rotor/omega-rad_sec'), v(U.OMEGA_TR / U.OMEGA)), v(550)))
        ax, ay, az = s.arm_rp(U.FS_TR, U.BL_TR, U.WL_TR)
        Fy, Fz = M(T, v(t[1])), M(T, v(t[2]))
        Mo = (D(M(ay, Fz), M(az, Fy)), NEG(M(ax, Fz)), M(ax, Fy))
        s.add((None, Fy, Fz), Mo)

    def downwash_correction(s):
        """GenHel's main rotor downwash correction terms [GH 5.10-19]: Sikorsky's empirical forces, from rotor-on
        wind tunnel and flight test data, for the main rotor wake swirl acting on the empennage. Added at the CG as
        a side force, a rolling moment and a pitching moment, with the free-stream dynamic pressure and the fuselage
        sideslip angle (deg, held beyond +-30)."""
        g = s.g
        if not U.DOWNWASH_CORRECTION:
            return
        q = p('aero/qbar-psf')
        b = g.fn('aero/add/beta-deg', MAX(v(-30), MIN(v(30), NEG(p('aero/fus/psi-deg')))))
        neg = LT(b, v(0))
        Fy = g.fn('aero/add/Y-lbs', M(v(U.ADD_Y_FT2), q))
        L = g.fn('aero/add/L-ftlb', NEG(M(S(v(U.ADD_L_FT3), M(IFTHEN(neg, v(U.ADD_L_BETA[0]), v(U.ADD_L_BETA[1])), b)), q)))
        Mm = g.fn('aero/add/M-ftlb', M(IFTHEN(neg, MAX(v(U.ADD_M_LIM[0]), M(v(U.ADD_M_BETA[0]), b)),
                                                MIN(v(U.ADD_M_LIM[1]), M(v(U.ADD_M_BETA[1]), b))), q))
        # the side force acts at the CG: cancel JSBSim's transfer from the aerodynamic reference point
        rx = Q(D(p('metrics/aero-rp-x-in'), p('inertia/cg-x-in')), v(12))
        rz = Q(D(p('metrics/aero-rp-z-in'), p('inertia/cg-z-in')), v(12))
        s.add((None, Fy, None), (S(L, NEG(M(rz, Fy))), Mm, M(rx, Fy)))

    # ------------------------------------------------------------ outputs for FlightGear (model, instruments, sound)
    def outputs(s):
        o = FCS('UH-60A outputs')
        o.channel('rotor')
        RPM = 60 / (2 * math.pi)
        W = p('rotor/omega-rad_sec')
        o.fn('out/rotor-rpm', M(v(RPM), W), out='/rotors/main/rpm')
        o.fn('out/rotor-rpm-pct', M(v(100 / U.OMEGA), W), out=['/rotors/main/rpm-pct', '/rotors/main/rpm_pourcent'])
        o.fn('out/tail-rpm', M(v(RPM * U.OMEGA_TR / U.OMEGA), W), out='/rotors/tail/rpm')
        R2D = 180 / math.pi
        o.fn('out/cone-deg', M(v(R2D / U.NB), S(*[p('rotor/b%d/beta-rad' % k) for k in range(U.NB)])), out='/rotors/main/cone-deg')
        # the model bends each blade at its hinge (cone1) and along its span (cone2-4), positive up (Models/Externals/
        # MainRotor/blade.xml: factor -1 about +y, the blade along +x): flapping, plus the static droop of the blades
        # that the centrifugal force takes out
        nr2 = SQR(MIN(v(1), Q(W, v(U.OMEGA))))
        o.fn('out/cone1', p('out/cone-deg'), out='/rotors/main/cone1-deg')
        for i, d in ((2, 0.6), (3, 0.8), (4, 1.0)):
            o.fn('out/cone%d' % i, M(v(-d), D(v(1), nr2)), out='/rotors/main/cone%d-deg' % i)
        o.lag('out/tpp-lat', 'aero/rotor/b1-deg', 0.15)
        o.lag('out/tpp-lon', 'aero/rotor/a1-deg', 0.15)
        o.fn('out/roll-deg', p('out/tpp-lat'), out='/rotors/main/roll-deg')
        o.fn('out/yaw-deg', NEG(p('out/tpp-lon')), out='/rotors/main/yaw-deg')
        o.fn('out/mr-incidence', D(p('ctl/theta0-deg'), v(7.5)), out='/rotors/main/blade[0]/incidence-deg')
        # each blade for the rotor's model (fdm/rotorvis.py): its hinge azimuth (from the tail, anticlockwise from
        # above), flap (up +), lag (lead +) and pitch (collective and cyclic); azimuth and flap are FlightGear's
        # multiplayer rotor properties
        wrap = lambda e: IFTHEN(GT(e, v(360)), D(e, v(360)), e)     # (JSBSim's <mod> is an integer modulus)
        for k in range(U.NB):
            o.fn('out/b%d-position' % k, wrap(S(M(v(R2D), p('rotor/psi-rad')), v(90.0 * k))),
                 out='/rotors/main/blade[%d]/position-deg' % k)
            o.fn('out/b%d-flap' % k, M(v(R2D), p('rotor/b%d/beta-rad' % k)), out='/rotors/main/blade[%d]/flap-deg' % k)
            o.fn('out/b%d-lag' % k, M(v(R2D), p('rotor/b%d/delta-rad' % k)), out='/rotors/main/blade[%d]/lag-deg' % k)
            o.fn('out/b%d-pitch' % k, p('aero/rotor/b%d/theta-deg' % k), out='/rotors/main/blade[%d]/pitch-deg' % k)
        for k in range(U.NB_TR):                                       # the tail rotor's blades: azimuth and flap
            o.fn('out/tr-b%d-position' % k, wrap(S(M(v(R2D), p('rotor/tr/psi-rad')), v(90.0 * k))),
                 out='/rotors/tail/blade[%d]/position-deg' % k)
            if U.TR_MODEL == 'blades':
                o.fn('out/tr-b%d-flap' % k, M(v(R2D), p('rotor/tr/b%d/beta-rad' % k)), out='/rotors/tail/blade[%d]/flap-deg' % k)
        o.fn('out/tr-incidence', D(p('ctl/thetaTR-deg'), v(10)), out=['/rotors/tail/blade[0]/incidence-deg', '/rotors/tail/blade/incidence-deg'])
        Qmr = p('aero/rotor/torque-ftlb')
        o.fn('out/total-torque', Qmr, out='/rotors/gear/total-torque')
        o.fn('out/torque-sound', M(S(v(0.75), M(v(0.25), MIN(v(1), MAX(v(0), Q(Qmr, v(40000)))))), MIN(v(1), MAX(v(0), D(M(v(0.02), p('out/rotor-rpm')), v(1))))),
             out='/rotors/gear[0]/torque-sound-filtered')
        o.channel('engines')

        def warm(name, target, tau, out):
            """a temperature approaching target with time constant tau, starting there at an initial condition"""
            o.declare(name + '-state', 0.0)
            st = p(name + '-state')
            o.fn(name + '-next', IFTHEN(GT(p('ctl/init-fast'), v(0.5)), target, S(st, M(Q(p('simulation/dt'), v(tau)), D(target, st)))),
                 out=name + '-state')
            o.fn(name, st, out=out)
        for n in range(2):
            g, e = 'eng[%d]' % n, '/engines/engine[%d]' % n
            o.fn('out/%s/n1' % g, p(g + '/ng-pct'), out=e + '/n1-pct')
            o.fn('out/%s/n2' % g, p(g + '/np-pct'), out=e + '/n2-pct')
            o.fn('out/%s/tgt' % g, p(g + '/tgt-degc'), out=[e + '/tot-degc', e + '/egt-degC'])
            o.fn('out/%s/itt' % g, Q(p(g + '/tgt-degc'), v(950)), out=e + '/itt-norm')
            o.fn('out/%s/rpm' % g, M(v(447), p(g + '/ng-pct')), out=e + '/rpm')
            o.fn('out/%s/ff' % g, Q(p(g + '/fuel-flow-pph'), v(6.7)), out=e + '/fuel-flow-gph')
            o.fn('out/%s/ffp' % g, p(g + '/fuel-flow-pph'), out=e + '/fuel-flow-pph')
            o.fn('out/%s/trq' % g, p(g + '/torque-pct'), out=e + '/torque-pct')
            o.fn('out/%s/run' % g, p(g + '/lit'), out=[e + '/running', e + '/on'])
            # oil (cosmetic): pressure with Ng, temperature warming up over a few minutes
            o.fn('out/%s/oilp' % g, M(v(0.62), p(g + '/ng-pct')), out=e + '/oil-pressure-psi')
            warm('out/%s/oilt' % g, S(M(D(p('atmosphere/T-R'), v(491.67)), v(5 / 9)), M(v(0.9), p(g + '/ng-pct'))), 120.0,
                 [e + '/oil-temperature-degc', e + '/oil-temperature-degC'])
        o.fn('out/torque-pct', M(v(0.5), S(p('eng[0]/torque-pct'), p('eng[1]/torque-pct'))), out='/sim/model/uh60/torque-pct')
        # main transmission oil (cosmetic, no UH-60 data): the pump turns with the rotor; the oil warms with the
        # power through the gearbox over a few minutes
        o.channel('transmission')
        wn = MIN(v(1.05), MAX(v(0), Q(W, v(U.OMEGA))))
        o.fn('out/xmsn-oilp', M(v(55), wn, wn), out='/sim/model/uh60/xmsn-oil-press-psi')
        warm('out/xmsn-oilt', S(M(D(p('atmosphere/T-R'), v(491.67)), v(5 / 9)), M(v(35), wn), M(v(0.35), p('out/torque-pct'))), 300.0,
             '/sim/model/uh60/xmsn-oil-temp-degc')
        # radar altimeter: height of the main wheels (the antennas are on the belly), reading 0 on the ground
        wl_wheels = U.yasim_to_station(4.136, 1.344, -2.317)[2]
        o.fn('out/radar-alt', MAX(v(0), D(p('position/h-agl-ft'), Q(D(p('inertia/cg-z-in'), v(wl_wheels)), v(12)))), out='/sim/model/uh60/radar-alt-ft')
        ng = MAX(p('eng[0]/ng-pct'), p('eng[1]/ng-pct'))
        lit = MAX(p('eng[0]/lit'), p('eng[1]/lit'))
        o.fn('out/state', IFTHEN(LT(ng, v(3)), v(0), IFTHEN(M(lit, GT(ng, v(60))), v(5), v(1))), out='/sim/model/uh60/state')
        # cockpit vibration (RMS g) for the view shake and the rattle: the 6-30 Hz part of the acceleration at the
        # pilot seat, which the blades' 4-per-rev hub loads and stall buffet put there (a broad band, as the rotor
        # speed droops in stall), plus the transition vibration around translational lift that the inflow model
        # cannot make (blade-vortex interaction; the level is an estimate)
        o.channel('vibration')
        hp = []
        for ax in 'xyz':
            o.washout('vib/%s-hp1' % ax, 'accelerations/a-pilot-%s-ft_sec2' % ax, 1 / (2 * math.pi * 6))
            hp.append(o.washout('vib/%s-hp' % ax, 'vib/%s-hp1' % ax, 1 / (2 * math.pi * 6)))
        o.fn('vib/ms', S(*[SQR(h) for h in hp]))
        o.lag('vib/ms-avg', 'vib/ms', 0.25)
        o.fn('vib/rotor-g', Q(SQ(MAX(v(0), p('vib/ms-avg'))), v(G0)))
        thrust = MAX(v(0), MIN(v(1.5), Q(p('aero/rotor/thrust-lbs'), v(16000))))
        o.fn('vib/bvi-g', M(v(0.05), p('rotor/turb/etl'), thrust, GT(W, v(0.5 * U.OMEGA))))
        # (faded in over the first seconds: the filters and the aircraft settling on its gear start from rest)
        fade = MAX(v(0), MIN(v(1), Q(D(p('simulation/sim-time-sec'), v(1.0)), v(3.0))))
        o.fn('vib/level-g', M(fade, D(v(1), p('ctl/init')), SQ(S(SQR(p('vib/rotor-g')), SQR(p('vib/bvi-g'))))),
             out='/sim/model/uh60/vibration-g')
        # master warning panel: the LOW ROTOR RPM light flashes 3-5 times a second [TM 2.81]
        o.channel('warning lights')
        o.fn('out/warning-flash', GT(SIN(M(v(2 * math.pi * 4), p('simulation/sim-time-sec'))), v(0)), out='/sim/model/uh60/warning-flash')
        return o

    # ------------------------------------------------------------ fuselage [GH 5.2]
    def wash_inputs(s):
        """wake skew (deg) and longitudinal flapping a1 (deg, + = disc aft) for the interference maps"""
        g = s.g
        g.fn('aero/rotor/chi-deg', M(v(180 / math.pi), p('aero/rotor/chi-rad')))
        g.fn('aero/rotor/a1-deg', M(v(-0.5 * 180 / math.pi), S(*[M(p('rotor/b%d/beta-rad' % k), p('aero/rotor/b%d/cpsi' % k)) for k in range(U.NB)])))
        g.fn('aero/rotor/b1-deg', M(v(-0.5 * 180 / math.pi), S(*[M(p('rotor/b%d/beta-rad' % k), p('aero/rotor/b%d/spsi' % k)) for k in range(U.NB)])))

    def fuselage(s):
        """Wind tunnel tables (tail off) at the mounting point, with the main rotor wash on the fuselage. The tables
        use the tunnel's angles; flying backwards, the forces are mirrored front to back so drag still opposes the
        motion, and the front-back asymmetric terms (lift, pitching and rolling moments) fade through zero."""
        g = s.g
        FW = FUS
        T = lambda name, var, xs, ys: g.fn(name, T1(var, xs, ys))
        wkx = T2('aero/rotor/chi-deg', 'aero/rotor/a1-deg', FW.WASH_CHI, FW.WASH_A1, [list(r) for r in zip(*FW.EKX)])
        wkz = T2('aero/rotor/chi-deg', 'aero/rotor/a1-deg', FW.WASH_CHI, FW.WASH_A1, [list(r) for r in zip(*FW.EKZ)])
        ekx = g.fn('aero/fus/ekx', wkx); ekz = g.fn('aero/fus/ekz', wkz)
        v0 = p('aero/rotor/vi-fps')
        ux, uy = p('aero/rotor/uhx'), p('aero/rotor/uhy')
        Vx = g.fn('aero/fus/Vx-fps', S(p('velocities/u-aero-fps'), M(ekx, v0, ux)))
        Vy = g.fn('aero/fus/Vy-fps', S(p('velocities/v-aero-fps'), M(ekx, v0, uy)))
        Vz = g.fn('aero/fus/Vz-fps', D(p('velocities/w-aero-fps'), M(ekz, v0)))
        Vxz = g.fn('aero/fus/Vxz-fps', MAX(SQ(S(SQR(Vx), SQR(Vz))), v(0.01)))
        V = g.fn('aero/fus/V-fps', MAX(SQ(S(SQR(Vx), SQR(Vy), SQR(Vz))), v(0.01)))
        R2D = 180 / math.pi
        g.fn('aero/fus/alpha-deg', M(v(R2D), ATAN2(Vz, ABS(Vx))), 'tunnel angle of attack (front-back mirrored)')
        g.fn('aero/fus/psi-deg', M(v(-R2D), ATAN2(Vy, Vxz)), 'tunnel yaw angle = -sideslip')
        kf = g.fn('aero/fus/kfwd', MAX(v(-1), MIN(v(1), Q(Vx, v(3.0)))))
        q = g.fn('aero/fus/qbar-psf', M(v(0.5), p('atmosphere/rho-slugs_ft3'), SQR(V)))
        A, P = 'aero/fus/alpha-deg', 'aero/fus/psi-deg'
        ax, dq = FW.merge(FW.LO, FW.DQ_LO, FW.HI, FW.DQ_HI)
        _, lq = FW.merge(FW.LO, FW.LQ_LO, FW.HI, FW.LQ_HI)
        _, mq = FW.merge(FW.LO, FW.MQ_LO, FW.HI, FW.MQ_HI)
        _, nq = FW.merge(FW.LO, FW.NQ_LO, FW.HI, FW.NQ_HI)
        bx, ddq = FW.merge(FW.ABS_LO, FW.DDQ_LO, FW.ABS_HI, FW.DDQ_HI)
        _, yq = FW.merge(FW.ABS_LO, FW.YQ_LO, FW.ABS_HI, FW.YQ_HI)
        _, rq = FW.merge(FW.ABS_LO, FW.RQ_LO, FW.ABS_HI, FW.RQ_HI)
        Dq = g.fn('aero/fus/D-ft2', S(T1(A, ax, dq), T1(P, *_even(bx, ddq))))
        Yq = g.fn('aero/fus/Y-ft2', T1(P, *_odd(bx, yq)))
        Lq = g.fn('aero/fus/L-ft2', M(kf, S(T1(A, ax, lq), T1(P, FW.LO, FW.DLQ_LO))))
        Rq = g.fn('aero/fus/R-ft3', M(kf, T1(P, *_odd(bx, rq))))
        # GenHel signs the pitching increment with the yaw angle; a symmetric body's is even in sideslip
        Mq = g.fn('aero/fus/M-ft3', M(kf, S(T1(A, ax, mq), T1(P, *_even(FW.ABS_LO, FW.DMQ_LO)))))
        Nq = g.fn('aero/fus/N-ft3', T1(P, ax, nq))
        # wind to body [GH 5.2-9] with the true flow angles: c1 = V/|V|, c2 = (ca sb, -cb, sa sb), c3 = (-sa, 0, ca)
        ca, sa = Q(Vx, Vxz), Q(Vz, Vxz)
        cb, sb = Q(Vxz, V), Q(Vy, V)
        c1 = (Q(Vx, V), Q(Vy, V), Q(Vz, V))
        c2 = (M(ca, sb), NEG(cb), M(sa, sb))
        c3 = (NEG(sa), v(0), ca)
        F = [g.fn('aero/fus/F%s-lbs' % a, M(q, NEG(S(M(Dq, c1[i]), M(Yq, c2[i]), M(Lq, c3[i]))))) for i, a in enumerate('XYZ')]
        Mw = [g.fn('aero/fus/M%s-wind' % a, M(q, S(M(Rq, c1[i]), NEG(M(Mq, c2[i])), M(Nq, c3[i])))) for i, a in enumerate('LMN')]
        rx, ry, rz = s.arm_rp(FW.FSWF, FW.BLWF, FW.WLWF)
        Mo = (S(Mw[0], D(M(ry, F[2]), M(rz, F[1]))), S(Mw[1], D(M(rz, F[0]), M(rx, F[2]))), S(Mw[2], D(M(rx, F[1]), M(ry, F[0]))))
        Mo = [g.fn('aero/fus/%s-ftlb' % a, e) for a, e in zip('LMN', Mo)]
        s.add(F, Mo)

    # ------------------------------------------------------------ stabilator and fin [GH 5.3]
    def empennage(s):
        """Each surface is a 2-D strip in its own plane (spanwise flow ignored), with the GenHel lift and drag
        tables extended over +-180 deg (reverse flow mirrors them), the main rotor wash, the fuselage downwash and
        sidewash, and the fuselage's dynamic pressure loss."""
        g = s.g
        E = EMP
        R2D = 180 / math.pi
        v0 = p('aero/rotor/vi-fps')
        ux, uy = p('aero/rotor/uhx'), p('aero/rotor/uhy')
        ekx = g.fn('aero/emp/ekx', T2('aero/rotor/chi-deg', 'aero/rotor/a1-deg', E.WASH_CHI, E.WASH_A1, [list(r) for r in zip(*E.EKX)]))
        ekz = g.fn('aero/emp/ekz', T2('aero/rotor/chi-deg', 'aero/rotor/a1-deg', E.WASH_CHI, E.WASH_A1, [list(r) for r in zip(*E.EKZ)]))
        kf = p('aero/fus/kfwd')
        ufwd = MAX(v(0), p('velocities/u-aero-fps'))
        kqh = g.fn('aero/emp/kqh', D(v(1), M(MAX(v(0), kf), D(v(1), SQ(T1('aero/fus/alpha-deg', E.LO, E.QH_LO))))))
        kqv = g.fn('aero/emp/kqv', D(v(1), M(MAX(v(0), kf), D(v(1), SQ(T1('aero/fus/psi-deg', *_even(E.ABS_LO, E.QV_LO)))))))
        eps = g.fn('aero/emp/eps-deg', T1('aero/fus/alpha-deg', *FUS.merge(E.LO, E.EPS_LO, E.HI, E.EPS_HI)))
        sx, sy = FUS.merge(E.ABS_LO, E.SIG_LO, E.SIG_HI_X, E.SIG_HI)
        sig = g.fn('aero/emp/sigma-deg', T1('aero/fus/psi-deg', *_odd(sx, sy)))
        Pa, Qa, Ra = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')

        def local(tag, fs, bl, wl, kq, extra, wash=None):
            rx, ry, rz = s.arm_cg(fs, bl, wl)
            wx, wy, wz = wash or (M(ekx, v0, ux), M(ekx, v0, uy), NEG(M(ekz, v0)))
            vb = (S(M(kq, p('velocities/u-aero-fps')), D(M(Qa, rz), M(Ra, ry)), wx, extra[0]),
                  S(M(kq, p('velocities/v-aero-fps')), D(M(Ra, rx), M(Pa, rz)), wy, extra[1]),
                  S(M(kq, p('velocities/w-aero-fps')), D(M(Pa, ry), M(Qa, rx)), wz, extra[2]))
            return [g.fn('aero/emp/%s/V%s-fps' % (tag, a), e) for a, e in zip('xyz', vb)]

        # stabilator: x-z plane, incidence = stabilator angle (TED +)
        stab_wash = None
        if getattr(U, 'STAB_WASH', 'genhel') == 'vortex':
            # GenHel's downwash factor holds for the wake blown aft (where it was calibrated, and where the ring-wake
            # model [mrwake.py] agrees with it from 40 deg of skew); the ring wake adds how it changes as the wake is
            # blown sideways or forward, off the tail
            mw = mrwake.load('stab')
            s.psi_flow()
            kz = g.fn('aero/emp/h/k-mr-z', T3('aero/rotor/chi-deg', 'aero/rotor/psi-flow-deg', 'aero/rotor/a1-deg', mw['chi'], mw['psi'], mw['a1'],
                                               [[[c[2] for c in row] for row in tab] for tab in mw['k']]))
            kz0 = g.fn('aero/emp/h/k-mr-z0', T2('aero/rotor/chi-deg', 'aero/rotor/a1-deg', mw['chi'], mw['a1'],
                                                 [[mw['k'][ia][ic][0][2] for ia in range(len(mw['a1']))] for ic in range(len(mw['chi']))]))
            stab_wash = (M(ekx, v0, ux), M(ekx, v0, uy), NEG(M(S(ekz, D(kz, kz0)), v0)))
        Vx, Vy, Vz = local('h', E.FS_H, E.BL_H, E.WL_H, kqh, (v(0), v(0), NEG(M(ufwd, eps, kqh, v(1 / R2D)))), stab_wash)
        Vn = g.fn('aero/emp/h/V-fps', MAX(SQ(S(SQR(Vx), SQR(Vz))), v(0.01)))
        ah = g.fn('aero/emp/h/alpha-raw', S(M(v(R2D), ATAN2(Vz, Vx)), p('stab/pos-deg')))
        ah = g.fn('aero/emp/h/alpha-deg', IFTHEN(GT(ah, v(180)), D(ah, v(360)), IFTHEN(LT(ah, v(-180)), S(ah, v(360)), ah)))
        xs, ys = _mirror180(*_odd(*FUS.merge(E.ABS_LO, E.CLH_LO, E.ABS_HI, E.CLH_HI)), negate=True)
        cl = g.fn('aero/emp/h/cl', T1('aero/emp/h/alpha-deg', xs, ys))
        xs, ys = _mirror180(*_even(*FUS.merge(E.ABS_LO, E.CDH_LO, E.ABS_HI, E.CDH_HI)), negate=False)
        cd = g.fn('aero/emp/h/cd', T1('aero/emp/h/alpha-deg', xs, ys))
        qs = g.fn('aero/emp/h/qS', M(v(0.5 * E.S_H), p('atmosphere/rho-slugs_ft3'), SQR(Vn)))
        # lift normal to the flow in the x-z plane, (sa, 0, -ca); drag along it
        Fx = g.fn('aero/emp/h/Fx-lbs', M(qs, Q(D(M(cl, Vz), M(cd, Vx)), Vn)))
        Fz = g.fn('aero/emp/h/Fz-lbs', M(qs, Q(NEG(S(M(cl, Vx), M(cd, Vz))), Vn)))
        rx, ry, rz = s.arm_rp(E.FS_H, E.BL_H, E.WL_H)
        s.add((Fx, None, Fz), (M(ry, Fz), D(M(rz, Fx), M(rx, Fz)), NEG(M(ry, Fx))))

        # fin: x-y plane, incidence 0. With the blade-element tail rotor's wake on it (TR_BLOCKAGE 'fin'), in spanwise
        # strips, each with its own sidewash from the tail rotor [trwake.py]: the hover's blockage and its remainder
        # in forward flight come from the fin's own lift and drag, not from a factor on the thrust
        strips = [(E.FS_V, E.WL_V, E.S_V, None, None)]
        if U.TR_BLOCKAGE == 'fin':
            import trwake
            tw = trwake.load()
            strips = [(st['fs'], st['wl'], st['area'], st['kx'], st['ky']) for st in tw['strips']]
            # the ring wake gives the fin's drag in the hover as 13.6 % of the thrust (Sikorsky measured 20.4 %);
            # TR_FIN_K (experiments, default 1) scales its velocity
            vtr = M(p('aero/rotor/tr/vi-fps'), v(getattr(U, 'TR_FIN_K', 1.0)))
        fin_wash = None
        if getattr(U, 'FIN_WASH', 'genhel') == 'vortex':
            # the main rotor's wake on the fin from the ring-wake model [mrwake.py] (mean over its planform): it follows
            # where the wake is blown, where GenHel's factors put the wash along the flight path at any azimuth
            mw = mrwake.load('fin')
            s.psi_flow()
            kf = [g.fn('aero/emp/v/k-mr-%s' % a, T3('aero/rotor/chi-deg', 'aero/rotor/psi-flow-deg', 'aero/rotor/a1-deg', mw['chi'], mw['psi'],
                                                     mw['a1'], [[[c[i] for c in row] for row in tab] for tab in mw['k']]),
                       'main rotor induced velocity over the fin, over its mean inflow') for i, a in enumerate('xy')]
            fin_wash = (NEG(M(kf[0], v0)), NEG(M(kf[1], v0)), v(0))
        xs_l, ys_l = _mirror180(*FUS.merge(E.LO, E.CLV_LO, E.HI, E.CLV_HI), negate=True)
        xs_d, ys_d = _mirror180(*FUS.merge(E.LO, E.CDV_LO, E.HI, E.CDV_HI), negate=False)
        FX, FY = [], []
        for i, (fs, wl, area, kx, ky) in enumerate(strips):
            tg = 'v' if len(strips) == 1 else 'v%d' % i
            extra = (v(0), NEG(M(ufwd, sig, kqv, v(1 / R2D))), v(0))
            if kx:
                ex = g.fn('aero/emp/%s/k-tr-x' % tg, T2('aero/rotor/tr/chi-deg', 'aero/rotor/tr/psi-wake-deg', tw['chi'], tw['psi'], kx))
                ey = g.fn('aero/emp/%s/k-tr-y' % tg, T2('aero/rotor/tr/chi-deg', 'aero/rotor/tr/psi-wake-deg', tw['chi'], tw['psi'], ky),
                          'the tail rotor\'s induced air velocity over the strip, body y, over its inflow')
                extra = (NEG(M(ex, vtr)), S(extra[1], NEG(M(ey, vtr))), v(0))
                jet2 = S(SQR(M(ex, vtr)), SQR(M(ey, vtr)))
            Vx, Vy, Vz = local(tg, fs, E.BL_V, wl, kqv, extra, fin_wash)
            Vn = g.fn('aero/emp/%s/V-fps' % tg, MAX(SQ(S(SQR(Vx), SQR(Vy))), v(0.01)))
            g.fn('aero/emp/%s/beta-deg' % tg, M(v(R2D), ATAN2(Vy, Vx)))
            cl = g.fn('aero/emp/%s/cl' % tg, T1('aero/emp/%s/beta-deg' % tg, xs_l, ys_l))
            cd = g.fn('aero/emp/%s/cd' % tg, T1('aero/emp/%s/beta-deg' % tg, xs_d, ys_d))
            cdf = getattr(U, 'TR_FIN_CD', 1.0)
            if kx and cdf != 1.0:
                # the fin's drag where the tail rotor's jet makes its flow: scaled to Sikorsky's hover blockage, in
                # proportion to the jet's share of the strip's dynamic pressure (the jet's velocity stays the ring wake's)
                cd = g.fn('aero/emp/%s/cd-jet' % tg, M(cd, S(v(1), M(v(cdf - 1), MIN(v(1), Q(jet2, SQR(Vn)))))))
            qs = g.fn('aero/emp/%s/qS' % tg, M(v(0.5 * area), p('atmosphere/rho-slugs_ft3'), SQR(Vn)))
            # lift (sb, -cb, 0): positive with flow from the right pushes the tail left
            Fx = g.fn('aero/emp/%s/Fx-lbs' % tg, M(qs, Q(D(M(cl, Vy), M(cd, Vx)), Vn)))
            Fy = g.fn('aero/emp/%s/Fy-lbs' % tg, M(qs, Q(NEG(S(M(cl, Vx), M(cd, Vy))), Vn)))
            rx, ry, rz = s.arm_rp(fs, E.BL_V, wl)
            s.add((Fx, Fy, None), (NEG(M(rz, Fy)), M(rz, Fx), D(M(rx, Fy), M(ry, Fx))))
            FX.append(Fx); FY.append(Fy)
        if len(strips) > 1:
            g.fn('aero/emp/v/Fx-lbs', S(*FX)); g.fn('aero/emp/v/Fy-lbs', S(*FY))

    def axes(s):
        def ax(name, terms):
            return '<axis name="%s">%s</axis>' % (name, ''.join('<function>%s</function>' % t for t in terms))
        return '\n'.join((ax('X', s.X), ax('Y', s.Y), ax('Z', s.Z), ax('ROLL', s.L), ax('PITCH', s.Mm), ax('YAW', s.N)))


# ---------------------------------------------------------------- mass, fuel and gear
# Loads (lb, FS, BL, WL); FlightGear's weight dialog edits the point masses, the fuel dialog the tanks.
POINT_MASSES = [('Pilot (right seat)', 200.0, 300.0, 232.0, 23.6, 225.0),
                ('Copilot (left seat)', 200.0, 300.0, 232.0, -23.6, 225.0),
                ('Crew chief and gunner', 400.0, 600.0, 300.0, 0.0, 220.0),
                ('Cabin load', 1200.0, 2640.0, 340.0, 0.0, 220.0)]
TANKS = [(1206.0, 395.0, -25.0, 210.0), (1206.0, 395.0, 25.0, 210.0)]   # 2 x 180 US gal, JP-8 (6.7 lb/gal)
# Fuselage contact points from the 3-D model's mesh (YASim frame: m, x forward, y left, z up)
STRUCTURE = [('NOSE', (7.65, 0.0, -1.2)),
             ('CHIN_LEFT', (6.0, 1.0, -1.75)), ('CHIN_RIGHT', (6.0, -1.0, -1.75)),
             ('BELLY_LEFT', (0.5, 1.1, -1.74)), ('BELLY_RIGHT', (0.5, -1.1, -1.74)),
             ('CABIN_ROOF_LEFT', (6.0, 1.0, 0.0)), ('CABIN_ROOF_RIGHT', (6.0, -1.0, 0.0)),
             ('AFT_ROOF_LEFT', (0.0, 1.2, 0.42)), ('AFT_ROOF_RIGHT', (0.0, -1.2, 0.42)),
             ('NACELLE_LEFT', (1.0, 1.4, 0.9)), ('NACELLE_RIGHT', (1.0, -1.4, 0.9)),
             ('ROTOR_HEAD', (3.04, 0.0, 1.4)),
             ('TAIL_BOOM', (-5.0, 0.0, -1.0)), ('TAIL_LOW', (-6.5, 0.0, -1.1)),
             ('STAB_LEFT', (-6.4, 2.34, -0.58)), ('STAB_RIGHT', (-6.4, -2.34, -0.58)),
             ('FIN_TOP', (-7.3, 0.0, 1.65))]


def mass_config(cg=None, weight=None):
    """Empty weight, its CG and inertias such that the default loading is GenHel's design point (16,000 lb class,
    FS 358 / WL 251, Table 4.2 inertias). For a test configuration (cg, weight) everything sits at that CG."""
    if cg:
        pms = [(n, 0.0, mx, cg[0], cg[1], cg[2]) for n, w, mx, x, y, z in POINT_MASSES]
        tanks = [(w, cg[0], cg[1], cg[2]) for w, x, y, z in TANKS]
        return dict(empty=weight - sum(t[0] for t in tanks), cg=cg, I=(U.IXX, U.IYY, U.IZZ, U.IXZ), pms=pms, tanks=tanks, full=True)
    loads = [(w, x, y, z) for n, w, mx, x, y, z in POINT_MASSES] + list(TANKS)
    total = U.GW_DESIGN
    we = total - sum(l[0] for l in loads)
    ex = (total * U.FS_CG - sum(w * x for w, x, y, z in loads)) / we
    ez = (total * U.WL_CG - sum(w * z for w, x, y, z in loads)) / we
    ey = -sum(w * y for w, x, y, z in loads) / we
    # empty inertias about the empty CG: design totals less the loads' and the empty mass' parallel-axis terms
    def para(w, x, y, z):
        m = w / G0; dx, dy, dz = (x - U.FS_CG) / 12, (y - 0) / 12, (z - U.WL_CG) / 12
        return m * (dy * dy + dz * dz), m * (dx * dx + dz * dz), m * (dx * dx + dy * dy), -m * dx * dz   # JSBSim's Ixz sign
    terms = [para(*l) for l in loads] + [para(we, ex, ey, ez)]
    I = tuple(Itot - sum(t[i] for t in terms) for i, Itot in enumerate((U.IXX, U.IYY, U.IZZ, U.IXZ)))
    return dict(empty=we, cg=(ex, ey, ez), I=I, pms=POINT_MASSES, tanks=TANKS, full=False)


def fdm_file(aero_functions, axes, cg=None, weight=None):
    m = U
    mc = mass_config(cg, weight)
    vrp = (m.FS_ORIGIN, 0.0, m.WL_ORIGIN)
    gear = []
    # positions from the FlightGear model (YASim frame), converted to stations; oleo + tyre about 4 in static
    for name, xyz, k, c, steer in (('LEFT_MAIN', (4.136, 1.344, -2.317), 20000, 2500, 0),
                                   ('RIGHT_MAIN', (4.136, -1.344, -2.317), 20000, 2500, 0),
                                   ('TAIL_WHEEL', (-4.664, 0.0, -2.217), 8000, 1000, 360)):
        fs, bl, wl = m.yasim_to_station(*xyz)
        gear.append('''    <contact type="BOGEY" name="%s">
      <location unit="IN"><x>%.2f</x><y>%.2f</y><z>%.2f</z></location>
      <static_friction>0.8</static_friction><dynamic_friction>0.5</dynamic_friction><rolling_friction>0.02</rolling_friction>
      <spring_coeff unit="LBS/FT">%d</spring_coeff><damping_coeff unit="LBS/FT/SEC">%d</damping_coeff>
      <max_steer unit="DEG">%d</max_steer><brake_group>%s</brake_group><retractable>1</retractable>
    </contact>''' % (name, fs, bl, wl, k, c, steer, 'NONE' if steer else ('LEFT' if 'LEFT' in name else 'RIGHT')))
    # structure: the hull of the 3-D model's fuselage, so that a hard landing or a roll-over ends on the skin
    # instead of sinking through the ground on the wheels (which then compress without limit and blow up).
    # Stiffness and damping are bounded by the explicit integration (set at 120 Hz; the 480 Hz the model runs at has
    # four times the margin): lying on its side, five contacts some 6 ft from the CG act on the small roll inertia
    # (Ixx 5,460 slug ft2), and the damping term sum(c r^2)/Ixx x dt must stay well under 1 (12,000 lb/ft/s made it
    # 3 at 120 Hz and the roll diverged)
    for name, (x, y, z) in STRUCTURE:
        fs, bl, wl = m.yasim_to_station(x, y, z)
        gear.append('''    <contact type="STRUCTURE" name="%s">
      <location unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
      <static_friction>0.8</static_friction><dynamic_friction>0.6</dynamic_friction>
      <spring_coeff unit="LBS/FT">40000</spring_coeff><damping_coeff unit="LBS/FT/SEC">2000</damping_coeff>
    </contact>''' % (name, fs, bl, wl))
    # fuel is two point masses (4, 5) that the drive system keeps equal to /consumables/fuel/tank[n]/level-lbs:
    # FlightGear 2024.1's JSBSim cannot run tanks without a JSBSim engine (FGPropulsion is never bound)
    pml = [(n, w, x, y, z) for n, w, mx, x, y, z in mc['pms']] + \
          [('Fuel tank %d' % (i + 1), w, x, y, z) for i, (w, x, y, z) in enumerate(mc['tanks'])]
    pms = '\n'.join('''    <pointmass name="%s">
      <weight unit="LBS">%.1f</weight>
      <location unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
    </pointmass>''' % pm for pm in pml)
    I = mc['I']
    return '''<?xml version="1.0"?>
<!-- UH-60A Black Hawk: generated by fdm/genuh60.py -->
<fdm_config name="UH-60A" version="2.0" release="ALPHA">
  <fileheader>
    <author>generated</author>
    <description>UH-60A Black Hawk, individual-blade main rotor from the Sikorsky GenHel data (NASA CR-166309)</description>
  </fileheader>
  <metrics>
    <wingarea unit="FT2">%.1f</wingarea>
    <wingspan unit="FT">%.2f</wingspan>
    <chord unit="FT">%.2f</chord>
    <location name="AERORP" unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
    <!-- pilot's design eye: on the seat centreline BL 24 at WL 257 (GenHel fig. 6.1.3) -->
    <location name="EYEPOINT" unit="IN"><x>229.0</x><y>24.0</y><z>257.0</z></location>
    <location name="VRP" unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
  </metrics>
  <mass_balance>
    <ixx unit="SLUG*FT2">%.0f</ixx><iyy unit="SLUG*FT2">%.0f</iyy><izz unit="SLUG*FT2">%.0f</izz><ixz unit="SLUG*FT2">%.0f</ixz>
    <emptywt unit="LBS">%.1f</emptywt>
    <location name="CG" unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
%s
  </mass_balance>
  <ground_reactions>
%s
  </ground_reactions>
  <system file="uh60a-controls"/>
  <system file="uh60a-rotor"/>
  <system file="uh60a-drive"/>
  <system file="uh60a-outputs"/>
  <aerodynamics file="Systems/uh60a-aero.xml"/>
</fdm_config>
''' % ((math.pi * m.R ** 2, 2 * m.R, m.CHORD) + (m.FS_CG, 0.0, m.WL_CG) + vrp + I + (mc['empty'],) + tuple(mc['cg']) + (pms, '\n'.join(gear)))


# offline test configurations: (file name, CG stations, gross weight)
TEST_CASES = [('uh60a', None, None),
              ('uh60a-ballin', (351.0, 0.0, 231.5), 16000.0)]   # Ballin TM-88360 fig. B1 level-flight trims


def build(out, fg=False):
    mdl = Model()
    mdl.controls()
    mdl.rotor_states()
    mdl.main_rotor()
    mdl.wash_inputs()
    mdl.fuselage()
    mdl.empennage()
    mdl.tail_rotor()
    mdl.downwash_correction()
    os.makedirs(os.path.join(out, 'Systems'), exist_ok=True)
    open(os.path.join(out, 'Systems', 'uh60a-controls.xml'), 'w').write('<?xml version="1.0"?>\n' + mdl.ctl.xml())
    open(os.path.join(out, 'Systems', 'uh60a-rotor.xml'), 'w').write('<?xml version="1.0"?>\n' + mdl.rot.xml())
    open(os.path.join(out, 'Systems', 'uh60a-drive.xml'), 'w').write('<?xml version="1.0"?>\n' + engines.build().xml())
    open(os.path.join(out, 'Systems', 'uh60a-outputs.xml'), 'w').write('<?xml version="1.0"?>\n' + mdl.outputs().xml())
    open(os.path.join(out, 'Systems', 'uh60a-aero.xml'), 'w').write(
        '<?xml version="1.0"?>\n<!-- generated by fdm/genuh60.py -->\n<aerodynamics>\n%s\n%s\n</aerodynamics>\n' % (mdl.g.xml(), mdl.axes()))
    if fg:
        open(os.path.join(out, 'uh60a.xml'), 'w').write(fdm_file(mdl.g, None))
        return len(mdl.g.items)
    open(os.path.join(out, 'uh60a-jsb.xml'), 'w').write(fdm_file(mdl.g, None))
    for name, cg, w in TEST_CASES:
        open(os.path.join(out, name + '.xml'), 'w').write(fdm_file(mdl.g, None, cg, w))
    return len(mdl.g.items)


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    out = args[0] if args else 'out'
    if '--fg' in sys.argv:
        # FlightGear aircraft folder: only the production FDM (uh60a.xml) and its systems
        n = build(out, fg=True)
    else:
        n = build(out)
    print('wrote', out, '-', n, 'aerodynamic functions')
