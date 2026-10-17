"""UH-60A Black Hawk data for the JSBSim model, with sources.

  [GH]  NASA CR-166309 (Howlett 1981), Sikorsky GenHel mathematical model, vol. I
  [HB]  NASA TM-85890 (Hilbert 1984), UH-60 math model parameters
  [BO]  NASA TP-2003-212265 (Bousman 2003), SC1095 / SC1094 R8 airfoils

Frames: Sikorsky stations in inches (FS aft, BL right, WL up) are the JSBSim structural frame.
"""
import math

DEG = math.pi / 180

# ---------------------------------------------------------------- main rotor [GH 5.1-49, Table 4.1]
R = 26.83               # ft, radius
OMEGA = 27.0            # rad/s, 100 % (258 rpm)
NB = 4                  # blades
CHORD = 1.73            # ft
E_FLAP = 1.25           # ft, flap hinge offset (4.66 % R)
R_ROOT = 3.83           # ft, inboard end of the aerodynamic blade (e' in GenHel)
W_BLADE = 256.9         # lb, blade weight
S_B = 86.7              # slug ft, first mass moment about the flap hinge
I_B = 1512.6            # slug ft^2, flap inertia about the hinge
TIP_LOSS = 0.97         # B, lift beyond B*R ignored
# tip loss in the momentum balance too: the wake carries the loading through the effective disc B^2 A, so the
# induced velocity is 1/B times the full-disc value in the hover and 1/B^2 edgewise (induced power factor 1/B in the hover)
# [Johnson, Rotorcraft Aeromechanics 3.6]. GenHel applies B to the blade lift only, which leaves the induced power
# ideal. Main rotor (wake forcing) and tail rotor (Bailey momentum).
TIP_LOSS_INFLOW = True
# section drag increment over the smooth wind-tunnel tables: a production blade's boundary layer is turbulent from
# near the leading edge. Tripping added about half of McCroskey's NACA 0012 increment to the SC1095 [Bousman
# NASA TP-2003-212265, experiment 3], 0.0012 at full-scale Reynolds number (6 million)
DCD = 0.0012
SHAFT_TILT = 3.0 * DEG  # forward
SWASH_PHASE = -9.7      # deg, swashplate phase angle
TIP_SWEEP_R = 299.0 / 322.0   # swept tip outboard of this (20 deg) [BO table 3]
TIP_SWEEP = 20.0 * DEG
FS_MR, BL_MR, WL_MR = 341.2, 0.0, 315.0   # hub
FLAP_STOP = (-6.0 * DEG, 25.0 * DEG)      # droop / anti-flap stops (assumed)
NSEG = 8                # blade segments, equal annulus area (GenHel uses 5)
# main rotor inflow: 'peters-he' (Peters & He generalized dynamic wake, finite states [peters_he.py]) or
# 'pitt-peters' (GenHel: uniform momentum inflow with a Glauert fore-aft gradient)
INFLOW_MODEL = 'peters-he'
# states (j, r) = (radial order, harmonic): three axisymmetric states (the hover radial distribution) and the first
# harmonic (moment-driven inflow, wake skew coupling). Adding the second harmonic (3, 2) or the j = 4 first-harmonic
# pair raises the retreating blade's angle of attack enough to stall it by 155-162 kt (the aircraft trims at 162 kt
# with 93 % collective); keep them out until blade torsion and dynamic stall are modelled.
PH_COS = [(1, 0), (3, 0), (5, 0), (2, 1)]   # cosine states
PH_SIN = [(2, 1)]                           # sine states
# ground effect in hover on the induced velocity, (1 + GE_K (R/Z)^2)^GE_EXP with Z the hub height above the ground.
# Fitted to the hover chart of TM 1-1520-237-10 (fig. 7-4, flight test: IGE/OGE torque 0.80, 0.86, 0.925, 0.972 at 5,
# 10, 20, 40 ft wheel height; hub 11.5 ft above the wheels). GenHel's form [GH 5.8] (0.115, -2/3) gives only half
# the chart's saving. Forward flight scales the hover value with Heyson's ratio [data/heyson_ground.py].
GE_K, GE_EXP = 0.3, -0.5
GE_ZR_MIN = 0.3
# lead-lag: the elastomeric bearing is a coincident flap/lag hinge at E_FLAP; same blade mass properties. The lag
# damper links a hub pickup and a pickup on the blade cuff (which turns with blade pitch); its force depends on its
# stroke speed [GH 5.1-30, 5.1-49, fig. 1.1.4, table 1.5.4]
LAG_DOF = True
LAG_STOP = (-20.0 * DEG, 12.0 * DEG)      # lag / lead stops (assumed)
LDM_A, LDM_B, LDM_C, LDM_D, LDM_R = 0.227, 3.242, 12.040, 10.0102, 6.898   # in
LDM_THETA_GEO = 17.481 * DEG
LDM_DELTA0 = 7.0 * DEG
LDM_V = [0.1 * i for i in range(21)] + [3.0, 4.0, 5.0, 6.0, 7.0]           # stroke speed, in/s
LDM_F = [0, 100, 230, 380, 600, 900, 1300, 1660, 2080, 2480, 2950, 3210, 3360, 3450, 3525, 3565, 3590, 3615,
         3630, 3650, 3660, 3742, 3805, 3875, 3940, 4000]                   # lb

# preset twist [GH table 1.5.1] against r/R
TWIST_X = [0.05 * i for i in range(21)]
TWIST_DEG = [0.0, 0.0, 0.0, 0.0, -0.15, -0.95, -1.8, -2.75, -3.55, -4.4,
             -5.3, -6.15, -7.1, -7.9, -8.8, -9.65, -10.3, -10.75, -12.3, -13.1, -10.9]

# ---------------------------------------------------------------- tail rotor [GH Table 4.1, HB Table 1]
R_TR = 5.5
OMEGA_TR = 124.62       # rad/s (gear ratio 4.62)
NB_TR = 4
CHORD_TR = 0.81
SIGMA_TR = 0.1875
TWIST_TR = -18.0 * DEG
CANT_TR = 20.0 * DEG    # thrust tilted upwards
K1_TR = 0.7002          # tan(delta3), pitch-flap coupling
GAMMA_TR = 3.3783       # Lock number
PRECONE_TR = 0.01309    # rad
A_TR = 5.73             # lift slope
CD0_TR = 0.0105         # profile drag (assumed, SC1095 at TR Reynolds number)
FS_TR, BL_TR, WL_TR = 732.0, 14.0, 324.7  # hub, right of the fin (GenHel's BL -14 is to the right) [GH 5.4-14]
TIP_LOSS_TR = 0.92
BIAS_TR = 6.0           # deg added to the rigged pitch for the Bailey solution (linear twist equivalent)
DA0_DT_TR = 0.001455    # deg of coning per lb of thrust (with tan delta-3: pitch-flap)
KBLK_TR, VBLK_TR = 0.796, 30.0   # fin blockage factor below 30 kt
# blade-element tail rotor (TR_MODEL 'blades'; 'bailey' is GenHel's closed form): four blades flapping on a hinge at
# the centre (the bearingless crossbeam), their flap inertia from the Lock number [HB]: the coning that gives per lb of
# thrust is GenHel's DA0_DT_TR within 15 %; a weak spring (the crossbeam's stiffness, assumed) holds them at precone
# at rest. Pitch as GenHel rigs it: cuff + BIAS_TR + TWIST_TR r/R - tan(delta-3) x flap; SC1095 sections
# The default is GenHel's closed form: flown in FlightGear at full power (a full collective pull, 140 % torque, the rotor
# drooping to 90 %), the blade-element rotor tops out near 1,900 lb at its pitch stop, its blades stalled, and the
# aircraft yaws away with full left pedal, where the closed form gives 2,400 lb and holds the heading. Its validation
# results are in the report; it stays an option until its stall at high thrust is understood.
TR_MODEL = 'bailey'
R_ROOT_TR = 0.2 * R_TR      # ft, inboard end of the aerodynamic blade (assumed)
NSEG_TR = 6
I_B_TR = 0.0023769 * A_TR * CHORD_TR * R_TR ** 4 / GAMMA_TR     # slug ft^2, flap inertia about the hinge
K_FLAP_TR = 0.05 * I_B_TR * OMEGA_TR ** 2                        # ft lb / rad: flap frequency 1.02 / rev
FLAP_STOP_TR = (-15.0 * DEG, 15.0 * DEG)
# Air at the tail: the main rotor's wake on the tail rotor, the fin and (its change with direction) the stabilator from the
# ring-wake model [mrwake.py] ('vortex'), or GenHel's empennage wash factors in any direction ('genhel'; 'none': no wash on
# the tail rotor). Fin blockage: the fin's own lift and drag in the tail rotor's ring wake [trwake.py] ('fin'; 13.6 % of
# the thrust in the hover, where Sikorsky measured 20.4 %), or GenHel's factor on the thrust, 0.796 fading to 1 by
# VBLK_TR ('genhel'). TR_FIN_K, TR_FIN_CD: experiments that scale the wake's velocity or the fin's drag in it towards
# Sikorsky's figure (1.25, 1.65); a velocity scaled up puts the jet near its far-wake speed at the fin, and both make the
# fin's force grow faster with the tail's sideways motion than the fin moves out of the jet (yaw anti-damping)
TR_WASH = 'vortex'
FIN_WASH = 'vortex'
STAB_WASH = 'vortex'
TR_BLOCKAGE = 'fin'
TR_FIN_K = 1.0
TR_FIN_CD = 1.0
TR_WASH_BLEND = None        # (chi0, chi1) deg: GenHel's wash on the tail rotor below chi0, the ring wake above chi1
# Dynamic stall of the blade-element tail rotor's sections: Gormont's stall delay [USAAMRDL TR 72-67, eqs. 39-45], or
# None (the static tables). The tail rotor turns at 20 Hz: on the retreating side at high speed its sections pitch at a
# reduced frequency near 0.2, where the lift runs well past the static stall before it breaks
TR_DYN_STALL = None
TC_SC1095 = 0.095                    # thickness ratio, for Gormont's stall delay functions
DS_K1_DOWN = 0.5                     # Gormont's K1 with the angle falling: the reattachment delay, once the section has stalled

# ---------------------------------------------------------------- controls [GH 5.5-20, fig. 6.3.1; HB Table 2]
COLL_IN = 10.0                       # collective travel, in (0 = down)
COLL_CUFF = (9.9, 25.9)              # blade root pitch, deg
LON_IN = 5.0                         # +-, positive aft
LON_DEG_IN = 2.88                    # B1s per inch (28.8 deg / 10 in)
B1S_LIM = (-12.5, 16.3)              # head travel limits (+ = forward), deg
LAT_IN = 5.0                         # +-, positive right
LAT_DEG_IN = 1.60
A1S_LIM = (-8.0, 8.0)
PED_IN = 2.69                        # +-, positive right
TR_CUFF_CENTER = 15.0                # deg, pedals centred (mid collective) [GH fig. 5.1.6(d): 21.9 deg at full left]
PED_BIAS_IN = 0.5                    # the aircraft flies with 0.5 in more left pedal than GenHel's rigging gives [Ballin]
TR_DEG_IN = -5.539                   # right pedal reduces tail rotor pitch
TR_CUFF_LIM = (4.5, 36.5)            # head travel limits [GH 5.5-20]
# mechanical mixing, inch of control per inch of collective (+up) / pedal (+right) [HB Table 2]
MIX_COLL_LON = -0.164
MIX_COLL_LAT = -0.16
MIX_COLL_PED = -0.2889
MIX_PED_LON = -0.5746

# ---------------------------------------------------------------- airframe [GH Tables 4.1, 4.2, HB Table 1]
FS_CG, WL_CG = 358.0, 251.0          # design mission (troops), 16000.9 lb
GW_DESIGN = 16000.9
# lb in s^2 -> slug ft^2 (divide by 12)
IXX, IYY, IZZ, IXZ = 65550 / 12, 473626 / 12, 442646 / 12, 18886 / 12
# main rotor downwash correction terms [GH 5.10-19]: empirical (rotor-on wind tunnel and flight test) forces for the
# main rotor wake swirl on the empennage, at the CG, x free-stream q: side force Y = ADD_Y_FT2 q (lb, + right),
# rolling moment L = -(ADD_L_FT3 + k beta) q with k = ADD_L_BETA (beta < 0, > 0), pitching moment M = m beta q with
# m = ADD_M_BETA limited to ADD_M_LIM (ft3); beta = fuselage sideslip (deg), held beyond +-30 deg
DOWNWASH_CORRECTION = True
ADD_Y_FT2 = -19.45
ADD_L_FT3, ADD_L_BETA = 70.0, (1.17, 5.54)
ADD_M_BETA, ADD_M_LIM = (38.9, 51.9), (-292.0, 390.0)
FS_STAB, WL_STAB, S_STAB, AR_STAB = 700.4, 244.0, 45.0, 4.6
FS_FIN, WL_FIN, S_FIN, AR_FIN = 695.0, 273.0, 32.3, 1.92
STAB_LIM = (-8.0, 39.0)              # deg, trailing edge down positive [HB]

# ---------------------------------------------------------------- 3-D model alignment
# The FlightGear model (YASim frame: x fwd, y left, z up, metres) puts the main rotor hub at
# (3.041, 0, 1.209). Matching it to FS 341.2 / WL 315.0 puts the model origin at:
IN_PER_M = 39.3701
FS_ORIGIN = FS_MR + 3.041 * IN_PER_M     # 460.9
WL_ORIGIN = WL_MR - 1.209 * IN_PER_M     # 267.4


def yasim_to_station(x, y, z):
    """YASim/model metres -> (FS, BL, WL) inches"""
    return FS_ORIGIN - x * IN_PER_M, -y * IN_PER_M, WL_ORIGIN + z * IN_PER_M


def segments(n=NSEG, r0=R_ROOT, r1=R):
    """equal annulus area segments: (r_mid, dr) in ft"""
    a = [r0 * r0 + (r1 * r1 - r0 * r0) * k / n for k in range(n + 1)]
    edges = [math.sqrt(x) for x in a]
    return [(math.sqrt((edges[k] ** 2 + edges[k + 1] ** 2) / 2), edges[k + 1] - edges[k]) for k in range(n)], edges


def twist(r):
    x = r / R
    for k in range(20):
        if x <= TWIST_X[k + 1]:
            f = (x - TWIST_X[k]) / (TWIST_X[k + 1] - TWIST_X[k])
            return TWIST_DEG[k] + f * (TWIST_DEG[k + 1] - TWIST_DEG[k])
    return TWIST_DEG[-1]
