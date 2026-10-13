"""USAAEFA lateral-directional trim data for the UH-60A, digitized from the scanned reports (1-bit 300 dpi scans;
symbols located by pixel after grid-line removal, axes from the labelled grid lines; ~0.2 deg / 0.03 in).

  [79-24]  USAAEFA 79-24 / DTIC ADA125428, RSIS validation flight test, UH-60A S/N 77-22716, appendix F
  [85]     DTIC ADA180404, expanded gross weight / cg test, same S/N 77-22716
  [77]     DTIC ADA132068, USAAEFA 77-17 A&FC;  [6Y] DTIC ADA180181, sixth-year production A&FC

Units: sideslip deg (+ = wind from the right), roll deg (+ = right wing down), lateral cyclic inches from full left
(10.0-10.2 in travel), pedal inches from full left (6.0 in travel). Trim points (shaded symbols) are flagged; all
sweeps were flown collective-fixed from a zero-sideslip trim with SAS on, FPS off, PBA off (failed), stabilator fixed
(31 deg TED at 60 KTAS, 8 at 100, 6 at 140). Referred true airspeed 60/100/140 kt; C_T ~ 69e-4 unless noted.
"""

# 79-24 fig. 2 (scan unreadable, values from the text, para 16): inherent sideslip in ball-centered (wings-level)
# level flight, varying linearly from -4 deg at 67 KCAS to -1 deg at 150 KCAS; ball-centered flight needs ~0.5 in
# more right pedal than zero sideslip. Mid cg, ~16,000 lb, C_T ~ 69e-4.
INHERENT_7924 = [(67, -4.0), (150, -1.0)]          # (KCAS, beta)

# [85] fig. 18: sideslip in ball-centered level flight vs advance ratio (curves), for six C_T; FS 347, BL 0.1 left.
# Above mu 0.22 the curves are flat: C_T 0.0070 -> -1.5, 0.0080 -> -1.0, 0.0090/0.0092 -> -0.5, 0.0100 -> 0,
# 0.0110 -> +2 deg. Below mu 0.18 they rise steeply to the right (+10..+20 at mu 0.13, large scatter).
# Text (para 21): ~5 deg RIGHT in ball-centered IRP climbs, ~5 deg LEFT in 1000 fpm descents.
INHERENT_85 = {0.0070: [(0.13, 11), (0.16, 2), (0.20, -1), (0.24, -1.5), (0.36, -1.5)],
               0.0080: [(0.13, 13), (0.15, 8), (0.18, 4), (0.20, 1), (0.22, -1), (0.36, -1)],
               0.0090: [(0.12, 18), (0.15, 11), (0.18, 2), (0.20, -0.5), (0.31, -0.5)],
               0.0100: [(0.13, 20), (0.16, 9), (0.20, 2), (0.24, 0), (0.30, 0)],
               0.0110: [(0.17, 17), (0.19, 11), (0.22, 4), (0.25, 2), (0.27, 2)]}
# [77] para 16: coordinated flight between best-endurance speed and VH "normally occurs between 5 and 0 degrees left";
# [6Y] para 13: "maximum left sideslip of approximately 1 degree", moving right with C_T.

# 79-24 figs. 20-28: static lateral-directional stability (sideslip sweeps). Each: dict(cond, roll, lat, ped) with
# (beta, value) points; TRIM marks the shaded trim point's beta.
SWEEPS = {
    20: dict(cond='baseline 60 KTAS (55 KCAS), level, FS ~352, BL 0, Hd ~5000, 258 rpm', trim=0.0,
             roll=[(-20.8, -2.11), (-15.9, -1.70), (-10.8, -0.65), (-6.5, -0.30), (0.0, 0.13), (4.0, -0.04),
                   (8.7, 1.07), (14.0, 1.85), (19.5, 2.10)],
             lat=[(-21.0, 4.25), (-16.1, 4.33), (-10.9, 4.47), (-6.7, 4.60), (-0.2, 4.83), (3.8, 4.94), (8.4, 5.19),
                  (13.8, 5.36), (19.3, 5.60)],
             ped=[(-21.1, 3.59), (-16.2, 3.40), (-11.0, 3.20), (-6.9, 2.95), (-0.2, 2.60), (3.7, 2.34), (8.3, 2.07),
                  (13.7, 1.81), (19.1, 1.48)]),
    21: dict(cond='aft cg FS ~359, 60 KTAS (54 KCAS), level, BL 0, Hd ~4800', trim=0.0,
             roll=[(-20.1, -6.31), (-15.9, -3.97), (-10.4, -2.89), (-6.1, -0.90), (-0.1, 1.63), (3.9, 2.39),
                   (8.6, 3.25), (13.6, 4.48), (18.8, 6.59)],
             lat=[(-20.3, 4.45), (-16.1, 4.52), (-10.7, 4.68), (-6.3, 4.84), (-0.2, 5.13), (3.6, 5.31), (8.2, 5.58),
                  (13.2, 5.83), (18.5, 6.07)],
             ped=[(-20.3, 4.91), (-16.1, 4.49), (-6.4, 3.54), (-0.4, 3.10), (3.5, 2.54), (8.0, 2.02), (13.1, 1.52),
                  (18.4, 0.92)]),
    22: dict(cond='baseline 140 KTAS (128 KCAS), level, BL -0.2, Hd ~6200', trim=0.25,
             roll=[(-16.1, -12.31), (-11.0, -5.64), (-6.1, -3.34), (0.25, 1.62), (4.2, 3.30), (8.8, 7.40)],
             lat=[(-16.1, 5.47), (-10.9, 5.50), (-6.2, 5.62), (-0.1, 5.86), (4.2, 6.17), (8.6, 6.46)],
             ped=[(-16.1, 5.51), (-11.0, 4.82), (-6.2, 4.06), (-0.8, 3.16), (4.2, 2.35), (8.7, 1.65)]),
    # fig. 23: lateral cg, 100 KTAS (92 KCAS). The roll axis runs -10..30 and the BL +2.1 circles plot at +17..+28:
    # read as displaced by +20 deg for clarity (no note on the figure; 20 deg is consistent with the lateral cyclic).
    '23_left': dict(cond='lateral cg BL -4.1 (left), 100 KTAS, 15,750 lb', trim=-1.0,
                    roll=[(-20.6, -6.67), (-17.1, -6.42), (-12.9, -4.60), (-7.0, -3.47), (-1.0, -0.71), (5.4, 0.12),
                          (8.9, 2.65), (13.8, 2.73), (18.8, 5.28)],
                    lat=[(-20.1, 5.45), (-16.9, 5.53), (-12.4, 5.66), (-6.7, 5.87), (-0.6, 6.10), (5.6, 6.40),
                         (9.5, 6.62), (14.1, 6.87), (19.1, 7.12)]),
    '23_right': dict(cond='lateral cg BL +2.1 (right), 100 KTAS, 15,480 lb; roll = plotted - 20 (see above)', trim=-1.1,
                     roll=[(-20.4, -2.95), (-16.1, -1.90), (-11.3, 0.37), (-6.4, 1.85), (-1.1, 2.51), (7.8, 5.62),
                           (13.7, 6.84), (18.7, 7.63)],
                     lat=[(-19.9, 4.06), (-15.5, 4.15), (-10.8, 4.34), (-5.9, 4.51), (-0.6, 4.61), (8.1, 5.17),
                          (14.2, 5.48), (19.1, 5.77)]),
    '23_ped': [(-20.0, 4.92), (-15.5, 4.54), (-12.2, 4.24), (-10.8, 4.06), (-6.4, 3.76), (-0.4, 3.19), (5.6, 2.50),
               (8.5, 2.07), (14.5, 1.58), (19.5, 0.97)],              # both cg cases (they overlap)
    # fig. 24: 1500 fpm climb and descent at 100 KTAS (93 KCAS), ~15,700 lb. Symbols overlap; read by eye at 3x zoom.
    '24_descent': dict(cond='1500 fpm descent, 100 KTAS', trim=-0.6,
                       roll=[(-20.5, -4.3), (-16.3, -3.5), (-11.0, -1.3), (-6.0, -0.35), (-0.6, 3.2), (5.0, 3.6),
                             (9.0, 3.9), (14.5, 6.4), (19.5, 7.7)]),
    '24_climb': dict(cond='1500 fpm climb, 100 KTAS', trim=-0.6,
                     roll=[(-20.7, -9.0), (-16.0, -5.6), (-11.0, -4.3), (-6.0, -2.5), (-0.6, -1.1), (5.6, 1.5),
                           (14.5, 5.3), (19.5, 5.7)]),
    '24_trim': dict(lat=(5.41, 5.72), ped=2.95),   # the two lateral trims can't be told apart; pedal trims coincide
    25: dict(cond='baseline 100 KTAS (92 KCAS), level, FS ~351, BL 0, Hd ~5000, 16,120 lb', trim=-0.8,
             roll=[(-20.0, -6.08), (-16.2, -3.85), (-11.6, -2.25), (-6.3, -1.61), (-0.8, 1.62), (4.7, 2.66),
                   (9.0, 4.46), (14.2, 5.00)],
             lat=[(-19.9, 4.85), (-16.1, 4.96), (-11.8, 5.12), (-6.3, 5.25), (-0.8, 5.44), (4.8, 5.75), (9.0, 5.99),
                  (14.1, 6.23)],
             ped=[(-19.9, 4.89), (-16.1, 4.48), (-11.7, 4.04), (-6.3, 3.55), (-0.5, 3.02), (4.9, 2.27), (9.3, 1.88),
                  (14.3, 1.36)]),
    # fig. 26: aft cg 100 KTAS. Lateral axis labels unreadable (values may be 1 in low); roll slope looks odd.
    26: dict(cond='aft cg FS ~359, 100 KTAS (96 KCAS), level (questionable read)', trim=-1.0,
             roll=[(-20.5, -2.59), (-16.3, -2.55), (-11.1, -2.37), (-6.1, -1.37), (-1.0, -1.07), (4.0, -0.20),
                   (7.8, -0.44), (12.9, 0.42), (19.3, 0.25)],
             ped=[(-20.7, 3.57), (-16.6, 3.42), (-11.1, 3.19), (-6.3, 2.95), (-1.0, 2.70), (3.8, 2.38), (7.5, 2.05),
                  (12.6, 1.86), (19.3, 1.57)]),
    27: dict(cond='mid C_T ~79e-4 (18,900 lb @ 4400 ft and 15,700 lb @ 9800 ft overlap), 100 KTAS', trim=-0.9,
             roll=[(-20.9, -5.51), (-15.8, -3.02), (-10.9, -1.66), (-5.9, -1.62), (-0.9, 2.06), (4.2, 2.34),
                   (9.1, 3.21), (14.2, 4.86), (19.1, 6.01)],
             lat=[(-21.0, 4.22), (-16.1, 4.44), (-11.1, 4.61), (-6.2, 4.71), (-1.0, 5.04), (4.0, 5.34), (9.0, 5.64),
                  (13.9, 5.86), (18.9, 6.24)],
             ped=[(-21.1, 5.19), (-16.0, 4.66), (-11.0, 4.22), (-6.0, 3.88), (-1.0, 3.33), (3.9, 2.64), (9.1, 2.08),
                  (13.9, 1.65), (18.9, 1.08)]),
    28: dict(cond='96 % referred rotor speed (248 rpm), 100 KTAS (92 KCAS), level', trim=0.2,
             roll=[(-19.7, -5.94), (-15.5, -3.20), (-10.2, -2.32), (-5.8, -1.54), (0.2, 1.93), (4.8, 2.61),
                   (9.3, 3.25), (14.8, 5.80), (19.9, 6.92)],
             lat=[(-20.0, 4.48), (-15.8, 4.56), (-10.5, 4.73), (-6.1, 4.84), (-0.2, 5.15), (4.6, 5.34), (9.0, 5.59),
                  (19.5, 6.13)],
             ped=[(-15.8, 4.58), (-6.2, 3.66), (-0.3, 3.19), (4.4, 2.53), (8.9, 2.08), (14.5, 1.50), (19.3, 0.95)]),
}
# 79-24 fig. 10 (climbs/descents at zero sideslip, three airspeeds, -4000..+3000 fpm): pedal stays within ~0.1 in of
# 2.9 in from autorotation to ~+1500 fpm (one series moves ~0.3 in left at +2000..3000 fpm); lateral cyclic flat
# for two series, the third moves right with climb rate.
