"""USAAEFA 77-17 (DTIC ADA132068) appendix E figs. 106-110: control positions at various relative wind azimuths,
10-40 KTAS, in ground effect (average wheel height 25 ft), AFCS on, winds under 3 kt (true airspeed = vector sum of
wind and ground speed, a ground pace vehicle and radar gun as the reference). Digitized from the scan at 220 dpi
(symbols located by pixel; a few read by eye where a symbol merged with its curve or the notes; ~0.05 in, ~10 hp).

Azimuth: degrees from the nose, clockwise (90 = wind from the right: right sideward flight; 180 = rearward flight).
Pedal: inches from full left (6.0 in travel). Tail rotor shaft horsepower: aircraft 716 only (not instrumented on 717).
Longitudinal cyclic: inches from full forward (10 in travel); symbols found per azimuth column, two overlapping
symbols split by the run length (~0.05 in). Pitch attitude was not read: the two aircraft's symbols merge.
The 0 and 360 deg points were flown separately (at 35 kt they differ by 0.4 in and 50 hp: the data's scatter).

  716: S/N 77-22716, normal utility; Coyote Flats; the weight is the one the plotted C_T 0.008033 gives at the
       plotted density altitude and rotor speed (the header's digits read 16,880)
  717: S/N 77-22717, IR suppressors, chaff dispenser, IR jammer and cargo hook; Edwards AFB; C_T read 0.006097
       (consistent with its 16,300-16,360 lb)
Text, para 81: the directional control margin was least, 0.5 in of left pedal remaining (10 %), at 60 deg and 35 KTAS
at Coyote Flats, and stayed about 10 % from 20 KTAS to the sideward limit of 35 KTAS."""

AZ = [0, 30, 60, 90, 120, 150, 180, 225, 270, 315, 360]
# weight lb, CG FS / BL in (forward CG), density altitude ft, OAT deg C, rotor rpm, C_T
COND = {'716': dict(w=16880, fs=347.0, bl=-0.2, da=9860, oat=-2.0, rpm=258, ct=0.008033),
        '717': dict(w=16330, fs=348.0, bl=-1.3, da=2080, oat=-1.0, rpm=259, ct=0.006097)}
WHEEL_FT = 25.0
N = None
PED = {('716', 10): [1.09, 0.90, 0.87, 0.92, 0.97, 0.93, 1.09, 1.20, 1.24, 1.21, 1.13],
       ('717', 10): [1.70, 1.52, 1.43, 1.42, 1.49, 1.60, 1.68, 1.70, 1.76, 1.82, 1.73],
       ('716', 20): [1.32, 0.86, 0.62, 0.69, 1.00, 1.18, 1.37, 1.52, 1.63, 1.51, 1.20],
       ('717', 20): [1.91, 1.41, 1.33, 1.48, 1.43, 1.73, 1.89, 2.05, 2.41, 2.00, 1.79],
       ('716', 30): [1.43, 0.70, 0.52, 0.63, 0.83, 1.08, 1.44, 2.06, 2.43, 2.28, 1.54],
       ('717', 30): [1.90, 1.04, 1.20, 1.23, 1.31, 1.55, 1.95, 2.76, 3.01, 2.69, 1.92],
       ('716', 35): [1.57, 0.63, 0.44, 0.42, 0.55, 0.81, 1.21, 1.97, 2.26, 2.16, 1.15],
       ('717', 35): [2.04, 1.16, 1.09, 0.99, 1.03, 1.22, 1.70, 2.52, 2.67, 2.49, 1.55],
       ('716', 40): [N, N, N, N, N, N, 1.57, 2.29, 2.70, 2.75, 1.70],
       ('717', 40): [2.24, 1.31, 1.13, 1.00, 1.09, 1.48, 2.01, 2.78, 3.09, 2.88, 2.13]}
TR_HP = {10: [267.9, 307.8, 318.5, 305.8, 303.1, 296.6, 272.9, 244.1, 235.1, 254.1, 270.9],
         20: [220.7, 303.2, 344.7, 314.8, 303.4, 271.9, 222.1, 160.0, 119.9, 182.3, 205.0],
         30: [167.9, 276.7, 338.7, 307.5, 271.8, 241.1, 199.8, 124.8, 87.0, 108.3, 155.7],
         35: [145.5, 238.5, 285.7, 278.4, 253.6, 210.5, 156.8, 77.1, 25.5, 27.1, 93.0],
         40: [N, N, N, N, N, N, 141.7, 73.0, 58.4, 73.6, 105.7]}
LON = {('716', 10): [5.60, 5.58, 5.70, 5.79, 5.86, 5.91, 5.92, 6.50, 6.20, 6.03, 5.66],
       ('717', 10): [6.14, 6.19, 6.20, 6.23, 6.34, 6.46, 6.59, 6.65, 6.62, 6.34, 6.15],
       ('716', 20): [4.92, 4.61, 5.29, 5.37, 5.66, 6.25, 6.68, 6.96, 6.74, 6.08, 4.79],
       ('717', 20): [5.49, 5.34, 5.71, 5.70, 5.80, 6.25, 6.88, 7.10, 7.20, 6.57, 5.30],
       ('716', 30): [4.63, 3.55, 5.16, 5.13, 5.41, 6.04, 6.67, 7.13, 7.02, 6.37, 4.70],
       ('717', 30): [5.69, 4.88, 5.57, 5.44, 5.87, 6.48, 7.03, 7.59, 7.65, 7.12, 5.70],
       ('716', 35): [4.75, 4.07, 4.94, 4.75, 5.09, 5.79, 6.56, 6.97, 6.74, 6.16, 4.32],
       ('717', 35): [5.96, 4.88, 5.45, 5.28, 5.51, 6.03, 6.64, 7.09, 7.06, 6.46, 5.45],
       ('716', 40): [N, N, N, N, N, N, 6.61, 7.31, 7.26, 6.66, 4.89],
       ('717', 40): [6.09, 5.05, 5.78, 5.46, 5.58, 6.12, 6.89, 7.61, 7.79, 7.41, 5.97]}
