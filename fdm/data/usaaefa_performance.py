"""USAAEFA hover and level-flight performance of the UH-60A, from the scanned reports.

  [77]  USAAEFA 77-17 / DTIC ADA132068, airworthiness and flight characteristics, UH-60A S/N 77-22716 (first-year
        production, normal utility configuration)
  [6Y]  USAAEFA 83-24 / DTIC ADA180181, sixth-year production UH-60A S/N 82-23748 (ESSS fairings)

Coefficients on the main rotor (R 26.833 ft, disc 2262 ft^2): C_T = gross weight / (rho A (Omega R)^2), C_P = total
engine output shaft power (both engines: main and tail rotor, transmission, accessories) / (rho A (Omega R)^3).
"""


def hover_oge(ct, ess=False):
    """tethered hover at 100 ft wheel height, C_P(C_T) [6Y app. E fig. 3, the fits printed on the figure]: first-year
    normal utility aircraft ([77] reanalysed in [6Y], dashed) or sixth-year with the ESSS fairings (solid; the
    operator's manual hover charts, ~3 % more power)"""
    if ct < 0.007:
        return 8.06470e-5 + (1.02707 if ess else 0.99309) * ct ** 1.5
    return (2.57292e-4 + 0.42384 * ct ** 1.5 + 515.0 * ct ** 3) if ess else (2.55577e-4 + 0.395715 * ct ** 1.5 + 510.0 * ct ** 3)


def tail_rotor_hover(ct_tr):
    """tail rotor shaft power in the OGE hover [77 app. E fig. 10], on the tail rotor (R 5.5 ft, disc 95.033 ft^2):
    C_P against the net anti-torque thrust (main rotor torque / 32.567 ft), the faired line read from the scan; data
    from C_T 102e-4 to 172e-4"""
    return (174.1 + 2.016 * (ct_tr * 1e4 - 102.2)) * 1e-5


# level flight [77 app. E figs. 15-26]: faired C_P x 1e5 against advance ratio and C_T x 1e4, read at a referred rotor
# speed of 263 rpm (the C_T 65 lines end at ~256 rpm and are read there); zero-sideslip trim, forward CG. Scale read
# from the axis labels (~0.055 per px at 160 dpi), +-0.3 on C_P. Check against a flight [77 fig. 52]: 16580 lb, DA
# 2650 ft, 6 C, C_T 65.5e-4, 95 KTAS (mu 0.225): 1123 shp = C_P 34.2e-5 (the fairing gives 33.7).
LEVEL = {
    0.10: {65: 34.21, 70: 37.99},
    0.12: {65: 33.78, 70: 37.31, 80: 45.50, 90: 54.24, 100: 65.81, 110: 88.24},
    0.14: {65: 32.89, 70: 35.82, 80: 42.79, 90: 50.68, 100: 60.91, 110: 79.50},
    0.16: {65: 32.32, 70: 35.00, 80: 40.91, 90: 48.26, 100: 57.55, 110: 73.27},
    0.18: {65: 32.05, 70: 34.42, 80: 39.92, 90: 46.74, 100: 55.64, 110: 70.05},
    0.20: {65: 32.09, 70: 34.33, 80: 39.68, 90: 46.52, 100: 55.06, 110: 70.19},
    0.22: {65: 33.14, 70: 35.19, 80: 40.44, 90: 47.30, 100: 56.14, 110: 72.19},
    0.24: {65: 34.70, 70: 36.57, 80: 41.94, 90: 48.99, 100: 58.52, 110: 78.76},
    0.26: {65: 38.01, 70: 40.00, 80: 45.47, 90: 52.97, 100: 63.33, 110: 92.95},
    0.28: {65: 41.86, 70: 43.71, 80: 49.54, 90: 58.03, 100: 71.22},
    0.30: {65: 46.87, 70: 48.82, 80: 55.17, 90: 65.98, 100: 83.16},
    0.32: {65: 53.60, 70: 55.24, 80: 62.16, 90: 75.21, 100: 98.21},
}
