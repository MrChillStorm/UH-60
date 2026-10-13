"""Figures for the report (vector PDF) from report/results/*.json.  python3 report/figures.py"""
import sys, os, json, math
HERE = os.path.dirname(os.path.abspath(__file__))
FDM = os.path.dirname(HERE)
sys.path.insert(0, FDM)
import numpy as np
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Circle
import uh60data as U
import genuh60 as G
from data import genhel_fuselage as FUS, usaaefa_performance as PERF

plt.rcParams.update({'font.family': 'serif', 'font.serif': ['STIX Two Text', 'STIXGeneral', 'DejaVu Serif'], 'mathtext.fontset': 'stix',
                     'font.size': 8.5, 'axes.labelsize': 8.5, 'axes.titlesize': 9, 'legend.fontsize': 7.5, 'xtick.labelsize': 7.5,
                     'ytick.labelsize': 7.5, 'axes.linewidth': 0.6, 'lines.linewidth': 1.1, 'grid.linewidth': 0.3, 'grid.alpha': 0.5,
                     'axes.grid': True, 'figure.dpi': 150, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02,
                     'axes.spines.top': False, 'axes.spines.right': False})
C_MOD, C_TEST, C_GH, C_REF = '#b2182b', '#222222', '#2166ac', '#7a7a7a'
W1, W2 = 3.35, 6.9          # single / double column width (in)
OUT = os.path.join(HERE, 'fig')
RES = os.path.join(HERE, 'results')


def res(name):
    return json.load(open(os.path.join(RES, name + '.json')))


def save(fig, name):
    fig.savefig(os.path.join(OUT, name + '.pdf')); plt.close(fig)


# ----------------------------------------------------------------------------------------------- geometry
SKIP = ('wings', 'grue', 'trougrue', 'barriere', 'probe', 'lampes', 'supportlampes', 'lampeverte', 'antennes', 'crochet', 'poignees')


def ac_polygons(fn):
    """the airframe's faces from the AC3D model, in stations (FS, WL)"""
    lines = open(fn, errors='replace').read().split('\n')
    polys = []

    def parse(i, off):
        name, loc, verts = '', [0.0, 0.0, 0.0], []
        i += 1
        while i < len(lines):
            t = lines[i].split()
            if not t:
                i += 1; continue
            if t[0] == 'name':
                name = ' '.join(t[1:]).strip('"')
            elif t[0] == 'loc':
                loc = [float(x) for x in t[1:4]]
            elif t[0] == 'numvert':
                n = int(t[1]); verts = [[float(x) for x in lines[i + 1 + k].split()[:3]] for k in range(n)]; i += n
            elif t[0] == 'numsurf':
                n = int(t[1]); i += 1; c = 0
                o = [off[k] + loc[k] for k in range(3)]
                while c < n:
                    if lines[i].startswith('refs'):
                        r = int(lines[i].split()[1])
                        idx = [int(lines[i + 1 + k].split()[0]) for k in range(r)]
                        if name not in SKIP and r >= 3:
                            pts = []
                            for j in idx:
                                ax, ay, az = (verts[j][k] + o[k] for k in range(3))
                                fs, bl, wl = U.yasim_to_station(-ax, az, ay)
                                pts.append((fs, wl))
                            polys.append(pts)
                        i += r; c += 1
                    i += 1
                continue
            elif t[0] == 'kids':
                kids = int(t[1]); i += 1
                o = [off[k] + loc[k] for k in range(3)]
                for _ in range(kids):
                    while not lines[i].startswith('OBJECT'):
                        i += 1
                    i = parse(i, o)
                return i
            i += 1
        return i
    i = 0
    while i < len(lines):
        if lines[i].startswith('OBJECT'):
            i = parse(i, [0.0, 0.0, 0.0])
        else:
            i += 1
    return polys


def fig_geometry():
    fig, (ax, bx) = plt.subplots(2, 1, figsize=(W2, 5.3), gridspec_kw=dict(height_ratios=[1.55, 1], hspace=0.32))
    polys = ac_polygons(os.path.join(FDM, '..', 'UH-60A', 'Models', 'uh60.ac'))
    ax.add_collection(PolyCollection(polys, facecolors='#ececec', edgecolors='#b8b8b8', linewidths=0.08, rasterized=True, zorder=1))
    # main rotor disc (3 deg forward shaft tilt) and hub
    t = U.SHAFT_TILT; r = U.R * 12
    ax.plot([U.FS_MR - r * math.cos(t), U.FS_MR + r * math.cos(t)], [U.WL_MR + r * math.sin(t), U.WL_MR - r * math.sin(t)], color=C_GH, lw=1.4, zorder=3)
    ax.plot([U.FS_MR, U.FS_MR + 40 * math.sin(t)], [U.WL_MR, U.WL_MR - 40 * math.cos(t)], color=C_GH, lw=0.8, zorder=3)
    for k, (rm, dr) in enumerate(U.segments()[0]):
        for sgn in (-1, 1):
            ax.plot(U.FS_MR + sgn * rm * 12 * math.cos(t), U.WL_MR - sgn * rm * 12 * math.sin(t), '|', color=C_GH, ms=4, mew=0.7, zorder=4)
    ax.text(U.FS_MR - r + 10, U.WL_MR + 26, 'main rotor: 4 blades x 8 segments, hub FS %.1f WL %.0f' % (U.FS_MR, U.WL_MR), color=C_GH, fontsize=7)
    # tail rotor (its disc seen nearly face on)
    ax.add_patch(Circle((U.FS_TR, U.WL_TR), U.R_TR * 12, fill=False, ec=C_GH, lw=1.0, ls='--', zorder=3))
    ax.text(U.FS_TR - 20, U.WL_TR + U.R_TR * 12 + 8, 'tail rotor, 20$^\\circ$ cant', color=C_GH, fontsize=7, ha='center')
    # stabilator, fin, fuselage reference point
    from data import genhel_empennage as EMP
    ch = math.sqrt(U.S_STAB / U.AR_STAB) * 12
    ax.plot([EMP.FS_H - 0.25 * ch, EMP.FS_H + 0.75 * ch], [EMP.WL_H, EMP.WL_H], color='#1b7837', lw=2.2, zorder=3)
    ax.plot(EMP.FS_H, EMP.WL_H, 'o', color='#1b7837', ms=3, zorder=4)
    ax.text(EMP.FS_H, EMP.WL_H - 30, 'stabilator\n(%.0f ft$^2$, $-8\\ldots39^\\circ$)' % U.S_STAB, color='#1b7837', fontsize=7, ha='center', va='top')
    ax.plot(EMP.FS_V, EMP.WL_V, 's', color='#1b7837', ms=3.5, zorder=4)
    ax.text(EMP.FS_V - 12, EMP.WL_V + 6, 'fin', color='#1b7837', fontsize=7, ha='right')
    ax.plot(FUS.FSWF, FUS.WLWF, 'D', color='0.3', ms=3.5, zorder=4)
    ax.text(FUS.FSWF + 8, FUS.WLWF - 14, 'fuselage tables', color='0.3', fontsize=7)
    # centres of gravity
    for (fs, wl), lab, mk in (((U.FS_CG, U.WL_CG), 'design (GenHel)', 'o'), ((351.0, 231.5), 'TM-88360 fig. B1', 's'), ((360.4, 247.2), 'TM-85890', '^')):
        ax.plot(fs, wl, mk, mfc='white', mec=C_MOD, mew=1.0, ms=4.5, zorder=5, label='CG: ' + lab)
    # ground contacts
    for name, xyz in (('L', (4.136, 1.344, -2.317)), ('T', (-4.664, 0.0, -2.217))):
        fs, bl, wl = U.yasim_to_station(*xyz)
        ax.plot(fs, wl, 'v', color='0.15', ms=4, zorder=5)
    S = np.array([U.yasim_to_station(*xyz) for _, xyz in G.STRUCTURE])
    ax.plot(S[:, 0], S[:, 2], 'x', color='0.45', ms=3, mew=0.6, zorder=4, label='structure contacts')
    ax.set_aspect('equal'); ax.set_xlim(-10, 830); ax.set_ylim(150, 455)
    ax.set_xlabel('fuselage station FS (in)'); ax.set_ylabel('waterline WL (in)')
    ax.legend(loc='upper left', ncol=4, frameon=False, fontsize=6.6, handletextpad=0.3, columnspacing=1.0)
    ax.set_title('(a) side view: 3-D model airframe, component reference points', loc='left', fontsize=8.5)
    # blade
    x = np.array(U.TWIST_X); y = np.array(U.TWIST_DEG)
    segs, edges = U.segments()
    for k in range(len(segs)):
        bx.axvspan(edges[k] / U.R, edges[k + 1] / U.R, color='#2166ac', alpha=0.06 if k % 2 else 0.13, lw=0)
        bx.text(segs[k][0] / U.R, 1.7, '%d' % (k + 1), ha='center', fontsize=6.5, color=C_GH)
    bx.plot(x, y, color=C_TEST, lw=1.3, label='preset twist [GenHel table 1.5.1]')
    for xv, lab, yy in ((U.E_FLAP / U.R, 'flap/lag hinge', -6.0), (U.R_ROOT / U.R, 'root cut-out', -8.5), (U.TIP_SWEEP_R, '20$^\\circ$ swept tip', -3.5),
                        (U.TIP_LOSS, 'tip loss $B$', -6.0)):
        bx.axvline(xv, color=C_REF, lw=0.7, ls='--')
        bx.text(xv - 0.008, yy, lab, rotation=90, ha='right', va='center', fontsize=6.5, color='0.25')
    bx.set_xlim(0, 1.0); bx.set_ylim(-14, 2.8)
    bx.set_xlabel('$r/R$'); bx.set_ylabel('twist (deg)')
    bx.set_title('(b) main rotor blade: twist and the eight segments of equal annulus area', loc='left', fontsize=8.5)
    bx.legend(loc='lower left', frameon=False)
    save(fig, 'geometry')


# ----------------------------------------------------------------------------------------------- level-flight trims
def fig_trim():
    R = res('b1'); M, T = R['model'], R['test']
    V = [m['V'] for m in M]
    panels = (('lon', 'longitudinal cyclic (% fwd)'), ('lat', 'lateral cyclic (% right)'), ('coll', 'collective (%)'), ('ped', 'pedal (% right)'),
              ('theta', 'pitch attitude (deg)'), ('phi', 'roll attitude (deg)'), ('hp', 'main rotor power (hp)'), ('stab', 'stabilator (deg TED)'))
    fig, axs = plt.subplots(2, 4, figsize=(W2, 3.9), sharex=True)
    for a, (k, lab) in zip(axs.flat, panels):
        a.plot(V, [m[k] for m in M], '-', color=C_MOD, label='model')
        tv = [t['V'] for t in T]; ty = [t[k] for t in T]
        low = [i for i, t in enumerate(T) if t['V'] < 35]
        a.plot([tv[i] for i in range(len(T)) if i not in low], [ty[i] for i in range(len(T)) if i not in low], 'o', mfc='white', mec=C_TEST, ms=4, label='USAAEFA test')
        a.plot([tv[i] for i in low], [ty[i] for i in low], 'o', mfc='0.75', mec=C_TEST, ms=4, label='test, flown at 3-15 KCAS')
        a.set_title(lab, fontsize=7.8)
        a.set_xlim(0, 170)
    for a in axs[1]:
        a.set_xlabel('true airspeed (kt)')
    axs[0, 0].legend(frameon=False, fontsize=6.3, loc='lower left')
    fig.tight_layout(pad=0.3, w_pad=0.6, h_pad=0.6)
    save(fig, 'trim')


# ----------------------------------------------------------------------------------------------- power
def fig_power():
    fig, axs = plt.subplots(2, 2, figsize=(W2, 5.0))
    # (a) OGE hover
    a = axs[0, 0]; Hv = res('hover')
    ct = np.linspace(0.0040, 0.0092, 60)
    a.plot(ct * 1e4, [PERF.hover_oge(c) * 1e5 for c in ct], '-', color=C_TEST, lw=1.0, label='test, first-year aircraft [77-17, 83-24]')
    a.plot(ct * 1e4, [PERF.hover_oge(c, True) * 1e5 for c in ct], '--', color=C_TEST, lw=0.8, label='test, sixth-year (ESSS) = flight manual')
    a.plot([h['ct'] * 1e4 for h in Hv], [h['cp'] * 1e5 for h in Hv], 'o-', color=C_MOD, ms=3.5, label='model')
    a.axvspan(50, 92, color='0.9', zorder=0, lw=0)
    a.set_xlabel('$C_T\\times10^4$ (gross weight)'); a.set_ylabel('$C_P\\times10^5$ (engine shaft power)')
    a.set_title('(a) OGE hover, 100 ft wheel height', loc='left', fontsize=8.5); a.legend(frameon=False, fontsize=6.5, loc='upper left')
    # (b) tail rotor in the hover
    b = axs[0, 1]
    x = np.linspace(0.0060, 0.0175, 50)
    ins = (x >= 0.0102) & (x <= 0.0172)
    b.plot(x[ins] * 1e4, [PERF.tail_rotor_hover(c) * 1e5 for c in x[ins]], '-', color=C_TEST, lw=1.0, label='test [77-17 fig. 10]')
    b.plot(x[~ins] * 1e4, [PERF.tail_rotor_hover(c) * 1e5 for c in x[~ins]], ':', color=C_TEST, lw=1.0, label='test line extrapolated')
    b.plot([h['ct_tr'] * 1e4 for h in Hv], [h['cp_tr'] * 1e5 for h in Hv], 'o-', color=C_MOD, ms=3.5, label='model')
    b.set_xlabel('tail rotor $C_T\\times10^4$ (net anti-torque thrust)'); b.set_ylabel('tail rotor $C_P\\times10^5$')
    b.set_title('(b) tail rotor shaft power, OGE hover', loc='left', fontsize=8.5); b.legend(frameon=False, fontsize=6.5, loc='upper left')
    # (c) level flight
    c = axs[1, 0]; L = res('level')
    cols = {'65': '#fdae61', '70': '#d7191c', '80': '#2c7bb6'}
    for k, col in cols.items():
        rows = [r for r in L[k] if r['resid'] < 0.05]
        c.plot([r['mu'] for r in rows], [r['cp'] * 1e5 for r in rows], '-', color=col, label='model, $C_T$ %s' % k)
        mus = sorted(PERF.LEVEL)
        c.plot(mus, [PERF.LEVEL[m].get(int(k), np.nan) for m in mus], 'o', mfc='white', mec=col, ms=3.5)
    c.plot([], [], 'o', mfc='white', mec='0.3', ms=3.5, label='test [77-17 figs. 15-26]')
    c.set_xlabel('advance ratio $\\mu$'); c.set_ylabel('$C_P\\times10^5$ (engine shaft power)')
    c.set_title('(c) level flight, $C_T\\times10^4$ = 65, 70, 80', loc='left', fontsize=8.5); c.legend(frameon=False, fontsize=6.3, loc='upper center', ncol=2)
    # (d) figure of merit
    d = axs[1, 1]; F = res('fm')
    m = [r for r in F['model'] if r['resid'] < 0.05]
    d.plot([r['ct'] / (U.NB * U.CHORD / (math.pi * U.R)) for r in m], [r['fm'] for r in m], 'o-', color=C_MOD, ms=3.5, label='model (isolated rotor)')
    d.plot(F['bemt_cts'], F['bemt_uniform_B'], '--', color=C_GH, lw=0.9, label='BEM: uniform inflow, lift cut at $B$')
    d.plot(F['bemt_cts'], F['bemt_annulus_F'], '-', color=C_GH, lw=0.9, label='BEM: annulus momentum, Prandtl $F$')
    nx, ny = zip(*F['norman'])
    d.errorbar(nx, ny, yerr=0.01, fmt='s', mfc='white', mec=C_TEST, ecolor=C_TEST, ms=3.5, elinewidth=0.6, capsize=1.5, label='measured [LRTA paper, fig. 22]')
    d.set_xlabel('$C_T/\\sigma$'); d.set_ylabel('figure of merit')
    d.set_title('(d) rotor figure of merit, sea level, $M_{tip}$ 0.65', loc='left', fontsize=8.5); d.legend(frameon=False, fontsize=6.3, loc='lower right')
    fig.tight_layout(pad=0.3, w_pad=1.0, h_pad=0.9)
    save(fig, 'power')


# ----------------------------------------------------------------------------------------------- ground effect
def fig_ge():
    E = res('ge')
    oge = [e for e in E if e['height'] == 100][0]['torque']
    fig, a = plt.subplots(figsize=(W1, 2.2))
    hw = np.linspace(2, 100, 200)
    a.plot([5, 10, 20, 40], [0.80, 0.86, 0.925, 0.972], 's', mfc='white', mec=C_TEST, ms=4.5, label='flight manual [TM fig. 7-4]')
    pts = [e for e in E if e['height'] < 100]
    a.plot([e['wheel_ft'] for e in pts], [e['torque'] / oge for e in pts], 'o', color=C_MOD, ms=4, label='model, flown by the test autopilot')
    a.axhline(1.0, color=C_REF, lw=0.6)
    a.set_xscale('log'); a.set_xticks([5, 10, 20, 40, 100]); a.set_xticklabels(['5', '10', '20', '40', '100'])
    a.set_xlabel('wheel height (ft)'); a.set_ylabel('torque IGE / OGE')
    a.legend(frameon=False, loc='lower right', fontsize=6.8)
    save(fig, 'ge')


# ----------------------------------------------------------------------------------------------- control responses
RESP = (('201', 'hover, 1 in right lateral cyclic'), ('204', 'hover, 0.5 in forward cyclic'), ('209', 'hover, 1 in left pedal'),
        ('504', '60 kt, 1 in left lateral cyclic'), ('401', '100 kt, 1 in aft cyclic'), ('102', '100 kt, 1 in right pedal'))


def fig_response():
    import dynamics as DY
    Dn = res('dyn')
    fig, axs = plt.subplots(2, 3, figsize=(W2, 3.9), sharex=True)
    names = {'p': 'roll rate', 'q': 'pitch rate', 'r': 'yaw rate'}
    for a, (fid, title) in zip(axs.flat, RESP):
        case = DY.D[fid]; t = np.array(case['t'])
        k, t0 = DY.primary(case); v = DY.AXIS[k]
        for who, st, col, lab in (('test', '--', C_TEST, 'flight test'), ('genhel', '-', C_GH, 'GenHel (Ames)')):
            y = DY.curve(case, v, who)
            if y is not None:
                a.plot(t, DY.rel(y, t, t0), st, color=col, lw=1.0, label=lab)
        a.plot(t, DY.rel(np.array(Dn[fid][v]), t, t0), '-', color=C_MOD, lw=1.3, label='model')
        a.axvline(t0, color=C_REF, lw=0.6)
        a.set_title('%s (file %s)' % (title, fid), fontsize=7.3)
        a.set_ylabel('%s (deg/s)' % names[v], fontsize=7.5)
    for a in axs[1]:
        a.set_xlabel('time (s)')
    axs[0, 0].legend(frameon=False, fontsize=6.3, loc='lower left')
    fig.tight_layout(pad=0.3, w_pad=0.6, h_pad=0.6)
    save(fig, 'response')


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    which = sys.argv[1:] or ['geometry', 'trim', 'power', 'ge', 'response']
    for w in which:
        globals()['fig_' + w]()
        print('fig', w)
