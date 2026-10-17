"""Tables (report/tab/*.tex) and the numbers quoted in the text (report/tab/numbers.tex) from report/results/*.json.
python3 report/mktables.py"""
import sys, os, json, math, re
HERE = os.path.dirname(os.path.abspath(__file__))
FDM = os.path.dirname(HERE)
sys.path.insert(0, FDM)
import numpy as np
import uh60data as U
import engines as E
import dynamics as DY
from data import usaaefa_performance as PERF

OUT = os.path.join(HERE, 'tab')
RES = os.path.join(HERE, 'results')
N = {}                      # \N<name> macros


def res(name):
    return json.load(open(os.path.join(RES, name + '.json')))


def put(name, value, fmt='%.1f'):
    s = fmt % value if not isinstance(value, str) else value
    N[name] = '$-$' + s[1:] if s.startswith('-') else s


def write(name, body):
    # typographic minus for negative numbers
    body = re.sub(r'(?<=[\s&(])-(?=\d)', '$-$', body)
    open(os.path.join(OUT, 'tab_%s.tex' % name), 'w').write(body)


def sgn(x, fmt='%.1f'):
    s = fmt % x
    return s if s.startswith('-') else ('+' + s if float(s) != 0 else s.lstrip('-'))


# ----------------------------------------------------------------------------------------------- data
def tab_params():
    sigma = U.NB * U.CHORD / (math.pi * U.R)
    lock = 0.0023769 * 5.73 * U.CHORD * U.R ** 4 / U.I_B
    segs, edges = U.segments()
    rows = [('\\multicolumn{2}{@{}l}{\\emph{Main rotor} \\cite{gh}}', ''),
            ('radius; blades; chord', '%.2f ft; %d; %.2f ft' % (U.R, U.NB, U.CHORD)),
            ('solidity; Lock number (sea level)', '%.4f; %.1f' % (sigma, lock)),
            ('speed (100\\,\\%); tip speed', '%.1f rad/s; %.0f ft/s' % (U.OMEGA, U.OMEGA * U.R)),
            ('flap/lag hinge offset; aerodynamic root', '%.2f ft (%.1f\\,\\%%); %.2f ft' % (U.E_FLAP, 100 * U.E_FLAP / U.R, U.R_ROOT)),
            ('blade weight; flap inertia', '%.1f lb; %.0f slug\\,ft$^2$' % (U.W_BLADE, U.I_B)),
            ('airfoil; twist; tip', 'SC1095 (GenHel tables); $-18^\\circ$ nonlinear; $20^\\circ$ sweep outboard of %.3f\\,$R$' % U.TIP_SWEEP_R),
            ('segments', '%d of equal annulus area, mid-points %.2f\\ldots%.2f\\,$R$' % (len(segs), segs[0][0] / U.R, segs[-1][0] / U.R)),
            ('tip loss $B$; section drag increment', '%.2f; %.4f' % (U.TIP_LOSS, U.DCD)),
            ('shaft tilt; swashplate phase', '%.0f$^\\circ$ forward; $%.1f^\\circ$' % (math.degrees(U.SHAFT_TILT), U.SWASH_PHASE)),
            ('lag damper', 'nonlinear force--speed table \\cite{gh}, pickup kinematics'),
            ('\\multicolumn{2}{@{}l}{\\emph{Tail rotor} \\cite{gh,hb}}', ''),
            ('radius; blades; solidity', '%.1f ft; %d; %.4f' % (U.R_TR, U.NB_TR, U.SIGMA_TR)),
            ('speed; cant; $\\tan\\delta_3$', '%.2f rad/s; %.0f$^\\circ$; %.2f' % (U.OMEGA_TR, math.degrees(U.CANT_TR), U.K1_TR)),
            ('twist; tip loss; fin blockage', '$%.0f^\\circ$; %.2f; %.3f below %.0f kt' % (math.degrees(U.TWIST_TR), U.TIP_LOSS_TR, U.KBLK_TR, U.VBLK_TR)),
            ('\\multicolumn{2}{@{}l}{\\emph{Airframe} \\cite{gh,bal}}', ''),
            ('design gross weight; CG', '%.0f lb; FS %.0f, WL %.0f' % (U.GW_DESIGN, U.FS_CG, U.WL_CG)),
            ('inertias $I_{xx}, I_{yy}, I_{zz}, I_{xz}$', '%.0f, %.0f, %.0f, %.0f slug\\,ft$^2$' % (U.IXX, U.IYY, U.IZZ, U.IXZ)),
            ('stabilator; fin', '%.0f ft$^2$, $-8\\ldots{+39}^\\circ$ TED; %.1f ft$^2$' % (U.S_STAB, U.S_FIN)),
            ('\\multicolumn{2}{@{}l}{\\emph{Drive} \\cite{gh,perf}}', ''),
            ('engines; transmission', '2 $\\times$ T700-GE-700, 1553 shp; 2828 shp'),
            ('rotor inertia (hub, drive, blades)', '%.0f slug\\,ft$^2$' % E.J_ROTOR),
            ('accessories; gearbox loss', '%.1f hp; 3\\,\\%% of main rotor torque' % (E.ACCESSORY_FTLB * E.OMEGA0 / 550))]
    body = '\\begin{tabularx}{\\columnwidth}{@{}>{\\raggedright\\arraybackslash}p{3.0cm}X@{}}\n\\toprule\n'
    for a, b in rows:
        body += ('%s \\\\\n' % a) if a.startswith('\\multicolumn') else '%s & %s \\\\\n' % (a, b)
    body += '\\bottomrule\n\\end{tabularx}\n'
    write('params', body)
    put('Lock', lock); put('Sigma', sigma, '%.4f')


# ----------------------------------------------------------------------------------------------- model size
def tab_model():
    sysdir = os.path.join(FDM, '..', 'UH-60A', 'Systems')
    comp = re.compile(r'<(fcs_function|lag_filter|washout_filter|second_order_filter|actuator|pure_gain|summer|switch|kinematic|integrator)\b')
    rows, tot = [], 0
    for fn, what in (('uh60a-controls.xml', 'rigging, mixing, servos, stabilator schedule, SAS, pitch bias actuator, FPS'),
                     ('uh60a-rotor.xml', 'blade flap and lag, lag dampers, Peters--He wake, tail rotor inflow, elastic twist, wake turbulence'),
                     ('uh60a-drive.xml', 'two T700 engines and their fuel controls, rotor speed, accessories, rotor brake'),
                     ('uh60a-outputs.xml', 'FlightGear outputs: rotor animation, instruments, vibration cue')):
        n = len(comp.findall(open(os.path.join(sysdir, fn)).read())); tot += n
        rows.append((fn.replace('uh60a-', '').replace('.xml', ''), what, '%d' % n))
    aero = open(os.path.join(sysdir, 'uh60a-aero.xml')).read()
    nf = aero.count('<function name=')
    rows.append(('aero', 'blade segments (4 $\\times$ 8), tail rotor, fuselage, stabilator, fin, interference', '%d' % nf))
    body = '\\begin{tabularx}{\\columnwidth}{@{}lXr@{}}\n\\toprule\nfile & content & items \\\\\n\\midrule\n'
    for r in rows:
        body += '%s & %s & %s \\\\\n' % r
    body += '\\bottomrule\n\\end{tabularx}\n'
    write('model', body)
    put('nFunc', nf, '%d'); put('nFcs', tot, '%d')
    put('aeroMB', os.path.getsize(os.path.join(sysdir, 'uh60a-aero.xml')) / 1e6, '%.1f')


# ----------------------------------------------------------------------------------------------- level-flight trims
def tab_b1():
    R = res('b1')
    MV = [m['V'] for m in R['model']]
    at = lambda V: {k: float(np.interp(V, MV, [m[k] for m in R['model']])) for k in R['model'][0]}
    keys = (('lon', '%'), ('lat', '%'), ('coll', '%'), ('ped', '%'), ('theta', 'deg'), ('phi', 'deg'))
    body = '\\begin{tabular}{@{}r' + 'r' * 7 + '@{}}\n\\toprule\n'
    body += 'kt & lon & lat & coll & ped & $\\theta$ & $\\phi$ & power \\\\\n & \\multicolumn{4}{c}{\\% of travel} & \\multicolumn{2}{c}{deg} & \\% \\\\\n\\midrule\n'
    dev = {k: [] for k, _ in keys}; pdev = []
    for t in R['test']:
        m = at(t['V'])
        d = [m[k] - t[k] for k, _ in keys]
        p = 100 * (m['hp'] / t['hp'] - 1)
        low = t['V'] < 35
        body += '%d%s & ' % (t['V'], '$^*$' if low else '') + ' & '.join(sgn(x) for x in d) + ' & %s \\\\\n' % sgn(p, '%.0f')
        if not low:
            for (k, _), x in zip(keys, d):
                dev[k].append(x)
            pdev.append(p)
    body += '\\bottomrule\n\\end{tabular}\n'
    write('b1', body)
    for k, _ in keys:
        put('Bmax' + k, max(abs(x) for x in dev[k]))
    put('Bpowmin', min(pdev), '%.0f'); put('Bpowmax', max(pdev), '%.0f')
    m = at(162); t = [t for t in R['test'] if t['V'] == 162][0]
    put('Bcollhi', t['coll'] - m['coll']); put('Bthetahi', m['theta'] - t['theta'])
    m50 = at(50); t50 = [t for t in R['test'] if t['V'] == 50][0]
    # main rotor power against 79-24 with that aircraft's instrumentation drag (2.8 ft2 [perf]) taken off the test
    rho = 0.0023769 * (1 - 6.8756e-6 * 5250) ** 4.2559
    corr = [100 * (at(x['V'])['hp'] / (x['hp'] - 0.5 * rho * (x['V'] * 1.68781) ** 3 * 2.8 / 550) - 1) for x in R['test'] if x['V'] >= 72]
    put('Bpowcorr', max(abs(c) for c in corr))
    put('Bpowfifty', 100 * (m50['hp'] / t50['hp'] - 1), '%.0f')
    # pedal, both tail rotor models: 50-139 kt range of the error, and 162 kt
    for suf, tag in (('', ''), ('-bailey', 'Y')):
        Rm = res_opt('b1' + suf)
        if not Rm:
            continue
        MM = Rm['model']; mv = [m['V'] for m in MM]
        d = [float(np.interp(t['V'], mv, [m['ped'] for m in MM])) - t['ped'] for t in R['test'] if 50 <= t['V'] <= 139]
        put('B%spedmin' % tag, min(d)); put('B%spedmax' % tag, max(d))
        t162 = [t for t in R['test'] if t['V'] == 162][0]
        m162 = [m for m in MM if m['V'] == 162][0]
        put('B%spedhs' % tag, m162['ped'] - t162['ped']); put('B%spedhsres' % tag, m162['resid'], '%.2f')
    # the tail rotor pitch that pedal and collective give through the rigging and the collective-to-pedal mixing (the
    # model's own, 0.5 in bias included), test against both models at 139 and 162 kt
    def trp(ped, coll):
        ped_in = (ped / 100 - 0.5) * 2 * U.PED_IN
        return U.TR_CUFF_CENTER + U.TR_DEG_IN * (ped_in + U.PED_BIAS_IN + U.MIX_COLL_PED * (coll / 100 * U.COLL_IN - U.COLL_IN / 2))
    tt = {t['V']: t for t in R['test']}
    put('BtrpTlo', trp(tt[139]['ped'], tt[139]['coll'])); put('BtrpThi', trp(tt[162]['ped'], tt[162]['coll']))
    put('BtrpTrise', trp(tt[162]['ped'], tt[162]['coll']) - trp(tt[139]['ped'], tt[139]['coll']))
    for suf, tag in (('', 'M'), ('-bailey', 'Y')):
        Rm = res_opt('b1' + suf)
        if not Rm:
            continue
        mm = {m['V']: m for m in Rm['model']}
        put('Btrp%slo' % tag, trp(mm[139]['ped'], mm[139]['coll'])); put('Btrp%shi' % tag, trp(mm[162]['ped'], mm[162]['coll']))
        put('Btrp%srise' % tag, trp(mm[162]['ped'], mm[162]['coll']) - trp(mm[139]['ped'], mm[139]['coll']))


# ----------------------------------------------------------------------------------------------- lateral-directional
def tab_latdir():
    R = res('latdir'); M, T = R['model'], R['test']
    rows = []
    for V in (60, 100, 140):
        m, t = M['%d_level_b0_0' % V], T['%d_level' % V]
        rows.append(('%d kt level' % V, t['roll'], m['roll'], t['lat'], m['lat'], t['ped'], m['ped']))
    for nm, key in (('100 kt, 1500 fpm descent', 'descent'), ('100 kt, 1500 fpm climb', 'climb')):
        m, t = M['100_%s_b0_0' % key], T['100_' + key]
        rows.append((nm, t['roll'], m['roll'], None, m['lat'], t['ped'], m['ped']))
    for bl in (2.1, -4.1):
        m, t = M['100_level_b0_%g' % bl], T['100_bl%g' % bl]
        rows.append(('100 kt, CG BL %+.1f in' % bl, t['roll'], m['roll'], t['lat'], m['lat'], None, m['ped']))
    f = lambda x, fmt='%.1f': '--' if x is None else fmt % x
    body = '\\begin{tabular}{@{}lrrrrrr@{}}\n\\toprule\n & \\multicolumn{2}{c}{roll (deg)} & \\multicolumn{2}{c}{lat. cyclic (in)} & \\multicolumn{2}{c}{pedal (\\%)} \\\\\n'
    body += '\\cmidrule(lr){2-3}\\cmidrule(lr){4-5}\\cmidrule(l){6-7}\nzero sideslip & test & model & test & model & test & model \\\\\n\\midrule\n'
    for r in rows:
        body += '%s & %s & %s & %s & %s & %s & %s \\\\\n' % (r[0], f(r[1]), f(r[2]), f(r[3], '%.2f'), f(r[4], '%.2f'), f(r[5], '%.0f'), f(r[6], '%.0f'))
    body += '\\bottomrule\n\\end{tabular}\n\n\\smallskip\\footnotesize\n'
    wl = ', '.join('%d\\,kt %s' % (V, sgn(M['%d_level_wl_0' % V]['beta'])) for V in (60, 100, 140))
    body += 'Wings level, sideslip (deg, + wind from the right): model %s; 100\\,kt climb %s, descent %s. ' % (wl, sgn(M['100_climb_wl_0']['beta']), sgn(M['100_descent_wl_0']['beta']))
    body += 'Test: $-4$ at 67\\,KCAS to $-1$ at 150\\,KCAS \\cite{a7924}; $+5$ in IRP climbs, $-5$ in 1000\\,fpm descents \\cite{a85}.\n'
    write('latdir', body)
    lv = [M['%d_level_b0_0' % V]['roll'] - T['%d_level' % V]['roll'] for V in (60, 100, 140)]
    put('Lrollerr', max(abs(x) for x in lv))
    lat = [M['%d_level_b0_0' % V]['lat'] - T['%d_level' % V]['lat'] for V in (60, 100, 140)]
    put('Llaterr', max(abs(x) for x in lat), '%.2f')
    sw_t = T['100_descent']['roll'] - T['100_climb']['roll']; sw_m = M['100_descent_b0_0']['roll'] - M['100_climb_b0_0']['roll']
    put('Lswingtest', sw_t); put('Lswingmodel', sw_m)
    dl_t = T['100_bl2.1']['lat'] - T['100_bl-4.1']['lat']; dl_m = M['100_level_b0_2.1']['lat'] - M['100_level_b0_-4.1']['lat']
    put('Lcgtest', dl_t, '%.2f'); put('Lcgmodel', dl_m, '%.2f')


# ----------------------------------------------------------------------------------------------- power
def tab_power():
    Hv = res('hover'); L = res('level'); F = res('fm'); S = res('split')
    body = '\\begin{tabular}{@{}rrrrrr@{}}\n\\toprule\n'
    body += 'weight & $C_T$ & \\multicolumn{2}{c}{$C_P\\times10^5$} & ratio & tail rotor \\\\\n'
    body += 'lb & $\\times10^4$ & model & test & & ratio \\\\\n\\midrule\n'
    rin, rtr = [], []
    for h in Hv:
        t = PERF.hover_oge(h['ct']); r = h['cp'] / t; rt = h['cp_tr'] / PERF.tail_rotor_hover(h['ct_tr'])
        intr = h['ct_tr'] >= 0.0102
        body += '%s & %.1f & %.2f & %.2f & %.3f & %s \\\\\n' % ('{:,}'.format(h['w']).replace(',', '\\,'), h['ct'] * 1e4, h['cp'] * 1e5, t * 1e5, r,
                                                            ('%.2f' % rt) if intr else '(%.2f)' % rt)
        if 0.0050 <= h['ct'] <= 0.0080:
            rin.append(r)
        if intr:
            rtr.append(rt)
    body += '\\bottomrule\n\\end{tabular}\n'
    write('hover', body)
    put('Hmin', min(rin), '%.3f'); put('Hmax', max(rin), '%.3f')
    put('Hheavy', [h['cp'] / PERF.hover_oge(h['ct']) for h in Hv if h['w'] == 22000][0], '%.3f')
    put('TRmin', min(rtr), '%.2f'); put('TRmax', max(rtr), '%.2f')
    # level flight
    rr = []
    for k in ('65', '70', '80'):
        for row in L[k]:
            t = PERF.LEVEL[round(row['mu'], 2)].get(int(k))
            if t and row['resid'] < 0.05 and row['mu'] >= 0.12:
                rr.append(row['cp'] * 1e5 / t)
    put('Cmin', 100 * (min(rr) - 1), '%.0f'); put('Cmax', 100 * (max(rr) - 1), '%.0f')
    r22 = [row['cp'] * 1e5 / PERF.LEVEL[0.22][70] for row in L['70'] if abs(row['mu'] - 0.22) < 1e-6][0]
    put('Cratio', 100 * (r22 - 1), '%.0f')
    # figure of merit
    sig = U.NB * U.CHORD / (math.pi * U.R)
    mod = [(r['ct'] / sig, r['fm']) for r in F['model'] if r['resid'] < 0.05]
    mx, my = zip(*sorted(mod))
    dn = [abs(np.interp(x, mx, my) - y) for x, y in F['norman']]
    put('FMnorman', max(dn), '%.3f')
    dF = [abs(np.interp(x, mx, my) - y) for x, y in zip(F['bemt_cts'], F['bemt_annulus_F']) if mx[0] <= x <= mx[-1]]
    put('FMbemt', max(dF), '%.3f')
    dB = [np.interp(x, mx, my) - y for x, y in zip(F['bemt_cts'], F['bemt_uniform_B']) if mx[0] <= x <= mx[-1]]
    put('FMuniB', -np.mean(dB), '%.3f')
    # power budget
    hv = [s for s in S if s['kind'] == 'hover'][0]; c22 = [s for s in S if s['kind'] == 'cruise' and abs(s['mu'] - 0.22) < 0.01][0]
    c30 = [s for s in S if s['kind'] == 'cruise' and abs(s['mu'] - 0.30) < 0.01][0]
    body = '\\begin{tabular}{@{}lrrr@{}}\n\\toprule\n & hover & $\\mu$ 0.22 & $\\mu$ 0.30 \\\\\n\\midrule\n'
    body += 'weight (lb) & %s \\\\\n' % ' & '.join('%.0f' % s['weight'] for s in (hv, c22, c30))
    body += 'main rotor, induced (hp) & %s \\\\\n' % ' & '.join('%.0f' % s['induced_hp'] for s in (hv, c22, c30))
    body += '\\quad blade profile drag & %s \\\\\n' % ' & '.join('%.0f' % s['profile_hp'] for s in (hv, c22, c30))
    body += '\\quad propulsive (airframe drag) & %s \\\\\n' % ' & '.join('%.0f' % s['propulsive_hp'] for s in (hv, c22, c30))
    body += 'tail rotor & %s \\\\\n' % ' & '.join('%.0f' % s['tr_hp'] for s in (hv, c22, c30))
    body += 'gearbox and accessories & %s \\\\\n' % ' & '.join('%.0f' % s['loss_hp'] for s in (hv, c22, c30))
    body += '\\midrule\nengine shaft power (hp) & %s \\\\\n' % ' & '.join('%.0f' % s['shp'] for s in (hv, c22, c30))
    body += 'induced power factor $\\kappa$ & %s \\\\\n' % ' & '.join('%.2f' % s['kappa'] for s in (hv, c22, c30))
    body += 'airframe drag area (ft$^2$) & -- & %.1f & %.1f \\\\\n' % (c22['fe_ft2'], c30['fe_ft2'])
    body += 'download (\\%% of weight) & %.1f & -- & -- \\\\\n' % (100 * hv['download_lb'] / hv['weight'])
    body += '\\bottomrule\n\\end{tabular}\n'
    write('budget', body)
    put('Kh', hv['kappa'], '%.2f'); put('Kc', c22['kappa'], '%.2f'); put('Fe', c22['fe_ft2'])
    put('Download', 100 * hv['download_lb'] / hv['weight']); put('Profilec', 100 * c22['profile_hp'] / c22['mr_hp'], '%.0f')


# ----------------------------------------------------------------------------------------------- ground effect
def tab_ge():
    Eg = res('ge'); oge = [e for e in Eg if e['height'] == 100][0]['torque']
    tm = {5: 0.80, 10: 0.86, 20: 0.925, 40: 0.972}
    body = '\\begin{tabular}{@{}rrrr@{}}\n\\toprule\nwheel height (ft) & model & flight manual & difference \\\\\n\\midrule\n'
    dev = []
    for e in Eg:
        if e['height'] == 100:
            continue
        r = e['torque'] / oge; dev.append(r - tm[e['height']])
        body += '%.1f & %.3f & %.3f & %s \\\\\n' % (e['wheel_ft'], r, tm[e['height']], sgn(r - tm[e['height']], '%.3f'))
    body += '\\bottomrule\n\\end{tabular}\n'
    write('ge', body)
    put('GEmax', max(abs(d) for d in dev), '%.3f'); put('GEoge', oge)


# ----------------------------------------------------------------------------------------------- control responses
def tab_resp():
    Dn = res('dyn')
    body = '\\begin{tabular}{@{}llllrrrrr@{}}\n\\toprule\n & & & & \\multicolumn{3}{c}{peak (deg/s)} & \\multicolumn{2}{c}{rms error} \\\\\n'
    body += '\\cmidrule(lr){5-7}\\cmidrule(l){8-9}\nfile & condition & input & & test & GenHel & model & GenHel & model \\\\\n\\midrule\n'
    better, rg, rm_, n = 0, [], [], 0
    for fid, case in DY.D.items():
        t = np.array(case['t']); k, t0 = DY.primary(case); ax = DY.AXIS[k]
        w = (t >= t0) & (t <= t0 + 2.5)
        test, gh = DY.curve(case, ax, 'test'), DY.curve(case, ax, 'genhel')
        m = np.array(Dn[fid][ax])

        def pk(y):
            if y is None:
                return float('nan')
            y = DY.rel(y, t, t0)[w]
            return y[np.nanargmax(np.abs(y))] if np.isfinite(y).any() else float('nan')

        def rms(a, b):
            if a is None or b is None:
                return float('nan')
            return float(np.sqrt(np.nanmean((DY.rel(a, t, t0)[w] - DY.rel(b, t, t0)[w]) ** 2)))
        g, mm = rms(gh, test), rms(m, test)
        bad = fid == '509'          # digitized test pitch rate corrupt
        cond = case['condition'].replace(' kt', '\\,kt')
        inp = case['input'].split(',')[0].replace('in.', 'in').replace('-in', '\\,in').rstrip('.')
        f = lambda x: '--' if not np.isfinite(x) else '%.1f' % x
        body += '%s & %s & %s & $%s$ & %s & %s & %s & %s & %s \\\\\n' % (fid, cond, inp, ax, f(pk(test)) + ('$^\\dagger$' if bad else ''),
                                                                      f(pk(gh)), f(pk(m)), f(g), ('\\textbf{%s}' % f(mm)) if (np.isfinite(g) and mm < g) else f(mm))
        if np.isfinite(g) and np.isfinite(mm) and not bad:
            n += 1; better += mm < g; rg.append(g); rm_.append(mm)
        if fid == '209':
            put('Rpedrms', mm); put('Rpedrmsgh', g)
    body += '\\bottomrule\n\\end{tabular}\n'
    write('resp', body)
    # hover cyclic steps: peak overshoot of GenHel and the model over the test; hover pedal step
    over = {}
    for fid in ('201', '202', '204', '209'):
        case = DY.D[fid]; t_ = np.array(case['t']); k, t0 = DY.primary(case); ax = DY.AXIS[k]; w = (t_ >= t0) & (t_ <= t0 + 2.5)
        pk = lambda y: (lambda z: z[np.nanargmax(np.abs(z))])(DY.rel(y, t_, t0)[w])
        tp = pk(DY.curve(case, ax, 'test'))
        over[fid] = (100 * (pk(DY.curve(case, ax, 'genhel')) / tp - 1), 100 * (pk(np.array(Dn[fid][ax])) / tp - 1))
    cyc = [over[f] for f in ('201', '202', '204')]
    put('Rghlo', min(c[0] for c in cyc), '%.0f'); put('Rghhi', max(c[0] for c in cyc), '%.0f')
    put('Rmlo', min(c[1] for c in cyc), '%.0f'); put('Rmhi', max(c[1] for c in cyc), '%.0f'); put('Rped', over['209'][1], '%.0f')
    put('nAftCG', sum(1 for fid in DY.D if DY.COND[fid][1] > 358), '%d')
    put('Rbetter', better, '%d'); put('Rn', n, '%d'); put('Rgh', np.mean(rg), '%.2f'); put('Rmod', np.mean(rm_), '%.2f')
    # the 100 kt aft step
    case = DY.D['401']; t = np.array(case['t']); k, t0 = DY.primary(case); w = (t >= t0) & (t <= t0 + 2.5)
    for who, key in (('test', 'Qtest'), ('genhel', 'Qgh')):
        y = DY.rel(DY.curve(case, 'q', who), t, t0)[w]; put(key, np.nanmax(y))
    put('Qmod', np.nanmax(DY.rel(np.array(Dn['401']['q']), t, t0)[w]))


# ----------------------------------------------------------------------------------------------- hover yaw: fitted derivatives
def yaw_fit(case, y):
    """first-order fit r' = N_r r + N_ped ped over 3 s after the input: (N_r 1/s, N_ped deg/s2 per in)"""
    from scipy.optimize import least_squares
    t = np.array(case['t']); k, t0 = DY.primary(case)
    t_in, inc = DY.inputs(case); ped = np.interp(t, t_in, inc['ped'])
    y = DY.rel(np.array(y, float), t, t0); ok = (t >= t0 - 0.2) & (t <= t0 + 3.0) & np.isfinite(y)
    def sim(prm):
        r = np.zeros_like(t)
        for i in range(1, len(t)):
            r[i] = r[i - 1] + (t[i] - t[i - 1]) * (prm[0] * r[i - 1] + prm[1] * ped[i])
        return r
    f = least_squares(lambda prm: sim(prm)[ok] - y[ok], [-0.3, 20.0], bounds=([-3, 0], [0.5, 200]))
    return f.x


def tab_yawfit():
    case = DY.D['209']
    for tag, y in (('test', DY.curve(case, 'r', 'test')), ('gh', DY.curve(case, 'r', 'genhel')), ('mod', res('dyn')['209']['r'])) + \
            ((('bai', res_opt('dyn-bailey')['209']['r']),) if res_opt('dyn-bailey') else ()):
        nr, npd = yaw_fit(case, y)
        put('Yaw%sNr' % tag, nr, '%.2f'); put('Yaw%sNp' % tag, npd, '%.0f')


def tab_yawdamp():
    """quasi-static hover yaw derivatives by component [yawdamp.py]: case 209's hover, and TM-85890's 1-kt condition"""
    D = res('yawdamp')
    d = D['209']
    put('Ydr', d['Nr']['total'], '%.2f'); put('Ydrtr', d['Nr']['tr'], '%.2f'); put('Ydrfin', d['Nr']['fin'], '%+.2f')
    put('Ydrmr', d['Nr']['rest'], '%.2f'); put('Ydp', -d['Nped']['total'], '%.2f'); put('Ydv', d['Nv']['total'], '%.4f')
    h = D['hb']
    put('Ydhbr', h['Nr']['total'], '%.2f'); put('Ydhbv', h['Nv']['total'], '%.4f'); put('Ydhbp', -h['Nped']['total'], '%.2f')
    Db = res_opt('yawdamp-bailey')
    if Db:
        put('YdYr', Db['209']['Nr']['total'], '%.2f')


# ----------------------------------------------------------------------------------------------- derivatives
def tab_derivs():
    Dv = res('derivs'); G, Aa, Ap = Dv['genhel'], Dv['augmented']['d'], Dv['aero']['d']
    units = dict(Xu='1/s', Xw='1/s', Xq='ft/s/rad', Zu='1/s', Zw='1/s', Zq='ft/s/rad', Mu='rad/s/ft', Mw='rad/s/ft', Mq='1/s',
                 Xe='ft/s$^2$/in', Ze='ft/s$^2$/in', Me='rad/s$^2$/in', Xc='ft/s$^2$/in', Zc='ft/s$^2$/in', Mc='rad/s$^2$/in')
    body = '\\begin{tabular}{@{}llrrr@{}}\n\\toprule\n & & TM-85890 & \\multicolumn{2}{c}{model} \\\\\n\\cmidrule(l){4-5}\n & unit & \\cite{hb} & PBA, stab. & aero only \\\\\n\\midrule\n'
    for k in G:
        sym = '$%s_{%s}$' % (k[0], {'e': '\\delta_e', 'c': '\\delta_c'}.get(k[1], k[1]))
        fmt = '%.4f' if abs(G[k]) < 0.1 else '%.3f' if abs(G[k]) < 10 else '%.2f'
        body += '%s & %s & %s & %s & %s \\\\\n' % (sym, units[k], fmt % G[k], fmt % Aa[k], fmt % Ap[k])
    body += '\\bottomrule\n\\end{tabular}\n'
    write('derivs', body)
    put('Mwgh', G['Mw'], '%.4f'); put('Mwmod', Ap['Mw'], '%.4f'); put('Zwgh', G['Zw'], '%.2f'); put('Zwmod', Ap['Zw'], '%.2f')
    put('Zqgh', G['Zq'], '%.1f'); put('Zqmod', Ap['Zq'], '%.1f'); put('Mqgh', G['Mq'], '%.2f'); put('Mqmod', Ap['Mq'], '%.2f')


# ----------------------------------------------------------------------------------------------- low speed, all azimuths
def res_opt(name):
    f = os.path.join(RES, name + '.json')
    return json.load(open(f)) if os.path.exists(f) else None


def xwind_errors(R, chan, ac, data):
    """model minus test over all speeds and azimuths (trimmed points with test data)"""
    from data import usaaefa_lowspeed as L
    d = []
    for key, rows in R.items():
        a, V = key.split('_')
        if a != ac:
            continue
        test = data[(a, int(V))] if data else L.TR_HP.get(int(V))
        for r in rows:
            if chan not in r or r['resid'] > 0.05:
                continue
            t = test[L.AZ.index(r['az'])]
            if t is not None:
                d.append(r[chan] - t)
    return np.array(d)


def tab_xwind():
    from data import usaaefa_lowspeed as L
    cols = [(n, lab, res_opt(n)) for n, lab in (('xwind-gwash', 'GenHel wash'), ('xwind', 'blade-element'), ('xwind-bailey', 'Bailey'))]
    cols = [c for c in cols if c[2]]
    rows = (('pedal 716', 'ped', '716', L.PED, '%.2f', 'in'), ('pedal 717', 'ped', '717', L.PED, '%.2f', 'in'),
            ('long.\\ cyclic 716', 'lon', '716', L.LON, '%.2f', 'in'), ('long.\\ cyclic 717', 'lon', '717', L.LON, '%.2f', 'in'),
            ('TR power 716', 'tr_hp', '716', None, '%.0f', 'hp'))
    body = '\\begin{tabular}{@{}l' + 'rr' * len(cols) + '@{}}\n\\toprule\n'
    body += ' & ' + ' & '.join('\\multicolumn{2}{c}{%s}' % lab for n, lab, R in cols) + ' \\\\\n'
    body += ''.join('\\cmidrule(%s){%d-%d}' % ('lr' if i < len(cols) - 1 else 'l', 2 + 2 * i, 3 + 2 * i) for i in range(len(cols))) + '\n'
    body += ' & ' + ' & '.join('mean & rms' for c in cols) + ' \\\\\n\\midrule\n'
    for lab, ch, ac, data, fmt, unit in rows:
        cells = []
        for n, _, R in cols:
            d = xwind_errors(R, ch, ac, data)
            cells += [sgn(d.mean(), fmt), fmt % np.sqrt((d ** 2).mean())] if len(d) else ['--', '--']
            tag = {'xwind': 'B', 'xwind-bailey': 'Y', 'xwind-gwash': 'G'}[n]
            if len(d):
                put('X%s%s%s' % (tag, ch.replace('_', ''), {'716': 'C', '717': 'E'}[ac]), np.sqrt((d ** 2).mean()), fmt)   # C: Coyote Flats, E: Edwards
                put('X%s%smean%s' % (tag, ch.replace('_', ''), {'716': 'C', '717': 'E'}[ac]), d.mean(), fmt)
        body += '%s (%s) & %s \\\\\n' % (lab, unit, ' & '.join(cells))
    body += '\\bottomrule\n\\end{tabular}\n'
    write('xwind', body)
    for n, _, R in cols:
        tag = {'xwind': 'B', 'xwind-bailey': 'Y', 'xwind-gwash': 'G'}[n]
        d = np.concatenate([xwind_errors(R, 'ped', ac, L.PED) for ac in ('716', '717')])
        put('X%sped' % tag, np.sqrt((d ** 2).mean()), '%.2f')
        d = np.concatenate([xwind_errors(R, 'lon', ac, L.LON) for ac in ('716', '717')])
        put('X%slon' % tag, np.sqrt((d ** 2).mean()), '%.2f')


# ----------------------------------------------------------------------------------------------- tail rotor power, level flight
def tab_trpower():
    from data import yeo_trpower as Y
    for n, tag in (('trpower', 'B'), ('trpower-bailey', 'Y'), ('trpower5', 'F')):
        R = res_opt(n)
        if not R:
            continue
        for lo, hi, band in ((0.12, 0.20, 'lo'), (0.22, 0.32, 'mid'), (0.33, 0.38, 'hi')):
            mm, tt = [], []
            for e in R:
                if e['cw'] > 0.0092:
                    continue
                pts = [r for r in e['rows'] if r['resid'] < 0.05]
                tp = [(m, c) for m, c in Y.TR_POWER[e['cw']] if lo <= m <= hi]
                if not tp or len(pts) < 2:
                    continue
                for m, c in tp:
                    if pts[0]['mu'] <= m <= pts[-1]['mu']:
                        mm.append(np.interp(m, [r['mu'] for r in pts], [r['cp_tr'] for r in pts])); tt.append(c)
            if mm:
                put('TP%s%s' % (tag, band), np.mean(mm) / np.mean(tt), '%.1f')
                put('TP%s%sm' % (tag, band), 1e3 * np.mean(mm), '%.2f'); put('TP%s%st' % (tag, band), 1e3 * np.mean(tt), '%.2f')
                print('trpower %s %s: %d points, model %.2f test %.2f (x1e3)' % (tag, band, len(mm), 1e3 * np.mean(mm), 1e3 * np.mean(tt)))


# ----------------------------------------------------------------------------------------------- the two tail rotor models
def tab_tr():
    """the blade-element and Bailey tail rotors against every test that sees the tail rotor"""
    from data import usaaefa_lowspeed as L
    import harness as H
    sets = [('', 'blade-element'), ('-bailey', 'Bailey')]
    have = [(suf, lab) for suf, lab in sets if res_opt('b1' + suf)]
    if len(have) < 2:
        return
    rows = []
    # hover trim pedal of the five hovers of TM-88360 app. D (in from centre, + right; 6.0 in travel)
    appd = DY.D
    hov = [f for f in appd if appd[f]['condition'] == 'hover']
    def first_test(fid):
        c = appd[fid]; t = np.array(c['t']); y = np.array([np.nan if v is None else v for v in (c['ped']['test'] or [])], float)
        w = (t < 0.5) & np.isfinite(y) if len(y) else np.zeros(0, bool)
        return float(np.median(y[w])) if w.any() else np.nan
    tt = np.nanmean([first_test(f) for f in hov])
    cells = []
    for suf, _ in have:
        Dn = res_opt('dyn' + suf)
        cells.append(np.mean([3 * Dn[f]['trim']['ped'] for f in hov]) if Dn and 'trim' in Dn[hov[0]] else np.nan)
    rows.append(('hover trim pedal, 5 hovers (in, + right)', '%.2f' % tt, ['%.2f' % c for c in cells]))
    # level flight pedal 50-139 kt, 162 kt
    test = {t[0]: t[4] for t in H.BALLIN_B1}
    for lab, Vs in (('level-flight pedal 50--139 kt, mean error (\\%)', (50, 72, 95, 117, 139)), ('level-flight pedal 162 kt, error (\\%)', (162,))):
        cells = []
        for suf, _ in have:
            M = res('b1' + suf)['model']
            d = [np.interp(V, [m['V'] for m in M], [m['ped'] for m in M]) - test[V] for V in Vs]
            bad = any(m['resid'] > 0.05 for m in M if m['V'] in Vs)
            cells.append(sgn(np.mean(d)) + ('$^*$' if bad else ''))
        rows.append((lab, '--', cells))
    # zero-sideslip pedal (lat-dir sweeps)
    cells = []
    for suf, _ in have:
        R = res_opt('latdir' + suf)
        cells.append(sgn(np.mean([R['model']['%d_level_b0_0' % V]['ped'] - R['test']['%d_level' % V]['ped'] for V in (60, 100, 140)])) if R else '--')
    rows.append(('zero-sideslip pedal 60--140 kt, mean error (\\%)', '--', cells))
    # crosswinds
    cells = []
    for suf, _ in have:
        R = res_opt('xwind' + suf)
        d = np.concatenate([xwind_errors(R, 'ped', ac, L.PED) for ac in ('716', '717')]) if R else []
        cells.append('%.2f' % np.sqrt(np.mean(np.square(d))) if len(d) else '--')
    rows.append(('pedal in crosswinds 10--40 kt, rms (in)', '--', cells))
    cells = []
    for suf, _ in have:
        R = res_opt('xwind' + suf)
        d = xwind_errors(R, 'tr_hp', '716', None) if R else []
        cells.append(sgn(np.mean(d), '%.0f') if len(d) else '--')
    rows.append(('tail rotor power in crosswinds, mean error (hp)', '--', cells))
    # hover tail rotor power
    cells = []
    for suf, _ in have:
        Hv = res_opt('hover' + suf)
        r = [h['cp_tr'] / PERF.tail_rotor_hover(h['ct_tr']) for h in Hv if h['ct_tr'] >= 0.0102] if Hv else []
        cells.append('%.2f--%.2f' % (min(r), max(r)) if r else '--')
    rows.append(('tail rotor power, OGE hover, model/test', '--', cells))
    # pedal steps: yaw-rate rms error
    peds = [f for f in appd if 'pedal' in appd[f]['input']]
    def rms_case(fid, m):
        case = appd[fid]; t = np.array(case['t']); k, t0 = DY.primary(case); ax = DY.AXIS[k]
        w = (t >= t0) & (t <= t0 + 2.5); test = DY.curve(case, ax, 'test')
        return float(np.sqrt(np.nanmean((DY.rel(m, t, t0)[w] - DY.rel(test, t, t0)[w]) ** 2)))
    cells = []
    for suf, _ in have:
        Dn = res_opt('dyn' + suf)
        cells.append('%.1f' % np.mean([rms_case(f, np.array(Dn[f][DY.AXIS[DY.primary(appd[f])[0]]])) for f in peds]) if Dn else '--')
    gh = np.mean([rms_case(f, DY.curve(appd[f], DY.AXIS[DY.primary(appd[f])[0]], 'genhel')) for f in peds])
    rows.append(('yaw rate after %d pedal steps, mean rms (deg/s; GenHel %.1f)' % (len(peds), gh), '--', cells))
    body = '\\begin{tabular}{@{}>{\\raggedright\\arraybackslash}p{3.7cm}r' + 'r' * len(have) + '@{}}\n\\toprule\n'
    body += ' & test & ' + ' & '.join(lab for _, lab in have) + ' \\\\\n\\midrule\n'
    for lab, t, cells in rows:
        body += '%s & %s & %s \\\\\n' % (lab, t, ' & '.join(cells))
    body += '\\bottomrule\n\\end{tabular}\n'
    write('tr', body)


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    for f in (tab_params, tab_model, tab_b1, tab_latdir, tab_power, tab_ge, tab_resp, tab_derivs, tab_xwind, tab_trpower, tab_tr, tab_yawfit, tab_yawdamp):
        try:
            f()
        except FileNotFoundError as e:
            print('skipped', f.__name__, '(no results:', os.path.basename(e.filename) + ')')
    with open(os.path.join(OUT, 'numbers.tex'), 'w') as fh:
        for k, v in sorted(N.items()):
            fh.write('\\newcommand{\\N%s}{%s}\n' % (k, v))
    print('wrote %d numbers' % len(N))
