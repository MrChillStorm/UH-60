# UH-60A navigation displays, command instrument system and panel systems, after TM 1-1520-237-10:
#
# - 3.25: the pilot's and copilot's VSI/HSI mode selectors, the CIS mode selector and the CIS processor (CISP),
#   which drives both VSIs' cyclic roll and pitch command bars and the collective position indicator;
# - the HSIs (compass card, course, deviation, No. 1 and No. 2 bearing pointers, KM) and VSIs (deviation pointers,
#   flags, DH and MB lights) fed from the selected sources;
# - 2.54 blade deice (control and test panels, ice rate meter), 4.8 IRCM, the panel lighting dimmers.
#
# Receivers: one VOR/ILS receiver (FlightGear's nav[0]: the course of the HSI that has CRS HDG control sets it), the
# LF/ADF (adf[0]) and, for the Doppler/GPS navigation set, FlightGear's GPS: its active waypoint is the FLY-TO
# destination. FM homing has no FlightGear counterpart, so its NAV flag stays in view. There is no icing in
# FlightGear: the ice detector reads zero except in its test. Power: the battery switch stands in for the ac and dc
# buses (the model has no APU).
#
# Commands are normalised to -1..1 (full deflection). Roll bar: 10 deg of roll error; pitch bar: 5 deg of pitch;
# collective pointer: 100 ft of altitude (with 6 s of vertical speed) or 400 ft/min against the glide slope.

var U = "/instrumentation/uh60/";
var DT = 1 / 30;                     # the step, measured each update from the simulation's elapsed time
var last_t = nil;

var val = func(p, d = 0) { var v = getprop(p); var x = (v == nil) ? nil : num(v); x == nil ? d : x; };
var b = func(p) { getprop(p) ? 1 : 0; };
var clamp = func(x, lo, hi) { x < lo ? lo : (x > hi ? hi : x); };
var wrap = func(a) { while (a > 180) a -= 360; while (a < -180) a += 360; a; };
var lag = func(y, x, tau) { y + (x - y) * clamp(DT / tau, 0, 1); };
var pwr = func { b("/controls/electric/battery-switch"); };

# a knob or switch one step along (the cockpit's clickable controls)
var step = func(p, d, lo, hi) { setprop(p, clamp(val(p) + d, lo, hi)); };

# ---------------------------------------------------------------- VSI/HSI mode selectors (3.25.4.1, 3.25.4.14-17)

var modesel = func(n, key) {
    if (!pwr()) return;
    var m = U ~ "modesel[" ~ n ~ "]/";
    if (key == "dplr" or key == "vor-ils") {
        var on = !b(m ~ key);
        setprop(m ~ key, on);
        if (on) setprop(m ~ (key == "dplr" ? "vor-ils" : "dplr"), 0);
    } elsif (key == "crs-hdg") {
        setprop(U ~ "crs-hdg-cplt", !b(U ~ "crs-hdg-cplt"));
    } else {
        setprop(m ~ key, !b(m ~ key));      # back-crs, fm-home, turn-altr, vgyro-altr, brg2-vor
    }
    if (n == 0 and cis.nav) nav_check();
};

var source = func(n, vsi) {
    # what a mode selector sends: FM homing goes to the VSI only, the HSI keeps the other selection
    var m = U ~ "modesel[" ~ n ~ "]/";
    if (vsi and b(m ~ "fm-home")) return "fm";
    if (b(m ~ "dplr")) return "dplr";
    if (b(m ~ "vor-ils")) return "vor";
    return "";
};

# ---------------------------------------------------------------- navigation data

var nav = {};

var read_nav = func {
    var hdg = val("/orientation/heading-magnetic-deg");
    nav.hdg = hdg;
    var N = "/instrumentation/nav[0]/";
    nav.vor_ok = pwr() and b(N ~ "in-range");
    nav.loc = b(N ~ "nav-loc");
    nav.gs_ok = nav.vor_ok and nav.loc and b(N ~ "has-gs") and b(N ~ "gs-in-range");
    nav.dev = val(N ~ "heading-needle-deflection-norm");                     # + : course to the right
    nav.gs = val(N ~ "gs-needle-deflection-norm");                           # + : glide slope above
    nav.to = b(N ~ "to-flag"); nav.from = b(N ~ "from-flag");
    var re = wrap(val(N ~ "radials/actual-deg") - val(N ~ "radials/selected-deg"));
    nav.radial_err = abs(re) > 90 ? wrap(re + 180) : re;                   # flying to the station
    nav.vor_brg = val(N ~ "radials/reciprocal-radial-deg");
    nav.gs_roc = val(N ~ "gs-rate-of-climb-fpm", val(N ~ "gs-rate-of-climb"));
    var G = "/instrumentation/gps/wp/wp[1]/";
    var id = getprop(G ~ "ID");
    nav.gps_ok = pwr() and id != nil and id != "" and val(G ~ "distance-nm", -1) >= 0;
    nav.gps_brg = val(G ~ "bearing-mag-deg");
    nav.gps_km = val(G ~ "distance-nm") * 1.852;
    nav.gps_crs = val(G ~ "desired-course-deg", nav.gps_brg);
    # Doppler/GPS course sensitivity: full scale 1000 m beyond 12 km, down to 200 m at 2 km and closer
    var fs = clamp(200 + (nav.gps_km - 2) * 80, 200, 1000);
    nav.gps_xtk = val(G ~ "course-error-nm") * 1852;                         # m, + : course to the right
    nav.gps_dev = clamp(nav.gps_xtk / fs, -1, 1);
    nav.adf_ok = pwr() and b("/instrumentation/adf/in-range");
    nav.adf_brg = hdg + val("/instrumentation/adf/indicated-bearing-deg");
};

var lateral = func(src, n) {
    # course deviation (-1..1, + : fly right), to-from and validity of a source
    if (src == "vor") {
        var s = (b(U ~ "modesel[" ~ n ~ "]/back-crs") and nav.loc) ? -1 : 1;
        return [s * nav.dev, nav.to, nav.from, nav.vor_ok];
    }
    if (src == "dplr")
        return [nav.gps_dev, nav.gps_ok, 0, nav.gps_ok];
    return [0, 0, 0, 0];
};

# ---------------------------------------------------------------- CIS processor (3.25.4.2 - 3.25.4.13)

var cis = {hdg: 0, nav: 0, alt: 0, src: "", ils: 0, captured: 0, app: 0, level: 0, passage: 0, last_to: 0,
           alt_ref: 0, ias_ref: 0, theta_lp: 0, coll_lp: 0, roll: 0, pitch: 0, coll: 0};

var ias = func { val("/instrumentation/airspeed-indicator/indicated-speed-kt"); };
var alt = func { val("/instrumentation/altimeter/indicated-altitude-ft"); };

var nav_check = func {
    # NAV needs data from the pilot's mode selector; without it the CISP stays off
    var s = source(0, 1);
    var ok = (s == "vor" and nav.vor_ok) or (s == "dplr" and nav.gps_ok);
    if (!ok) { cis.nav = 0; return 0; }
    if (s != cis.src) { cis.src = s; cis.captured = 0; cis.app = 0; }
    return 1;
};

var press = func(key) {
    if (!pwr()) return;
    if (key == "hdg") {
        if (cis.hdg and !cis.nav) cis.hdg = 0;
        else { cis.hdg = 1; cis.nav = 0; cis.app = 0; }
    } elsif (key == "nav") {
        if (cis.nav) {
            cis.nav = 0; cis.hdg = 0; cis.app = 0; cis.level = 0;
        } else {
            cis.src = ""; cis.nav = 1;
            if (!nav_check()) return;
            cis.hdg = 0; cis.captured = 0; cis.app = 0; cis.passage = 0; cis.last_to = nav.to;
            cis.ils = (cis.src == "vor" and nav.loc);
            if (cis.ils) { cis.ias_ref = ias(); cis.alt = 1; cis.alt_ref = alt(); }
        }
    } elsif (key == "alt") {
        if (cis.alt) { cis.alt = 0; cis.level = 0; }
        elsif (!(cis.nav and cis.ils)) { cis.alt = 1; cis.alt_ref = alt(); }   # not by hand during an ILS
    }
};

var step_cis = func {
    if (!pwr()) {
        cis.hdg = 0; cis.nav = 0; cis.alt = 0; cis.app = 0; cis.level = 0;
    }
    var c = b(U ~ "crs-hdg-cplt");
    var crs = val(U ~ "hsi[" ~ c ~ "]/course-deg");
    var bug = c ? val(U ~ "hsi[1]/heading-bug-deg") : val("/autopilot/settings/heading-bug-deg");
    var phi = val("/orientation/roll-deg"); var theta = val("/orientation/pitch-deg");
    var vs = val("/velocities/vertical-speed-fps");
    var coll = val("/fdm/jsbsim/sens/coll-pct");
    cis.coll_lp = lag(cis.coll_lp, coll, 3.0);
    cis.theta_lp = lag(cis.theta_lp, theta, 10.0);
    var roll_vis = 0; var pitch_vis = 0; var coll_vis = 0;
    var phi_cmd = 0; var lim = 20;
    var hdg_cmd = func(h) { clamp(wrap(h - nav.hdg), -20, 20); };            # 1 deg of roll per deg, 20 deg limit

    if (cis.nav) nav_check();
    if (cis.nav) {
        roll_vis = 1;
        if (cis.src == "dplr") {
            # straight wind-corrected line to the destination; over it (0.2 km) the CISP drops to heading mode
            if (nav.gps_km < 0.2) { cis.nav = 0; cis.hdg = 1; }
            phi_cmd = hdg_cmd(nav.gps_crs + clamp(45 * nav.gps_dev, -45, 45));
        } else {
            var dev = lateral("vor", 0)[0];
            if (!cis.captured) {
                # beyond the capture (15 deg of radial, two dots of localizer) the intercept is flown in heading mode
                var near = cis.ils ? abs(nav.dev) < 0.8 : abs(nav.radial_err) < 15;
                if (near) { cis.captured = 1; cis.hdg = 0; }
                else { cis.hdg = 1; phi_cmd = hdg_cmd(bug); }
            }
            if (cis.captured) {
                if (!cis.ils and nav.to != cis.last_to) cis.passage = 30;           # VOR station passage
                if (cis.passage > 0) {
                    cis.passage -= DT; phi_cmd = hdg_cmd(crs);
                } else {
                    var k = cis.ils ? 60 : 45 / 0.6;
                    phi_cmd = hdg_cmd(crs + clamp(k * dev, -45, 45));
                }
            }
            cis.last_to = nav.to;
            if (cis.ils) {
                # pitch bar: hold the airspeed at engagement; collective: altitude, then the glide slope
                pitch_vis = 1;
                var theta_cmd = cis.theta_lp + clamp((ias() - cis.ias_ref) * 0.5, -8, 8);
                cis.pitch = lag(cis.pitch, clamp((theta_cmd - theta) / 5, -1, 1), 0.3);
                if (!cis.app and nav.gs_ok and cis.captured and abs(nav.gs) < 0.15) { cis.app = 1; cis.alt = 0; }
                if (cis.app) {
                    lim = 15; coll_vis = 1;
                    var vs_req = nav.gs_roc + 400 * nav.gs;                  # fpm: the glide slope, closing the error
                    cis.coll = lag(cis.coll, clamp((vs * 60 - vs_req) / 400 + (coll - cis.coll_lp) / 10, -1, 1), 0.3);
                }
            }
        }
        # level-off: below the higher of the two radar altimeters' LO bugs in the NAV modes, hold that height
        var ralt = val("/sim/model/uh60/radar-alt-ft"); var lo = lo_bug();
        if (!cis.app and lo > 0 and ralt < lo) { cis.level = 1; cis.alt = 1; }
    } elsif (cis.hdg) {
        roll_vis = 1; phi_cmd = hdg_cmd(bug);
    }
    if (cis.alt and !cis.app) {
        coll_vis = 1;
        var err = cis.level ? lo_bug() - val("/sim/model/uh60/radar-alt-ft")
                            : cis.alt_ref - alt();
        var scale = cis.level ? 50 : 100;
        cis.coll = lag(cis.coll, clamp((6 * vs - err) / scale + (coll - cis.coll_lp) / 10, -1, 1), 0.3);
    }
    cis.roll = lag(cis.roll, clamp((clamp(phi_cmd, -lim, lim) - phi) / 10, -1, 1), 0.2);
    setprop(U ~ "cis/hdg-on", cis.hdg); setprop(U ~ "cis/nav-on", cis.nav); setprop(U ~ "cis/alt-on", cis.alt);
    setprop(U ~ "cis/roll-cmd", cis.roll); setprop(U ~ "cis/pitch-cmd", cis.pitch); setprop(U ~ "cis/coll-cmd", cis.coll);
    setprop(U ~ "cis/roll-vis", roll_vis); setprop(U ~ "cis/pitch-vis", pitch_vis); setprop(U ~ "cis/coll-vis", coll_vis);
    # the receiver follows the course selector of the HSI with CRS HDG control
    setprop("/instrumentation/nav[0]/radials/selected-deg", crs);
};

# ---------------------------------------------------------------- HSIs and VSIs

var displays = func {
    var p = pwr();
    for (var n = 0; n < 2; n += 1) {
        var H = U ~ "hsi[" ~ n ~ "]/";
        var hs = lateral(source(n, 0), n);
        setprop(H ~ "dev-norm", hs[0]); setprop(H ~ "to", hs[3] and hs[1]); setprop(H ~ "from", hs[3] and hs[2]);
        setprop(H ~ "nav-flag", !hs[3]); setprop(H ~ "hdg-flag", !p);
        # bearing pointers: without a signal they park at 3 o'clock
        setprop(H ~ "brg1-rel", nav.gps_ok ? wrap(nav.gps_brg - nav.hdg) : 90); setprop(H ~ "brg1-vis", nav.gps_ok);
        var vor2 = b(U ~ "modesel[" ~ n ~ "]/brg2-vor");
        var ok2 = vor2 ? (nav.vor_ok and !nav.loc) : nav.adf_ok;
        setprop(H ~ "brg2-rel", ok2 ? wrap((vor2 ? nav.vor_brg : nav.adf_brg) - nav.hdg) : 90);
        setprop(H ~ "brg2-vis", ok2);
        setprop(H ~ "km", clamp(nav.gps_km, 0, 9999)); setprop(H ~ "km-vis", nav.gps_ok);
        var V = U ~ "vsi[" ~ n ~ "]/";
        var src = source(n, 1);
        var vs = lateral(src, n);
        setprop(V ~ "dev-norm", vs[0]); setprop(V ~ "nav-flag", !vs[3]);
        var gs = src == "vor" and nav.gs_ok;
        setprop(V ~ "gs-norm", gs ? nav.gs : 0); setprop(V ~ "gs-flag", !gs);
        setprop(V ~ "att-flag", !p); setprop(V ~ "cmd-flag", !p);
        setprop(V ~ "dh", b(U ~ "radalt[" ~ n ~ "]/lo-light"));               # with the radar altimeter's LO light
        setprop(V ~ "mb", p and (b("/instrumentation/marker-beacon/outer") or b("/instrumentation/marker-beacon/middle")
                                 or b("/instrumentation/marker-beacon/inner")));
        # turn rate: one needle width for a 4-minute turn (1.5 deg/s)
        setprop(V ~ "turn-widths", p ? clamp(val("/orientation/yaw-rate-degps") / 1.5, -3, 3) : 0);
    }
};

# ---------------------------------------------------------------- radar altimeters AN/APN-209 (3.29)
# Each pilot's own: LO SET turns it on (counterclockwise past 0: OFF) and sets the L bug and his LO light, HI SET the H
# bug and HI light; pushed, it tests (1000 ft, HI on, LO off, no flag). About 2 minutes of warm-up after power comes on;
# above 1500 ft and when off or not tracking the pointer is out of sight behind the mask.

var ra = {on_t: [999, 999], was: [1, 1]};
var lo_bug = func {
    var lo = -1;
    foreach (var n; [0, 1]) {
        var v = val(U ~ "radalt[" ~ n ~ "]/lo-ft", -10);
        if (v > lo) lo = v;
    }
    lo;
};
var radalt_bug = func(n, which, dir) {
    var R = U ~ "radalt[" ~ n ~ "]/" ~ which ~ "-ft";
    var v = val(R, which == "lo" ? 80 : 800);
    var st = (v + dir * 10 < 200 or (v <= 200 and dir < 0)) ? 10 : 50;
    v = clamp(v + dir * st, which == "lo" ? -10 : 0, 1500);
    setprop(R, which == "lo" and v < 0 ? -10 : v);
};
var step_radalt = func {
    var h = val("/sim/model/uh60/radar-alt-ft");
    foreach (var n; [0, 1]) {
        var R = U ~ "radalt[" ~ n ~ "]/";
        var lo = val(R ~ "lo-ft", 80); var hi = val(R ~ "hi-ft", 800);
        var on = pwr() and lo >= 0;
        if (on and !ra.was[n]) ra.on_t[n] = 0;
        ra.was[n] = on;
        if (on) ra.on_t[n] += DT;
        var warm = ra.on_t[n] >= 120;
        var test = on and b(R ~ "test");
        var ft = test ? 1000 : h;
        var vis = on and (test or (warm and h <= 1500));
        setprop(R ~ "pointer-ft", ft); setprop(R ~ "pointer-vis", vis);
        setprop(R ~ "digits", math.round(ft)); setprop(R ~ "digits-vis", vis);
        setprop(R ~ "off-flag", !(test or (on and warm)));
        setprop(R ~ "lo-light", on and !test and warm and h <= lo);
        setprop(R ~ "hi-light", on and (test or (warm and h > hi)));
    }
};

# ---------------------------------------------------------------- blade deice and ice rate meter (2.54), IRCM (4.8)

var D = "/controls/anti-ice/blade-deice/";
var ice = {test: 0, meter_test: 0, ircm_cool: 0, ircm_was: 0};

var deice_power = func(pos) {        # 0 OFF, 1 ON, 2 TEST: one test cycle, TEST IN PROGRESS for 105-135 s
    if (pos == 2 and val(D ~ "power") != 2) ice.test = 120;
    setprop(D ~ "power", pos);
};
var ice_meter_test = func { if (pwr()) ice.meter_test = 75; };
var deice_step = func(dir) {         # the POWER switch from the bottom up: TEST, OFF, ON
    var order = [2, 0, 1]; var i = 1;
    forindex (var k; order) if (order[k] == val(D ~ "power")) i = k;
    deice_power(order[clamp(i + dir, 0, 2)]);
};

var step_systems = func {
    var p = pwr();
    # ice rate meter: the detector reads no ice; PRESS TO TEST drives it to half scale for about 50 s, then to zero
    var lwc = 0;
    if (ice.meter_test > 0) { ice.meter_test -= DT; lwc = ice.meter_test > 25 ? 1.0 : 0; }
    var m = val(D ~ "meter-lwc");
    setprop(D ~ "meter-lwc", lag(m, p ? lwc : 0, 2.0));
    setprop(U ~ "ice-flag", !p);
    var detected = p and !val(D ~ "power") and ice.meter_test > 25 and ice.meter_test < 60;  # from 15 s into the test
    setprop("/sim/model/uh60/cap/ice", detected);
    # deice test cycle: TEST IN PROGRESS, MAIN RTR monitor for the first 3 s; the test panel's failures
    var sel = val(D ~ "test-select");       # 0 NORM, 1 SYNC 1, 2 SYNC 2, 3 OAT, 4 EOT
    var testing = p and val(D ~ "power") == 2;
    if (!testing) ice.test = 0;
    if (ice.test > 0) ice.test -= DT;
    setprop(D ~ "test-in-progress", testing and sel == 0 and ice.test > 0);
    setprop(D ~ "pwr-main", p and (b(D ~ "press-pwr") or (testing and sel == 0 and ice.test > 117)));
    setprop(D ~ "pwr-tail", p and b(D ~ "press-pwr"));
    var eot = p and sel == 4 and val(D ~ "power") == 1 and val(D ~ "mode") == 3;     # EOT with POWER ON, MODE M
    setprop("/sim/model/uh60/cap/mr-deice-fail", (testing and sel >= 1 and sel <= 3) or eot);
    setprop("/sim/model/uh60/cap/tr-deice-fail", (testing and sel == 3) or eot);
    # IRCM: turned off after running, it cools down for 60 s with IRCM INOP on
    var on = p and b("/controls/ircm/on");
    if (ice.ircm_was and !on and p) ice.ircm_cool = 60;
    ice.ircm_was = on;
    if (ice.ircm_cool > 0) ice.ircm_cool -= DT;
    setprop("/sim/model/uh60/cap/ircm-inop", p and ice.ircm_cool > 0);
};

# ---------------------------------------------------------------- instrument and panel lights
# TM 2.70.3: NVG blue-green bezel lights on three circuits with their INSTR LT rheostats (OFF-BRT, 0-1 here):
# /controls/lighting/instr-plt-flt, instr-cplt-flt and instr-non-flt (Aircraft menu > Cockpit lights). The pilot's
# flight instruments and the nonflight ones are on the No. 2 ac primary bus, the copilot's on No. 1; the generators
# feed them, off line on the ground below 93.75 % rotor speed (the simulation has no APU or external power). The radar
# altimeters' lights dim besides with RAD ALT DIMMING. Levels for the instruments' XML in /instrumentation/uh60/lights.
#
# TM 2.81.2, 2.70.4: the caution/advisory and master warning lights are bright, or dim once the BRT/DIM-TEST switch
# goes to BRT/DIM with the PILOT FLT control out of OFF (again: bright); without dc power they return to bright. Dim,
# the NVG dimming panel's MA WRN and CAUT/ADVSY controls set them, and the cockpit indicator lights dim with the
# caution/advisory panel; the lighted switches (mode selectors) with the LIGHTED SWITCHES control. The radar altimeters'
# LO/HI lights and digits here likewise, with RAD ALT DIMMING. The material animations take the 0-1 levels; the HDR
# pipeline (Effects/Lit) takes cd/m2, in lights/hdr: bright LAMP_BRT, dim DIM_LO to DIM_HI over the control's travel,
# the display units' lamps from VIDS_NIGHT to VIDS_DAY with the light outside (their photocells, TM 2.10.3).
#
# TM 2.70.2, secondary lighting (HDR only: spot lights in Panels/panel.xml): the glareshield lights, GLARESHIELD
# LIGHTS (/controls/lighting/glareshield, OFF-BRT), No. 1 ac primary bus; the cockpit floodlights, BLUE/OFF/WHITE
# (/controls/lighting/cockpit-flood: 1, 0, -1), dc essential bus.
#
# The flight starts with every dimmer of the Cockpit lights dialog at its first notch (0.05), the floods off and the
# caution panel DIM.
var LT = U ~ "lights/";
var LAMP_BRT = 600;
var DIM_LO = 0.05; var DIM_HI = 2;            # dim, at most as bright as the instruments' markings at BRT
var VIDS_NIGHT = 3; var VIDS_DAY = 400;
var CAP_DIM = "/sim/model/uh60/cap-dim";
var FIRST_NOTCH = 0.05;
var dc_was = 0;

var brt_dim = func {         # the BRT/DIM-TEST switch to BRT/DIM
    if (b("/controls/electric/battery-switch") and val("/controls/lighting/instr-plt-flt") > 0)
        setprop(CAP_DIM, !b(CAP_DIM));
};

var lights = func {
    var wow = b("/gear/gear[0]/wow") or b("/gear/gear[1]/wow") or b("/gear/gear[2]/wow");
    var ac = !(wow and val("/rotors/main/rpm-pct") < 93.75);
    var knob = func(k) { ac ? clamp(val("/controls/lighting/instr-" ~ k), 0, 1) : 0; };
    var plt = knob("plt-flt"); var cplt = knob("cplt-flt");
    setprop(LT ~ "plt-flt", plt);
    setprop(LT ~ "cplt-flt", cplt);
    setprop(LT ~ "non-flt", knob("non-flt"));
    var ra = clamp(val("/controls/lighting/radalt-norm", 1), 0, 1);
    setprop(LT ~ "radalt[0]", plt * ra);
    setprop(LT ~ "radalt[1]", cplt * ra);
    var dc = b("/controls/electric/battery-switch");
    if (dc_was and !dc) setprop(CAP_DIM, 0);              # power cut: bright when it comes back
    dc_was = dc;
    var dim = b(CAP_DIM);
    setprop(LT ~ "cap-mode", dim ? "DIM" : "BRT");
    foreach (var k; [["ma-wrn", "ma-wrn-norm"], ["caut-advsy", "caut-advsy-norm"], ["switches", "lighted-switches"],
                     ["radalt-lamps", "radalt-norm"]]) {
        var f = clamp(val("/controls/lighting/" ~ k[1], 1), 0, 1);
        setprop(LT ~ k[0], dim ? f : 1);
        setprop(LT ~ "hdr/" ~ k[0], dim ? DIM_LO * math.pow(DIM_HI / DIM_LO, f) : LAMP_BRT);
    }
    var day = clamp((90 - val("/sim/time/sun-angle-rad") * R2D + 6) / 12, 0, 1);     # sun 6 deg below to above
    setprop(LT ~ "hdr/vids", VIDS_NIGHT * math.pow(VIDS_DAY / VIDS_NIGHT, day));
    setprop(LT ~ "glareshield", ac ? clamp(val("/controls/lighting/glareshield"), 0, 1) : 0);
    var flood = b("/controls/electric/battery-switch") ? val("/controls/lighting/cockpit-flood") : 0;
    setprop(LT ~ "flood-blue", flood > 0.5);
    setprop(LT ~ "flood-white", flood < -0.5);
};

# ---------------------------------------------------------------- init and loop

var init = func {
    setprop(U ~ "hsi[0]/course-deg", val("/instrumentation/nav[0]/radials/selected-deg"));
    setprop(U ~ "hsi[1]/course-deg", val("/instrumentation/nav[0]/radials/selected-deg"));
    setprop(U ~ "hsi[1]/heading-bug-deg", val("/autopilot/settings/heading-bug-deg"));
    foreach (var n; [0, 1])
        foreach (var k; ["dplr", "vor-ils", "back-crs", "fm-home", "turn-altr", "vgyro-altr", "brg2-vor"])
            if (getprop(U ~ "modesel[" ~ n ~ "]/" ~ k) == nil) setprop(U ~ "modesel[" ~ n ~ "]/" ~ k, 0);
    if (getprop(U ~ "crs-hdg-cplt") == nil) setprop(U ~ "crs-hdg-cplt", 0);
    foreach (var k; ["power", "mode", "test-select", "meter-lwc", "press-pwr"])
        if (getprop(D ~ k) == nil) setprop(D ~ k, 0);
    if (getprop("/controls/ircm/on") == nil) setprop("/controls/ircm/on", 0);
    foreach (var n; [0, 1]) {         # radar altimeters on, L bug 80 ft, H bug 800 ft (TM 3.29.3)
        if (getprop(U ~ "radalt[" ~ n ~ "]/lo-ft") == nil) setprop(U ~ "radalt[" ~ n ~ "]/lo-ft", 80);
        if (getprop(U ~ "radalt[" ~ n ~ "]/hi-ft") == nil) setprop(U ~ "radalt[" ~ n ~ "]/hi-ft", 800);
    }
    foreach (var k; ["ma-wrn-norm", "caut-advsy-norm", "radalt-norm"])
        if (getprop("/controls/lighting/" ~ k) == nil) setprop("/controls/lighting/" ~ k, 1);
    foreach (var k; ["instr-plt-flt", "instr-cplt-flt", "instr-non-flt", "lighted-switches", "glareshield"])
        if (getprop("/controls/lighting/" ~ k) == nil) setprop("/controls/lighting/" ~ k, FIRST_NOTCH);
    if (getprop("/controls/lighting/cockpit-flood") == nil) setprop("/controls/lighting/cockpit-flood", 0);
    if (getprop(CAP_DIM) == nil) setprop(CAP_DIM, 1);
};

var update = func {
    var t = val("/sim/time/elapsed-sec");
    DT = last_t == nil ? 1 / 30 : clamp(t - last_t, 0, 0.5);
    last_t = t;
    read_nav();
    step_cis();
    step_radalt();
    displays();
    step_systems();
    lights();
};

init();
var timer = maketimer(DT, update);
timer.start();
