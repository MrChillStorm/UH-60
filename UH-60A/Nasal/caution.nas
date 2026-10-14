# MASTER CAUTION (TM 1-1520-237-10 2.81): lights whenever an amber capsule of the caution/advisory panel goes on
# and stays on until pressed, so that a second caution lights it again. The capsules' flags come from
# Systems/uh60a-cap.xml, the AFCS's from here. Written by fdm/cockpit/cap.py.

var CAUTIONS = ["fuel-low-1", "gen-1", "gen-2", "fuel-low-2", "fuel-press-1", "fuel-press-2", "oil-press-1", "conv-1", "conv-2", "oil-press-2", "oil-temp-1", "ac-ess", "oil-temp-2", "starter-1", "starter-2", "pri-servo-1", "hyd-pump-1", "hyd-pump-2", "pri-servo-2", "ircm-inop", "xmsn-oil-temp", "boost-servo", "stabilator", "sas-off", "flt-path-stab", "mr-deice-fail", "tr-deice-fail", "ice", "xmsn-oil-press"];
var lit = {};
var master = props.globals.getNode("/sim/model/uh60/master-caution", 1);
master.setBoolValue(0);

# STABILATOR, SAS OFF (both SAS off, or no hydraulics below half rotor speed) and FLT PATH STAB. getprop, unlike a
# property rule or a listener, doesn't create JSBSim's switches before JSBSim has set them on.
var off = func(p) { var v = getprop(p); return v != nil and v < 0.5; };
var afcs = func {
    var C = "/sim/model/uh60/cap/";
    setprop(C ~ "stabilator", off("/fdm/jsbsim/stab/auto"));
    setprop(C ~ "sas-off", (off("/fdm/jsbsim/ctl/sas1") and off("/fdm/jsbsim/ctl/sas2"))
                           or (getprop("/rotors/main/rpm-pct") or 0) < 50);
    setprop(C ~ "flt-path-stab", off("/fdm/jsbsim/ctl/fps"));
};

var update = func {
    afcs();
    foreach (var c; CAUTIONS) {
        if (getprop("/sim/model/uh60/cap/" ~ c)) {
            if (!contains(lit, c)) {
                lit[c] = 1;
                master.setBoolValue(1);
            }
        } elsif (contains(lit, c)) {
            delete(lit, c);
        }
    }
};

var reset = func { master.setBoolValue(0); };

var timer = maketimer(0.25, update);
timer.start();
