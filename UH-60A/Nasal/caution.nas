# MASTER CAUTION (TM 1-1520-237-10 2.81): lights whenever an amber capsule of the caution/advisory panel goes on
# and stays on until pressed, so that a second caution lights it again. The capsules' flags come from
# Systems/uh60a-cap.xml. Written by fdm/cockpit/cap.py.

var CAUTIONS = ["fuel-low-1", "gen-1", "gen-2", "fuel-low-2", "fuel-press-1", "fuel-press-2", "oil-press-1", "conv-1", "conv-2", "oil-press-2", "oil-temp-1", "ac-ess", "oil-temp-2", "starter-1", "starter-2", "pri-servo-1", "hyd-pump-1", "hyd-pump-2", "pri-servo-2", "xmsn-oil-temp", "boost-servo", "stabilator", "sas-off", "flt-path-stab", "xmsn-oil-press"];
var lit = {};
var master = props.globals.getNode("/sim/model/uh60/master-caution", 1);
master.setBoolValue(0);

var update = func {
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
