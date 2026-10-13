# UH-60A: what the JSBSim model does not do. The rotor, engines, drive train, flight controls, SAS and
# stabilator all run in the FDM (fdm/genuh60.py); this file only has the autostart/autostop sequencer, skid
# sounds and the dynamic view. Based on the bo105/uh60 files by Melchior FRANZ and Maik Justus.

var sin      = func(a) { math.sin(a * math.pi / 180.0) }
var cos      = func(a) { math.cos(a * math.pi / 180.0) }
var clamp    = func(v, min = 0, max = 1) { v < min ? min : v > max ? max : v }
var normatan = func(x) { math.atan2(x, 1) * 2 / math.pi }

# timers ============================================================
var turbine_timer = aircraft.timer.new("/sim/time/hobbs/turbines", 10);
aircraft.timer.new("/sim/time/hobbs/helicopter", nil).start();

# engines ===========================================================
# The ENG POWER CONT levers are the mixture controls: 0 OFF, 0.5 IDLE, 1 FLY. A start is: starter pressed,
# lever to IDLE at about 20 % Ng (light-off), starter drops out at 52-65 % Ng, idle 63-69 % Ng; then the
# second engine; then both levers to FLY.
var eng   = func(n, p) { "/controls/engines/engine[" ~ n ~ "]/" ~ p }
var ng    = func(n) { getprop("/fdm/jsbsim/eng[" ~ n ~ "]/ng-pct") or 0 }
var phase = 0;      # 0 sequencer idle, 1..5 start steps, 10..12 stop steps
var t0    = 0;

var press_starter = func(n) {
  setprop(eng(n, "starter"), 1);
  settimer(func { setprop(eng(n, "starter"), 0) }, 0.6);
}

var sequencer = func {
  var t = getprop("/sim/time/elapsed-sec");
  if (phase == 1) {                                   # engine 1: starter
    setprop(eng(0, "mixture"), 0);
    press_starter(0);
    phase = 2;
  } elsif (phase == 2 and ng(0) > 20) {               # light-off
    setprop(eng(0, "mixture"), 0.5);
    phase = 3;
  } elsif (phase == 3 and ng(0) > 62) {               # engine 2
    setprop(eng(1, "mixture"), 0);
    press_starter(1);
    phase = 4;
  } elsif (phase == 4 and ng(1) > 20) {
    setprop(eng(1, "mixture"), 0.5);
    phase = 5;
  } elsif (phase == 5 and ng(1) > 62) {               # run up
    interpolate(eng(0, "mixture"), 1, 8);
    interpolate(eng(1, "mixture"), 1, 8);
    turbine_timer.start();
    phase = 0;
  } elsif (phase == 10) {                             # stop: idle for a cool-down, then off, then the brake
    setprop(eng(0, "mixture"), 0.5);
    setprop(eng(1, "mixture"), 0.5);
    t0 = t;
    phase = 11;
  } elsif (phase == 11 and t > t0 + 10) {
    setprop(eng(0, "mixture"), 0);
    setprop(eng(1, "mixture"), 0);
    turbine_timer.stop();
    phase = 12;
  } elsif (phase == 12 and (getprop("/rotors/main/rpm-pct") or 0) < 40) {
    interpolate("/controls/rotor/brake", 1, 3);
    phase = 0;
  }
}
var seq_timer = maketimer(0.25, sequencer);
seq_timer.start();

# called by the AutoStart key and menu
var engines = func(on) {
  if (on) {
    setprop("/controls/engines/ignition", 1);
    setprop("/controls/rotor/brake", 0);
    setprop("/controls/engines/engine[0]/throttle", 1);   # collective down
    phase = 1;
  } else {
    phase = 10;
  }
}

# skid/tyre sounds ==================================================
var Skid = {
  new : func(n) {
    var m          = { parents : [Skid] };
    var soundN     = props.globals.getNode("sim/sound", 1).getChild("slide", n, 1);
    var gearN      = props.globals.getNode("gear", 1).getChild("gear", n, 1);
    m.compressionN = gearN.getNode("compression-norm", 1);
    m.rollspeedN   = gearN.getNode("rollspeed-ms", 1);
    m.frictionN    = gearN.getNode("ground-friction-factor", 1);
    m.wowN         = gearN.getNode("wow", 1);
    m.volumeN      = soundN.getNode("volume", 1);
    m.pitchN       = soundN.getNode("pitch", 1);
    m.volumeN.setDoubleValue(0);
    m.pitchN.setDoubleValue(0);
    return m;
  },
  update : func {
    me.wowN.getBoolValue() or return;
    var rollspeed = abs(me.rollspeedN.getValue() or 0);
    me.pitchN.setDoubleValue(rollspeed * 0.6);
    var s = normatan(20 * rollspeed);
    var f = clamp(((me.frictionN.getValue() or 1) - 0.5) * 2);
    var c = clamp((me.compressionN.getValue() or 0) * 2);
    me.volumeN.setDoubleValue(s * f * c * 2);
  },
};

var skid = [];
for (var i = 0; i < 3; i += 1) {
  append(skid, Skid.new(i));
}
var skid_timer = maketimer(0, func { foreach (var s; skid) s.update() });

# view management ===================================================
var elapsedN  = props.globals.getNode("/sim/time/elapsed-sec", 1);
var flap_mode = 0;
var down_time = 0;

controls.flapsDown = func(v) {
  if (!flap_mode) {
    if (v < 0) {
      down_time = elapsedN.getValue();
      flap_mode = 1;
      dynamic_view.lookat(5, -20, 0, 0.2, 0.6, 0.85, 0.2, 55);
    } elsif (v > 0) {
      flap_mode = 2;
      var p = "/sim/view/dynamic/enabled";
      setprop(p, !getprop(p));
    }
  } else {
    if (flap_mode == 1) {
      if (elapsedN.getValue() < down_time + 0.2) {
        return;
      }
      dynamic_view.resume();
    }
    flap_mode = 0;
  }
}

dynamic_view.register(func {
  var lowspeed = 1 - normatan(me.speedN.getValue() / 50);
  var r        = sin(me.roll) * cos(me.pitch);
  me.heading_offset = (me.roll < 0 ? -50 : -30) * r * abs(r);
  me.pitch_offset   = -50 * sin(me.pitch) * lowspeed + 15 * sin(me.roll) * sin(me.roll);
  me.roll_offset    = -15 * r * lowspeed;
});

# cockpit vibration =================================================
# The FDM gives the vibration at the pilot seat (/sim/model/uh60/vibration-g, RMS g of the 6-30 Hz band: the
# rotor's 4-per-rev hub loads, stall buffet, transition). The real motion is far too small to see (0.1 g at 17 Hz
# moves the eye 0.1 mm) but blurs the panel, so the eye shakes by an exaggerated amount: the panel shakes, the
# distant world stays steady. /sim/model/uh60/vibration-view-scale scales it (0 = off). Like the dynamic view,
# it adds its offsets to the view's own, so head movement and the dynamic view still work.
var vibN       = props.globals.getNode("/sim/model/uh60/vibration-g", 1);
var vibScaleN  = props.globals.initNode("/sim/model/uh60/vibration-view-scale", 1.0, "DOUBLE");
var internalN  = props.globals.getNode("/sim/current-view/internal", 1);
var shake_axes = [];
var shake_wt   = [0.6, 1.0, 0.8];     # right, up, aft: mostly vertical
var shake_now  = [0, 0, 0];
foreach (var a; ["x-offset-m", "y-offset-m", "z-offset-m"]) {
  append(shake_axes, props.globals.getNode("/sim/current-view/" ~ a, 1));
}
var shake = func {
  var amp = 0;
  if (internalN.getBoolValue()) {
    amp = clamp((vibN.getValue() or 0) * 0.02 * (vibScaleN.getValue() or 0), 0, 0.03);   # 2 cm per g
  }
  forindex (var i; shake_axes) {
    var s = amp * shake_wt[i] * (2 * rand() - 1);
    var n = shake_axes[i];
    n.setDoubleValue((n.getValue() or 0) - shake_now[i] + s);
    shake_now[i] = s;
  }
}
var shake_timer = maketimer(0, shake);
setlistener("/sim/current-view/view-number", func { shake_now = [0, 0, 0] });

# AFCS cautions =====================================================
# The caution panel shows a stability function that is off; here the message goes on the screen when a switch
# changes, at lift-off, and every 30 s while airborne with something still off.
var afcs = [["/fdm/jsbsim/ctl/sas1", "SAS 1 OFF"], ["/fdm/jsbsim/ctl/sas2", "SAS 2 OFF"], ["/fdm/jsbsim/ctl/fps", "FPS OFF"],
            ["/fdm/jsbsim/ctl/pba", "PITCH BIAS OFF"], ["/fdm/jsbsim/stab/auto", "STABILATOR MANUAL"]];
var afcs_off = func {
  var off = [];
  foreach (var a; afcs) {
    if (!(getprop(a[0]) or 0)) append(off, a[1]);
  }
  return off;
}
var afcs_report = func(all_on_too = 0) {
  var off = afcs_off();
  if (size(off)) {
    screen.log.write("CAUTION: " ~ string.join(", ", off), 1.0, 0.6, 0.1);
  } elsif (all_on_too) {
    screen.log.write("AFCS: all on", 0.5, 1.0, 0.5);
  }
}
# The listeners go on after the FDM has started: a listener creates its property, and JSBSim does not set its
# default (switch on) on a property that already exists.
var afcs_listening = 0;
var afcs_listen = func {
  afcs_listening and return;
  afcs_listening = 1;
  foreach (var a; afcs) {
    setlistener(a[0], func { afcs_report(1) }, 0, 0);
  }
  setlistener("/gear/gear[0]/wow", func(n) { if (!n.getBoolValue()) afcs_report() }, 0, 0);
}
var afcs_timer = maketimer(30, func { if (!getprop("/gear/gear[0]/wow")) afcs_report() });

# initialization ====================================================
setlistener("/sim/signals/fdm-initialized", func {
  # FlightGear starts JSBSim with the CG at ground level. The UH-60's wheels are 6.2 ft below its CG, close to
  # the reach of FlightGear's ground probe (2 m above each contact): on uneven ground a probe could start under
  # the surface. Start at the height it rests at on its gear (CG 5.9 ft up), so it doesn't drop onto the wheels.
  if (getprop("/sim/presets/onground")) {
    setprop("/position/altitude-ft", getprop("/position/ground-elev-ft") + 5.9);
  }
  skid_timer.start();
  shake_timer.start();
  afcs_listen();
  afcs_timer.start();
  if ((getprop("/position/altitude-agl-ft") or 0) > 50) {
    turbine_timer.start();
  }
  setlistener("/sim/signals/reinit", func {
    cmdarg().getBoolValue() and return;
    turbine_timer.stop();
    phase = 0;
  });
});
