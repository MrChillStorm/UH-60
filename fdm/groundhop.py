"""On-ground lift-off with fixed controls, as the FlightGear telnet test (controls in FlightGear's convention)."""
import sys, math
import harness as H
lon_fg, lat_fg, rud_fg, cmax = [float(a) for a in sys.argv[1:5]]
f = H.load(model='uh60a')
f['eng/init-running'] = 1
f['ic/terrain-elevation-ft'] = 177; f['ic/h-agl-ft'] = 6.0; f['ic/vt-kts'] = 0; f['ic/psi-true-deg'] = 227
f.run_ic()
dt = f['simulation/dt']
for i in range(int(3/dt)):   # settle on the gear
    H.controls(f, 0.0, lon_fg, lat_fg, rud_fg); f.run()
print('   t   agl  pitch   roll    hdg     p     q     r  coll  wowL wowR wowT  Fy_gear')
for i in range(int(14/dt)):
    t = i * dt
    c = min(cmax, t * 0.12)
    H.controls(f, c, lon_fg, lat_fg, rud_fg)
    f.run()
    if i % int(0.5/dt) == 0:
        print('%4.1f %5.1f %6.1f %6.1f %6.1f %5.1f %5.1f %5.1f %5.2f %4d %4d %4d %8.0f' % (t, f['position/h-agl-ft'], f['attitude/theta-deg'], f['attitude/phi-deg'],
              f['attitude/psi-deg'], math.degrees(f['velocities/p-rad_sec']), math.degrees(f['velocities/q-rad_sec']), math.degrees(f['velocities/r-rad_sec']), c,
              f['gear/unit[0]/WOW'], f['gear/unit[1]/WOW'], f['gear/unit[2]/WOW'], f['forces/fby-gear-lbs']))
    if abs(f['attitude/phi-deg']) > 40: print('rolled over'); break
