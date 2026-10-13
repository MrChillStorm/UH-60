import harness as H, math
f = H.load(model='uh60a')
f['eng/init-running'] = 0
for n in range(2):
    f['/controls/engines/engine[%d]/mixture' % n] = 0.0
f['ic/h-agl-ft'] = 6.4; f['ic/terrain-elevation-ft'] = 0; f['ic/vt-kts'] = 0; f['ic/theta-deg'] = 0; f['ic/phi-deg'] = 0
H.controls(f, 0.0, 0, 0, 0)
f.run_ic()
dt = f['simulation/dt']
keys = ['drive/nr-pct','eng[0]/ng-pct','eng[0]/np-pct','eng[0]/tgt-degc','eng[0]/torque-pct','eng[0]/fuel-flow-pph','eng[0]/starter-on','eng[0]/lit',
        'eng[1]/ng-pct','eng[1]/np-pct','eng[1]/tgt-degc','position/h-agl-ft','attitude/psi-deg','gear/unit[0]/WOW']
print('    t ' + ' '.join('%9s' % (k.split('/')[0][-2:] + k.split('/')[-1][:7]) for k in keys))
events = []
t = 0.0; phase = 0
for i in range(int(150/dt)):
    t = i*dt
    if phase == 0 and t > 2:
        f['/controls/engines/engine[0]/starter'] = 1; phase = 1; events.append((t, 'starter 1'))
    if phase == 1 and t > 2.5:
        f['/controls/engines/engine[0]/starter'] = 0
        if f['eng[0]/ng-pct'] > 20:
            f['/controls/engines/engine[0]/mixture'] = 0.5; phase = 2; events.append((t, 'PCL1 IDLE'))
    if phase == 2 and f['eng[0]/ng-pct'] > 62 and f['eng[0]/starter-on'] < 0.5:
        f['/controls/engines/engine[1]/starter'] = 1; phase = 3; events.append((t, 'starter 2'))
    if phase == 3 and f['eng[1]/ng-pct'] > 20:
        f['/controls/engines/engine[1]/starter'] = 0
        f['/controls/engines/engine[1]/mixture'] = 0.5; phase = 4; events.append((t, 'PCL2 IDLE'))
    if phase == 4 and f['eng[1]/ng-pct'] > 62:
        phase = 5; t5 = t; events.append((t, 'PCLs to FLY'))
    if phase == 5:
        p = min(1.0, 0.5 + (t - t5) * 0.05)
        f['/controls/engines/engine[0]/mixture'] = p; f['/controls/engines/engine[1]/mixture'] = p
    if i % int(5/dt) == 0:
        print('%5.0f ' % t + ' '.join('%9.1f' % f[k] for k in keys))
    f.run()
print(events)
