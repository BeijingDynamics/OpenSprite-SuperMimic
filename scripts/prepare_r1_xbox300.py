"""Extend the exact validated R1 Xbox profile to five minutes in an isolated stage."""
from pathlib import Path
import hashlib,json,tarfile

root=Path(__file__).resolve().parents[1]
base=root/'outputs/deploy-sm-walk-r1-xbox300-20261119';base.mkdir(exist_ok=False)
src=base/'source';src.mkdir()
old=root/'outputs/deploy-sm-walk-r1-fast-20261119/source-final-no-spin.tar.gz'
assert hashlib.sha256(old.read_bytes()).hexdigest()=='73d752522aa16adc745028827658878399620d26ec40b6493958e9ca3a5dff5f'
with tarfile.open(old) as archive:archive.extractall(src,filter='data')
old_stage='/home/tony/sprite-sm-walk-r1-fast-20261119'
new_stage='/home/tony/sprite-sm-walk-r1-xbox300-20261119'
changes=[]
def edit(name,a,b,count=1):
    p=src/name;t=p.read_text()
    if t.count(a)!=count:raise ValueError(name+': '+a+' count='+str(t.count(a)))
    p.write_text(t.replace(a,b),newline='\n');changes.append(name)
edit('native/src/sling_window.hpp','kXboxControlFrames = 120 * 50','kXboxControlFrames = 300 * 50')
edit('native/src/sling_window.hpp','kXboxDurationSeconds = 146.0','kXboxDurationSeconds = 326.0')
edit('native/src/sling_window.hpp','kXboxEndNoticeSeconds == 145','kXboxEndNoticeSeconds == 325')
edit('native/src/sprite_supermimic_replay.cpp','XBOX_WINDOW_OPEN: 120 seconds','XBOX_WINDOW_OPEN: 300 seconds')
edit('native/src/sprite_can_shadow.cpp','exact 146-second Xbox profile','exact 326-second Xbox profile')
for name in ['native/src/sprite_can_shadow.cpp','native/tests/test_bounded_preview.cpp','tools/run_supermimic_commissioning.py']:
    p=src/name;t=p.read_text();assert 'SUPERMIMIC_XBOX_120S_040_FAST_TORQUE_OBSERVE' in t
    p.write_text(t.replace('SUPERMIMIC_XBOX_120S_040_FAST_TORQUE_OBSERVE','SUPERMIMIC_XBOX_300S_040_FAST_TORQUE_OBSERVE'),newline='\n')
edit('native/src/action_tx_trace.hpp','events.reserve(600000)','events.reserve(700000)')
edit('native/tests/test_action_tx.cpp','i<500000','i<650000')
edit('native/tests/test_action_tx.cpp','events.size()==500000','events.size()==650000')
edit('native/tests/test_action_tx.cpp','trace.stamp(499999,','trace.stamp(649999,')
edit('native/tests/test_gamepad_control.cpp','==6000','==15000')
edit('native/tests/test_gamepad_control.cpp','==146.0','==326.0')
edit('native/tests/test_gamepad_control.cpp','frame<7301','frame<16301')
edit('native/tests/test_gamepad_control.cpp','frame==7001','frame==16001')
edit('native/tests/test_gamepad_control.cpp','frame>=7203','frame>=16203')
edit('native/tests/test_gamepad_control.cpp','allowed_frames==5997','allowed_frames==14997')
edit('native/tests/test_gamepad_control.cpp','PASS 120-second','PASS 300-second')
p=src/'native/tests/test_bounded_preview.cpp';t=p.read_text()
t=t.replace('accepts_duration("146",','accepts_duration("326",')
t=t.replace('{"54", "120", "144", "145", "147", "200"}', '{"54", "120", "146", "300", "325", "327", "600"}')
t=t.replace('exact 146-second profile','exact 326-second profile')
p.write_text(t,newline='\n')
edit('tools/run_supermimic_commissioning.py','XBOX_DURATION = 146.0','XBOX_DURATION = 326.0')
edit('tools/run_supermimic_commissioning.py','120-second Xbox window','300-second Xbox window',2)
edit('tools/run_supermimic_commissioning.py','"180" if args.gamepad_control_device','"360" if args.gamepad_control_device')
edit('tools/run_supermimic_commissioning.py','135.5 if args.gamepad_control_device','315.5 if args.gamepad_control_device')
edit('tools/run_supermimic_commissioning.py','6700 if args.gamepad_control_device','15700 if args.gamepad_control_device')
edit('tools/run_supermimic_commissioning.py','command[7205:]','command[16205:]')
edit('tools/run_supermimic_commissioning.py','capture_output=True, text=True, timeout=30)',
     'capture_output=True, text=True, timeout=90)')
for name in ['operator-xbox.sh','run-disabled.py']:
    p=src/name;t=p.read_text().replace(old_stage,new_stage)
    t=t.replace('sleeping-disabled-01','disabled-01').replace('sleeping-fault-01','fault-01')
    if name.endswith('.sh'):
        for a,b in [('== 146.0','== 326.0'),('(7301, 71)','(16301, 71)'),('[7205:,','[16205:,'),
                    ('120-second','300-second'),('146 seconds','326 seconds'),('120 s','300 s'),
                    ('about 48.0 m','about 120.0 m'),('allow 50 m','allow 122 m')]:t=t.replace(a,b)
    else:t=t.replace('timeout=240','timeout=480')
    p.write_text(t,newline='\n')
# Use the already qualified temporary driver and loader instead of copying module dependencies.
(src/'load-echo-order-driver.sh').write_text('#!/usr/bin/env bash\nset -euo pipefail\nexec bash '+old_stage+'/load-echo-order-driver.sh "$@"\n',newline='\n')
with tarfile.open(base/'source.tar.gz','w:gz') as archive:
    for p in src.iterdir():archive.add(p,arcname=p.name)
(base/'preparation.json').write_text(json.dumps(dict(base_source_sha256=hashlib.sha256(old.read_bytes()).hexdigest(),
    control_window_s=300,total_control_s=326,policy_frames=16301,tx_events_per_socket=700000,
    model_and_gains_changed=False,latency_guards_changed=False,driver_changed=False),indent=2)+'\n')
print('Prepared isolated 300-second Xbox profile; duration and recording capacity only.')
