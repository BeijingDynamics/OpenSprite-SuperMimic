"""Prepare an isolated, exactly bounded 120-second / 0.40 m/s Xbox profile."""
from pathlib import Path
import shutil
import tarfile

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'outputs/deploy-sm-walk-r1-xbox120-20261119'
OLD='/home/tony/sprite-sm-walk-r1-20261119'
NEW='/home/tony/sprite-sm-walk-r1-xbox120-20261119'


def main():
    src=BASE/'source';src.mkdir(exist_ok=False)
    for name in ['native','tools','runtime']:
        shutil.copytree(BASE/'original'/name,src/name)
    for name in ['operator-xbox.sh','run-disabled.py','ensure-can-scheduler.sh']:
        shutil.copyfile(BASE/'original'/name,src/name)
    def edit(name,a,b,count=1):
        p=src/name;t=p.read_text()
        if t.count(a)!=count:raise ValueError(f'Unexpected patch anchor: {name}: {a}')
        p.write_text(t.replace(a,b), newline=chr(10))
    edit('native/src/sling_window.hpp','kXboxControlFrames = 30 * 50','kXboxControlFrames = 120 * 50')
    edit('native/src/sling_window.hpp','2.5 s maximum braking','4 s maximum braking')
    edit('native/src/sling_window.hpp','kXboxDurationSeconds = 54.0','kXboxDurationSeconds = 146.0')
    edit('native/src/policy_gamepad.hpp','bool xbox_control_025=false;','bool xbox_control_025=false;\n  bool xbox_control_040=false;')
    edit('native/src/policy_gamepad.hpp','forward>(xbox_control_025?.25:.10)',
         'forward>(xbox_control_040?.40:xbox_control_025?.25:.10)')
    edit('native/src/policy_gamepad.hpp','if(deadzone>=1 ||',
         'if(xbox_control_025 && xbox_control_040)throw std::runtime_error("conflicting Xbox profiles");\n    if(deadzone>=1 ||')
    edit('native/src/sprite_supermimic_replay.cpp','gamepad_config.xbox_control_025=true;\n      gamepad_config.forward=.25;',
         'gamepad_config.xbox_control_040=true;\n      gamepad_config.forward=.40;')
    edit('native/src/sprite_supermimic_replay.cpp','XBOX_WINDOW_OPEN: 30 seconds of control.','XBOX_WINDOW_OPEN: 120 seconds of control.')
    edit('native/src/sprite_can_shadow.cpp','if (!(result.duration_s > 0.0 && result.duration_s <= 120.0)) {',
         'const bool xbox_long_duration = result.xbox_body_admission && result.duration_s == sprite_policy::kXboxDurationSeconds;\n  if (!(result.duration_s > 0.0 && (result.duration_s <= 120.0 || xbox_long_duration))) {')
    edit('native/src/sprite_can_shadow.cpp','duration must be in (0, 120]',
         'duration must be in (0, 120], except the exact 146-second Xbox profile')
    edit('native/src/sprite_can_shadow.cpp','if (result.duration_s > 60.0) {',
         'if (result.duration_s > 60.0 && !xbox_long_duration) {')
    edit('native/src/sprite_can_shadow.cpp','extended protected policy actuation must not exceed 60 seconds',
         'extended protected policy actuation must not exceed 60 seconds outside the exact Xbox profile')
    for name in ['native/src/sprite_can_shadow.cpp','tools/run_supermimic_commissioning.py','native/tests/test_bounded_preview.cpp']:
        p=src/name;p.write_text(p.read_text().replace('SUPERMIMIC_XBOX_30S','SUPERMIMIC_XBOX_120S_040'))
    edit('tools/run_supermimic_commissioning.py','XBOX_DURATION = 54.0','XBOX_DURATION = 146.0')
    edit('tools/run_supermimic_commissioning.py','30-second Xbox window','120-second Xbox window',2)
    edit('tools/run_supermimic_commissioning.py','"85" if (args.walk_speed or args.gamepad_control_device)',
         '"180" if args.gamepad_control_device else "85" if args.walk_speed')
    edit('tools/run_supermimic_commissioning.py','45.5 if args.gamepad_control_device','135.5 if args.gamepad_control_device')
    edit('tools/run_supermimic_commissioning.py','2200 if args.gamepad_control_device','6700 if args.gamepad_control_device')
    edit('tools/run_supermimic_commissioning.py','(.250001 if args.gamepad_control_device else .100001)',
         '(.400001 if args.gamepad_control_device else .100001)')
    edit('tools/run_supermimic_commissioning.py','command[2630:]','command[7205:]')
    p=src/'native/tests/test_bounded_preview.cpp';t=p.read_text()
    t=t.replace('accepts_duration("54",','accepts_duration("146",')
    anchor='  constexpr auto sling_ack = "ENABLE_SUPERMIMIC_ZERO_SPD_SLING_WINDOW";'
    t=t.replace(anchor,anchor+'''
  for (const char* invalid : {"54", "120", "144", "145", "147", "200"})
    require(!accepts_duration(invalid, "ENABLE_SUPERMIMIC_XBOX_120S_040", "50", true, nullptr, true),
            "Xbox extension requires the exact 146-second profile");
  require(!accepts_duration("146", "ENABLE_SUPERMIMIC_XBOX_30S", "50", true, nullptr, true),
          "old acknowledgement must not admit the extended profile");
''');p.write_text(t)
    # The new standalone window test replaces brittle fixed end-frame assertions.
    shutil.copyfile(ROOT/'scripts/test_r1_xbox120_040.cpp',src/'native/tests/test_gamepad_control.cpp')
    p=src/'operator-xbox.sh';t=p.read_text().replace(OLD,NEW)
    for a,b in [('duration_s\'] == 54.0','duration_s\'] == 146.0'),('(2701, 71)','(7301, 71)'),
        ('inputs[2630:, 68:71]','inputs[7205:, 68:71]'),('forward up to 0.25','forward up to 0.40'),
        ('up to 2.5 s from full forward speed','up to 4 s from full forward speed'),
        ('30-second','120-second'),('30 s','120 s'),('54 seconds','146 seconds'),
        ('about 7.2 m','about 47.2 m'),('allow 8 m','allow 50 m'),('a full 30 s','a full 120 s')]:t=t.replace(a,b)
    t=t.replace("echo 'A continuous full-forward", "echo 'A continuous full-forward")
    p.write_text(t)
    p=src/'run-disabled.py';t=p.read_text().replace(OLD,NEW).replace('timeout=140','timeout=240');p.write_text(t)
    p=src/'ensure-can-scheduler.sh';p.write_text(p.read_text().replace(OLD,NEW))
    for shell in src.glob('*.sh'):
        shell.write_text(shell.read_text(), newline=chr(10))
    from patch_r1_xbox120_trace_capacity import patch
    patch(src)
    with tarfile.open(BASE/'source.tar.gz','w:gz') as archive:
        for p in src.iterdir():archive.add(p,arcname=p.name,filter=lambda i:None if '__pycache__' in i.name else i)
    print('Isolated 120-second / 0.40 m/s source prepared.')


if __name__=='__main__':main()
