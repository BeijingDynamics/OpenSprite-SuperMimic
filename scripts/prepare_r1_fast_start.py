"""Build an isolated operator-requested faster-start and torque-observation profile."""
from pathlib import Path
import shutil
import tarfile

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'outputs/deploy-sm-walk-r1-fast-20261119'
OLD = '/home/tony/sprite-sm-walk-r1-xbox120-20261119'
NEW = '/home/tony/sprite-sm-walk-r1-fast-20261119'


def main():
    BASE.mkdir(exist_ok=True)
    src = BASE / 'source'
    shutil.copytree(ROOT / 'outputs/deploy-sm-walk-r1-xbox120-20261119/source', src)
    def edit(name, before, after, count=1):
        path = src / name
        text = path.read_text()
        if text.count(before) != count:
            raise ValueError('Unexpected anchor: ' + name + ': ' + before)
        path.write_text(text.replace(before, after), newline='\n')

    for name in ['operator-xbox.sh', 'run-disabled.py']:
        p = src / name
        p.write_text(p.read_text().replace(OLD, NEW), newline='\n')
    for name in ['native/src/sprite_can_shadow.cpp', 'native/tests/test_bounded_preview.cpp',
                 'tools/run_supermimic_commissioning.py']:
        p = src / name
        p.write_text(p.read_text().replace('SUPERMIMIC_XBOX_120S_040',
                                          'SUPERMIMIC_XBOX_120S_040_FAST_TORQUE_OBSERVE'), newline='\n')
    edit('native/src/sprite_supermimic_replay.cpp', 'gamepad_config.forward=.40;',
         'gamepad_config.forward=.40;\n      gamepad_config.acceleration=.10;')
    edit('native/tests/test_gamepad_control.cpp', 'cfg.forward=.40;', 'cfg.forward=.40;cfg.acceleration=.10;')
    edit('native/tests/test_gamepad_control.cpp', 'if(frame==7001)',
         'if(frame==1205)assert(v[0]==.40);\n    if(frame==7001)')
    edit('operator-xbox.sh', 'Forward acceleration is 0.05', 'Forward acceleration is 0.10')
    edit('operator-xbox.sh', 'about 47.2 m', 'about 48.0 m')
    edit('operator-xbox.sh', "echo 'After enable, policy takes over",
         "echo 'Instantaneous feedback torque excursions are recorded, not stop triggers. Motor faults still stop control.'\n"
         "echo 'PD command saturation, position, velocity, temperature and timing guards remain active.'\n"
         "echo 'After enable, policy takes over")
    edit('operator-xbox.sh', "    can = json.loads((directory / 'can.json').read_text())",
         "    can = json.loads((directory / 'can.json').read_text())\n"
         "    if not all(m.get('feedback_torque_observe_only') is True for m in can['motors'].values()):\n"
         "        raise SystemExit('Expected feedback torque observation profile was not verified.')")

    native = 'native/src/sprite_can_shadow.cpp'
    edit(native, '  double feedback_torque_cap = 0.0;',
         '  double feedback_torque_cap = 0.0;\n'
         '  bool feedback_torque_observe_only = false;\n'
         '  std::size_t feedback_torque_excursions = 0, feedback_torque_full_scale_samples = 0;\n'
         '  double first_torque_excursion_position = 0., first_torque_excursion_velocity = 0.;\n'
         '  double first_torque_excursion_nm = 0.;')
    helper = '''void check_dynamic_feedback(Motor& motor, double feedback_torque, unsigned raw_torque) {
  const bool torque_excursion = std::abs(feedback_torque) > motor.feedback_torque_cap;
  if (raw_torque == 0 || raw_torque == 4095) ++motor.feedback_torque_full_scale_samples;
  if (torque_excursion) {
    if (motor.feedback_torque_excursions++ == 0) {
      motor.first_torque_excursion_position = motor.last_position;
      motor.first_torque_excursion_velocity = motor.last_velocity;
      motor.first_torque_excursion_nm = feedback_torque;
    }
  }
  const char* reason = nullptr;
  if (motor.last_position < motor.hard_position_min || motor.last_position > motor.hard_position_max)
    reason = "position";
  else if (std::abs(motor.last_velocity) > motor.deployment_velocity_max) reason = "velocity";
  else if (torque_excursion && !motor.feedback_torque_observe_only) reason = "feedback_torque";
  if (reason) {
    std::ostringstream message;
    message << "protected-policy dynamic guard failed for " << motor.name << " reason=" << reason
            << " q=" << motor.last_position << " q_min=" << motor.hard_position_min
            << " q_max=" << motor.hard_position_max << " qd=" << motor.last_velocity
            << " qd_limit=" << motor.deployment_velocity_max << " tau=" << feedback_torque
            << " tau_observation_boundary=" << motor.feedback_torque_cap;
    throw std::runtime_error(message.str());
  }
}

'''
    edit(native, 'void drain_policy_feedback(', helper + 'void drain_policy_feedback(')
    before = '''      if (enforce_dynamic_guards &&
          (motor.last_position < motor.hard_position_min ||
           motor.last_position > motor.hard_position_max ||
           std::abs(motor.last_velocity) > motor.deployment_velocity_max ||
           std::abs(feedback_torque) > motor.feedback_torque_cap)) {
        throw std::runtime_error("protected-policy dynamic guard failed for " + motor.name);
      }'''
    edit(native, before, '      if (enforce_dynamic_guards) check_dynamic_feedback(motor, feedback_torque, raw_torque);')
    edit(native, '  const bool walking = options.walk_speed > 0.0;',
         '  const bool walking = options.walk_speed > 0.0;\n'
         '  // Only the explicitly acknowledged extended Xbox profile observes instantaneous torque.\n'
         '  for (auto& motor : motors) motor.feedback_torque_observe_only = strict && options.xbox_body_admission;')
    anchor = r'        << ", \"feedback_torque_cap_nm\": " << motor.feedback_torque_cap'
    addition = r'''
        << ", \"feedback_torque_observe_only\": " << (motor.feedback_torque_observe_only ? "true" : "false")
        << ", \"feedback_torque_excursions\": " << motor.feedback_torque_excursions
        << ", \"feedback_torque_full_scale_samples\": " << motor.feedback_torque_full_scale_samples
        << ", \"first_torque_excursion_position_rad\": " << motor.first_torque_excursion_position
        << ", \"first_torque_excursion_velocity_rad_s\": " << motor.first_torque_excursion_velocity
        << ", \"first_torque_excursion_nm\": " << motor.first_torque_excursion_nm'''
    edit(native, anchor, anchor + addition)
    shutil.copyfile(ROOT / 'scripts/test_r1_feedback_observation.inc', src / 'native/tests/test_feedback_observation.inc')
    edit('native/tests/test_bounded_preview.cpp', 'int main() {',
         '#include "test_feedback_observation.inc"\n\nint main() {\n  test_feedback_observation();')
    edit('tools/run_supermimic_commissioning.py', 'training_full_body=args.training_full_body, standing_admission=standing_admission,',
         'training_full_body=args.training_full_body, standing_admission=standing_admission,\n'
         '                    feedback_torque_mode="observe_only" if args.gamepad_control_device else "stop",\n'
         '                    xbox_forward_acceleration_m_s2=.10 if args.gamepad_control_device else None,')
    with tarfile.open(BASE / 'source.tar.gz', 'w:gz') as stream:
        for p in src.iterdir():
            stream.add(p, arcname=p.name, filter=lambda m: None if '__pycache__' in m.name else m)
    print('Prepared isolated faster-start profile; no hardware accessed.')


if __name__ == '__main__':
    main()
