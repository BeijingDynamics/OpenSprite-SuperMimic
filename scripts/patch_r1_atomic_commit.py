"""Refresh all protected motor targets on a newly accepted coherent action."""
from pathlib import Path
import shutil

root=Path(__file__).resolve().parents[1]
src=root/'outputs/deploy-sm-walk-r1-fast-20261119/source'
def edit(name,old,new):
    p=src/name;t=p.read_text()
    if t.count(old)!=1:raise ValueError(name+': '+old)
    p.write_text(t.replace(old,new),newline='\n')

edit('native/src/action_tx_trace.hpp',
     'inline bool coherent_commit_due(std::size_t slot,std::uint64_t pending,std::uint64_t active)',
     'inline bool coherent_commit_due(std::size_t /*slot*/,std::uint64_t pending,std::uint64_t active)')
edit('native/src/action_tx_trace.hpp','return slot%4==0 && pending!=active;',
     'return pending!=active;')
edit('native/src/sprite_can_shadow.cpp',
     'bool pd_position_saturation = false) {\n  if (!ipc.has_target)',
     'bool pd_position_saturation = false, bool refresh_all = false) {\n  if (!ipc.has_target)')
edit('native/src/sprite_can_shadow.cpp',
     'const bool update_other_motors = ipc.motor_preview_count % 10 == 0;',
     'const bool update_other_motors = refresh_all || ipc.motor_preview_count % 10 == 0;')
edit('native/src/sprite_can_shadow.cpp',
     'if (slot % 4 == 0) {\n        preview_final_motor_commands(motors, ipc, kinematics, bound_command, pd_position_saturation);\n      }',
     '// A new action refreshes every motor before the coherent group is sent.\n'
     '      // Periodic ankle remapping continues between policy updates.\n'
     '      if (commit || slot % 4 == 0) {\n'
     '        preview_final_motor_commands(motors, ipc, kinematics, bound_command, pd_position_saturation, commit);\n      }')
edit('native/tests/test_action_tx.cpp',
     '// An arriving target must wait for at most the next 2 ms mapping boundary,\n  // not the next 20 ms low-rate cycle. No skipped sequence is admissible.',
     '// An accepted action commits in the current transport slot.\n  // No skipped sequence is admissible.')
edit('native/tests/test_action_tx.cpp','assert(slot-arrival<=3);','assert(slot==arrival);')
edit('native/tests/test_bounded_preview.cpp','#include "test_feedback_observation.inc"',
     '#include "test_feedback_observation.inc"\n#include "test_atomic_commit.inc"')
edit('native/tests/test_bounded_preview.cpp','  test_feedback_observation();',
     '  test_feedback_observation();\n  test_atomic_commit();')
shutil.copyfile(root/'scripts/test_r1_atomic_commit.inc',src/'native/tests/test_atomic_commit.inc')
for target in ['sprite_can_shadow','test_bounded_preview']:
    edit('native/CMakeLists.txt',
         f'target_compile_options({target} PRIVATE -Wall -Wextra -Wpedantic -Werror)',
         f'target_compile_options({target} PRIVATE -O2 -ffp-contract=off -Wall -Wextra -Wpedantic -Werror)')
