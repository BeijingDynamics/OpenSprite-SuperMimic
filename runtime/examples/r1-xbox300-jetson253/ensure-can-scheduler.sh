#!/usr/bin/env bash
# Install beside the stage's operator script. No CAN or IMU access.
set -euo pipefail
stage=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
binary="$stage/build/sprite_can_shadow"
[[ -f "$binary" && -x "$binary" ]] || { echo 'CAN binary missing.' >&2; exit 2; }
cpu=$(/home/tony/open_sprite_runtime/.venv/bin/python - "$stage/tools/run_supermimic_commissioning.py" <<'PY'
import ast, sys
tree = ast.parse(open(sys.argv[1]).read())
values = {n.targets[0].id: ast.literal_eval(n.value) for n in tree.body
          if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name)
          and n.targets[0].id in ('CAN_CPU', 'CAN_REALTIME_PRIORITY', 'ACTOR_CPUS')}
assert values['CAN_REALTIME_PRIORITY'] == 50
assert values['CAN_CPU'] not in values['ACTOR_CPUS']
print(values['CAN_CPU'])
PY
)
if ! getcap "$binary" | grep -q 'cap_sys_nice=ep'; then
  echo 'The new CAN binary needs realtime permission. sudo may request the onsite password.'
  sudo setcap cap_sys_nice=ep "$binary"
fi
getcap "$binary" | grep -q 'cap_sys_nice=ep' || { echo 'Realtime permission missing.' >&2; exit 2; }
"$binary" --check-commissioning-scheduler "$cpu"
echo "Scheduler verified: CAN CPU $cpu / SCHED_FIFO 50. No motor frames sent."
