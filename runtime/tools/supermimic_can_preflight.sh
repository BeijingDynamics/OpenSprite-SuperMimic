#!/usr/bin/env bash
# No CAN/IMU access. Install beside the commissioning stage's operator script.
set -euo pipefail
stage=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
binary="$stage/build/sprite_can_shadow"
[[ -f "$binary" && -x "$binary" ]] || { echo 'CAN binary missing or not executable; stopping.'; exit 2; }
# Read the launcher's single source of truth without importing or executing it.
cpu=$(/home/tony/open_sprite_runtime/.venv/bin/python - "$stage/tools/run_supermimic_commissioning.py" <<'PY'
import ast,sys
tree=ast.parse(open(sys.argv[1]).read())
values={node.targets[0].id:ast.literal_eval(node.value) for node in tree.body
        if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name)
        and node.targets[0].id in ('CAN_CPU','CAN_REALTIME_PRIORITY','ACTOR_CPUS')}
assert values['CAN_REALTIME_PRIORITY']==50
assert values['CAN_CPU'] not in values['ACTOR_CPUS']
print(values['CAN_CPU'])
PY
)
if ! getcap "$binary" | grep -q 'cap_sys_nice=ep'; then
  echo 'CAN binary lacks realtime permission; configuring it automatically. sudo may request the onsite password.'
  sudo setcap cap_sys_nice=ep "$binary"
fi
getcap "$binary" | grep -q 'cap_sys_nice=ep' || { echo 'Realtime permission not active; stopping.'; exit 2; }
"$binary" --check-commissioning-scheduler "$cpu"
echo "Scheduler preflight passed: CAN CPU $cpu / SCHED_FIFO 50. The actual control process will be checked again."
if [[ -f "$stage/tools/can_tx_timestamp_setup.py" ]]; then
  if ! /usr/bin/python3 "$stage/tools/can_tx_timestamp_setup.py" --lock "$stage/../commissioning.lock"; then
    echo 'CAN adapter TX hardware timestamps must be enabled. This configures timestamps without sending CAN frames; sudo may request the onsite password.'
    sudo /usr/bin/python3 "$stage/tools/can_tx_timestamp_setup.py" --apply --lock "$stage/../commissioning.lock"
  fi
fi
