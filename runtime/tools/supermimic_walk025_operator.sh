#!/usr/bin/env bash
set -euo pipefail
# Install as operator-test.sh in the reviewed isolated stage directory.
stage=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
python=/home/tony/open_sprite_runtime/.venv/bin/python
mode=${1:-run}
if [[ $# -gt 1 || ( "$mode" != run && "$mode" != --verify-only && "$mode" != --revalidate && "$mode" != --check-cache-only ) ]]; then
  echo 'Usage: operator-test.sh [--verify-only | --revalidate | --check-cache-only]'; exit 2
fi
session="$stage/runs/$(date +%Y%m%d-%H%M%S)-$$"
mkdir -p "$session"
if [[ "$mode" != --check-cache-only ]]; then
  printf '%s\n' "$session" > "$stage/latest-session.txt"
fi
exec > >(tee -a "$session/operator-run.log") 2>&1
printf '\n[Configuration check; no hardware TX] All 31 motors at 500 Hz, policy at 50 Hz; full training gains; forward command 0.25 m/s for 10 seconds; lateral/yaw commands zero.\n'
validation="$session"
if [[ "$mode" == run || "$mode" == --check-cache-only ]]; then
  set +e
  "$python" "$stage/select-validation.py" --stage "$stage" --direct-policy-start --coherent-action-send --all-joint-500hz --report "$session/validation-cache.json" > "$session/config-preflight.log" 2>&1
  cache_result=$?
  set -e
  if [[ $cache_result -eq 0 ]]; then
    validation=$("$python" -c 'import json,sys; print(json.load(open(sys.argv[1]))["selected"])' "$session/validation-cache.json")
    printf '[Reusing validation] Build, model, gains, guards and speed profile match; action history and disable records rechecked.\nSource: %s\n' "$validation"
    printf 'Skipping the full disabled rehearsal and timeout injection; fresh CAN, feedback, scheduler and IMU checks still precede powered control.\n'
  elif [[ $cache_result -eq 1 ]]; then
    printf '[Validation required] No complete passing record matches this configuration.\n'
  else
    printf 'Configuration check failed; exiting without motor commands.\n'
    cat "$session/config-preflight.log"
    printf 'See validation-cache.json.\n'; exit 2
  fi
  if [[ "$mode" == --check-cache-only ]]; then
    printf 'File-only check complete; no CAN/IMU access, permission changes or motor control.\n'; exit "$cache_result"
  fi
fi
printf '%s\n' "$validation" > "$session/validation-source.txt"
bash "$stage/ensure-can-scheduler.sh"
if [[ "$validation" == "$session" ]]; then
  printf '\n[Validation 1/2; motors disabled] Full command rehearsal: 0.25 m/s plateau for 10 seconds, continuous action history and all 31 motors disabled.\n'
  bash "$stage/run-stage.sh" dry "$session"
  printf '\n[Validation 2/2; motors disabled] Pause inference deliberately to verify the original 20 ms guard and shutdown records.\n'
  bash "$stage/run-stage.sh" fault "$session"
  "$python" "$stage/check-records.py" "$session"
fi
if [[ "$mode" == --verify-only ]]; then
  printf 'Both disabled validations completed; no powered test started.\n'; exit 0
fi
printf '\n[Onsite confirmation required] The next stage enables motors and sends real forward commands.\n'
printf 'Keep still for approximately 15 seconds of IMU calibration. Control lasts about 50 seconds: 20 seconds zero command, 10-second sling window, 3-second acceleration, 10-second forward plateau, 3-second deceleration and final zero command.\n'
printf 'Require at least 5 metres of clear level travel with fall protection throughout. Start with the sling supporting the robot, both feet on the ground and nobody touching it.\n'
printf 'Stay at the main power switch with this terminal visible. Slacken the sling from outside the motion area only after the body-stability confirmation; retain fall protection.\n'
printf 'If disable feedback is incomplete, communication is lost, or any motor remains green, CUT MOTOR POWER.\n'
read -r -p 'If all onsite conditions are satisfied, type READY to start; anything else exits > ' answer
if [[ "$answer" != READY ]]; then printf 'Exited without starting the powered stage.\n'; exit 0; fi
set +e
bash "$stage/run-stage.sh" powered "$session" "$validation"
result=$?
set -e
"$python" "$stage/check-records.py" "$session" --powered
exit "$result"
