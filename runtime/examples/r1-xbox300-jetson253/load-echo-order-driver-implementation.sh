#!/usr/bin/env bash
# Temporary timestamp bookkeeping fix; no motor frames are sent.
set -euo pipefail
stage=/home/tony/sprite-sm-walk-r1-fast-20261119
base=/home/tony/supermimic-run002-20261106
previous=$base/walk025-sync-1110/driver-tx-ioctl-v2/kcan.ko
[[ $(uname -r) == 5.15.199-tegra ]] || { echo 'Unexpected kernel.'; exit 2; }
for dependency in "$previous" "$stage/driver-echo-order/kcan.ko" \
  "$stage/driver-echo-order.sha256" "$stage/tools/can_tx_timestamp_setup.py" \
  "$stage/build/sprite_can_shadow" /home/tony/setup_sprite0825_kcanfd_on_253.sh; do
  [[ -f "$dependency" && -r "$dependency" ]] || { echo "Missing dependency: $dependency"; exit 2; }
done
for command in modprobe insmod udevadm ip flock sha256sum /usr/bin/python3; do
  command -v "$command" >/dev/null || { echo "Missing command: $command"; exit 2; }
done
(cd "$stage" && sha256sum -c driver-echo-order.sha256)
/usr/bin/python3 "$stage/tools/can_tx_timestamp_setup.py" --help >/dev/null
if [[ ${1:-} == --check ]]; then
  echo 'Loader dependencies and candidate hash verified. No hardware access.'
  exit 0
fi
[[ $# -eq 0 ]] || { echo 'Usage: load-echo-order-driver.sh [--check]'; exit 2; }
[[ $EUID -eq 0 ]] || { echo 'Run this script with sudo.'; exit 2; }
exec 9>>"$base/commissioning.lock"
flock -n 9 || { echo 'Controller/commissioning lock busy; stop.'; exit 2; }
test -f "$previous"
(cd "$stage" && sha256sum -c driver-echo-order.sha256)
rollback() {
  code=$?
  trap - EXIT
  if [[ $code -ne 0 ]]; then
    echo 'Candidate setup failed; restoring the previous temporary timestamp driver.'
    set +e
    modprobe -r kcan
    insmod "$previous"
    udevadm settle
    bash /home/tony/setup_sprite0825_kcanfd_on_253.sh
    /usr/bin/python3 "$stage/tools/can_tx_timestamp_setup.py" --apply --lock "$base/commissioning.lock" --inherited-lock-fd 9
  fi
  exit "$code"
}
trap rollback EXIT
modprobe -r kcan
insmod "$stage/driver-echo-order/kcan.ko"
udevadm settle
for retry in {1..50}; do
  [[ -e /sys/class/net/kcan1 && -e /sys/class/net/kcan2 && -e /sys/class/net/kcan3 && -e /sys/class/net/kcan4 ]] && break
  sleep 0.1
done
bash /home/tony/setup_sprite0825_kcanfd_on_253.sh
/usr/bin/python3 "$stage/tools/can_tx_timestamp_setup.py" --apply --lock "$base/commissioning.lock" --inherited-lock-fd 9
"$stage/build/sprite_can_shadow" --check-commissioning-scheduler 2
echo 'Echo registration now precedes TX publication. Four buses and timestamps are configured.'
echo 'No motor frames sent. Installed driver unchanged; reboot restores the installed driver.'
