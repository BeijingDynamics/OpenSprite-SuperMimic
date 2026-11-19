#!/usr/bin/env bash
# One post-reboot entry point for the qualified Jetson CAN/timestamp setup.
# Delegates to the sealed loader, including its controller lock and rollback.
set -euo pipefail

stage=/home/tony/supermimic-run002-20261106/walk025-sync-en-1111
loader="$stage/load-tx-timestamp-driver.sh"
loader_sha=ff5c97b558e2fe244e8d5f3c7bc28535fcafc4b2259e7727fe068b5fcf5860b1
can_setup=/home/tony/setup_sprite0825_kcanfd_on_253.sh
can_setup_sha=909de0e4b30cf0bfdfb0177f346b32e08f72fcd85aaccb70fc591fe3e33176e5

if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --check ) ]]; then
  echo 'Usage: sudo bash setup_supermimic_can_on_253.sh [--check]' >&2
  exit 2
fi
[[ $(uname -r) == 5.15.199-tegra ]] || { echo 'Unsupported kernel; refusing setup.' >&2; exit 2; }
for spec in "$loader_sha:$loader" "$can_setup_sha:$can_setup"; do
  expected=${spec%%:*}
  path=${spec#*:}
  actual=$(sha256sum -- "$path")
  [[ ${actual%% *} == "$expected" ]] || { echo "Setup file identity mismatch: $path" >&2; exit 2; }
done
(cd "$stage" && sha256sum -c driver-candidate.sha256)
if [[ ${1:-} == --check ]]; then
  echo 'Setup dependencies verified. No driver, interface or timestamp setting changed.'
  echo 'This check does not certify that the running CAN interfaces are ready.'
  exit 0
fi
if (( EUID != 0 )); then
  echo 'Root is required to reload the CAN driver and configure interfaces.'
  exec sudo -- bash "$(readlink -f -- "${BASH_SOURCE[0]}")"
fi
echo 'Configuring kcan1..kcan4, hardware TX/RX timestamps, and CAN CPU 2 / FIFO 50 permission.'
echo 'The qualified loader refuses a busy commissioning lock. No motor frames are sent.'
bash "$loader"
echo 'SuperMimic CAN setup complete. No separate CAN setup or timestamp command is needed.'
echo 'Run this setup again after reboot, before starting a robot test.'
