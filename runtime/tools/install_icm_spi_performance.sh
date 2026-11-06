#!/usr/bin/env bash
# One-time root setup. Only SPI0 runtime PM; no CAN, motor, clock or governor setup.
set -euo pipefail
runtime_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="$runtime_root/config/systemd"
control=/sys/bus/platform/devices/3210000.spi/power/control
rule=/etc/udev/rules.d/80-open-sprite-spi-performance.rules
unit=/etc/systemd/system/open-sprite-spi-performance.service
if [[ "$(readlink -f /sys/class/spidev/spidev0.0/device)" != *'/3210000.spi/'* ]]; then
  echo 'spidev0.0 does not map to the qualified 3210000.spi controller' >&2
  exit 1
fi
if [[ ! -f "$control" ]]; then
  echo "Qualified Orin SPI0 controller is missing: $control" >&2
  exit 1
fi
for name in 80-open-sprite-spi-performance.rules open-sprite-spi-performance.service; do
  [[ -f "$source_dir/$name" ]] || { echo "Missing source: $name" >&2; exit 1; }
done
if [[ "${1:-}" == --dry-run ]]; then
  printf 'Current SPI0 runtime PM: '; cat "$control"
  printf 'Would install %s and %s, enable the service and set SPI0 to on.\n' "$rule" "$unit"
  exit 0
fi
if [[ $# != 0 ]]; then echo 'Usage: install_icm_spi_performance.sh [--dry-run]' >&2; exit 2; fi
if (( EUID != 0 )); then
  echo 'Run once with sudo; no password is stored by this script.' >&2
  exit 3
fi
for destination in "$rule" "$unit"; do
  if [[ -e "$destination" ]]; then
    cp --preserve=all -- "$destination" "${destination}.before-$(date +%Y%m%d_%H%M%S)"
  fi
done
install -m 0644 "$source_dir/80-open-sprite-spi-performance.rules" "$rule"
install -m 0644 "$source_dir/open-sprite-spi-performance.service" "$unit"
udevadm control --reload-rules
systemctl daemon-reload
systemctl enable open-sprite-spi-performance.service
systemctl restart open-sprite-spi-performance.service
[[ "$(cat "$control")" == on ]]
echo 'SPI0 runtime PM is on, persisted for boot/device bind. Exit does not restore auto.'
echo 'No CAN configuration, motor frame, enable, clock or CPU-governor change was made.'
