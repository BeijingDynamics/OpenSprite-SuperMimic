#!/usr/bin/env bash
# Install only the root-owned narrow CPU frequency service, without robot IO.
set -euo pipefail
if [[ $EUID -ne 0 ]]; then
  echo 'Run this installer with sudo; it only installs the CPU frequency lease service.' >&2
  exit 1
fi
source_dir=$(cd -- "$(dirname -- "$0")" && pwd)
lease_user=${SUDO_USER:-tony}
lease_uid=$(id -u "$lease_user")
lease_group=$(id -gn "$lease_user")
install -d -m 0755 /usr/local/libexec
install -o root -g root -m 0755 "$source_dir/cpu_frequency_guard.py" /usr/local/libexec/open-sprite-cpu-frequency.py
cat > /etc/systemd/system/open-sprite-cpu-frequency.service <<EOF
[Unit]
Description=Open Sprite inference CPU 4/5 frequency lease
After=local-fs.target

[Service]
Type=simple
User=root
Group=$lease_group
UMask=0007
CPUAffinity=0
RuntimeDirectory=open-sprite-cpu-frequency
RuntimeDirectoryMode=0750
RuntimeDirectoryPreserve=yes
ExecStart=/usr/bin/python3 /usr/local/libexec/open-sprite-cpu-frequency.py --uid $lease_uid
ExecStopPost=/usr/bin/python3 /usr/local/libexec/open-sprite-cpu-frequency.py --uid $lease_uid --restore
Restart=on-failure
RestartSec=1
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable --now open-sprite-cpu-frequency.service
echo 'CPU frequency lease service installed. Frequency changes only while a client lease is active.'
echo 'No CAN, IMU, policy or motor process was started.'
