#!/usr/bin/env bash
set -euo pipefail

sudo systemctl stop ariel-mcp.service 2>/dev/null || true
sudo systemctl stop ariel-armpifpv-hardware.service 2>/dev/null || true
sudo systemctl disable ariel-mcp.service 2>/dev/null || true
sudo systemctl disable ariel-armpifpv-hardware.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/ariel-mcp.service
sudo rm -f /etc/systemd/system/ariel-armpifpv-hardware.service
sudo systemctl daemon-reload

echo "Removed Ariel ArmPi-FPV systemd services."
