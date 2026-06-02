#!/usr/bin/env bash
set -euo pipefail

sudo systemctl stop ariel-mcp.service
sudo systemctl stop ariel-armpifpv-hardware.service
systemctl --no-pager --full status ariel-armpifpv-hardware.service ariel-mcp.service || true
