#!/usr/bin/env bash
set -euo pipefail

sudo systemctl start ariel-armpifpv-hardware.service
sudo systemctl start ariel-mcp.service
systemctl --no-pager --full status ariel-armpifpv-hardware.service ariel-mcp.service
