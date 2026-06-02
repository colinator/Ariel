#!/usr/bin/env bash
set -euo pipefail

systemctl --no-pager --full status ariel-armpifpv-hardware.service ariel-mcp.service
