#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

SERVICE_USER="${ARIEL_SYSTEMD_USER:-$(id -un)}"
PYTHON_BIN="${ARIEL_PYTHON:-$REPO_ROOT/robots/armpifpv/.pyvenv/bin/python}"
REALSENSE_ENABLE="${ARMPIFPV_REALSENSE_ENABLE:-1}"
IMAGE_RETURN_MODE="${ARIEL_IMAGE_RETURN_MODE:-inline}"

HARDWARE_UNIT="/etc/systemd/system/ariel-armpifpv-hardware.service"
MCP_UNIT="/etc/systemd/system/ariel-mcp.service"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Python not found or not executable: $PYTHON_BIN" >&2
  echo "Set ARIEL_PYTHON=/path/to/python if your venv lives elsewhere." >&2
  exit 1
fi

if [[ ! -f "$REPO_ROOT/robot.conf" ]]; then
  echo "robot.conf not found under repo root: $REPO_ROOT" >&2
  exit 1
fi

if ! grep -qx "robots.armpifpv.robot:ArmPiFPVRobotProxy" "$REPO_ROOT/robot.conf"; then
  echo "WARNING: robot.conf does not point at ArmPi-FPV:" >&2
  cat "$REPO_ROOT/robot.conf" >&2
  echo >&2
  echo "Expected: robots.armpifpv.robot:ArmPiFPVRobotProxy" >&2
fi

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

cat > "$tmpdir/ariel-armpifpv-hardware.service" <<EOF
[Unit]
Description=Ariel ArmPi-FPV hardware server
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$SERVICE_USER
WorkingDirectory=$REPO_ROOT
Environment=GST_DEBUG=0
Environment=G_MESSAGES_DEBUG=
Environment=ARMPIFPV_REALSENSE_ENABLE=$REALSENSE_ENABLE
ExecStart=$PYTHON_BIN -m robots.armpifpv.hardware
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

cat > "$tmpdir/ariel-mcp.service" <<EOF
[Unit]
Description=Ariel MCP server
After=network-online.target ariel-armpifpv-hardware.service
Wants=network-online.target
Requires=ariel-armpifpv-hardware.service

[Service]
Type=simple
User=$SERVICE_USER
WorkingDirectory=$REPO_ROOT
Environment=ARIEL_IMAGE_RETURN_MODE=$IMAGE_RETURN_MODE
Environment=ARMPIFPV_REALSENSE_ENABLE=$REALSENSE_ENABLE
ExecStart=$PYTHON_BIN -m server.mcp_server
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

echo "Installing systemd units:"
echo "  $HARDWARE_UNIT"
echo "  $MCP_UNIT"
echo
echo "Service user: $SERVICE_USER"
echo "Repo root:    $REPO_ROOT"
echo "Python:       $PYTHON_BIN"

sudo install -m 0644 "$tmpdir/ariel-armpifpv-hardware.service" "$HARDWARE_UNIT"
sudo install -m 0644 "$tmpdir/ariel-mcp.service" "$MCP_UNIT"
sudo systemctl daemon-reload
sudo systemctl enable ariel-armpifpv-hardware.service
sudo systemctl enable ariel-mcp.service

echo
echo "Installed and enabled. Start now with:"
echo "  $SCRIPT_DIR/start.sh"
