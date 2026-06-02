# ArmPi-FPV Systemd Boot Services

This directory contains small helpers for running the ArmPi-FPV Ariel stack at
boot on the Raspberry Pi.

The normal boot stack is:

1. `ariel-armpifpv-hardware.service`
   - runs `python -m robots.armpifpv.hardware`
   - owns `/dev/rrc`, cameras, and the local Roboflex graph

2. `ariel-mcp.service`
   - runs `python -m server.mcp_server`
   - launches the Ariel REPL subprocess automatically
   - exposes `http://<pi-ip>:8750/mcp`

ArmPi-FPV currently uses local `ipc://...` ZMQ endpoints, so the hardware
process and MCP server should run on the same machine.

## Prerequisites

Run this from the Ariel checkout on the Raspberry Pi, after the ArmPi-FPV
virtualenv is already created:

```bash
cd /home/pi/arieltesting/Ariel
test -x robots/armpifpv/.pyvenv/bin/python
cat robot.conf
```

`robot.conf` should contain:

```text
robots.armpifpv.robot:ArmPiFPVRobotProxy
```

The vendor stack must not own `/dev/rrc` while Ariel is running. Disable it once:

```bash
sudo systemctl disable --now start_node.service
docker stop armpi_fpv
docker update --restart=no armpi_fpv
```

If the Docker container is not installed, the `docker` commands can fail safely.

## Install

From the repo root:

```bash
robots/armpifpv/systemd/install.sh
```

By default the services run as the current user and use:

- repo root: auto-detected from this script location
- Python: `robots/armpifpv/.pyvenv/bin/python`
- RealSense: enabled
- image return mode: inline

Useful overrides:

```bash
ARIEL_SYSTEMD_USER=pi robots/armpifpv/systemd/install.sh
ARMPIFPV_REALSENSE_ENABLE=0 robots/armpifpv/systemd/install.sh
ARIEL_IMAGE_RETURN_MODE=file robots/armpifpv/systemd/install.sh
```

## Start, Stop, Status

```bash
robots/armpifpv/systemd/start.sh
robots/armpifpv/systemd/status.sh
robots/armpifpv/systemd/stop.sh
```

Follow logs:

```bash
journalctl -u ariel-armpifpv-hardware.service -f
journalctl -u ariel-mcp.service -f
```

## Uninstall

```bash
robots/armpifpv/systemd/uninstall.sh
```

This stops and disables the two Ariel services and removes their unit files from
`/etc/systemd/system`.

## Notes

- The MCP service has `Requires=ariel-armpifpv-hardware.service`, so stopping
  the hardware service also stops MCP.
- Both services use `Restart=always` so they come back after crashes.
- Systemd runs the services with `WorkingDirectory` set to the repo root. This
  matters for the current `ipc://...` ZMQ endpoint names.
- The installer does not mutate `robot.conf`; check it before installing.
