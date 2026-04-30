# ArmPi-FPV Client Monitors

These scripts display the low-rate TCP monitor feeds published by the ArmPi-FPV hardware process.

They are meant to run on your client computer, not necessarily on the Raspberry Pi.

## Requirements

Install the viewer dependencies:

```bash
pip install -r robots/armpifpv/clientmonitors/requirements.txt
```

`roboflex.visualization` uses SDL2, so the client machine also needs SDL2 available.

## Hardware Side

Start the hardware process on the robot. The monitor feeds are enabled by default:

```bash
python -m robots.armpifpv.hardware
```

Default monitor endpoints:

- regular camera RGB/JPEG feed: `tcp://0.0.0.0:5560`
- RealSense frameset feed: `tcp://0.0.0.0:5561`

The monitor rate defaults to 1 Hz and can be changed with:

```bash
ARMPIFPV_MONITOR_HZ=2.0 python -m robots.armpifpv.hardware
```

## View Regular Camera

```bash
python -m robots.armpifpv.clientmonitors.view_camera_rgb 192.168.0.195
```

## View RealSense RGB

```bash
python -m robots.armpifpv.clientmonitors.view_realsense_rgb 192.168.0.195
```

Optional arguments for both scripts:

```text
host width height fps
```

Example:

```bash
python -m robots.armpifpv.clientmonitors.view_camera_rgb 192.168.0.195 640 480 1
```
