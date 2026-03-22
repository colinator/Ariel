#!/usr/bin/env python3
"""Discover available cameras via roboflex_webcam_gst / GStreamer."""

import roboflex.webcam_gst as rcw

print("=== Available Cameras ===\n")
print(rcw.get_device_list_string())

devices = rcw.get_device_list()
print(f"\n{len(devices)} device(s) found.")
for i, dev in enumerate(devices):
    print(f"\n--- Device index {i} ---")
    print(f"  Name: {dev.display_name}")
    print(f"  Path: {dev.device_path}")
    print(f"  Caps:  {dev.caps_strings}")
