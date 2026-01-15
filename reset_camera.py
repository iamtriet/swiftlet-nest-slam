#!/usr/bin/env python3
"""
RealSense Camera Hardware Reset Tool
Unlocks the camera when it shows "Camera Locked: YES"
"""
import pyrealsense2 as rs
import time

print("🔍 Searching for RealSense devices...")
ctx = rs.context()
devs = ctx.query_devices()

if len(devs) == 0:
    print("❌ No RealSense devices found!")
    print("   Make sure your L515 is connected via USB")
    exit(1)

print(f"✅ Found {len(devs)} device(s)")

for dev in devs:
    print(f"   Device: {dev.get_info(rs.camera_info.name)}")
    print(f"   Serial: {dev.get_info(rs.camera_info.serial_number)}")
    print("🔄 Resetting hardware...")
    dev.hardware_reset()

print("⏳ Waiting for device to reinitialize...")
time.sleep(3)

print("✅ Camera reset complete!")
print("   You can now run your SLAM system")
