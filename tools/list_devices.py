#!/usr/bin/env python3
"""Print STABLE device identifiers for arms (by-id) and cameras (by-path).

Use these in devices.env instead of /dev/ttyACMn or /dev/videoN, which renumber
on reboot/replug.
"""
import glob
import os

print("== Arm serial ports (by-id) ==")
found = sorted(glob.glob("/dev/serial/by-id/*"))
for p in found:
    print(f"  {p}  ->  {os.path.realpath(p)}")
if not found:
    print("  (none -- plug the servo-bus adapters in)")

print("\n== Cameras (by-path, capture nodes) ==")
found = sorted(glob.glob("/dev/v4l/by-path/*-index0"))
for p in found:
    print(f"  {p}  ->  {os.path.realpath(p)}")
if not found:
    print("  (none -- plug the cameras in)")
