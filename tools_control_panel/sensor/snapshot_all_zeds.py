#!/usr/bin/env python3
"""
Detect all connected ZED cameras, grab one frame each, save as PNG.

Known cameras (from config):
  back  -> 48335070
  front -> 49537850

New cameras (right/left) are auto-detected from whatever serials remain.
Output: test_back.png, test_front.png, test_right.png, test_left.png
"""

import sys
import os
import cv2
import pyzed.sl as sl

KNOWN = {
    48335070: "back",
    49537850: "front",
}

NEW_NAMES = ["right", "left"]


def get_connected_serials() -> list[int]:
    devices = sl.Camera.get_device_list()
    return [int(d.serial_number) for d in devices]


def grab_snapshot(serial: int, label: str, out_path: str) -> bool:
    cam = sl.Camera()
    init = sl.InitParameters()
    init.set_from_serial_number(serial)
    init.depth_mode        = sl.DEPTH_MODE.NONE
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.camera_fps        = 30
    init.sdk_verbose       = False
    init.sensors_required  = False

    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        print(f"[{label}] open failed (serial={serial})")
        return False

    mat     = sl.Mat()
    runtime = sl.RuntimeParameters()

    grabbed = False
    for _ in range(30):
        if cam.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            grabbed = True
            break

    if not grabbed:
        print(f"[{label}] grab failed")
        cam.close()
        return False

    cam.retrieve_image(mat, sl.VIEW.LEFT)
    rgba = mat.get_data()
    rgb  = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
    cv2.imwrite(out_path, rgb)
    cam.close()
    print(f"[{label}] saved -> {out_path}")
    return True


def main():
    serials = get_connected_serials()
    if not serials:
        sys.exit("No ZED cameras detected.")

    print(f"Detected serials: {serials}")

    known_serials = set(KNOWN.keys())
    new_serials   = [s for s in serials if s not in known_serials]

    if len(new_serials) != 2:
        print(f"Warning: expected 2 new cameras, found {len(new_serials)}: {new_serials}")

    label_map: dict[int, str] = dict(KNOWN)
    for i, serial in enumerate(new_serials):
        label_map[serial] = NEW_NAMES[i] if i < len(NEW_NAMES) else f"cam_{i}"

    out_dir = os.path.dirname(os.path.abspath(__file__))
    results = {}
    for serial in serials:
        label    = label_map[serial]
        out_path = os.path.join(out_dir, f"test_{label}.png")
        results[label] = grab_snapshot(serial, label, out_path)

    print("\n--- Summary ---")
    for label, ok in results.items():
        print(f"  {label}: {'OK' if ok else 'FAILED'}")


if __name__ == "__main__":
    main()