#!/usr/bin/env python3
"""
Open ALL connected ZED cameras at the SAME TIME, hold them open,
grab a frame from each, save as PNG. This mirrors real runtime where
all 4 Argus sessions compete for GMSL bandwidth/power simultaneously.

Output: test_<label>.png for each camera.
"""

import sys
import os
import time
import cv2
import pyzed.sl as sl

KNOWN = {
    48335070: "back",
    49537850: "front",
}
NEW_NAMES = ["right", "left"]


def open_cam(serial: int, label: str):
    cam  = sl.Camera()
    init = sl.InitParameters()
    init.set_from_serial_number(serial)
    init.depth_mode        = sl.DEPTH_MODE.NONE
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.camera_fps        = 30
    init.sdk_verbose       = False
    init.sensors_required  = False
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[{label}] open FAILED: {status}")
        return None
    print(f"[{label}] open SUCCESS (serial={serial})")
    return cam


def main():
    devices = sl.Camera.get_device_list()
    serials = [int(d.serial_number) for d in devices]
    if not serials:
        sys.exit("No ZED cameras detected.")
    print(f"Detected serials: {serials}")

    new_serials = [s for s in serials if s not in KNOWN]
    label_map = dict(KNOWN)
    for i, s in enumerate(new_serials):
        label_map[s] = NEW_NAMES[i] if i < len(NEW_NAMES) else f"cam_{i}"

    # 1) open ALL first, keep them all open simultaneously
    cams = {}
    for s in serials:
        label = label_map[s]
        cam = open_cam(s, label)
        if cam is not None:
            cams[label] = cam

    if len(cams) != len(serials):
        print(f"\n>>> Only {len(cams)}/{len(serials)} cameras opened concurrently <<<")
    else:
        print(f"\n>>> All {len(serials)} cameras open at once <<<")

    # 2) hold a moment so all streams run together under load
    time.sleep(2.0)

    # 3) grab from each while all are still open
    out_dir = os.path.dirname(os.path.abspath(__file__))
    runtime = sl.RuntimeParameters()
    results = {}
    for label, cam in cams.items():
        ok = False
        for _ in range(60):
            if cam.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                ok = True
                break
            time.sleep(0.01)
        if ok:
            mat = sl.Mat()
            cam.retrieve_image(mat, sl.VIEW.LEFT)
            rgb = cv2.cvtColor(mat.get_data(), cv2.COLOR_RGBA2BGR)
            path = os.path.join(out_dir, f"test_{label}.png")
            cv2.imwrite(path, rgb)
            print(f"[{label}] grabbed -> {path}")
        else:
            print(f"[{label}] grab FAILED while concurrent")
        results[label] = ok

    # 4) close all
    for cam in cams.values():
        cam.close()

    print("\n--- Summary (concurrent) ---")
    for s in serials:
        label = label_map[s]
        state = "OK" if results.get(label) else "FAILED/NOT-OPENED"
        print(f"  {label}: {state}")


if __name__ == "__main__":
    main()