#!/usr/bin/env python3

import re
from pathlib import Path

NEW_RESOLUTION = 0.3
# -------------------------

RESOLUTION_KEYS = {
    "downsample_resolution", "vgicp_resolution", "ivox_resolution",
    "voxel_resolution", "keyframe_voxel_resolution",
    "submap_downsample_resolution", "submap_voxel_resolution",
    "vgicp_voxel_resolution", "gicp_max_correspondence_dist",
}

# Matches: "some_key": 0.5  or  "some_key": 1  (with optional trailing comma/comment)
KEY_PATTERN = re.compile(
    r'("(?P<key>[^"]+)"\s*:\s*)(?P<val>-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)'
)

def replace_resolutions(text: str) -> tuple[str, list[str]]:
    changes = []

    def replacer(m):
        key = m.group("key")
        old_val = m.group("val")
        if key in RESOLUTION_KEYS:
            if key == "downsample_resolution":
                new_val = str(float(NEW_RESOLUTION)*3)
            else:
                new_val = str(float(NEW_RESOLUTION))
            changes.append(f"  {key}: {old_val} -> {new_val}")
            return m.group(1) + new_val
        return m.group(0)

    new_text = KEY_PATTERN.sub(replacer, text)
    return new_text, changes


for path in sorted(Path(__file__).parent.glob("config*.json")):
    raw = path.read_text()
    new_text, changes = replace_resolutions(raw)
    if changes:
        print(f"\n{path.name}")
        for c in changes:
            print(c)
        path.write_text(new_text)
    else:
        print(f"\n{path.name}  (no resolution keys found)")

print(f"\n✓ NEW_RESOLUTION={NEW_RESOLUTION}")