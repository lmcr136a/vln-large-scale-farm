#!/usr/bin/env python3

import re
from pathlib import Path

RESOLUTION_FINE   = 0.05   # feature matching / new points
RESOLUTION_COARSE = 0.05   # global map

# matching/correspondence (fine)
FINE_KEYS = {
    "ivox_resolution",
    "vgicp_resolution",
    "vgicp_voxel_resolution",
    "gicp_max_correspondence_dist",
    "keyframe_voxel_resolution",
}

COARSE_KEYS = {
    "submap_voxel_resolution",
    "submap_downsample_resolution",
    "voxel_resolution",
}

KEY_PATTERN = re.compile(
    r'("(?P<key>[^"]+)"\s*:\s*)(?P<val>-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)'
)

def replace_resolutions(text: str) -> tuple[str, list[str]]:
    changes = []

    def replacer(m):
        key = m.group("key")
        old_val = m.group("val")
        if key in FINE_KEYS:
            new_val = str(RESOLUTION_FINE)
        elif key in COARSE_KEYS:
            new_val = str(RESOLUTION_COARSE)
        else:
            return m.group(0)
        changes.append(f"  {key}: {old_val} -> {new_val}")
        return m.group(1) + new_val

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

print(f"\n✓ fine={RESOLUTION_FINE}  coarse={RESOLUTION_COARSE}  preprocess={RESOLUTION_FINE}")