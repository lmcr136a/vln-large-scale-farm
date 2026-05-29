#!/usr/bin/env python3

import re
from pathlib import Path

# Point/map density: smaller = finer detail. Safe to go to 5cm or below.
RESOLUTION_POINTS = 0.05
# Submap downsample / stored map: keep coarser so submaps stay light on Jetson.
RESOLUTION_COARSE = 0.10
# VGICP registration voxel: must NOT be tiny, or per-voxel covariances become
# unstable and rotation alignment breaks. 0.25-0.5m for a closed indoor room.
RESOLUTION_VGICP = 0.30

# Registration voxels (handled separately from density)
VGICP_KEYS = {
    "vgicp_resolution",
    "vgicp_voxel_resolution",
}

# Fine: point density / correspondence distance
FINE_KEYS = {
    "ivox_resolution",
    "gicp_max_correspondence_dist",
    "keyframe_voxel_resolution",
}

# Coarse: submap downsample / stored map
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
        if key in VGICP_KEYS:
            new_val = str(RESOLUTION_VGICP)
        elif key in FINE_KEYS:
            new_val = str(RESOLUTION_POINTS)
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

print(f"\n✓ points={RESOLUTION_POINTS}  coarse={RESOLUTION_COARSE}  vgicp={RESOLUTION_VGICP}")