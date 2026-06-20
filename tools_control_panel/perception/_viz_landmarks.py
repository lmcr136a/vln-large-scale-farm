"""
Quick diagnostic plot: GPS track + crop field boundary + detected landmarks.
Run manually: python3 perception/_viz_landmarks.py [out.png]
"""
import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _gps_to_enu(lat, lon, olat, olon):
    R = 6378137.0
    x = R * math.radians(lon - olon) * math.cos(math.radians(olat))
    y = R * math.radians(lat - olat)
    return x, y


def main(out_path):
    landmarks_path = os.path.join(BASE, 'data', 'landmarks.json')
    track_path     = os.path.join(BASE, 'data', 'gps_track.json')
    crop_path      = os.path.join(BASE, 'data', 'crop_field.json')

    lm_data = json.load(open(landmarks_path))
    origin  = lm_data.get('origin') or {}
    olat, olon = origin.get('lat'), origin.get('lon')
    landmarks = lm_data.get('landmarks', [])

    track = []
    if os.path.exists(track_path):
        try:
            track = json.load(open(track_path))
        except Exception as e:
            print(f'(track file mid-write, skipping: {e})')
    if olat is None and track:
        olat, olon = track[0]['lat'], track[0]['lon']

    crop = json.load(open(crop_path)) if os.path.exists(crop_path) else None

    fig, ax = plt.subplots(figsize=(10, 14), facecolor='white')

    # GPS track
    if track:
        xs = [p['x'] for p in track]
        ys = [p['y'] for p in track]
        ax.plot(xs, ys, '-', color='#3399ff', linewidth=0.8, alpha=0.7, label=f'GPS track ({len(track)} pts)')

    # Crop field boundary (south fixed, north sheared — matches landmark_map.py)
    if crop:
        STRIPE_TOTAL_W = 15 * (9 * 0.3048) + 14 * (5 * 0.3048)
        theta = math.radians(crop.get('rotation_deg', 0.0))
        dx_shift = (crop['y_north'] - crop['y_south']) * math.sin(theta)
        ref_x, y_s, y_n = crop['ref_x'], crop['y_south'], crop['y_north']
        west_x = ref_x - STRIPE_TOTAL_W
        poly = [(ref_x, y_s), (west_x, y_s), (west_x + dx_shift, y_n), (ref_x + dx_shift, y_n)]
        ax.add_patch(mpatches.Polygon(poly, closed=True, fill=False,
                                       edgecolor='green', linewidth=1.5, linestyle='--',
                                       label='Crop field boundary'))

    # Landmarks
    colors = {'metal_box': '#cc6600', 'trailer': '#cc0000', 'crop_stick': '#aa8800'}
    for lm in landmarks:
        if olat is None:
            break
        x, y = _gps_to_enu(lm['lat'], lm['lon'], olat, olon)
        w = lm.get('width_m', 1.0)
        col = colors.get(lm['type'], '#888888')
        ax.add_patch(mpatches.Circle((x, y), max(w / 2, 0.3), color=col, alpha=0.8))
        ax.text(x + 1, y, f"{lm['type'][:3]} n={lm['sighting_count']}", fontsize=6, color=col)

    ax.set_aspect('equal')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title(f'Landmarks={len(landmarks)}  Track pts={len(track)}')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, dpi=130)
    print(f'Saved {out_path}')


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/landmarks_viz.png'
    main(out)
