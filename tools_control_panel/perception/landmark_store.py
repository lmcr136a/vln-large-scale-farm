"""
LandmarkStore: thread-safe JSON-backed store for farm landmarks.

Landmark types:
  metal_box   — large metal storage container
  trailer     — vehicle trailer or large movable structure
  crop_stick  — numbered stake near crop row

Each landmark is stored once and updated (position averaged, sighting count increased)
when a new detection falls within the merge radius.
"""
import json
import logging
import math
import os
import threading
import uuid
from datetime import datetime

log = logging.getLogger(__name__)

# Radius within which two detections of the same type are merged (metres)
MERGE_RADIUS = {
    'metal_box':  6.0,
    'trailer':    8.0,
    'crop_stick': 1.5,
}
DEFAULT_MERGE_RADIUS = 4.0


def _haversine_m(lat1, lon1, lat2, lon2) -> float:
    R = 6378137.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return R * 2 * math.asin(math.sqrt(min(a, 1.0)))


class LandmarkStore:
    def __init__(self, path: str):
        self._path = path
        self._lock = threading.Lock()
        self._landmarks: list[dict] = []
        self._origin: dict | None = None
        self._load()

    # ── Public API ───────────────────────────────────────────────────────────

    def set_origin(self, lat: float, lon: float):
        with self._lock:
            self._origin = {'lat': lat, 'lon': lon}

    def add_landmark(self,
                     lm_type: str,
                     lat: float,
                     lon: float,
                     description: str = '',
                     number: str | None = None,
                     width_m: float = 2.0,
                     height_m: float = 2.0,
                     confidence: float = 0.7) -> str:
        """
        Add or merge a landmark.  Returns the landmark id.
        """
        now = datetime.utcnow().isoformat()
        radius = MERGE_RADIUS.get(lm_type, DEFAULT_MERGE_RADIUS)

        with self._lock:
            for lm in self._landmarks:
                if lm['type'] != lm_type:
                    continue
                if _haversine_m(lat, lon, lm['lat'], lm['lon']) < radius:
                    # Merge: running average position
                    n = lm['sighting_count']
                    lm['lat'] = (lm['lat'] * n + lat) / (n + 1)
                    lm['lon'] = (lm['lon'] * n + lon) / (n + 1)
                    lm['sighting_count'] = n + 1
                    lm['last_seen'] = now
                    if number and not lm.get('number'):
                        lm['number'] = number
                    if description and not lm.get('description'):
                        lm['description'] = description
                    self._update_bbox(lm, lat, lon, width_m, height_m)
                    return lm['id']

            # New entry
            lm_id = str(uuid.uuid4())[:8]
            lm = {
                'id':             lm_id,
                'type':           lm_type,
                'lat':            lat,
                'lon':            lon,
                'description':    description,
                'number':         number,
                'confidence':     confidence,
                'sighting_count': 1,
                'first_seen':     now,
                'last_seen':      now,
            }
            self._update_bbox(lm, lat, lon, width_m, height_m)
            self._landmarks.append(lm)
            log.info(f'New landmark [{lm_type}] #{lm_id} at ({lat:.6f}, {lon:.6f})')
            return lm_id

    def get_all(self) -> list[dict]:
        with self._lock:
            return [dict(lm) for lm in self._landmarks]

    def save(self):
        with self._lock:
            data = {
                'landmarks': list(self._landmarks),
                'origin':    self._origin,
                'updated':   datetime.utcnow().isoformat(),
            }
        try:
            os.makedirs(os.path.dirname(self._path) or '.', exist_ok=True)
            with open(self._path, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            log.error(f'LandmarkStore.save: {e}')

    # ── Internal ─────────────────────────────────────────────────────────────

    @staticmethod
    def _update_bbox(lm: dict, lat: float, lon: float, width_m: float, height_m: float):
        half_lat = (height_m / 2) / 111319.5
        half_lon = (width_m  / 2) / (111319.5 * math.cos(math.radians(lat)))
        lm['lat_min'] = min(lm.get('lat_min',  1e9), lat - half_lat)
        lm['lat_max'] = max(lm.get('lat_max', -1e9), lat + half_lat)
        lm['lon_min'] = min(lm.get('lon_min',  1e9), lon - half_lon)
        lm['lon_max'] = max(lm.get('lon_max', -1e9), lon + half_lon)

    def _load(self):
        try:
            with open(self._path) as f:
                data = json.load(f)
            self._landmarks = data.get('landmarks', [])
            self._origin    = data.get('origin')
            log.info(f'Loaded {len(self._landmarks)} landmarks from {self._path}')
        except FileNotFoundError:
            pass
        except Exception as e:
            log.warning(f'LandmarkStore.load: {e}')
