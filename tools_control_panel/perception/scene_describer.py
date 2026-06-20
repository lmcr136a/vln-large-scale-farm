"""
SceneDescriber — periodic per-camera scene description from the 4 RGB cameras.

Every `vlm.scene_interval_sec` (default 10s), each of the front/back/left/right
frames is sent to the VLM (same backend/model as the landmark detector's vlm:
config in farm_config.yaml) SEPARATELY, with a camera-specific prompt asking
for a one-sentence description of the flax crop's condition. The 4 results are
pushed to the web panel as one 'scene_description' event ({'descriptions':
{camera: text}}), and each camera's frame is logged to disk under
<data_dir>/scene_log/<timestamp>_<camera>.png with its own description burned
into a caption bar at the bottom of the image.
"""
import base64
import io
import logging
import os
import threading
import time

log = logging.getLogger(__name__)

CAM_LABEL = {'front': 'front', 'back': 'back', 'left': 'left', 'right': 'right'}


def _build_prompt(camera: str) -> str:
    """Camera-specific prompt — one independent description per camera, not a
    fused summary, so each camera's view of the crop can be judged on its own."""
    return (
        f'This is the {CAM_LABEL.get(camera, camera)} camera of a robot driving '
        'around a flax farm. There might be utility boxes and trailers. Focusing on '
        "the flax crop's size, growth stage, and any signs of pests or disease "
        'damage, describe what is visible in 2-3 short plain sentences '
        '(max ~50 words), in English.'
    )


MAX_IMG_WIDTH = 480


class SceneDescriber:
    def __init__(self, config: dict, proxy):
        vlm_cfg = config.get('vlm', {})
        self._proxy      = proxy
        self._backend    = vlm_cfg.get('backend',    'ollama')
        self._model      = vlm_cfg.get('model',      'qwen2.5vl:3b')
        self._api_key    = vlm_cfg.get('api_key',    '')
        self._ollama_url = vlm_cfg.get('ollama_url', 'http://localhost:11434')
        self._keep_alive = vlm_cfg.get('keep_alive', '10m')
        self._timeout    = float(vlm_cfg.get('timeout_sec',        150.0))
        self._interval   = float(vlm_cfg.get('scene_interval_sec', 10.0))
        self._enabled    = (self._backend == 'ollama') or \
                           (self._backend == 'claude' and bool(self._api_key))
        self._log_dir    = os.path.join(config.get('paths', {}).get('data_dir', '.'), 'scene_log')

        self._lock    = threading.Lock()
        self._images: dict[str, tuple[float, bytes]] = {}
        self._running = False

    def on_image_b64(self, camera: str, b64: str):
        try:
            data = base64.b64decode(b64)
        except Exception:
            return
        with self._lock:
            self._images[camera] = (time.time(), data)

    def start(self):
        if not self._enabled:
            log.info('SceneDescriber disabled (check vlm config)')
            return
        self._running = True
        threading.Thread(target=self._loop, daemon=True, name='scene-describer').start()
        log.info(f'SceneDescriber started [{self._backend}] interval={self._interval}s')

    def stop(self):
        self._running = False

    def _loop(self):
        while self._running:
            time.sleep(self._interval)
            try:
                self._describe_once()
            except Exception as e:
                log.warning(f'SceneDescriber: {e}')

    def _images_fresh(self, max_age: float = 15.0) -> bool:
        now = time.time()
        with self._lock:
            if not self._images:
                return False
            return all(now - ts < max_age for ts, _ in self._images.values())

    def _describe_once(self):
        if not self._images_fresh():
            return
        with self._lock:
            snapshot = {cam: data for cam, (_, data) in self._images.items()}

        descriptions: dict[str, str] = {}
        for cam in ('front', 'back', 'left', 'right'):
            if cam not in snapshot:
                continue
            b64  = base64.b64encode(snapshot[cam]).decode()
            text = self._call_ollama(cam, b64) if self._backend == 'ollama' \
                   else self._call_claude(cam, b64)
            if not text:
                continue
            descriptions[cam] = ' '.join(text.strip().split())  # collapse whitespace/newlines

        if not descriptions:
            return
        log.info(f'[SceneDescriber] {descriptions}')
        self._proxy.emit('scene_description', {'descriptions': descriptions}, namespace='/')
        self._save_log(snapshot, descriptions)

    LOG_MAX_SIDE  = 720   # resize so the long edge is this many pixels
    LOG_FONT_PT   = 10    # caption font size, ~10pt @ 96dpi
    _FONT_PATHS = (
        '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
        '/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf',
        '/usr/share/fonts/truetype/freefont/FreeSans.ttf',
    )

    @classmethod
    def _load_font(cls, px: int):
        from PIL import ImageFont
        for p in cls._FONT_PATHS:
            try:
                return ImageFont.truetype(p, px)
            except Exception:
                continue
        return ImageFont.load_default()

    def _save_log(self, snapshot: dict, descriptions: dict):
        """Save one image per camera, overwriting the same path each cycle
        (latest_<camera>.png) rather than accumulating a new file every time."""
        try:
            from PIL import Image, ImageDraw
        except ImportError:
            return
        try:
            os.makedirs(self._log_dir, exist_ok=True)
        except Exception as e:
            log.warning(f'SceneDescriber: cannot create log dir: {e}')
            return
        font_px   = round(self.LOG_FONT_PT * 96 / 72)
        font      = self._load_font(font_px)
        line_h    = font_px + 4
        for cam, data in snapshot.items():
            text = descriptions.get(cam)
            if not text:
                continue
            try:
                img = Image.open(io.BytesIO(data)).convert('RGB')
                w, h = img.size
                if max(w, h) != self.LOG_MAX_SIDE:
                    scale = self.LOG_MAX_SIDE / max(w, h)
                    w, h = max(1, round(w * scale)), max(1, round(h * scale))
                    img = img.resize((w, h), Image.LANCZOS)
                draw = ImageDraw.Draw(img)
                n_lines = max(1, min(6, (h // 3) // line_h))   # 2-3 sentences need more room
                bar_h   = n_lines * line_h + 6
                draw.rectangle([0, h - bar_h, w, h], fill=(0, 0, 0))
                # Word-wrap to the actual rendered text width so lines don't run off
                words, lines, cur = text.split(), [], ''
                for word in words:
                    trial = f'{cur} {word}'.strip()
                    if cur and draw.textlength(trial, font=font) > w - 12:
                        lines.append(cur)
                        cur = word
                    else:
                        cur = trial
                if cur:
                    lines.append(cur)
                ty = h - bar_h + 3
                for line in lines[:n_lines]:
                    draw.text((6, ty), line, fill=(255, 255, 255), font=font)
                    ty += line_h
                path = os.path.join(self._log_dir, f'latest_{cam}.png')
                img.save(path, 'PNG')
            except Exception as e:
                log.warning(f'SceneDescriber: log save failed [{cam}]: {e}')

    def _call_ollama(self, camera: str, b64: str) -> str | None:
        try:
            import requests
        except ImportError:
            return None
        payload = {
            'model': self._model, 'prompt': _build_prompt(camera),
            'images': [self._resize_jpeg(b64)], 'stream': False,
            'keep_alive': self._keep_alive,
            'options': {'temperature': 0.2, 'num_predict': 140},
        }
        try:
            resp = requests.post(f'{self._ollama_url}/api/generate',
                                 json=payload, timeout=self._timeout)
            resp.raise_for_status()
            return resp.json().get('response', '').strip() or None
        except Exception as e:
            log.warning(f'Ollama scene call failed [{camera}]: {e}')
            return None

    def _call_claude(self, camera: str, b64: str) -> str | None:
        try:
            import anthropic
        except ImportError:
            log.warning('pip install anthropic')
            return None
        try:
            client  = anthropic.Anthropic(api_key=self._api_key)
            content = [
                {'type': 'image',
                 'source': {'type': 'base64', 'media_type': 'image/jpeg', 'data': b64}},
                {'type': 'text', 'text': _build_prompt(camera)},
            ]
            resp = client.messages.create(model=self._model, max_tokens=140,
                                          messages=[{'role': 'user', 'content': content}])
            return resp.content[0].text.strip() or None
        except Exception as e:
            log.warning(f'Claude scene call failed [{camera}]: {e}')
            return None

    @staticmethod
    def _resize_jpeg(b64: str, max_width: int = MAX_IMG_WIDTH) -> str:
        try:
            import io
            from PIL import Image
            raw = base64.b64decode(b64)
            img = Image.open(io.BytesIO(raw)).convert('RGB')
            w, h = img.size
            if w > max_width:
                img = img.resize((max_width, int(h * max_width / w)), Image.LANCZOS)
            buf = io.BytesIO()
            img.save(buf, 'JPEG', quality=85)
            return base64.b64encode(buf.getvalue()).decode()
        except Exception:
            return b64
