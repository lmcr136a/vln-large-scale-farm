"""
SvoPlayer: reads SVO2 files and pushes frames to landmark_detector + web stream.

Used in replay mode alongside `ros2 bag play` to feed camera data.
One thread per camera, paced at stream_fps.
"""
import base64
import logging
import os
import threading
import time

import cv2

log = logging.getLogger(__name__)

CAMERA_NAMES = ('front', 'back', 'left', 'right')


class SvoPlayer:
    """Replays SVO2 files from a session directory.

    Args:
        session_dir: path containing rgbd_<name>.svo2 files
        on_frame:    callback(camera_name: str, jpeg_bytes: bytes) called per frame
        stream_fps:  target playback rate (web stream rate, typically 5 Hz)
    """

    def __init__(self, session_dir: str, on_frame=None, on_depth=None,
                 stream_fps: float = 5.0):
        self._session_dir   = session_dir
        self._on_frame      = on_frame
        self._on_depth      = on_depth      # cb(camera, depth_arr: np.ndarray, intrinsics: dict)
        self._stream_period = 1.0 / max(stream_fps, 0.1)
        self._running       = False
        self._threads: list[threading.Thread] = []

    def start(self):
        self._running = True
        for name in CAMERA_NAMES:
            svo_path = os.path.join(self._session_dir, f'rgbd_{name}.svo2')
            if not os.path.exists(svo_path):
                log.info(f'SvoPlayer: {name} not found, skipping')
                continue
            t = threading.Thread(
                target=self._play_cam,
                args=(name, svo_path),
                daemon=True,
                name=f'svo-{name}',
            )
            t.start()
            self._threads.append(t)
        log.info(f'SvoPlayer: started {len(self._threads)} camera(s) from {self._session_dir}')

    def stop(self):
        self._running = False

    def join(self, timeout: float = 10.0):
        for t in self._threads:
            t.join(timeout=timeout)

    # ── Per-camera thread ────────────────────────────────────────────────────

    def _play_cam(self, name: str, svo_path: str):
        try:
            import pyzed.sl as sl
        except ImportError:
            log.warning('pyzed not installed — SvoPlayer disabled')
            return

        init = sl.InitParameters()
        init.set_from_svo_file(svo_path)
        init.svo_real_time_mode = True   # SDK paces grab() to original recording timestamps
        init.coordinate_units   = sl.UNIT.METER
        init.depth_mode         = sl.DEPTH_MODE.QUALITY   # accurate stereo depth for landmark localization

        cam = sl.Camera()
        err = cam.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            log.warning(f'SvoPlayer [{name}]: cannot open {svo_path}: {err}')
            return

        # Camera intrinsics for back-projection (pixel + depth → 3D)
        cam_info = cam.get_camera_information()
        calib    = cam_info.camera_configuration.calibration_parameters.left_cam
        intrinsics = {
            'fx': float(calib.fx),
            'fy': float(calib.fy),
            'cx': float(calib.cx),
            'cy': float(calib.cy),
        }

        mat          = sl.Mat()
        depth_mat    = sl.Mat()
        rt           = sl.RuntimeParameters()
        next_send_t  = time.time()

        try:
            while self._running:
                # grab() blocks until the next frame's original timestamp arrives → real-time sync
                if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
                    log.info(f'SvoPlayer [{name}]: end of file')
                    break

                # Throttle web stream to stream_fps; skip intermediate frames
                now = time.time()
                if now < next_send_t:
                    continue
                next_send_t = now + self._stream_period

                cam.retrieve_image(mat, sl.VIEW.LEFT, sl.MEM.CPU)
                rgba = mat.get_data()
                bgr  = cv2.cvtColor(rgba, cv2.COLOR_BGRA2BGR)  # ZED returns BGRA on Linux/CUDA

                ok, buf = cv2.imencode(
                    '.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85]
                )
                if ok and self._on_frame:
                    try:
                        self._on_frame(name, buf.tobytes())
                    except Exception as e:
                        log.debug(f'SvoPlayer on_frame error: {e}')

                # Depth at same rate as RGB — used by landmark_detector for 3D localization
                if self._on_depth:
                    try:
                        cam.retrieve_measure(depth_mat, sl.MEASURE.DEPTH, sl.MEM.CPU)
                        self._on_depth(name, depth_mat.get_data().copy(), intrinsics)
                    except Exception as e:
                        log.debug(f'SvoPlayer on_depth error: {e}')
        finally:
            cam.close()
            log.info(f'SvoPlayer [{name}]: closed')
