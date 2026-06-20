"""
SvoPlayer: reads SVO2 files and pushes frames to landmark_detector + web stream.

Used in replay mode alongside `ros2 bag play --clock`. Frame pacing is driven by
the ROS /clock topic rather than an independent wall-clock timer: each SVO
frame carries its own recorded nanosecond timestamp (ZED TIME_REFERENCE.IMAGE),
which lives in the *same* epoch as the rosbag's recorded message timestamps
(verified directly — a frame and the GPS fix recorded ~0.66s later carry
timestamps 0.66s apart). Waiting for /clock to reach a frame's own timestamp
before delivering it keeps GPS/IMU pose and camera frames in lockstep for the
whole session, immune to the two playback processes drifting apart under load
(independent wall-clock pacing on each side does NOT have this guarantee).
"""
import logging
import os
import threading
import time

import cv2

log = logging.getLogger(__name__)

CAMERA_NAMES = ('front', 'back', 'left', 'right')

CLOCK_WAIT_TIMEOUT_S = 5.0   # give up waiting (treat as "bag has moved on") after this long


class SvoPlayer:
    """Replays SVO2 files from a session directory.

    Args:
        session_dir: path containing rgbd_<name>.svo2 files
        on_frame:    callback(camera_name: str, jpeg_bytes: bytes) called per frame
        stream_fps:  target playback rate (web stream rate, typically 5 Hz)
        speed:       playback speed multiplier (1.0 = real-time, 3.0 = 3x faster).
                     Only used as a fallback when /clock sync is unavailable —
                     normally pacing is driven entirely by /clock (see module
                     docstring), which already advances at whatever --rate the
                     bag player was started with.
    """

    def __init__(self, session_dir: str, on_frame=None, on_depth=None,
                 stream_fps: float = 5.0, speed: float = 1.0):
        self._session_dir   = session_dir
        self._on_frame       = on_frame
        self._on_depth       = on_depth     # cb(camera, depth_arr: np.ndarray, intrinsics: dict)
        self._stream_period = 1.0 / max(stream_fps, 0.1)
        self._speed          = max(speed, 0.01)
        self._running        = False
        self._threads: list[threading.Thread] = []

        # First-frame sync point, set once any camera delivers its first frame.
        self.ready_event       = threading.Event()
        self.first_frame_ts_ns: int | None = None
        self._first_frame_lock = threading.Lock()

        # /clock subscription state (shared across all camera threads)
        self._clock_lock   = threading.Lock()
        self._sim_clock_ns: int | None = None
        self._clock_node    = None
        self._clock_thread  = None

    def start(self):
        self._running = True
        self._start_clock_sync()
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
        if self._clock_node is not None:
            try:
                self._clock_node.destroy_node()
            except Exception:
                pass

    def join(self, timeout: float = 10.0):
        for t in self._threads:
            t.join(timeout=timeout)

    # ── /clock sync ──────────────────────────────────────────────────────────

    def _start_clock_sync(self):
        try:
            import rclpy
            from rosgraph_msgs.msg import Clock
        except ImportError:
            log.warning('SvoPlayer: rosgraph_msgs unavailable — falling back to speed-based pacing')
            return
        try:
            self._clock_node = rclpy.create_node('svo_clock_sync')

            def _on_clock(msg):
                with self._clock_lock:
                    self._sim_clock_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec

            self._clock_node.create_subscription(Clock, '/clock', _on_clock, 10)
            self._clock_thread = threading.Thread(
                target=lambda: __import__('rclpy').spin(self._clock_node),
                daemon=True, name='svo-clock-sync',
            )
            self._clock_thread.start()
            log.info('SvoPlayer: subscribed to /clock for frame-accurate sync')
        except Exception as e:
            log.warning(f'SvoPlayer: /clock subscription failed ({e}) — falling back to speed-based pacing')
            self._clock_node = None

    def _get_sim_clock_ns(self) -> int | None:
        with self._clock_lock:
            return self._sim_clock_ns

    def _wait_for_clock(self, frame_ts_ns: int):
        """Block until /clock reaches frame_ts_ns, or give up after CLOCK_WAIT_TIMEOUT_S."""
        deadline = time.time() + CLOCK_WAIT_TIMEOUT_S
        while self._running and time.time() < deadline:
            sim_ns = self._get_sim_clock_ns()
            if sim_ns is None or sim_ns >= frame_ts_ns:
                return
            time.sleep(0.005)

    def _mark_first_frame(self, ts_ns: int):
        with self._first_frame_lock:
            if self.first_frame_ts_ns is None:
                self.first_frame_ts_ns = ts_ns
                self.ready_event.set()

    # ── Per-camera thread ────────────────────────────────────────────────────

    def _play_cam(self, name: str, svo_path: str):
        try:
            import pyzed.sl as sl
        except ImportError:
            log.warning('pyzed not installed — SvoPlayer disabled')
            return

        init = sl.InitParameters()
        init.set_from_svo_file(svo_path)
        # We pace explicitly (via /clock, or the speed fallback) — let grab()
        # return as fast as it can decode rather than the SDK's own real-time gate.
        init.svo_real_time_mode = False
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
        prev_ts_ns   = None
        prev_wall    = time.time()
        clock_ever_seen = False

        try:
            while self._running:
                if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
                    log.info(f'SvoPlayer [{name}]: end of file')
                    break

                cur_ts_ns = cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
                self._mark_first_frame(cur_ts_ns)

                if self._get_sim_clock_ns() is not None:
                    clock_ever_seen = True
                    self._wait_for_clock(cur_ts_ns)
                elif self._speed != 1.0:
                    # No /clock yet (bag not started, or rosgraph_msgs missing) —
                    # pace from consecutive frame timestamps as a fallback.
                    if prev_ts_ns is not None:
                        delta_play = (cur_ts_ns - prev_ts_ns) / 1e9 / self._speed
                        elapsed    = time.time() - prev_wall
                        if delta_play > elapsed:
                            time.sleep(delta_play - elapsed)
                    prev_ts_ns = cur_ts_ns
                    prev_wall  = time.time()

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
            log.info(f'SvoPlayer [{name}]: closed (clock-synced={clock_ever_seen})')
