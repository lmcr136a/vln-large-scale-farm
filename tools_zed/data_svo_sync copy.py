import os
import time
import threading
import argparse
import signal
import sys
import pyzed.sl as sl
import cv2

stop_event = threading.Event()

class ZEDImageRecorder(threading.Thread):
    def __init__(self, serial_number, name, base_path, sync_time, image_interval=1):
        super().__init__(name=name)
        self.serial = serial_number
        self.name = name
        self.base_path = base_path
        self.sync_time = sync_time
        self.running = True
        self.ready_sent = False
        self.image_interval = image_interval

        self.cam = sl.Camera()

        init = sl.InitParameters()
        input_type = sl.InputType()
        input_type.set_from_serial_number(serial_number)
        init.input = input_type
        init.depth_mode = sl.DEPTH_MODE.NEURAL
        init.coordinate_units = sl.UNIT.METER
        init.camera_resolution = sl.RESOLUTION.HD1080
        init.camera_fps = 15

        status = self.cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED camera {serial_number}: {status}")

        # SVO 녹화 제거!

        self.runtime = sl.RuntimeParameters()
        self.info_csv = open(os.path.join(self.base_path, f"{self.name}_info.csv"), "w")
        self.info_csv.write("frame_id,rel_time,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n")
        self.sensors_data = sl.SensorsData()
        
        # 이미지 저장용
        self.image_left = sl.Mat()

    def run(self):
        count = 0
        print(111)
        while self.running and not stop_event.is_set():
            print(222)
            if self.cam.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
                print(333)
                now = time.time()
                rel_time = now - self.sync_time

                if not self.ready_sent:
                    print(f"[READY] {self.name}")
                    self.ready_sent = True

                if self.cam.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
                    imu = self.sensors_data.get_imu_data()
                    accel = imu.get_linear_acceleration()
                    gyro = imu.get_angular_velocity()

                    self.info_csv.write(f"{count},{rel_time:.6f},{accel[0]},{accel[1]},{accel[2]},{gyro[0]},{gyro[1]},{gyro[2]}\n")
                    self.info_csv.flush()

                # 이미지 저장
                if count % self.image_interval == 0:
                    print("Trying to save image")
                    self.cam.retrieve_image(self.image_left, sl.VIEW.LEFT)
                    image_np = self.image_left.get_data()
                    
                    img_path = os.path.join(self.base_path, f"rgb.png")
                    cv2.imwrite(img_path, image_np)
                    
                    if count % (self.image_interval * 10) == 0:
                        print(f"[{self.name}] Saved frame {count}")

                count += 1
            else:
                time.sleep(0.005)
        print(f"[{self.name}] Exiting safely.")

    def stop(self):
        self.running = False
        stop_event.set()
        self.join()
        self.info_csv.close()
        self.cam.close()
        print(f"[{self.name}] Stopped.")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, help="Output directory path")
    parser.add_argument("--sync_time", required=True, type=float, help="Shared sync timestamp (float, seconds)")
    parser.add_argument("--image_interval", type=int, default=1, help="Save every N frames (default: 1)")
    args = parser.parse_args()

    base_path = os.path.expanduser(args.output)
    os.makedirs(base_path, exist_ok=True)

    rec_L = ZEDImageRecorder(
        serial_number=48335070, 
        name="L", 
        base_path=base_path, 
        sync_time=args.sync_time,
        image_interval=args.image_interval
    )
    rec_R = ZEDImageRecorder(
        serial_number=49537850, 
        name="R", 
        base_path=base_path, 
        sync_time=args.sync_time,
        image_interval=args.image_interval
    )
    print("SSSSSSSSSSSSSstarted")
    rec_L.start()
    rec_R.start()

    def signal_handler(sig, frame):
        print("[🛑] Stopping...")
        rec_L.stop()
        rec_R.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        signal_handler(None, None)

if __name__ == "__main__":
    main()