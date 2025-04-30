
import sys
import os
import argparse
import threading
import serial
import cv2
import numpy as np
import time
import signal
import csv
from datetime import datetime
pyd_path = "C:\\Program Files\\Intel RealSense SDK 2.0\\bin\\x64\\"
sys.path.append(pyd_path)
import pyrealsense2 as rs

# Package is for 3.11

# py -3.11 -m venv py311
# .\py311\Scripts\Activate

parser = argparse.ArgumentParser(description="Stream collector")
parser.add_argument("--trial_name" , "-t", type=str)

if len(sys.argv) < 2:
    print("Provide -t <trial_name>")
    exit()

args = parser.parse_args()

DATA_DIR = f"./{args.trial_name}/"
CAM_DIR = DATA_DIR + "camera/"
IMU_FILE = DATA_DIR + "imu.csv"
UWB_FILE = DATA_DIR + "uwb.json"
try: os.makedirs(DATA_DIR, exist_ok=True)
except OSError as e: print(e)

try: os.makedirs(CAM_DIR, exist_ok=True)
except OSError as e: print(e)

# To check what the possible streaming rates are
def check_sensors():
    sensors = device.query_sensors()
    for sensor in sensors:
        print(f"Sensor: {sensor.get_info(rs.camera_info.name)}")
        for p in sensor.get_stream_profiles():
            print(f"  {p.stream_type()}, {p.format()}, {p.fps()} fps")


pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
print(f"{str(device.get_info(rs.camera_info.product_line))=}")
print(f"{str(device.get_info(rs.camera_info.name))=}")


config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

end_threads_event = threading.Event()

T_START = time.perf_counter()

# Start realsense pipeline
pipeline.start(config)
def realsense_listener():
    frame_idx = 0

    imu = [ s for s in device.query_sensors() if s.get_info(rs.camera_info.name) == 'Motion Module']
    camera = [ s for s in device.query_sensors() if s.get_info(rs.camera_info.name) == 'Motion Module']
    
    imu_fs = csv.writer(open(IMU_FILE, 'w'), newline="")

    accel = []
    gyro = []
    def imu_callback(frame):
        global accel, gyro
        stype = frame.get_profile().stream_type()
        data = frame.as_motion_frame().get_motion_data()

        sample = {"t_h": frame.get_timestamp(), "t_s": time.perf_counter(), "data":(data.x, data.y, data.z)}
        if stype == rs.stream.gyro:
            gyro.append(sample)
        elif stype == rs.stream.accel:
            accel.append(sample)

    imu.start(imu_callback)


    def camera_callback(frame):

    rgb_frames = []
    depth_frames = []

    global end_threads_event
    while not end_threads_event.is_set():
        # Create a pipeline object. This object configures the streaming camera and owns it's handle

        # is motion frame is inclusive of gyro and accel
        # The way I have configured it, gyro and accel are synced.


        print()
         # pipeline.wait_for_frames batches imu and gyro frames with camera depth frames
        for frame in pipeline.wait_for_frames():
            # print(frame.profile.as_stream_profile)

            frame_type = frame.get_profile().stream_type()
            print(frame_type)
            # print(frame.data)

            # # if frame_type == rs.stream.color:
                
            # # elif frame_type == rs.stream.depth: # never runs

            # elif frame_type == rs.stream.accel:
                
            # elif frame_type == rs.stream.gyro:




        # print(frames)

        # rgb_frame = frames.get_color_frame()
        # depth_frame = frames.get_depth_frame()

        # depth_image = np.asanyarray(depth_frame.get_data())
        # color_image = np.asanyarray(rgb_frame.get_data())

        # # Save images to directory
        # cv2.imwrite(CAM_DIR+f"depth_{frame_idx}.png", depth_image)
        # cv2.imwrite(CAM_DIR+f"color_{frame_idx}.png", color_image)


        # accel = frames.first_or_default(rs.stream.accel)
        # gyro = frames.first_or_default(rs.stream.gyro)

        frame_idx += 1

    pipeline.stop()
    exit()




# TAG_PORT = '/dev/ttyX'
# TAG_SER = serial.Serial(TAG_PORT, 115200)
# def read_from_serial( ser):
#     ser.reset_input_buffer()
#     return ser.readline().decode('utf-8')

def decawave_listener():
    global end_threads
    while not end_threads_event.is_set():
        # print("uwb")
        continue
    exit()

def on_interrupt(sig, frame):
    print("Interrupt")
    global end_threads_event
    end_threads_event.set()

    realsense_thread.join()
    decawave_thread.join()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, on_interrupt)


    realsense_thread = threading.Thread(target=realsense_listener)
    realsense_thread.start()

    decawave_thread = threading.Thread(target=decawave_listener)
    decawave_thread.start()

    # Need to keep main thread alive to capture ctrl c
    try:
        while not end_threads_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        # Fallback in case signal handler didn't fire
        on_interrupt(None, None)
        exit()



