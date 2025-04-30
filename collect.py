
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


# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
# config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

imu = [ s for s in device.query_sensors() if s.get_info(rs.camera_info.name) == 'Motion Module'][0]
rgb_camera = [ s for s in device.query_sensors() if s.get_info(rs.camera_info.name) == 'RGB Camera'][0]
depth_camera = [ s for s in device.query_sensors() if s.get_info(rs.camera_info.name) == 'Stereo Module'][0]

# For color sensor
color_profiles = rgb_camera.get_stream_profiles()
color_profile = next(p for p in color_profiles if 
                     p.stream_type() == rs.stream.color and 
                     p.format() == rs.format.bgr8 and 
                     p.as_video_stream_profile().width() == 640 and 
                     p.as_video_stream_profile().height() == 480 and 
                     p.fps() == 30)
rgb_camera.open(color_profile)

# For depth sensor
depth_profiles = depth_camera.get_stream_profiles()
depth_profile = next(p for p in depth_profiles if 
                     p.stream_type() == rs.stream.depth and 
                     p.format() == rs.format.z16 and 
                     p.as_video_stream_profile().width() == 640 and 
                     p.as_video_stream_profile().height() == 480 and 
                     p.fps() == 30)
depth_camera.open(depth_profile)

# For IMU (accel & gyro)
motion_profiles = imu.get_stream_profiles()

accel_profile = next(p for p in motion_profiles if 
                     p.stream_type() == rs.stream.accel and 
                     p.format() == rs.format.motion_xyz32f and 
                     p.fps() == 200)

gyro_profile = next(p for p in motion_profiles if 
                    p.stream_type() == rs.stream.gyro and 
                    p.format() == rs.format.motion_xyz32f and 
                    p.fps() == 200)

imu.open([accel_profile, gyro_profile])



end_threads_event = threading.Event()

def realsense_listener():

    accel = []
    gyro = []
    def imu_callback(frame):
        stype = frame.get_profile().stream_type()
        data = frame.as_motion_frame().get_motion_data()

        sample = {"t_h": frame.get_timestamp(), "t_s": time.perf_counter(), "data":(data.x, data.y, data.z)}
        if stype == rs.stream.gyro:
            gyro.append(sample)
        elif stype == rs.stream.accel:
            accel.append(sample)

    rgb = []
    def rgb_camera_callback(frame):
        rgb.append({"t_h": frame.get_timestamp(), "t_s": time.perf_counter(), "data":np.asanyarray(frame.get_data())})

    depth = []
    def depth_camera_callback(frame):
        depth.append({"t_h": frame.get_timestamp(), "t_s": time.perf_counter(), "data":np.asanyarray(frame.get_data())})

    # These cameras are async by default, realsense spawns internal threads
    imu.start(imu_callback) # They also have some kind of syncer object
    rgb_camera.start(rgb_camera_callback)
    depth_camera.start(depth_camera_callback)

    global end_threads_event
    while not end_threads_event.is_set():
        continue
    
    # With I/O, is it best to sleep the thread or run it in a loop, I can't remember?
    # I think its that you sleep, and only wake up when there is data in your buffer.

    imu.stop()
    rgb_camera.stop()
    depth_camera.stop()

    print(accel)
    print(gyro)
    print(rgb)

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

    
    T_START = time.perf_counter()

    print("Starting realsense_thread")
    realsense_thread = threading.Thread(target=realsense_listener)
    realsense_thread.start()

    print("Starting decawave_thread")
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



