
import sys
import os
import argparse
import threading
import multiprocessing
import serial
import cv2
import numpy as np
import time
import signal
import csv
import json
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

accel = []
gyro = []
rgb = []
depth = []

def realsense_listener():
    global accel, gyro, rgb, depth

    def imu_callback(frame):
        stype = frame.get_profile().stream_type()
        data = frame.as_motion_frame().get_motion_data()

        sample = {"t_h": frame.get_timestamp(), "t_s": time.perf_counter(), "data":(data.x, data.y, data.z)}
        if stype == rs.stream.gyro:
            gyro.append(sample)
        elif stype == rs.stream.accel:
            accel.append(sample)

    def rgb_camera_callback(frame):
        rgb.append({"t_h": frame.get_timestamp(), "t_s": time.perf_counter(), "data":np.asanyarray(frame.get_data())})

    def depth_camera_callback(frame):
        depth.append({"t_h": frame.get_timestamp(), "t_s": time.perf_counter(), "data":np.asanyarray(frame.get_data())})

    # These cameras are async by default, realsense spawns internal threads
    # I don't even really need to be spinning off this as an extra thread in that case
    imu.start(imu_callback) # They also have some kind of syncer object
    rgb_camera.start(rgb_camera_callback)
    depth_camera.start(depth_camera_callback)
    print(f" Realsense set up callbacks ")


TAG_PORT = 'COM4'
TAG_SERIAL = serial.Serial(TAG_PORT, 115200)
def read_from_serial( ser):
    ser.reset_input_buffer()
    return ser.readline().decode('utf-8')

# Remember you need to run the AT Commands first! -> Remember I was planning to set the setup.json file to do that for me on boot
# Need to do that before I can get them working on the wall outlets.
uwb = []
def decawave_listener(): # Can use the timestamp I log in decawave serial as that hardware time
    global end_threads
    while True:
        data = read_from_serial(TAG_SERIAL)
        if data is not None:
            uwb.append({"t_s": time.perf_counter(), "data":json.loads(data)})
        # if end_threads_event.is_set(): break # It seems like the end threads event takes a super long time to run?

    # while not end_threads_event.is_set(): # I think this loop runs at far too low a rate
    #     # print("uwb
    # exit()

def on_interrupt(sig, frame):
    print("Interrupt")


    imu.stop()
    rgb_camera.stop()
    depth_camera.stop()

    print(uwb)
    # print(rgb)

    exit()

if __name__ == "__main__":

    signal.signal(signal.SIGINT, on_interrupt)

    T_START = time.perf_counter()
    # Realsense_HT_START = # TODO: Somehow get start time recorded on the realsense hardware.

    print("Starting Realsense thread") # We specify a target, but GPT implies its a C-style process clone
    realsense_listener() # Library implicitly spawns off 3 threads
    print("Starting Decawave thread")
    decawave_listener() # Can run on main thread



