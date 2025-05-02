
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
import re

import matplotlib.pyplot as plt
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
        for p in sensor.get_hosttream_profiles():
            print(f"  {p.stream_type()}, {p.format()}, {p.fps()} fps")

def host_timestamp(): return time.perf_counter_ns() / 1e6 # Return host time in ms.


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



start_collection = threading.Event()
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

        sample = {"t_dev": frame.get_timestamp(), "t_host": host_timestamp(), "data":(data.x, data.y, data.z)}
        if stype == rs.stream.gyro:
            gyro.append(sample)
        elif stype == rs.stream.accel:
            accel.append(sample)

    def rgb_camera_callback(frame):
        rgb.append({"t_dev": frame.get_timestamp(), "t_host": host_timestamp(), "data":np.asanyarray(frame.get_data())})

    def depth_camera_callback(frame):
        depth.append({"t_dev": frame.get_timestamp(), "t_host": host_timestamp(), "data":np.asanyarray(frame.get_data())})
    
    print(f"Realsense set up callbacks ")

    start_collection.wait()
    
    imu.start(imu_callback)
    rgb_camera.start(rgb_camera_callback)
    depth_camera.start(depth_camera_callback)
    print(f"Realsense started collection")


TAG_PORT = 'COM4'
TAG_SERIAL = serial.Serial(TAG_PORT, 115200)
def read_from_serial( ser):
    ser.reset_input_buffer()
    return ser.readline().decode('utf-8')

def write_to_serial(ser, str):
    ser.write((str+"\n").encode('utf-8'))

def fetch_decawave_time(): write_to_serial(TAG_SERIAL, "AT+TIME") # Issue command to get hardware timestamp


uwb = []
deca_time = []
def decawave_listener(): # Can use the timestamp I log in decawave serial as that hardware time
    global end_threads
    ht_query_limit = 5 # Very laggy with a query limit of 5
    range_counter = 0

    start_collection.wait()

    fetch_decawave_time()
    while True:
        line = read_from_serial(TAG_SERIAL)

        if line is not None:
            if "{" in line and "}" in line: 
                print(line)
                uwb.append({"t_host": host_timestamp(), "data":json.loads(line)})
                range_counter += 1
            elif "Time:" in line:
                # deca_time.append({"t_host": time.perf_counter(), "t_dev": int(line.split()[1]) })
                match = re.search(r'\d+', line)
                if match:
                    deca_time.append({"t_dev": int(match.group()), "t_host": host_timestamp()})


        if range_counter > ht_query_limit:  
            fetch_decawave_time()


def on_interrupt(sig, frame):
    print("Interrupt")

    # They seem off by around 40ms, it might take the Realsense threads a few seconds to boot.
    # Set a timer, and make sure data collection on all threads only begins when that timer expires
    # This should align the host start timestamps even enough

    # Even though the timestamps are unaligned, for now, we will treat the decawave as the HOST Start
    HOST_START = deca_time[0]["t_host"]
    DECAWAVE_START = deca_time[0]["t_dev"]
    REALSENSE_START = accel[0]["t_dev"]

    host_ts = []
    decawave_ts = []
    realsense_ts = []

    # Currently the start timestamps are a little unsynced (40ms), we will treat the decawave host timestamp start
    # TODO: Plotting to see any clock drift
    # x-axis software timestamp
    # y-axis decawave or realsense
    for d in deca_time:
        decawave_ts.append(d["t_dev"] - DECAWAVE_START)
        host_ts.append(d["t_host"] - HOST_START)
    
    for r in accel:
        realsense_ts.append(r["t_dev"] - REALSENSE_START)

    # print(host_ts)
    # print(f"{max(host_ts)=}")
    ts_x = np.linspace(0, max(host_ts), int(max(host_ts)))

    plt.title(" Hardware (Decawave, Realsense IMU) Timestamps vs Host Timestamps")
    plt.plot( realsense_ts, label='Realsense IMU')
    plt.plot( decawave_ts, label='Decawave')
    plt.plot( host_ts, label='Host')

    plt.xlim((0, max(host_ts)))
    plt.legend()

    plt.show()


    imu.stop()
    rgb_camera.stop()
    depth_camera.stop()
    realsense_process.kill()
    realsense_process.close()

    exit()

if __name__ == "__main__":

    signal.signal(signal.SIGINT, on_interrupt)


    print("Starting Realsense thread") 
    realsense_process = multiprocessing.Process(target=realsense_listener) 
    # Problem, a process is a full process clone; and changes data arrays in its own memory -> Can't use threads because GIL
    realsense_process.start()

    time.sleep(0.1)
    start_collection.set() # Goal: Start both data collection threads off at the same time. 

    print("Starting Decawave thread")
    decawave_listener() # Can run on main thread




