
import sys
import os
import argparse
import threading
from multiprocessing import Queue, Process, Event
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

DATA_DIR = f"./data/{args.trial_name}_raw/"
RGB_DIR = DATA_DIR + "rgb/"
DEPTH_DIR = DATA_DIR + "depth/"
ACCEL_FILE = DATA_DIR + "accel.json"
GYRO_FILE = DATA_DIR + "gyro.json"
UWB_FILE = DATA_DIR + "uwb.json"

try: os.makedirs(DATA_DIR, exist_ok=True)
except OSError as e: print(e)

try: os.makedirs(RGB_DIR, exist_ok=True)
except OSError as e: print(e)

try: os.makedirs(DEPTH_DIR, exist_ok=True)
except OSError as e: print(e)

def host_timestamp(): return time.perf_counter_ns() / 1e6 # Return host time in ms.



def realsense_listener(stop_signal, stop_complete_signal):
    # global stop_signal, stop_complete_signal

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

    accel = Queue(maxsize=200)
    gyro = Queue(maxsize=200)
    rgb = Queue(maxsize=10)
    depth = Queue(maxsize=10)

    ACCEL_HANDLE = open(ACCEL_FILE, 'w', newline='')
    ACCEL_FS = csv.writer(ACCEL_HANDLE)
    GYRO_HANDLE = open(GYRO_FILE, 'w', newline='')
    GYRO_FS = csv.writer(GYRO_HANDLE)

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

    def imu_callback(frame):
        stype = frame.get_profile().stream_type()
        data = frame.as_motion_frame().get_motion_data()

        sample = {"t_dev": frame.get_timestamp(), "t_host": host_timestamp(), "data":(data.x, data.y, data.z)}
        if stype == rs.stream.gyro:
            gyro.put(sample)
        elif stype == rs.stream.accel:
            accel.put(sample)

    def rgb_camera_callback(frame):
        rgb.put({"t_dev": frame.get_timestamp(), "t_host": host_timestamp(), "data":np.asanyarray(frame.get_data())})

    def depth_camera_callback(frame):
        # print("Depth callback") # Depth just runs at very sparse (sub 30fps intervals), a bursrt at the beginning, and then very low frequency
        depth.put({"t_dev": frame.get_timestamp(), "t_host": host_timestamp(), "data":np.asanyarray(frame.get_data())})

    def flush_buffers():
        if accel.full():
            print("Flushing accel")
            while not accel.empty():
                x = accel.get(block=False)
                ACCEL_FS.writerow([x['t_dev'], x['t_host'], x['data'][0], x['data'][1], x['data'][2]])
                # json.dump(accel.get(block=False), ACCEL_FS, indent=1)
        if gyro.full(): 
            print("Flushing gyro")
            while not gyro.empty():
                x = gyro.get(block=False)
                GYRO_FS.writerow([x['t_dev'], x['t_host'], x['data'][0], x['data'][1], x['data'][2]])
        if rgb.full():
            print("Flushing RGB")
            while not rgb.empty():
                frame = rgb.get()
                np.savez( RGB_DIR+"/"+str(frame["t_host"]/1e3)+".npz", array= frame["data"], t_dev=frame["t_dev"], t_host = frame["t_host"] )
        if depth.full():
            print("Flushing depth")
            while not depth.empty():
                frame = depth.get()
                np.savez( DEPTH_DIR+"/"+str(frame["t_host"]/1e3)+".npz", array= frame["data"], t_dev=frame["t_dev"], t_host = frame["t_host"] )
    
    print(f"Realsense set up callbacks ")
    imu.start(imu_callback)
    rgb_camera.start(rgb_camera_callback)
    depth_camera.start(depth_camera_callback)
    print(f"Realsense started collection")

    while not stop_signal.is_set():
        # print(stop_signal)
        flush_buffers()
    flush_buffers()

    print("Escaped loop")

    imu.stop()
    rgb_camera.stop()
    depth_camera.stop()
    print("Stopped realsense")
    ACCEL_HANDLE.close()
    GYRO_HANDLE.close()
    print("Realsense done closing")

    stop_complete_signal.set()
    sys.exit(0)

def read_from_serial( ser):
    ser.reset_input_buffer()
    return ser.readline().decode('utf-8')

def write_to_serial(ser, str):
    ser.write((str+"\n").encode('utf-8'))

def fetch_decawave_time(ser): write_to_serial(ser, "AT+TIME") # Issue command to get hardware timestamp


# def on_interrupt(sig, frame):
#     print("Interrupt")
#     stop_signal.set()
#     stop_complete_signal.wait()
#     realsense_process.join()
#     print("Exited realsense")
#     with open(UWB_FILE, 'w') as fs: json.dump(uwb, fs, indent=1) # Dump UWB data

#     sys.exit(0)

if __name__ == "__main__":

    # signal.signal(signal.SIGINT, on_interrupt)

    print("Starting Realsense thread") 
    # realsense_listener() # Library implicitly spawns off 3 threads

    stop_signal = Event()
    stop_complete_signal = Event()

    realsense_process = Process(target=realsense_listener, args = (stop_signal, stop_complete_signal))
    realsense_process.start()

    uwb = []
    deca_time = []
    def decawave_listener(): # Can use the timestamp I log in decawave serial as that hardware time
        
        TAG_PORT = 'COM4'
        TAG_SERIAL = serial.Serial(TAG_PORT, 115200)

        ht_query_limit = 5 # Very laggy with a query limit of 1
        range_counter = 0

        fetch_decawave_time(TAG_SERIAL)
        print(f"Decawave started collection")

        while True:
            line = read_from_serial(TAG_SERIAL)
            if line is not None:
                if "{" in line and "}" in line: 
                    # print(line)
                    if len(deca_time) > 0:
                        estimated_t_dev = (host_timestamp() - deca_time[-1]["t_host"]) + deca_time[-1]["t_dev"]
                        uwb.append({"t_dev": estimated_t_dev, "t_host": host_timestamp(), "data":json.loads(line)})
                        range_counter += 1
                elif "Time:" in line:
                    # deca_time.append({"t_host": time.perf_counter(), "t_dev": int(line.split()[1]) })
                    match = re.search(r'\d+', line)
                    if match:
                        deca_time.append({"t_dev": int(match.group()), "t_host": host_timestamp()})

            if range_counter > ht_query_limit:  
                fetch_decawave_time(TAG_SERIAL)
                range_counter = 0

            if time.perf_counter() - START_T > 120: break
        return

    START_T = time.perf_counter()
    print("Starting Decawave thread")
    decawave_listener() # Can run on main thread

    print("Interrupt")
    stop_signal.set()
    print("Between stop set and wait complete")
    stop_complete_signal.wait()
    print("stop completed, passing on")
    # realsense_process.join() # Don't join an exited process
    print("Exited realsense")

    with open(UWB_FILE, 'w') as fs: json.dump(uwb, fs, indent=1) # Dump UWB data
    print("UWB dump complete")

    # Force kill if process still alive
    if realsense_process.is_alive():
        print("Force terminating realsense process")
        realsense_process.terminate()
        realsense_process.join()
    else:
        print("Realsense process exited cleanly")
        # Never run this as process hangs on exiting realsense
            
        # TODO: They only allow you to have 16 frames in memory at a given time, which is why my frames cap out at 16
        # So I would have to flush frames out of memory asynchronously for the realsense
        # To do this I need the process API, which is a mess of its own because its a full process clone.


