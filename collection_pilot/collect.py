
import os
import argparse
import sys
import threading
import serial
import pyrealsense2 as rs
import cv2
import numpy as np

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

try:
    os.makedirs(DATA_DIR, exist_ok=True)
except OSError as e:
    print(e)

try:
    os.makedirs(CAM_DIR, exist_ok=True)
except OSError as e:
    print(e)

pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

# Enable RGB camera
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Enable depth camera
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Enable IMU streams
# config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
# config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

# Start pipeline
pipeline.start(config)

def realsense_listener():
    frame_idx = 0
    while True:
        # Create a pipeline object. This object configures the streaming camera and owns it's handle
        frames = pipeline.wait_for_frames() # frames is a set of depth, rgb, or inertial information
        # Don't quite understand what 'frames' is in relation to streams

        print()
        print(frames)
        print()

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



TAG_PORT = '/dev/ttyX'
TAG_SER = serial.Serial(TAG_PORT, 115200)
def read_from_serial( ser):
    ser.reset_input_buffer()
    return ser.readline().decode('utf-8')

def decawave_listener():
    while True:
        print("uwb")


realsense_thread = threading.Thread(realsense_listener)
# decawave_thread = threading.Thread(decawave_listener)
