

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
RAW_DATA_DIR = f"./data/{args.trial_name}_raw/"
RGB_DIR = RAW_DATA_DIR + "rgb/"
DEPTH_DIR = RAW_DATA_DIR + "depth/"
ACCEL_FILE = RAW_DATA_DIR + "accel.json"
GYRO_FILE = RAW_DATA_DIR + "gyro.json"
UWB_FILE = RAW_DATA_DIR + "uwb.json"

accel, gyro, rgb, uwb = ([], [], [], [])

# for depth_file in os.listdir(DEPTH_DIR):
#     data = np.load(DEPTH_DIR+depth_file)
#     depth.append({key: data[key].item() if data[key].ndim == 0 else data[key] for key in data.files})

for rgb_file in os.listdir(RGB_DIR):
    data = np.load(RGB_DIR+rgb_file)
    rgb.append({key: data[key].item() if data[key].ndim == 0 else data[key] for key in data.files})

with open(ACCEL_FILE, 'r') as fs: 
    csvreader = csv.reader(fs)
    for row in csvreader:
        accel.append({"t_dev": float(row[0]), "t_host": float(row[1]), "data":(float(row[2]), float(row[3]), float(row[4]))})

with open(GYRO_FILE, 'r') as fs: 
    csvreader = csv.reader(fs)
    for row in csvreader:
        gyro.append({"t_dev": float(row[0]), "t_host": float(row[1]), "data":(float(row[2]), float(row[3]), float(row[4]))})

uwb = json.load(open(UWB_FILE, 'r'))


def postprocess_data():

    sensor_streams = [accel, gyro, rgb,  uwb]

    HOST_START = max([ s[0]["t_host"] for s in sensor_streams])
    print(f" {HOST_START=}")

    print(f" Starting host timestamps {[ s[0]['t_host'] for s in sensor_streams]}")

    for i in range(len(sensor_streams)): # CROP all sensor streams to start as close as possible to host start.
        sensor_streams[i] = [ x for x in sensor_streams[i] if x["t_host"] >= HOST_START]

    print(f" Starting host timestamps {[ s[0]['t_host'] for s in sensor_streams]}")

    DECAWAVE_START = uwb[0]["t_dev"]
    REALSENSE_START = min(accel[0]["t_dev"], gyro[0]["t_dev"], rgb[0]["t_dev"])

    for sensordata in [accel, gyro, rgb]:
        for a in sensordata: 
            a["t_host"] -= HOST_START
            a["t_dev"] -= REALSENSE_START

    for sensordata in [uwb]:
        for a in sensordata: 
            a["t_host"] -= HOST_START
            a["t_dev"] -= DECAWAVE_START

    decawave_ts = []
    accel_arr = []
    gyro_arr = []
    realsense_ts = []

    for d in uwb:
        decawave_ts.append([d["t_host"], d["t_dev"]])

    for i in range(0, len(accel)):
        dat = [accel[i]["t_host"], accel[i]["t_dev"], accel[i]["data"][0], accel[i]["data"][1], accel[i]["data"][2]]
        accel_arr.append(dat)

    for i in range(0, len(gyro)):
        dat = [gyro[i]["t_host"], gyro[i]["t_dev"], gyro[i]["data"][0], gyro[i]["data"][1], gyro[i]["data"][2]]
        gyro_arr.append(dat)

    for c in rgb:
        realsense_ts.append([c["t_host"], c["t_dev"]])

    decawave_ts = np.array(decawave_ts)
    accel_arr = np.array(accel_arr)
    gyro_arr = np.array(gyro_arr)
    realsense_ts = np.array(realsense_ts)

    # plt.title(" Hardware (Decawave, Realsense IMU, RGB) Timestamps vs Host Timestamps")
    # plt.plot( accel_arr[:,0], accel_arr[:,1], label='Realsense IMU')
    # plt.plot( realsense_ts[:,0], realsense_ts[:,1], label='Realsense RGB')
    # plt.plot( decawave_ts[:,0], decawave_ts[:,1], label='Decawave')
    # plt.legend()
    # plt.show()

    print(" Clock Drift Slope: ")
    m, b = np.polyfit(accel_arr[:,0], accel_arr[:,1], 1)
    print(f" Accel = {m}")
    m, b = np.polyfit(realsense_ts[:,0], realsense_ts[:,1], 1)
    print(f" RGB = {m}")
    m, b = np.polyfit(decawave_ts[:,0], decawave_ts[:,1], 1)
    print(f" UWB = {m}")


    imu = []
    for g_idx in range(gyro_arr.shape[0]):
        # find the nearest accelerometer hardware timestamp, to each gyro.
        gyro_dev_timestamp = gyro_arr[g_idx, 1]
        near_idx = np.argmin(np.abs(accel_arr[:,1] - gyro_dev_timestamp)) # Fetch nearest accel  hardware timestamp to this gyro's hardware timestamp
        row = np.concat((accel_arr[near_idx, :], gyro_arr[g_idx, 2:]), axis = 0)
        # Append the gx, gy, gz values to the acceleration we've selected
        imu.append(row)
    
    print(imu[-1])



   

postprocess_data()