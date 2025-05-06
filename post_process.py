

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

    # print(accel[0])
    HOST_START = max([ s[0]["t_host"] for s in sensor_streams])
    print(f" {HOST_START=}")

    for s in sensor_streams:
        print(len(s))

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

    # ITS like the for loop just doesn't iterate over the full list?
    # Could it be that the realsense thread is still modifying the list after I told it to stop? No...
    # I have absolutely no clue what the fuck is wrong with this program.

    # for i in range(0,3): # Realsense
    #     for j in range(0, len(sensor_streams[i])):
    #         sensor_streams[i][j]["t_dev"] -= REALSENSE_START
    #         sensor_streams[i][j]["t_host"] -= HOST_START
    
    # i=0
    # for i in range(3,4): # UWB
    #     for j in range(len(sensor_streams[i])):
    #         sensor_streams[i][j]["t_dev"] -= DECAWAVE_START
    #         sensor_streams[i][j]["t_host"] -= HOST_START


    decawave_ts = []
    accel_ts = []
    realsense_ts = []

    for d in uwb:
        decawave_ts.append([d["t_host"], d["t_dev"]])

    # print(f"{len(accel)=}")
    # print(f"{len(accel_ts)=}")
    for i in range(0, len(accel)):
        dat = [accel[i]["t_host"], accel[i]["t_dev"]]
        # print(dat)
        accel_ts.append(dat)
    # print(f"{len(accel_ts)=}")

    for c in rgb:
        realsense_ts.append([c["t_host"], c["t_dev"]])

    decawave_ts = np.array(decawave_ts)
    accel_ts = np.array(accel_ts)
    print(accel_ts)
    # print(accel_ts.shape)
    realsense_ts = np.array(realsense_ts)

    plt.title(" Hardware (Decawave, Realsense IMU, RGB) Timestamps vs Host Timestamps")
    plt.plot( accel_ts[:,0], accel_ts[:,1], label='Realsense IMU')
    plt.plot( realsense_ts[:,0], realsense_ts[:,1], label='Realsense RGB')
    plt.plot( decawave_ts[:,0], decawave_ts[:,1], label='Decawave')
    # plt.scatter( accel_ts, label='Realsense IMU')
    # plt.scatter( realsense_ts, label='Realsense RGB')
    # plt.scatter( decawave_ts, label='Decawave')
    plt.legend()
    plt.show()

    print(" Clock Drift Slope: ")
    m, b = np.polyfit(accel_ts[:,0], accel_ts[:,1], 1)
    print(f" Accel = {m}")
    m, b = np.polyfit(realsense_ts[:,0], realsense_ts[:,1], 1)
    print(f" RGB = {m}")
    m, b = np.polyfit(decawave_ts[:,0], decawave_ts[:,1], 1)
    print(f" UWB = {m}")

    # for a in accel:
    #     at = accel["t_host"]


    # print(f"{HOST_START=} {REALSENSE_START=}")


    # # TODO: First align timestamps to starting point
    # for u in uwb: 
    #     u["t_host"] -= HOST_START
    #     # u["t_dev"] -= DECAWAVE_START

    # print("before")
    # print(len(accel))
    
    # for a in accel:
    #     t =a['t_host']
    #     a["t_host"] -= HOST_START
    #     # print(f" Host {t} - {HOST_START} = {a['t_host']}")
    #     a["t_dev"] -= REALSENSE_START

    # # You will not always get the same amount of gyro samples as you will accel samples
    
    # print(f"{len(accel)=} {len(gyro)=}")
    # # # with open(IMU_FILE):
    # for a in accel:
    #     print()
    #     print(f"{a=}")

postprocess_data()