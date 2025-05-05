

def postprocess_data():

    sensor_streams = [accel, gyro, rgb, depth, uwb]
    HOST_START = max([ s[0]["t_host"] for s in sensor_streams])
    print(f" {HOST_START=}")

    for s in sensor_streams:
        print(len(s))

    print(f" Starting host timestamps {[ s[0]['t_host'] for s in sensor_streams]}")

    for i in range(len(sensor_streams)): # CROP all sensor streams to start as close as possible to host start.
        sensor_streams[i] = [ x for x in sensor_streams[i] if x["t_host"] >= HOST_START]

    print(f" Starting host timestamps {[ s[0]['t_host'] for s in sensor_streams]}")

    # # for sensordata in [accel, gyro, rgb, depth]:
    # #     for a in sensordata: 
    # #         a["t_host"] -= HOST_START
    # #         a["t_dev"] -= REALSENSE_START

    DECAWAVE_START = uwb[0]["t_dev"]
    REALSENSE_START = min(accel[0]["t_dev"], gyro[0]["t_dev"], rgb[0]["t_dev"], depth[0]["t_dev"])

    # ITS like the for loop just doesn't iterate over the full list?
    # Could it be that the realsense thread is still modifying the list after I told it to stop? No...
    # I have absolutely no clue what the fuck is wrong with this program.

    for i in range(0,4): # Realsense
        for j in range(0, len(sensor_streams[i])):
            sensor_streams[i][j]["t_dev"] -= REALSENSE_START
            sensor_streams[i][j]["t_host"] -= HOST_START
    
    i=0
    for i in range(4,5): # UWB
        for j in range(len(sensor_streams[i])):
            sensor_streams[i][j]["t_dev"] -= DECAWAVE_START
            sensor_streams[i][j]["t_host"] -= HOST_START


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
    # print(accel_ts)
    # print(accel_ts.shape)
    realsense_ts = np.array(realsense_ts)

    plt.title(" Hardware (Decawave, Realsense IMU, RGB) Timestamps vs Host Timestamps")
    plt.scatter( accel_ts[:,0], accel_ts[:,1], label='Realsense IMU')
    plt.scatter( realsense_ts[:,0], realsense_ts[:,1], label='Realsense RGB')
    plt.scatter( decawave_ts[:,0], decawave_ts[:,1], label='Decawave')
    # plt.scatter( accel_ts, label='Realsense IMU')
    # plt.scatter( realsense_ts, label='Realsense RGB')
    # plt.scatter( decawave_ts, label='Decawave')
    plt.legend()
    plt.show()

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
