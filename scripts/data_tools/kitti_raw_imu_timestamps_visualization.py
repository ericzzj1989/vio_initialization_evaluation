# coding:utf-8
#!/usr/bin/python

import time
import datetime as dt
import glob
import os
import matplotlib.pyplot as plt
import numpy as np
import codecs
from itertools import islice

data_path = "/home/eric/datasets/KITTI/2011_09_26_drive_0039/2011_09_26_drive_0039_extract/output"
# data_path = "/home/eric/datasets/imav/mav0"
def load_timestamps(data='cam0'):
    """Load timestamps from file."""
    timestamp_file = os.path.join(
        data_path, data, 'data.csv')

    # Read and parse the timestamps
    timestamps = []
    # with open(timestamp_file, 'r') as f:
    with codecs.open(timestamp_file, 'r', 'utf-8') as f:
        # for line in f.readlines():
        for line in islice(f, 1, None):
            # t = dt.datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            # t = dt.datetime.timestamp(t)
            # print(t)
            t = float("{:.9f}".format(float(line.split(',')[0]) / 1e9))
            print(t)
            timestamps.append(t)       

    # Subselect the chosen range of frames, if any
    return timestamps
timestamps = np.array(load_timestamps())
x = np.arange(0, len(timestamps))
print(x)

last_timestamp = timestamps[:-1]
curr_timestamp = timestamps[1:]
dt = np.array(curr_timestamp - last_timestamp) #计算前后帧时间差
print(dt)

print("dt > 0.15: \n{}".format(dt[dt> 0.15])) # 打印前后帧时间差大于0.015的IMU index
dt = dt.tolist()
dt.append(0.01)
dt = np.array(dt)
print("dt > 0.15: \n{}".format(x[dt> 0.15])) #打印时间差大于0.015的具体时间差
plt.plot(x, timestamps, 'r', label='imu')
plt.show()