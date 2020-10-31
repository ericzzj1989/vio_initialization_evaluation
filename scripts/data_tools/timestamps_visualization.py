# coding:utf-8
#!/usr/bin/python

import os
import argparse

import time
import datetime as dt
import glob
import matplotlib.pyplot as plt
import numpy as np
import codecs
from itertools import islice



def load_timestamps(timestamps_data_path):
    """Load timestamps from file."""
    timestamp_file = os.path.join(
        timestamps_data_path, 'data.csv')

    timestamps = []
    with codecs.open(timestamp_file, 'r', 'utf-8') as f:
        for line in islice(f, 1, None):
            t = float("{:.9f}".format(float(line.split(',')[0]) / 1e9))
            timestamps.append(t)       

    # Subselect the chosen range of frames, if any
    return timestamps

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='''Visualize the timestamps in a data file.''')
    parser.add_argument(
        'timestamps_data_path', type=str,
        help="Folder containing the data with timestamps.")
    args = parser.parse_args()

    assert os.path.exists(args.timestamps_data_path)

    timestamps = np.array(load_timestamps(args.timestamps_data_path))
    x = np.arange(0, len(timestamps))

    last_timestamp = timestamps[:-1]
    curr_timestamp = timestamps[1:]
    dt = np.array(curr_timestamp - last_timestamp)

    print("dt > 0.15: \n{}".format(dt[dt> 0.015]))
    dt = dt.tolist()
    dt.append(0.01)
    dt = np.array(dt)
    print("dt > 0.15: \n{}".format(x[dt> 0.015]))
    plt.plot(x, timestamps, 'r', label='imu')
    plt.show()