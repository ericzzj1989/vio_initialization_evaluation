# coding:utf-8
#!/usr/bin/python

import os
import re
import sys
import argparse
import codecs
import time
import datetime
import cv2
import csv

def construct(kitti_path, cam0_path, cam1_path, imu0_path):
    f_cam0 = codecs.open(cam0_path + 'data.csv', 'w', 'utf-8')
    f_cam1 = codecs.open(cam1_path + 'data.csv', 'w', 'utf-8')
    f_imu = codecs.open(imu0_path + 'data.csv', 'w', 'utf-8')
    cam0_dat_dir = cam0_path + 'data/'
    cam1_dat_dir = cam1_path + 'data/'
    if not os.path.exists(cam0_dat_dir):
        os.makedirs(cam0_dat_dir)
    if not os.path.exists(cam1_dat_dir):
        os.makedirs(cam1_dat_dir)

    f_cam0_timestamps = codecs.open(cam0_path + 'cam0_timestamps.txt', 'w', 'utf-8')
    f_cam1_timestamps = codecs.open(cam1_path + 'cam1_timestamps.txt', 'w', 'utf-8')
    f_cam0_raw_timestamps = codecs.open(kitti_path + '/image_00/timestamps.txt', 'r', 'utf-8')
    f_cam1_raw_timestamps = codecs.open(kitti_path + '/image_01/timestamps.txt', 'r', 'utf-8')
    f_imu_raw_timestamps = codecs.open(kitti_path + '/oxts/timestamps.txt', 'r', 'utf-8')

    csv_writer_cam0 = csv.writer(f_cam0)
    csv_writer_cam1 = csv.writer(f_cam1)
    csv_writer_imu = csv.writer(f_imu)
    csv_writer_cam0.writerow(['#timestamp [ns]', 'filename'])
    csv_writer_cam1.writerow(['#timestamp [ns]', 'filename'])
    csv_writer_imu.writerow(['#timestamp [ns]', 'w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]', 'w_RS_S_z [rad s^-1]', 
                             'a_RS_S_x [m s^-2]', 'a_RS_S_y [m s^-2]', 'a_RS_S_z [m s^-2]'])
    
    img0_list = os.listdir(kitti_path + '/image_00/data')
    img1_list = os.listdir(kitti_path + '/image_01/data')
    imu_list = os.listdir(kitti_path + '/oxts/data')
    img0_list.sort(key=lambda x:int(x.split('.')[0]))
    img1_list.sort(key=lambda x:int(x.split('.')[0]))
    imu_list.sort(key=lambda x:int(x.split('.')[0]))
    for ts_img0, f_img0, in zip(f_cam0_raw_timestamps, img0_list):
        str_time0 = ts_img0.split('.')
        time0_array = time.strptime(str_time0[0], "%Y-%m-%d %H:%M:%S")
        timer0 = str("{:.0f}".format(float(time.mktime(time0_array)) * 1e9 + int(str_time0[1])))
        f_cam0_timestamps.write(timer0 + '\n')

        img0 = cv2.imread(kitti_path + '/image_00/data/' + f_img0)
        img0_name = timer0 + ".png"
        cv2.imwrite(cam0_dat_dir + img0_name, img0)
        csv_writer_cam0.writerow([timer0, img0_name])

    for ts_img1, f_img1 in zip(f_cam1_raw_timestamps, img1_list):
        str_time1 = ts_img1.split('.')
        time1_array = time.strptime(str_time1[0], "%Y-%m-%d %H:%M:%S")
        timer1 = str("{:.0f}".format(float(time.mktime(time1_array)) * 1e9 + int(str_time1[1])))
        f_cam1_timestamps.write(timer1 + '\n')

        img1 = cv2.imread(kitti_path + '/image_01/data/' + f_img1)
        img1_name = timer1 + ".png"
        cv2.imwrite(cam1_dat_dir + img1_name, img1)
        csv_writer_cam1.writerow([timer1, img1_name])

    for ts_imu, f_imu in zip(f_imu_raw_timestamps, imu_list):
        str_time_imu = ts_imu.split('.')
        time_imu_array = time.strptime(str_time_imu[0], "%Y-%m-%d %H:%M:%S")
        timer_imu = str("{:.0f}".format(float(time.mktime(time_imu_array)) * 1e9 + int(str_time_imu[1])))

        with codecs.open(kitti_path + '/oxts/data/' + f_imu, 'r', 'utf-8') as f:
            read_data = f.read()
            imu_data = read_data.split()
            csv_writer_imu.writerow([timer_imu, imu_data[17], imu_data[18], imu_data[19], 
                                     imu_data[11], imu_data[12], imu_data[13]])

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Construct EuRoC format datasets from kitti raw data.
    ''')
    parser.add_argument('kitti', help='kitti raw path')
    parser.add_argument('--output', default='output')
    args = parser.parse_args()

    in_dir = os.path.abspath(args.kitti)
    out_dir = os.path.join(in_dir, args.output)

    print in_dir
    
    cam0_dir = out_dir + '/cam0/'
    cam1_dir = out_dir + '/cam1/'
    imu0_dir = out_dir + '/imu0/'

    if not os.path.exists(cam0_dir):
        os.makedirs(cam0_dir)
    if not os.path.exists(cam1_dir):
        os.makedirs(cam1_dir)
    if not os.path.exists(imu0_dir):
        os.makedirs(imu0_dir)

    print('Construct EuRoC format datasets from kitti path ' + out_dir)
    # print('Saving files to path ' + out_fn)
    construct(in_dir, cam0_dir, cam1_dir, imu0_dir)