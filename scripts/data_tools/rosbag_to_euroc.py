# coding:utf-8
#!/usr/bin/python
  
# Extract images from a bag file.
import os
import sys
import argparse

import roslib
import rosbag
import rospy
import decimal
import cv2
import csv
import codecs
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError



def construct(bagfile, out_filename):
    bridge = CvBridge()
    cam0_cnt = 0
    cam1_cnt = 0
    imu_cnt = 0
    n = 0
    f_cam0 = codecs.open(out_filename + 'cam0/data.csv', 'w', 'utf-8')
    f_cam1 = codecs.open(out_filename + 'cam1/data.csv', 'w', 'utf-8')
    f_imu = codecs.open(out_filename + 'imu0/data.csv', 'w', 'utf-8')
    f_timestampscam = codecs.open(out_filename + 'cam_timestamps.txt', 'w', 'utf-8')
    csv_writer_cam0 = csv.writer(f_cam0)
    csv_writer_cam1 = csv.writer(f_cam1)
    csv_writer_imu = csv.writer(f_imu)
    csv_writer_cam0.writerow(['#timestamp [ns]', 'filename'])
    csv_writer_cam1.writerow(['#timestamp [ns]', 'filename'])
    csv_writer_imu.writerow(['#timestamp [ns]', 'w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]', 'w_RS_S_z [rad s^-1]', 
                             'a_RS_S_x [m s^-2]', 'a_RS_S_y [m s^-2]', 'a_RS_S_z [m s^-2]'])
    with rosbag.Bag(bagfile, 'r') as bag:
        for topic,msg,t in bag.read_messages():
            if topic == "/cam0/image_raw":
                try:
                    cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
                except CvBridgeError as e:
                    print e
                timestr = "{:.9f}".format(msg.header.stamp.to_sec())
                timer = str("{:.0f}".format(1e9 * float(timestr)))
                image_name = timer + ".png"
                cv2.imwrite(out_filename + "mav0/cam0/data/" + image_name, cv_image)
                csv_writer_cam0.writerow([timer, image_name])
                f_timestampscam.write(timer + '\n')
                cam0_cnt += 1
            elif topic == "/cam1/image_raw":
                try:
                    cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
                except CvBridgeError as e:
                    print e
                timestr = "{:.9f}".format(msg.header.stamp.to_sec())
                timer = str("{:.0f}".format(1e9 * float(timestr)))
                image_name = timer + ".png"
                cv2.imwrite(out_filename + "mav0/cam1/data/" + image_name, cv_image)
                csv_writer_cam1.writerow([timer, image_name])
                cam1_cnt += 1
            elif topic == "/imu0":
                timestr = "{:.9f}".format(msg.header.stamp.to_sec())
                timer = str("{:.0f}".format(1e9 * float(timestr)))
                csv_writer_imu.writerow([timer, "{:.18f}".format(msg.angular_velocity.x), "{:.18f}".format(msg.angular_velocity.y), "{:.18f}".format(msg.angular_velocity.z), 
                                         "{:.18f}".format(msg.linear_acceleration.x), "{:.18f}".format(msg.linear_acceleration.y), "{:.18f}".format(msg.linear_acceleration.z)])
                imu_cnt += 1
            n += 1
    print('wrote cam0 ' + str(cam0_cnt) + ' image data to the file: ' + out_filename + 'mav0/cam0/')
    print('wrote cam1 ' + str(cam1_cnt) + ' image data to the file: ' + out_filename + 'mav0/cam1/')
    print('wrote ' + str(imu_cnt) + ' imu data to the file: ' + out_filename + 'mav0/imu0/')
    print('wrote ' + str(n) + ' messages to the file: ' + out_filename + 'mav0/')
    f_cam0.close()
    f_cam1.close()
    f_imu.close()
    f_timestampscam.close()

 
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Construct datasets from bagfile.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('--output', default='')
    args = parser.parse_args()

    out_dir = os.path.dirname(os.path.abspath(args.bag))
    out_fn = os.path.join(out_dir, args.output)

    print('Construct dataset from bag ' + args.bag)
    print('Saving to file ' + out_fn)
    construct(args.bag, out_fn)
