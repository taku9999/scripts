#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
import numpy as np
import ros_numpy
import open3d as o3d
import time
import datetime
import os

dirpass = "/workspace/docker_bind_Local"

def get_time():
    now_time = datetime.datetime.now() + datetime.timedelta(hours=9)
    time_str = str(now_time.strftime("%Y-%m-%d_%H:%M:%S_%f")[:-4])
    return time_str

def make_dirname():
    global dirpass
    dirpass += "/lidar-data/" + get_time()
    os.makedirs(dirpass)

def lidar_callback(message):
    print("========== PCD data receive ==========")
    pc = ros_numpy.numpify(message)
    points = np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float32))
    o3d.io.write_point_cloud(dirpass + "/" + get_time() +".pcd", pcd)

def imu_callback(imu_data):
    print("========== IMU Data receive ==========")
    with open(dirpass + "/imu_log.csv", mode="a") as f:
        f.write(get_time())
        f.write("," + str(imu_data.angular_velocity.x) + "," + str(imu_data.angular_velocity.y) + "," + str(imu_data.angular_velocity.z))
        f.write("," + str(imu_data.linear_acceleration.x) + "," + str(imu_data.linear_acceleration.y) + "," + str(imu_data.linear_acceleration.z) + "\r\n")
    # time.sleep(5)

def subscriber():
    rospy.init_node('point_sub') #ノードの初期化
    rospy.Subscriber('/livox/lidar', PointCloud2 , lidar_callback)  #点群トピックSubscribe用
    rospy.Subscriber('/livox/imu', Imu , imu_callback) #IMUトピックSubscribe用

    make_dirname() # データ保存先の設定

    rospy.spin() #コールバック関数を繰り返し呼び出す

if __name__ == "__main__":
    subscriber()