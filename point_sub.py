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

filepass = "/workspace/docker_bind_Local/lidar"

def get_time():
    now_time = datetime.datetime.now() + datetime.timedelta(hours=9)
    time_str = str(now_time.strftime("%Y-%m-%d_%H:%M:%S_%f")[:-4])
    return time_str

def lidar_callback(message):
    print("========== PCD data receive ==========")
    pc = ros_numpy.numpify(message)
    points = np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float32))
    o3d.io.write_point_cloud(filepass + "/" + get_time() +".pcd", pcd)

def imu_callback(imu_data):
    print("========== IMU Data receive ==========")
    with open(filepass + "/imu_log.csv", mode="a") as f:
        f.write(get_time())
        f.write("," + str(imu_data.angular_velocity.x) + "," + str(imu_data.angular_velocity.y) + "," + str(imu_data.angular_velocity.z))
        f.write("," + str(imu_data.linear_acceleration.x) + "," + str(imu_data.linear_acceleration.y) + "," + str(imu_data.linear_acceleration.z) + "\r\n")
    # time.sleep(5)

def subscriber():
    #ノードの初期化
    rospy.init_node('point_sub') 
    #点群トピックSubscribe用
    rospy.Subscriber('/livox/lidar', PointCloud2 , lidar_callback)
    #IMUトピックSubscribe用
    rospy.Subscriber('/livox/imu', Imu , imu_callback)
    #コールバック関数を繰り返し呼び出す
    rospy.spin()

if __name__ == "__main__":
    subscriber()