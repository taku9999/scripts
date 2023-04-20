#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
import numpy as np
import ros_numpy
import open3d as o3d
import time

def lidar_callback(message):
    pc = ros_numpy.numpify(message)
    points = np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float32))
    o3d.io.write_point_cloud("/workspace/docker_bind/test.pcd", pcd)

def imu_callback(imu_data):
    print("IMU Data:")
    print("Timestamp: ", imu_data.header.stamp)
    print("Orientation: x=", imu_data.orientation.x, ", y=", imu_data.orientation.y, ", z=", imu_data.orientation.z, ", w=", imu_data.orientation.w)
    print("Angular Velocity: x=", imu_data.angular_velocity.x, ", y=", imu_data.angular_velocity.y, ", z=", imu_data.angular_velocity.z)
    print("Linear Acceleration: x=", imu_data.linear_acceleration.x, ", y=", imu_data.linear_acceleration.y, ", z=", imu_data.linear_acceleration.z)
    print("")
    time.sleep(1)
    with open("/workspace/docker_bind/log_imu.csv", mode="a") as f:
        f.write(str(imu_data.header.stamp) + "," + str(imu_data.angular_velocity.x) + "," + str(imu_data.angular_velocity.y) + "," + str(imu_data.angular_velocity.z) + "\r\n")

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