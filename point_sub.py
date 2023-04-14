#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import ros_numpy
import open3d as o3d

def callback(message):
    pc = ros_numpy.numpify(message)
    points = np.zeros((pc.shape[0],3)) # (pc.shape[0])行 3列の初期化配列
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float32))
    o3d.io.write_point_cloud("/workspace/docker_bind/test.pcd", pcd)


def subscriber():
    #ノードの初期化
    rospy.init_node('point_sub') 
    #subscriberの作成
    sub = rospy.Subscriber('/livox/lidar', PointCloud2 , callback)
    #コールバック関数を繰り返し呼び出す
    rospy.spin()

if __name__ == "__main__":
    subscriber()