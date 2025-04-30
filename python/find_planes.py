#!/usr/bin/env python

import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

import rospy
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu, CameraInfo, Image

from depth_to_pcd import depth_to_pcd
from get_normal import get_normal

import matplotlib.pyplot as plt

class FindPlaneNode:
    def __init__(self):

        self.imu = None
        self.camera_intrinsic = None

        rospy.init_node('find_plane_node')

        self.imu_topic = rospy.get_param('imu_filtered_topic', 100)
        self.depth_img_topic = rospy.get_param('depth_img_topic', 20)
        self.depth_intrinsic_topic = rospy.get_param('depth_intrinsic_topic', 1)
        self.rescale_depth = 1000

        self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imuCallback)
        self.depth_img_sub = rospy.Subscriber(self.depth_img_topic, Image, self.depthImageCallback)
        self.depth_intrinsic_sub = rospy.Subscriber(self.depth_intrinsic_topic, CameraInfo, self.depthIntrinsicCallback)
    def imuCallback(self, msg):
        self.imu = msg
    
    def depthIntrinsicCallback(self, msg):
        self.camera_intrinsic = msg
    
    def depthImageCallback(self, msg):
        if self.imu == None or self.camera_intrinsic == None:
            return
        
        W = msg.width
        H = msg.height

        depth = np.frombuffer(msg.data, dtype=np.int16)/self.rescale_depth
        
        pts_3d, _ = depth_to_pcd(depth, self.camera_intrinsic.K, W, H)

        grav_normal = np.array([self.imu.linear_acceleration.x, self.imu.linear_acceleration.y, self.imu.linear_acceleration.z])
        grav_normal = grav_normal / np.linalg.norm(grav_normal)

        # Find img normal
        img_normal = get_normal(depth.reshape(H,W), self.camera_intrinsic.K)
        img_normal_pos = img_normal.reshape(-1, 3)
        img_normal_neg = -img_normal_pos
        dot1 = np.dot(img_normal_pos, grav_normal).reshape(-1, 1)
        dot2 = np.dot(img_normal_neg, grav_normal).reshape(-1, 1)
        
        dot = np.concatenate((dot1, dot2), axis=1)
        normal_index = np.argmax(dot, axis=1)
        img_normal = np.zeros_like(img_normal_pos)
        img_normal[normal_index == 0] = img_normal_pos[normal_index == 0]
        img_normal[normal_index == 1] = img_normal_neg[normal_index == 1]

        if False:
            img_normal_rgb = (img_normal + 1)/2 * 255
            img_normal_rgb = img_normal_rgb.astype(np.uint8)
            img_normal_rgb = img_normal_rgb.reshape(H, W, 3)
            plt.imsave("/catkin_ws/src/gravity_plane_est/normal.png", img_normal_rgb)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = FindPlaneNode()
    node.run()