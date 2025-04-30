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
from gravity_correction import gravity_correction
from get_mask import get_mask
from hsv import hsv_img

import matplotlib.pyplot as plt

class FindPlaneNode:
    def __init__(self):
        self.rescale_depth = 1000
        self.dot_bound = 0.9
        self.correction_iteration = 5
        self.kernel_size = 21
        self.cluster_size = 5

        self.imu = None
        self.camera_intrinsic = None

        rospy.init_node('find_plane_node')

        self.imu_topic = rospy.get_param('imu_filtered_topic', 100)
        self.depth_img_topic = rospy.get_param('depth_img_topic', 20)
        self.depth_intrinsic_topic = rospy.get_param('depth_intrinsic_topic', 1)

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
        
        pts_3d, index_2d = depth_to_pcd(depth, self.camera_intrinsic.K, W, H)

        grav_normal = np.array([self.imu.linear_acceleration.x, self.imu.linear_acceleration.y, self.imu.linear_acceleration.z])
        grav_normal = grav_normal / (np.linalg.norm(grav_normal) + 1e-15)

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

        grav_normal = gravity_correction(grav_normal,img_normal, pts_3d, self.dot_bound,self.correction_iteration)



        if True:
            dot1 = np.dot(img_normal, grav_normal).reshape(-1,1)
            dot2 = np.dot(img_normal, -grav_normal).reshape(-1,1)

            angle_dist = np.concatenate((dot1, dot2), axis=1)
            angle_dist = np.max(angle_dist, axis=1)
            scalar_dist = np.dot(pts_3d, grav_normal)
            scalar_dist[angle_dist < self.dot_bound] = 0
            scalar_dist[pts_3d[:, 2] == 0] = 0

            # Plot histogram
            fig, ax = plt.subplots()
            ax.hist(scalar_dist[scalar_dist!=0], bins=1000)
            plt.xlabel("Distance")
            plt.ylabel("Count")
            plt.title("Histogram of Distance")
            fig.savefig("/catkin_ws/src/gravity_plane_est/histogram.png")

        
        mask = get_mask(grav_normal, img_normal, pts_3d, self.dot_bound, self.kernel_size, self.cluster_size)

        if True:
            plt.imsave("/catkin_ws/src/gravity_plane_est/mask.png", hsv_img(mask.reshape(H,W)))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = FindPlaneNode()
    node.run()