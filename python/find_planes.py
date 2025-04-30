#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu, CameraInfo, Image

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
        if self.imu == None or self. camera_intrinsic == None:
            return
        
        W = msg.width
        H = msg.height

        depth = np.frombuffer(msg.data, dtype=np.int16)/self.rescale_depth
        depth = depth.reshape(H,W,1)
        

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = FindPlaneNode()
    node.run()