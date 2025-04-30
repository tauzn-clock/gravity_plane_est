#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu

class FindPlaneNode:
    def __init__(self):
        rospy.init_node('find_plane_node')

        self.imu_topic = rospy.get_param('imu_filtered_topic', 100)

        rospy.loginfo("IMU Topic: %s", self.imu_topic)

        self.sub = rospy.Subscriber(self.imu_topic, Imu, self.callback)
        self.pub = rospy.Publisher("/output", Imu, queue_size=100)

    def callback(self, msg):
        rospy.loginfo("Received: %f", msg.linear_acceleration.x)
        self.pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = FindPlaneNode()
    node.run()