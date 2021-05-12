#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import degrees

class TB3Odometry(object):
    def odom_cb(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')

        self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)

        if self.startup:
            self.startup = False
            self.posx0 = self.posx
            self.posy0 = self.posy
            self.yaw0 = self.yaw
            self.start_point_yaw

    def __init__(self):
        self.startup = True
        self.start_point_yaw = 0.0
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0
        self.posx0 = 0.0
        self.posy0 = 0.0
        self.yaw0 = 0.0
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)

    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)
