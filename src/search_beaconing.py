#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry
from math import sqrt,radians, pi
import datetime as dt
import os
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class search_beaconing:
    def __init__(self):
        self.detect_colour = False
        self.found_colour = False

        self.forward_speed = 0.2
        self.turn_speed = 0.3

        rospy.init_node('search_beaconing_node', anonymous=True)
        self.rate = rospy.Rate(10)
        self.camera_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cvbridge_interface = CvBridge()
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        self.front_min_distance = 0.0
        self.left_min_distance = 0.0
        self.right_min_distance = 0.0

        self.front_min_angle = 0.0
        self.left_min_angle = 0.0
        self.right_min_angle = 0.0

        self.m00 = 0
        self.m00_min = 10000

        self.colours = ['Blue','Red','Green','Turquoise','Yellow','Purple']
        self.lower = [(115, 224, 100), (0, 185, 100), (55, 150, 100), (75, 50, 150), (20, 100, 100), (146, 150, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (62, 255, 255), (100, 255, 255), (30, 255, 255), (153, 255, 255)]

        self.mask = None
        self.max_white_space_index = 0

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        if self.detect_colour:
            white_space = [None]*6
            for i in range(6):
                mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
                white_space[i] = cv2.countNonZero(mask)
            self.max_white_space_index = white_space.index(max(white_space))
            self.mask = cv2.inRange(hsv_img, self.lower[self.max_white_space_index], self.upper[self.max_white_space_index])
            print("SEARCH INITIATED: The target colour is " + self.colours[self.max_white_space_index])
            self.detect_colour = False
        else:
            self.mask = cv2.inRange(hsv_img, self.lower[self.max_white_space_index], self.upper[self.max_white_space_index])

        m = cv2.moments(self.mask)

        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def scan_callback(self, scan_data):
        angle_tolerance = 5

        front_left_arc = scan_data.ranges[0:21]
        front_right_arc = scan_data.ranges[-20:]
        front_front_arc = np.array(front_left_arc[::-1] + front_right_arc[::-1])
        front_arc_angles = np.arange(-20, 21)
        self.front_min_distance = front_front_arc.min()
        self.front_min_angle = front_arc_angles[np.argmin(front_front_arc)]

        left_left_arc = scan_data.ranges[40:61]
        left_right_arc = scan_data.ranges[21:40]
        left_front_arc = np.array(left_left_arc[::-1] + left_right_arc[::-1])
        left_arc_angles = np.arange(-61, -21)
        self.left_min_distance = left_front_arc.min()
        self.left_min_angle = left_arc_angles[np.argmin(left_front_arc)]

        right_left_arc = scan_data.ranges[-40:-20]
        right_right_arc = scan_data.ranges[-60:-40]
        right_front_arc = np.array(right_left_arc[::-1] + right_right_arc[::-1])
        right_arc_angles = np.arange(20, 60)
        self.right_min_distance = right_front_arc.min()
        self.right_min_angle = right_arc_angles[np.argmin(right_front_arc)]

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        self.robot_controller.stop()

    def main_loop(self):
        while not self.ctrl_c:
            print("Working")

if __name__ == '__main__':
    search_beaconing_instance = search_beaconing()
    try:
        search_beaconing_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
