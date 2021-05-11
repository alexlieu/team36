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

class object_detection:
    def __init__(self):
        self.startup = False
        self.turn_left_90 = False
        self.turn_left_180 = False
        self.turn_right_180 = False
        self.found_colour = False
        self.forward = True

        self.forward_speed = 0.2
        self.turn_speed = 0.3

        rospy.init_node('avoid_node', anonymous=True)
        self.rate = rospy.Rate(10)
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

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

        if self.startup:
            white_space = [None]*6
            for i in range(6):
                mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
                white_space[i] = cv2.countNonZero(mask)
            self.max_white_space_index = white_space.index(max(white_space))
            self.mask = cv2.inRange(hsv_img, self.lower[self.max_white_space_index], self.upper[self.max_white_space_index])
            print("SEARCH INITIATED: The target colour is " + self.colours[self.max_white_space_index])
            self.turn_left_90 = True
            self.startup = False
        else:
            self.mask = cv2.inRange(hsv_img, self.lower[self.max_white_space_index], self.upper[self.max_white_space_index])

        m = cv2.moments(self.mask)

        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        self.robot_controller.stop()

    def main_loop(self):
        while not self.ctrl_c:
            if self.forward == True:
                self.robot_controller.set_move_cmd(linear=self.forward_speed)
                if sqrt(pow(self.robot_odom.posx0 - self.robot_odom.posx, 2) + pow(self.robot_odom.posy0 - self.robot_odom.posy, 2)) >= 1:
                    self.robot_controller.stop()
                    self.robot_odom.posx0 = self.robot_odom.posx
                    self.robot_odom.posy0 = self.robot_odom.posy
                    self.forward = False
                    self.turn_right_180 = True
            elif self.turn_right_180 == True:
                self.robot_controller.set_move_cmd(angular=-self.turn_speed)
                if abs(self.robot_odom.yaw0 - self.robot_odom.yaw) >= 180:
                    self.robot_controller.stop()
                    self.robot_odom.yaw0 = self.robot_odom.yaw
                    self.turn_right_180 = False
                    self.startup = True
            elif self.turn_left_90 == True:
                self.robot_controller.set_move_cmd(angular=self.turn_speed)
                if abs(self.robot_odom.yaw0 - self.robot_odom.yaw) >= 90:
                    self.robot_controller.stop()
                    self.robot_odom.yaw0 = self.robot_odom.yaw
                    self.turn_left_90 = False
                    self.turn_left_180 = True
            elif self.turn_left_180 == True:
                self.robot_controller.set_move_cmd(angular=self.turn_speed)
                if self.m00 > self.m00_min:
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        self.robot_controller.stop()
                        self.turn_left_180 = False
                        self.found_colour = True
                if abs(self.robot_odom.yaw0 - self.robot_odom.yaw) >= 180:
                    self.robot_controller.stop()
                    self.robot_odom.yaw0 = self.robot_odom.yaw
                    self.turn_left_180 = False
                    print("SEARCH FAILED: A pillar of target colour " + self.colours[self.max_white_space_index] + " is not present.")
                    self.ctrl_c = True
            elif self.found_colour == True:
                print("SEARCH COMPLETE: The robot is now facing the target pillar.")
                self.ctrl_c = True
            self.robot_controller.publish()

if __name__ == '__main__':
    object_detection_instance = object_detection()
    try:
        object_detection_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
