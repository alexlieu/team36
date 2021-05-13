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

        """
        self.bootup = True
        self.start_bootup = True
        """
        self.bootup = False
        self.start_bootup = False

        self.start_position = False

        self.search = True

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

        self.origin_x = 0.0
        self.origin_y = 0.0

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
            self.start_position = True
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

        left_left_arc = scan_data.ranges[90:111]#scan_data.ranges[40:61]
        left_right_arc = scan_data.ranges[71:90]#scan_data.ranges[21:40]
        left_front_arc = np.array(left_left_arc[::-1] + left_right_arc[::-1])
        left_arc_angles = np.arange(-111,-71)#np.arange(-61, -21)
        self.left_min_distance = left_front_arc.min()
        self.left_min_angle = left_arc_angles[np.argmin(left_front_arc)]

        right_left_arc = scan_data.ranges[-90:-70]#scan_data.ranges[-40:-20]
        right_right_arc = scan_data.ranges[-110:-90]#scan_data.ranges[-60:-40]
        right_front_arc = np.array(right_left_arc[::-1] + right_right_arc[::-1])
        right_arc_angles = np.arange(70,110)#np.arange(20, 60)
        self.right_min_distance = right_front_arc.min()
        self.right_min_angle = right_arc_angles[np.argmin(right_front_arc)]

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        self.robot_controller.stop()

    def has_moved_distance(self, distance):
        if sqrt(pow(self.robot_odom.posx0 - self.robot_odom.posx, 2) + pow(self.robot_odom.posy0 - self.robot_odom.posy, 2)) >= distance:
            return True
        else:
            return False

    def has_turned_angle(self, angle):
        if abs(self.robot_odom.yaw0 - self.robot_odom.yaw) >= angle:
            return True
        else:
            return False

    def detect_obj_front(self, angle_l, angle_r, distance):
        if self.front_min_angle >= angle_l and self.front_min_angle <= angle_r and self.front_min_distance <= distance:
            return True
        else:
            return False

    def detect_obj_left(self, angle_l, angle_r, distance):
        if self.left_min_angle >= angle_l and self.left_min_angle <= angle_r and self.left_min_distance <= distance:
            return True
        else:
            return False

    def detect_obj_right(self, angle_l, angle_r, distance):
        if self.right_min_angle >= angle_l and self.right_min_angle <= angle_r and self.right_min_distance <= distance:
            return True
        else:
            return False

    def set_coordinate(self):
        self.robot_odom.posx0 = self.robot_odom.posx
        self.robot_odom.posy0 = self.robot_odom.posy

    def set_orientation(self):
        self.robot_odom.yaw0 = self.robot_odom.yaw

    def in_exclusion_zone(self):
        half_width = 1
        x1 = self.robot_odom.originx - half_width
        x2 = self.robot_odom.originx + half_width
        y1 = self.robot_odom.originy - half_width
        y2 = self.robot_odom.originy + half_width
        if self.robot_odom.posx >= x1 and self.robot_odom.posx <= x2 and self.robot_odom.posy >= y1 and self.robot_odom.posy <= y2:
            return True
        else:
            return False

    def startup(self):
        if self.start_bootup == True:
            self.robot_controller.set_move_cmd(linear=self.forward_speed)
            if self.has_moved_distance(0.5):
                print(self.robot_odom.originx)
                print(self.robot_odom.originy)
                print(self.in_exclusion_zone())
                print(self.robot_odom.posx)
                print(self.robot_odom.posy)
                self.robot_controller.stop()
                self.set_coordinate()
                self.start_bootup = False
        else:
            if self.robot_odom.yaw0 < 0:
                self.robot_controller.set_move_cmd(angular=self.turn_speed)
            else:
                self.robot_controller.set_move_cmd(angular=-self.turn_speed)
            if self.has_turned_angle(180):
                self.robot_controller.stop()
                self.set_orientation()
                self.bootup = False
                self.detect_colour = True


    def main_loop(self):
        check_right_wall = True
        right_wall = False
        scan = False
        count = 0
        left_wall_adjustment = False

        while not self.ctrl_c:
            if self.bootup == True:
                self.startup()
            elif self.start_position == True:
                if check_right_wall == True:
                    right_wall = self.detect_obj_right(50, 60, 1)
                    check_right_wall = False
                if right_wall == True:
                    self.robot_controller.set_move_cmd(angular=self.turn_speed)
                    if self.has_turned_angle(90):
                        self.robot_controller.stop()
                        self.set_orientation()
                        self.set_coordinate()
                        self.start_position = False
                        self.search = True
                else:
                    self.robot_controller.set_move_cmd(angular=-self.turn_speed)
                    if self.has_turned_angle(90):
                        self.robot_controller.stop()
                        self.set_orientation()
                        self.set_coordinate()
                        self.start_position = False
                        self.search = True
            elif self.search == True:
                self.robot_controller.set_move_cmd(linear=self.forward_speed)
                if scan == True:
                    if count <= 4:
                        self.robot_controller.set_move_cmd(angular=self.turn_speed)
                        if self.has_turned_angle(90):
                            self.robot_controller.stop()
                            self.set_orientation()
                            self.set_coordinate()
                            count+=1
                    else:
                        count = 0
                        scan = False
                else:
                    if self.has_moved_distance(7):
                        self.robot_controller.stop()
                        self.set_orientation()
                        self.set_coordinate()
                        scan = True
                    else:
                        if self.front_min_distance<0.5:
                            self.set_orientation()
                            wall_adjustment = True
                            if wall_adjustment == True:
                                self.robot_controller.set_move_cmd(angular=-0.4)
                                if self.has_turned_angle(90):
                                    self.robot_controller.stop()
                                    wall_adjustment = False
                        if self.detect_obj_left(-90,-71,0.5) and self.front_min_distance<0.5:
                            self.set_orientation()
                            wall_adjustment = True
                            if wall_adjustment == True:
                                self.robot_controller.set_move_cmd(angular=-0.4)
                                if self.has_turned_angle(90):
                                    self.robot_controller.stop()
                                    wall_adjustment = False
                        elif self.detect_obj_left(-90,-71,0.5):
                            wall_adjustment = True
                            if wall_adjustment == True:
                                self.robot_controller.set_move_cmd(angular=-0.4)
                                if self.left_min_angle == -90:
                                    wall_adjustment = False
                        elif self.detect_obj_right(70,90,0.5) and self.front_min_distance<0.5:
                            self.set_orientation()
                            wall_adjustment = True
                            if wall_adjustment == True:
                                self.robot_controller.set_move_cmd(angular=0.4)
                                if self.has_turned_angle(90):
                                    self.robot_controller.stop()
                                    wall_adjustment = False
                        elif self.detect_obj_right(70,90,0.5):
                            wall_adjustment = True
                            if wall_adjustment == True:
                                self.robot_controller.set_move_cmd(angular=0.4)
                                if self.right_min_angle == -90:
                                    wall_adjustment = False
                        elif self.detect_obj_left(-90,-71,0.5) and self.detect_obj_right(70,90,0.5):
                            wall_adjustment = True
                            if wall_adjustment == True:
                                if count <= 2:
                                    self.robot_controller.set_move_cmd(angular=self.turn_speed)
                                    if self.has_turned_angle(90):
                                        self.robot_controller.stop()
                                        self.set_orientation()
                                        self.set_coordinate()
                                        count+=1
                                else:
                                    count = 0
                                    wall_adjustment = False






            # print("current odometry: x = {:.3f}, y = {:.3f}, theta_z = {:.3f}".format(self.robot_odom.posx, self.robot_odom.posy, self.robot_odom.yaw))

            self.robot_controller.publish()

if __name__ == '__main__':
    search_beaconing_instance = search_beaconing()
    try:
        search_beaconing_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
