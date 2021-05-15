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


        self.bootup = True
        self.start_bootup = True
        """
        self.bootup = False
        self.start_bootup = False
        """

        self.start_position = False

        self.scan_interval = 4.5
        # start_position B
        # scan_interval 3
        # x = -1.2409, y = 2.0656
        #
        # start_position C
        # x = 2.0684, y = 1.9707
        # scan_interval 1.5
        #
        # start_position A
        # x = -2.078, y = -1.47
        # scan_interval 1.5
        self.search = False

        self.jiggle_counter_l = 0
        self.jiggle_counter_r = 0
        self.reset_jiggle_counter = False

        self.test3 = False
        self.test1 = False
        self.test2 = False

        self.forward_speed = 0.2
        self.turn_speed = 0.5

        self.count = 0

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
        self.lower = [(115, 224, 100), (0, 185, 100), (55, 150, 100), (75, 150, 100), (20, 100, 100), (146, 150, 100)]
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
        crop_height = 100#400
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

    def has_moved_vertical_distance(self, distance):
        if abs(self.robot_odom.posy0 - self.robot_odom.posy) >= distance:
            return True
        else:
            return False

    def move_distance(self, distance):
        self.robot_controller.stop()
        self.set_coordinate()
        self.set_orientation()
        if not self.has_moved_distance(distance):
            self.robot_controller.set_move_cmd(self.forward_speed,0)
        else:
            self.set_coordinate()
            self.set_orientation()


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

    def detect_freespace_front(self, angle_l, angle_r, distance):
        if self.front_min_angle >= angle_l and self.front_min_angle <= angle_r and self.front_min_distance >= distance:
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

    def detect_corner_left(self, angle_l, angle_r, distance):
        if self.corner_left_min_angle >= angle_l and self.corner_left_min_angle <= angle_r and self.corner_left_min_distance <= distance:
            return True
        else:
            return False

    def detect_corner_right(self, angle_l, angle_r, distance):
        if self.corner_right_min_angle >= angle_l and self.corner_right_min_angle <= angle_r and self.corner_right_min_distance <= distance:
            return True
        else:
            return False

    def set_coordinate(self):
        self.robot_odom.posx0 = self.robot_odom.posx
        self.robot_odom.posy0 = self.robot_odom.posy

    def set_orientation(self):
        self.robot_odom.yaw0 = self.robot_odom.yaw

    def start_zone_A(self):
        half_width = 1
        x1 = self.robot_odom.originx - half_width
        x2 = self.robot_odom.originx + half_width
        y1 = self.robot_odom.originy - half_width
        y2 = self.robot_odom.originy + half_width
        if self.robot_odom.posx >= x1 and self.robot_odom.posx <= x2 and self.robot_odom.posy >= y1 and self.robot_odom.posy <= y2:
            return True
        else:
            return False

    def start_zone_C(self):
        half_width = 1
        x1 = self.robot_odom.originx - half_width
        x2 = self.robot_odom.originx + half_width
        y1 = self.robot_odom.originy - half_width
        y2 = self.robot_odom.originy + half_width
        if self.robot_odom.posx >= x1 and self.robot_odom.posx <= x2 and self.robot_odom.posy >= y1 and self.robot_odom.posy <= y2:
            return True
        else:
            return False


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
        escape_corner = False

        while not self.ctrl_c:
            if self.bootup == True:
                self.startup()
            elif self.start_position == True:
                if check_right_wall == True:
                    right_wall = self.detect_obj_right(70, 110, 1)
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
                #print("jiggle counter: l = {:.3f}, r = {:.3f}".format(self.jiggle_counter_l, self.jiggle_counter_r))
                print("distance travelled: d = {:.3f}".format(sqrt(pow(self.robot_odom.posx0 - self.robot_odom.posx, 2) + pow(self.robot_odom.posy0 - self.robot_odom.posy, 2))))
                #print("vertical distance travelled: d = {:.3f}".format(abs(self.robot_odom.posy0 - self.robot_odom.posy)))
                if scan == True:
                    print("scanning")
                    if self.count < 190000:
                        self.robot_controller.set_move_cmd(angular=0.3)

                        if self.m00 > self.m00_min:
                            if self.cy >= 560-100 and self.cy <= 560+100:
                                print("BEACON DETECTED: Beaconing initiated.")
                                print(self.robot_odom.originx)
                                print(self.robot_odom.originy)
                                self.robot_controller.stop()
                                self.search = False
                                self.test1 = True
                                scan = False
                        self.count += 1
                    else:
                        self.robot_controller.stop()
                        self.set_orientation()
                        self.set_coordinate()
                        self.count = 0
                        scan = False
                else:
                    if self.has_moved_distance(self.scan_interval): #self.has_moved_vertical_distance(2):
                        self.robot_controller.stop()
                        self.set_orientation()
                        self.set_coordinate()
                        scan = True
                    else:
                        if self.detect_freespace_front(-20,20,1):
                            self.robot_controller.set_move_cmd(linear=self.forward_speed)
                            self.jiggle_counter_l = 0
                            self.jiggle_counter_r = 0
                        else:
                            if self.jiggle_counter_l >= 6 and self.jiggle_counter_r >= 6:
                                print("Stuck in corner")
                                escape_corner = True
                            else:
                                wall_adjustment = True
                                if wall_adjustment == True:
                                    if self.front_min_angle < 0:
                                        self.robot_controller.set_move_cmd(angular=-self.turn_speed)
                                        self.jiggle_counter_l += 1
                                        wall_adjustment = False
                                    else:
                                        self.robot_controller.set_move_cmd(angular=self.turn_speed)
                                        self.jiggle_counter_r += 1
                                        wall_adjustment = False
                                elif escape_corner == True:
                                    if self.front_min_angle < 0:
                                        self.robot_controller.set_move_cmd(angular=-self.turn_speed)
                                    else:
                                        self.robot_controller.set_move_cmd(angular=self.turn_speed)
            elif self.test1 == True:
                if not self.in_exclusion_zone():
                    if self.detect_obj_front(-20,20,0.4):
                        self.robot_controller.set_move_cmd(0.15, 0)
                        if self.detect_obj_front(-20,20,0.3):
                            print("BEACONING COMPLETE: The robot has now stopped.")
                            self.robot_controller.stop()
                            self.ctrl_c = True
                    else:
                        self.robot_controller.set_move_cmd(self.forward_speed, 0)
                else:
                    self.robot_controller.stop()
                    self.test1 = False
                    self.test2 = True
            elif self.test2 == True:
                self.robot_controller.set_move_cmd(angular=self.turn_speed)
                print(self.robot_odom.yaw)
                if self.robot_odom.yaw >= 90:
                    self.robot_controller.stop()
                    print("turned away")
                    self.set_orientation()
                    self.set_coordinate()
                    self.robot_controller.stop()
                    self.test2 = False
                    self.test3 = True

                """
                if self.count < 150000:
                    self.robot_controller.set_move_cmd(angular=self.turn_speed)
                    self.count += 1
                else:
                    self.count = 0
                    print("turned away")
                    self.set_orientation()
                    self.set_coordinate()
                    self.robot_controller.stop()
                    self.test2 = False
                    self.test3 = True
                """
            elif self.test3 == True:
                if self.count < 130000:
                    self.robot_controller.set_move_cmd(angular=self.turn_speed)
                    self.count += 1
                    if self.m00 > self.m00_min:
                        if self.cy >= 560-100 and self.cy <= 560+100:
                            print("BEACON DETECTED: Beaconing initiated.")
                            self.robot_controller.stop()
                            self.test2 = False
                            self.test1 = True
                else:
                    self.count = 0
                    print("scan turned away")
                    self.set_orientation()
                    self.set_coordinate()
                    self.robot_controller.stop()
                    self.scan_interval += 1.5
                    self.test3 = False
                    self.search = True






            # print("current odometry: x = {:.3f}, y = {:.3f}, theta_z = {:.3f}".format(self.robot_odom.posx, self.robot_odom.posy, self.robot_odom.yaw))

            self.robot_controller.publish()

if __name__ == '__main__':
    search_beaconing_instance = search_beaconing()
    try:
        search_beaconing_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
