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

class final_challenge:
    def __init__(self):
        self.detect_colour = False
        self.found_colour = False

        self.bootup = False
        self.search = True

        self.count = 0
        self.step = 0

        rospy.init_node('final_node', anonymous=True)
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
            self.search = True
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

    def set_coordinate(self):
        self.robot_odom.posx0 = self.robot_odom.posx
        self.robot_odom.posy0 = self.robot_odom.posy

    def set_orientation(self):
        self.robot_odom.yaw0 = self.robot_odom.yaw

    def in_exclusion_zone(self):
        half_width = 1

        if self.start_zone_B():
            half_width = 0.3

        if self.start_zone_A():
            half_width = 0.8#1.2
        x1 = self.robot_odom.originx - half_width
        x2 = self.robot_odom.originx + half_width
        y1 = self.robot_odom.originy - half_width
        y2 = self.robot_odom.originy + half_width
        if self.robot_odom.posx >= x1 and self.robot_odom.posx <= x2 and self.robot_odom.posy >= y1 and self.robot_odom.posy <= y2:
            return True
        else:
            return False

    def bootup_action(self):
        if self.step == 0:
            self.robot_controller.set_move_cmd(0.18,0)
            if self.has_moved_distance(0.4):
                self.robot_controller.stop()
                self.step += 1

        if self.step == 1:
            self.count += 1
            if self.count < 60000:
                self.robot_controller.set_move_cmd(0,0.7)
            else:
                self.count = 0
                self.robot_controller.stop()
                self.detect_colour = True
                self.step += 1

        if self.step == 2:
            if self.detect_freespace_front(-20,20,0.8):
                self.robot_controller.stop()
                self.step += 1
            else:
                self.robot_controller.set_move_cmd(0,0.7)

        if self.step == 3:
            self.count += 1
            if self.count < 10000:
                self.robot_controller.set_move_cmd(0.2,0)
            else:
                self.count = 0
                self.robot_controller.stop()
                self.bootup = False
                self.step += 1

    def avoid(self):
        if self.front_min_distance<0.5 and not self.ctrl_c:
            #print("DETECTED OBJECT AHEAD")
            self.robot_controller.stop()
            if self.front_min_angle<0:
                #print("| ANGLE IS LESS THAN 0")
                while self.front_min_distance<0.5 and not self.ctrl_c:
                    self.robot_controller.set_move_cmd(angular=-0.5)
                    self.robot_controller.publish()
            elif self.front_min_angle>=0:
                #print("| ANGLE IS GREATER THAN OR EQUAL TO 0")
                while self.front_min_distance<0.5 and not self.ctrl_c:
                    self.robot_controller.set_move_cmd(angular=0.4) # was 0.5
                    self.robot_controller.publish()
        elif self.left_min_distance<=0.37 and self.right_min_distance<=0.37 and not self.ctrl_c: # was 0.5 then 0.6
            #print("DETECTED A NARROW GAP")
            self.robot_controller.stop()
            while self.left_min_distance<=0.37 and self.right_min_distance<=0.37 and not abs(abs(self.left_min_angle) - abs(self.right_min_angle))<=5 and not self.ctrl_c: # was 0.5
                self.left_min_angle == -61 and self.right_min_angle == 60
                self.left_min_angle != self.right_min_angle
                if abs(self.left_min_angle) > abs(self.right_min_angle):
                    #print("CASE 1")
                    self.robot_controller.set_move_cmd(angular=0.1)
                    self.robot_controller.publish()
                else:
                    #print("CASE 2")
                    self.robot_controller.set_move_cmd(angular=-0.1)
                    self.robot_controller.publish()
            #print("EQUALITY")
        elif self.left_min_distance<0.37:
            #print("CAUGHT IN THE LEFT")
            self.robot_controller.stop()
            while self.left_min_distance<0.37 and not self.ctrl_c:
                self.robot_controller.set_move_cmd(angular=-0.4) # was 0.5
                self.robot_controller.publish()
        elif self.right_min_distance<0.37:
            #print("CAUGHT IN THE RIGHT")
            self.robot_controller.stop()
            while self.right_min_distance<0.37 and not self.ctrl_c:
                self.robot_controller.set_move_cmd(angular=0.4) # was 0.5
                self.robot_controller.publish()

    def test(self):
        wall_adjustment = False
        case1 = False
        case2 = False
        case3 = False
        if self.detect_freespace_front(-20,20,0.5):
            case1 = False
            case2 = False
            case3 = False
            self.robot_controller.set_move_cmd(0.2,0)
            """
            if self.detect_obj_left(-90,-71,0.3):
                self.robot_controller.stop()
                wall_adjustment = True
                if wall_adjustment == True:
                    self.robot_controller.set_move_cmd(angular=-0.4)
                    if self.left_min_angle == -90:
                        wall_adjustment = False
            """
        else:
            """
            if self.detect_obj_right(70,110,1) and not self.detect_obj_left(-111,-71,0.7):
                print("wall right")
            """
            if self.detect_obj_right(70,110,1) and not self.detect_obj_left(-111,-71,0.7):
                case1 = True
            elif not self.detect_obj_right(70,110,0.7) and self.detect_obj_left(-111,-71,1):
                case2 = True
            else:
                case3 = True

            if case1 == True:
                print("trying to turn left")
                case2 = False
                case3 = False
                while not self.detect_freespace_front(-20,20,0.5):
                    self.robot_controller.set_move_cmd(0,0.5)
            elif case2 == True:
                print("trying to turn right 111111")
                case1 = False
                case3 = False
                while not self.detect_freespace_front(-20,20,0.5):
                    self.robot_controller.set_move_cmd(0,-0.5)
            elif case3 == True:
                print("trying to turn right 222222")
                case2 = False
                case1 = False
                while not self.detect_freespace_front(-20,20,0.5):
                    self.robot_controller.set_move_cmd(0,-0.5)

    def main_loop(self):
        while not self.ctrl_c == True:
            if self.bootup == True:
                self.bootup_action()
            elif self.search == True:
                self.test()
            self.robot_controller.publish()


if __name__ == '__main__':
    final_challenge_instance = final_challenge()
    try:
        final_challenge_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
