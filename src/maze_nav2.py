#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry
from math import radians, pi
import datetime as dt
import os
import numpy as np

class avoid:
    def __init__(self):
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.init_node('avoid_node', anonymous=True)
        self.rate = rospy.Rate(10)
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        self.turn = False

        self.front_min_distance = 0.0
        self.left_min_distance = 0.0
        self.right_min_distance = 0.0

        self.front_min_angle = 0.0
        self.left_min_angle = 0.0
        self.right_min_angle = 0.

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)


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

        x = self.robot_odom.posx
        x0 = self.robot_odom.posx
        y = self.robot_odom.posy
        y0 = self.robot_odom.posy
        z = self.robot_odom.yaw
        z0 = self.robot_odom.yaw
        total_distance = 0.0

        while not self.ctrl_c:
            self.robot_controller.set_move_cmd(linear=0.26) # was 0.26 then 0.22
            self.robot_controller.publish()
            x0 = self.robot_odom.posx
            y0 = self.robot_odom.posy
            z0 = self.robot_odom.yaw

            if self.front_min_distance<0.5 and not self.ctrl_c:
                print("DETECTED OBJECT AHEAD")
                self.robot_controller.stop()
                if self.front_min_angle<0:
                    print("| ANGLE IS LESS THAN 0")
                    while self.front_min_distance<0.5 and not self.ctrl_c:
                        self.robot_controller.set_move_cmd(angular=-0.5)
                        self.robot_controller.publish()
                elif self.front_min_angle>=0:
                    print("| ANGLE IS GREATER THAN OR EQUAL TO 0")
                    while self.front_min_distance<0.5 and not self.ctrl_c:
                        self.robot_controller.set_move_cmd(angular=0.4) # was 0.5
                        self.robot_controller.publish()
            elif self.left_min_distance<=0.57 and self.right_min_distance<=0.57 and not self.ctrl_c: # was 0.5 then 0.6
                print("DETECTED A NARROW GAP")
                self.robot_controller.stop()
                while self.left_min_distance<=0.57 and self.right_min_distance<=0.57 and not abs(abs(self.left_min_angle) - abs(self.right_min_angle))<=5 and not self.ctrl_c: # was 0.5
                    self.left_min_angle == -61 and self.right_min_angle == 60
                    self.left_min_angle != self.right_min_angle
                    if abs(self.left_min_angle) > abs(self.right_min_angle):
                        print("CASE 1")
                        self.robot_controller.set_move_cmd(angular=0.1)
                        self.robot_controller.publish()
                    else:
                        print("CASE 2")
                        self.robot_controller.set_move_cmd(angular=-0.1)
                        self.robot_controller.publish()
                print("EQUALITY")
            elif self.left_min_distance<0.37:
                print("CAUGHT IN THE LEFT")
                self.robot_controller.stop()
                while self.left_min_distance<0.37 and not self.ctrl_c:
                    self.robot_controller.set_move_cmd(angular=-0.4) # was 0.5
                    self.robot_controller.publish()
            elif self.right_min_distance<0.37:
                print("CAUGHT IN THE RIGHT")
                self.robot_controller.stop()
                while self.right_min_distance<0.37 and not self.ctrl_c:
                    self.robot_controller.set_move_cmd(angular=0.4) # was 0.5
                    self.robot_controller.publish()



if __name__ == '__main__':
    avoid_instance = avoid()
    try:
        avoid_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
