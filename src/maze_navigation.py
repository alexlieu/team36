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

class maze_navigation:
    def __init__(self):
        # Forward, find wall, turn to wall, follow wall
        self.startup = False
        self.turn_left_90 = False
        self.turn_left_180 = False
        self.turn_right_180 = False
        self.turn_initiated = False
        self.forward = True
        self.found_wall = False
        self.following_wall = False

        self.turn_count = 0

        self.forward_speed = 0.2
        self.turn_speed = 0.3

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        rospy.init_node('maze_navigation_node', anonymous=True)
        self.rate = rospy.Rate(10)
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        self.front_min_distance = 0.0
        self.left_min_distance = 0.0
        self.right_min_distance = 0.0

        self.front_min_angle = 0.0
        self.left_min_angle = 0.0
        self.right_min_angle = 0.0

        self.origin_x = 0.0
        self.origin_y = 0.0

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

    def set_coordinate(self):
        self.robot_odom.posx0 = self.robot_odom.posx
        self.robot_odom.posy0 = self.robot_odom.posy

    def set_orientation(self):
        self.robot_odom.yaw0 = self.robot_odom.yaw

    def has_turned_angle(self, angle):
        #print ("Yaw 0 - ")
        #print (self.robot_odom.yaw0)
        #print "Yaw - "
        #print (self.robot_odom.yaw)
        #print ("Angle - ")
        #print (angle)


        if abs(self.robot_odom.yaw0 - self.robot_odom.yaw) >= angle:
            print ("Yaw0 - Yaw >= Angle")
            return True
        else:
            #print ("Yaw0 - Yaw not >= Angle")
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

    def detect_obj_front(self, angle_l, angle_r, distance):
        if self.front_min_angle >= angle_l and self.front_min_angle <= angle_r and self.front_min_distance <= distance:
            return True
        else:
            return False

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        self.robot_controller.stop()

    def main_loop(self):
        while not self.ctrl_c:
            if self.forward == True: # Set off out start zone

                #print ("Going Forward")
                self.robot_controller.set_move_cmd(linear=self.forward_speed)

                #if self.front_min_distance < 0.5 and self.front_min_angle > -5 and self.front_min_angle < 5 and self.front_min_distance != 0 and self.front_min_angle != 0:

                if self.detect_obj_front(-5, 5, 0.5) and self.front_min_distance != 0 and self.front_min_angle != 0:
                    #print ("Detected object - front")

                    self.robot_controller.stop()
                    self.set_orientation()
                    self.set_coordinate()
                    self.forward = False
                    self.found_wall = True

            elif self.found_wall == True: # Find a wall
                #print ("Found Wall")
                self.found_wall = False
                self.turn_initiated = True

            elif self.turn_initiated == True: # Turn right
                #print ("Turn initiated")
                self.robot_controller.stop()
                self.robot_odom.yaw0 = self.robot_odom.yaw
                self.set_orientation()
                self.set_coordinate()

                #wall_adjustment = True
                #if wall_adjustment == True:
                self.robot_controller.set_move_cmd(angular=-0.4)
                #print ("Wall adjustment is true")

                #print (self.left_min_angle)


                if self.left_min_angle == -90:

                    #print ("Left min angle = -90")

                    if self.detect_obj_front(-5, 5, 0.5) == False:

                        print (self.detect_obj_front(-5, 5, 0.5))

                        self.robot_controller.stop()
                        self.robot_odom.yaw0 = self.robot_odom.yaw

                        #wall_adjustment = False

                        #print ("Wall adjustment now false")

                    #self.set_orientation()
                    #self.set_coordinate()
                        self.turn_initiated = False
                        self.turn_count += 1
                        self.following_wall = True

                    elif turn_count > 0:

                        self.robot_controller.set_move_cmd(angular=-0.4)

                        if self.left_min_angle == -180:

                            self.robot_controller.stop()
                            self.robot_odom.yaw0 = self.robot_odom.yaw

                            self.turn_initiated = True
                            self.following_wall = False


                #if self.detect_obj_left(-90,-71,0.5):

                    #print ("Detected object - left")

                    #wall_adjustment = True
                    #if wall_adjustment == True:
                        #self.robot_controller.set_move_cmd(angular=-0.4)
                        #print ("Wall adjustment is true")

                        #if self.left_min_angle == -90:
                            #wall_adjustment = False

                            #print ("Wall adjustment now false")

                            #self.set_orientation()
                            #self.set_coordinate()

                    #elif self.detect_obj_right(70, 110, 1):

                        #print ("Detected object - right")

                        #wall_adjustment = True
                        #if wall_adjustment == True:
                            #self.robot_controller.set_move_cmd(angular=-0.4)
                            #print ("Wall adjustment is true")

                            #if self.right_min_angle == 70:
                                #wall_adjustment = False

                                #print ("Wall adjustment now false")

                                #self.set_orientation()
                                #self.set_coordinate()

                    #elif self.detect_obj_front():
                        #print ("Detected object - front")

                        #wall_adjustment = True
                        #if wall_adjustment == True:
                            #self.robot_controller.set_move_cmd(angular=-0.4)
                            #print ("Wall adjustment is true")

                            #if self.right_min_angle == 70:
                                #wall_adjustment = False

                                #print ("Wall adjustment now false")

                                #self.set_orientation()
                                #self.set_coordinate()


                #else:
                    #self.turn_right_90 = False
                    #self.following_wall = True

                    #self.turn_right_90 = False

                    #self.following_wall = True

            elif self.following_wall == True: # Follow wall - go parallel to wall
                print ("Following Wall")

                self.robot_odom.yaw0 = self.robot_odom.yaw
                #self.robot_controller.set_move_cmd(linear=self.forward_speed)

                #print ("Going Forward")
                self.robot_controller.stop()
                self.set_orientation()
                self.set_coordinate()
                self.robot_controller.set_move_cmd(linear=self.forward_speed)


                self.following_wall = False
                self.forward = True

                # To do:
                # 1. Set off and find wall
                # 2. Turn right - follow wall
                # 3. Move forward, avoid hitting wall


            # final task - stop in end zone
            # Include in final step of if statement - self.ctrl_c = True
            self.robot_controller.publish()




if __name__ == '__main__':
    maze_navigation_instance = maze_navigation()
    try:
        maze_navigation_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
