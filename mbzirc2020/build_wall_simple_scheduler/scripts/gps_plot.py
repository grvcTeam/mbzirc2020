#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import numpy as np
import PyKDL #Vector, Rotation, Quaternion, Frame
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from novatel_msgs.msg import BESTPOS



class PlannedMotion(object):
    """
    Components that move the arm in a planned motion.

    """
    def __init__(self):
        
        
    # Subscribers

        rospy.Subscriber("/novatel/novatel_node/full_psr", BESTPOS, self.gps_cb)
        rospy.Subscriber("/odom_calibration/odom_calibration_node/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/odometry/filtered", Odometry, self.ekf_cb)
        
        self.max_x = 0
        self.max_y = 0
        self.max_velx = 0
        self.max_vely = 0
        self.count = 1


    def gps_cb(self, msg):

        self.gps_msg = msg

        #self.count = self.count + 1 
        #print self.count

        #print self.gps_msg.lat, self.gps_msg.lon

        with open('coordinates11.txt', 'a') as the_file:
            the_file.write(str(self.gps_msg.lat) + "," + str(self.gps_msg.lon)+ "\n")

    def odom_cb(self, msg):
        """
        Obtains the joint configuration where the arm will be moved.

        """
        self.odom_msg = msg

        if abs(self.odom_msg.pose.pose.position.x)> self.max_x:
            self.max_x=abs(self.odom_msg.pose.pose.position.x)
            print "XXXXXXXXXXX", self.max_x

        if abs(self.odom_msg.pose.pose.position.y)> self.max_y:
            self.max_y=abs(self.odom_msg.pose.pose.position.y)
            print "YYYYYYYYYYYYYYY", self.max_y

        if abs(self.odom_msg.twist.twist.linear.x)> self.max_velx:
            self.max_velx=abs(self.odom_msg.twist.twist.linear.x)
            print "vellllllllllllllllXXXXXXXXXXX", self.max_velx

        if abs(self.odom_msg.twist.twist.linear.y)> self.max_vely:
            self.max_vely=abs(self.odom_msg.twist.twist.linear.y)
            print "vellllllllllllllllYYYYYYYYYYYYY", self.max_vely


    def ekf_cb(self, msg):
        """
        Obtains the joint configuration where the arm will be moved.

        """
        self.odom_msg = msg

        if abs(self.odom_msg.pose.pose.position.x)> self.max_x:
            self.max_x=abs(self.odom_msg.pose.pose.position.x)
            print "EKF_XXXXXXXXXXX", self.max_x

        if abs(self.odom_msg.pose.pose.position.y)> self.max_y:
            self.max_y=abs(self.odom_msg.pose.pose.position.y)
            print "EKF_YYYYYYYYYYYYYYY", self.max_y

        if abs(self.odom_msg.twist.twist.linear.x)> self.max_velx:
            self.max_velx=abs(self.odom_msg.twist.twist.linear.x)
            print "EKF_vellllllllllllllllXXXXXXXXXXX", self.max_velx

        if abs(self.odom_msg.twist.twist.linear.y)> self.max_vely:
            self.max_vely=abs(self.odom_msg.twist.twist.linear.y)
            print "EKF_vellllllllllllllllYYYYYYYYYYYYY", self.max_vely




def main():

    rospy.init_node('wall_scheduler')
    planned_motion= PlannedMotion()
    rospy.spin()

if __name__ == '__main__':
    main()
