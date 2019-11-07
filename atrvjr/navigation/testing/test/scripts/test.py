#!/usr/bin/env python
import rospy
import numpy as np

from nav_msgs.msg import Odometry

if __name__ == '__main__':
    rospy.init_node('origin_odom_rl_stat_trans')
    sub_utm = rospy.Publisher('/novatel/novatel_node/gps_odom', Odometry, queue_size=10)
    sub_utm2 = rospy.Publisher('/odom_calibration/odom_calibration_node/odom', Odometry, queue_size=10)

    first_utm = Odometry()

    first_utm.header.frame_id = 'origin'
    for i in range(6):
        first_utm.pose.covariance[7*i] = 1
    r=rospy.Rate(1)
    while not rospy.is_shutdown():
        first_utm.header.stamp = rospy.Time.now()
        first_utm.pose.pose.position.x = 0
        first_utm.pose.pose.position.y = 0
        sub_utm.publish(first_utm)

        first_utm.pose.pose.position.x = 3
        sub_utm2.publish(first_utm)
        r.sleep
