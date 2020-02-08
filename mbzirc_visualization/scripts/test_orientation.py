#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def pose_callback(data):
    x = data.pose.orientation.x
    y = data.pose.orientation.y
    rospy.loginfo("sq(x) = %f"%(x*x))
    rospy.loginfo("sq(y) = %f"%(y*y))
    rospy.loginfo("sq(x) + sq(y) = %f"%(x*x + y*y))

def main():
    rospy.init_node('test_orientation')
    pose_sub = rospy.Subscriber('ual/pose', PoseStamped, pose_callback)
    rospy.spin()


if __name__ == '__main__':
    main()

