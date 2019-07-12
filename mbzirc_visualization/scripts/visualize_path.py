#!/usr/bin/env python
import rospy
import collections
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

path = collections.deque(maxlen=1000)  # TODO: maxlen as a parameter?
path_pub = rospy.Publisher('path', Path, queue_size=10)

def odom_callback(data):
    global path
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.append(pose)
    path_to_pub = Path()
    path_to_pub.header = data.header
    for p in path:
        path_to_pub.poses.append(p)
    path_pub.publish(path_to_pub)

def main():
    rospy.init_node('visualize_path')
    odom_sub = rospy.Subscriber('ual/odom', Odometry, odom_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
