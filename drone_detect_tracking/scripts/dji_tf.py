#!/usr/bin/env python
import rospy

import tf_conversions
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped


def pose_cb(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = 'base_link'
    t.transform.translation = msg.position
    t.transform.rotation = msg.orientation
    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('dji_tf')

    pose_pub = rospy.Publisher('/dji_control/pose_stamped', PoseStamped, queue_size=1)
    rospy.Subscriber('/dji_control/pose', Pose, pose_cb)

    while not rospy.is_shutdown():
        rospy.spin()
