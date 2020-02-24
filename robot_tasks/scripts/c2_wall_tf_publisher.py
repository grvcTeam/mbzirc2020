#!/usr/bin/env python

import rospy
import tf2_ros
import tf.transformations
from geometry_msgs.msg import PoseStamped, TransformStamped
from mbzirc_comm_objs.srv import PublishWallTfs, PublishWallTfsResponse

broadcaster = tf2_ros.StaticTransformBroadcaster()

def publishWallTfsCallback(req):
    global broadcaster
    broadcaster.sendTransform(req.wall_tfs)
    rospy.loginfo('Publishing wall tfs!')
    return PublishWallTfsResponse()

def main():
    rospy.init_node('wall_tf_publisher')

    service = rospy.Service('publish_wall_tfs', PublishWallTfs, publishWallTfsCallback)

    rospy.spin()

if __name__ == '__main__':
    main()