#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import numpy as np
import PyKDL #Vector, Rotation, Quaternion, Frame

from mbzirc_comm_objs.srv import Magnetize, MagnetizeResponse
from mbzirc_comm_objs.msg import GripperAttached
from IOService.srv import setIO, setIORequest
from geometry_msgs.msg import WrenchStamped

class node():

    def magnetize_cb(self,req):
        v = 24 if req.magnetize else 0
        self.set_io(fun=setIORequest.FUN_SET_TOOL_VOLTAGE,state=v)
        return MagnetizeResponse(success=True)

    def wrench_cb(self, msg):

        if abs(msg.wrench.force.y) > self.threshold and not self.attached:
            self.attached = True
            self.pub.publish(attached=True)
        elif abs(msg.wrench.force.y) < self.threshold and self.attached
            self.attached = False
            self.pub.publish(attached=False)

    def __init__(self):

        self.set_io = rospy.ServiceProxy('ur_driver/set_io', setIO)
        rospy.Service('magnetize', Empty, self.magnetize_cb)

        self.pub = rospy.Publisher('attached', GripperAttached, queue_size=1)
        self.sub = rospy.Subscriber('/wrench', WrenchStamped, wrench_cb)

        self.attached = False
        self.threshold = 15 #N


def main():

    rospy.init_node('ur5_magnetic_gripper')
    n = node()
    rospy.spin()

if __name__ == '__main__':
    main()
