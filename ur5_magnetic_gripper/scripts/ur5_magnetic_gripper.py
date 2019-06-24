#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import numpy as np
import PyKDL #Vector, Rotation, Quaternion, Frame

from mbzirc_comm_objs.srv import Magnetize, MagnetizeResponse
from mbzirc_comm_objs.msg import GripperAttached
from ur_msgs.srv import SetIO, SetIORequest
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String

class node():

    def magnetize_cb(self,req):
        v = 0 if req.magnetize else 24
        #self.set_io(fun=SetIORequest.FUN_SET_TOOL_VOLTAGE,state=v)
        self.pub2.publish(data='set_tool_voltage({v})'.format(v=v))
        return MagnetizeResponse(success=True)

    def wrench_cb(self, msg):

        if abs(msg.wrench.force.y) > self.threshold and not self.attached:
            self.attached = True
            self.pub.publish(attached=True)
        elif abs(msg.wrench.force.y) < self.threshold and self.attached:
            self.attached = False
            self.pub.publish(attached=False)

    def __init__(self):

        #self.set_io = rospy.ServiceProxy('ur_driver/set_io', SetIO)
        rospy.Service('magnetize', Magnetize, self.magnetize_cb)

        self.pub = rospy.Publisher('attached', GripperAttached, queue_size=1)
        self.pub2 = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
        self.sub = rospy.Subscriber('/wrench', WrenchStamped, self.wrench_cb)

        self.attached = False
        self.threshold = 15 #N


def main():

    rospy.init_node('ur5_magnetic_gripper')
    n = node()
    rospy.spin()

if __name__ == '__main__':
    main()
