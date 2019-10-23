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

from math import sqrt, pow

class node():

    def __init__(self):

        #self.set_io = rospy.ServiceProxy('ur_driver/set_io', SetIO)
        rospy.Service('magnetize', Magnetize, self.magnetize_cb)

        self.pub = rospy.Publisher('attached', GripperAttached, queue_size=1)
        self.pub2 = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=1)
        self.sub = rospy.Subscriber('/wrench', WrenchStamped, self.wrench_cb)

        self.attached = False
        self.threshold = 25  #N



    def magnetize_cb(self,req):

        if not req.magnetize:
            
            self.pub2.publish(data='sec voltageProgram():\nset_tool_voltage(24)\nend ')


            rospy.sleep(1)

            # while self.force<0:
            #     rospy.sleep(1)
            #     self.pub2.publish(data='set_tool_voltage({v}) '.format(v=v))
                
            # else:
            return MagnetizeResponse(success=True)   

        if req.magnetize:
            
            self.pub2.publish(data='sec voltageProgram():\nset_tool_voltage(0)\nend ')

            rospy.sleep(1)

            return MagnetizeResponse(success=True)

    def wrench_cb(self, msg):

        self.force = msg.wrench.force.z

         # to compensate gripper weigth ~ 15 N
        #print f

        '''def norm(v):
            return sqrt(pow(v.x,2)+pow(v.y,2)+pow(v.z,2))'''

        #print norm(msg.wrench.force) - 15

        if self.force < -self.threshold and not self.attached:
            print self.force
            self.attached = True
            self.pub.publish(attached=True)
        elif abs(self.force) < self.threshold and self.attached:
            print self.force
            self.attached = False
            self.pub.publish(attached=False)

    


def main():

    rospy.init_node('ur5_magnetic_gripper')
    n = node()
    rospy.spin()

if __name__ == '__main__':
    main()
