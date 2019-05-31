import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

# message definitions

# main class
class Task(smach.State):

    def __init__(self, name, interface):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = [])

        # members
        self.name = name

        self.iface = interface

    # main function
    def execute(self, userdata):
        self.recall_preempt()

        #wait until state is preempted, ie. until a task execution
        #is requested
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                return 'success'
            r.sleep()

        return 'error'
