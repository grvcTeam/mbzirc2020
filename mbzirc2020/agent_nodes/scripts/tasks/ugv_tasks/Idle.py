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

        self.iface = interface

        #properties. TODO: properties should be part of the Task module and checking if they are present in AgentInterface be done automatically for every task
        properties = []
        for prop in properties:
            if prop not in interface.agent_props:
                raise AttributeError('{task} is missing required property {prop} and cannot '\
                'be instantiated.'.format(task=name,prop=prop))

        self.props = self.iface.agent_props

        # members
        self.name = name

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
