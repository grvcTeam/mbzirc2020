import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

# message definitions
from uav_abstraction_layer.srv import TakeOff, TakeOffRequest, Land, LandRequest

# main class. TODO: not tested
class Task(smach.State):

    def __init__(self, name, interface, uav_ns):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = [])

        self.iface = interface

        #properties. TODO: properties should be part of the Task module and checking if they are present in AgentInterface be done automatically for every task
        properties = ['height']
        for prop in properties:
            if prop not in interface.agent_props:
                raise AttributeError('{task} is missing required property {prop} and cannot '\
                'be instantiated.'.format(task=name,prop=prop))

        self.props = self.iface.agent_props

        # members
        self.name = name

        # interface elements
        interface.add_client('cli_land',uav_ns+'/'+'land',Land)
        interface.add_client('cli_take_off',uav_ns+'/'+'take_off',TakeOff)

    # main function
    def execute(self, userdata):
        self.recall_preempt()

        #land UAVAgent
        self.iface['cli_land'](LandRequest(blocking=True))

        #wait until state is preempted, ie. until a task execution
        #is requested
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.iface['cli_take_off'](TakeOffRequest(height=self.props['height'],
                                           blocking=True))
                return 'success'
            r.sleep()

        return 'error'
