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

    def __init__(self, name, interface, uav_ns, height):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = [])

        # members
        self.name = name
        self.height = height

        # interface elements
        interface.add_client('cli_land',uav_ns+'/'+'land',Land)
        interface.add_client('cli_take_off',uav_ns+'/'+'take_off',TakeOff)

        self.iface = interface

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
                self.iface['cli_take_off'](TakeOffRequest(height=self.height,
                                           blocking=True))
                return 'success'
            r.sleep()

        return 'error'
