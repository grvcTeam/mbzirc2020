import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

# message definitions
from uav_abstraction_layer.srv import TakeOff, TakeOffRequest

# main class
class Task(smach.State):

    def __init__(self, name, interface, uav_ns, height, global_frame, uav_frame):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = [])

        # members
        self.name = name
        self.height = height
        self.global_frame = global_frame
        self.uav_frame = uav_frame

        # interface elements
        interface.add_client('cli_take_off',uav_ns+'/'+'take_off',TakeOff)

        self.iface = interface

    # main function
    def execute(self, userdata):
        self.recall_preempt()

        try:
            trans_global2uav = lookup_tf_transform(self.global_frame, self.uav_frame, self.iface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print self.name + ' Task could not be executed'
            return 'error'

        self.iface['cli_take_off'](TakeOffRequest(height=self.height-trans_global2uav.transform.translation.z,blocking=True))
        
        #wait until state is preempted, ie. until a task execution
        #is requested
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                return 'success'
            r.sleep()

        return 'error'
