import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

# message definitions
from uav_abstraction_layer.srv import TakeOff, TakeOffRequest, GoToWaypoint, GoToWaypointRequest
from uav_abstraction_layer.msg import State
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

# main class
class Task(smach.State):

    def state_cb(self, msg):
        self.uav_state = msg.state

    def __init__(self, name, interface, uav_ns):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = [])

        self.iface = interface

        #properties. TODO: properties should be part of the Task module and checking if they are present in AgentInterface be done automatically for every task
        properties = ['height', 'global_frame', 'agent_frame']
        for prop in properties:
            if prop not in interface.agent_props:
                raise AttributeError('{task} is missing required property {prop} and cannot '\
                'be instantiated.'.format(task=name,prop=prop))

        self.props = self.iface.agent_props

        # members
        self.uav_state = State.UNINITIALIZED
        self.name = name

        # interface elements
        interface.add_client('cli_take_off',uav_ns+'/'+'take_off',TakeOff)
        interface.add_client('cli_go_waypoint',uav_ns+'/'+'go_to_waypoint',
                                GoToWaypoint)

        interface.add_subscriber(self,uav_ns+'/'+'state', State,
                                self.state_cb)

    # main function
    def execute(self, userdata):
        self.recall_preempt()

        rate = rospy.Rate(1.0)
        while self.uav_state == State.UNINITIALIZED:
            print self.name + ' task waiting for UAL to initialize'
            rate.sleep()

        if self.uav_state == State.LANDED_ARMED:
            self.iface['cli_take_off'](TakeOffRequest(height=self.props['height'],blocking=True))
        elif self.uav_state == State.FLYING_AUTO:
            try:
                trans_global2uav = lookup_tf_transform(self.props['global_frame'], self.props['agent_frame'], self.iface['tf_buffer'],5)
            except Exception as error:
                print repr(error)
                print self.name + ' Task could not be executed'
                return 'error'

            pose = from_geom_msgs_Transform_to_geom_msgs_Pose(trans_global2uav.transform)
            pose.position.z = self.props['height']
            way = GoToWaypointRequest(waypoint=PoseStamped(
            header=Header(frame_id=self.props['global_frame'],stamp=rospy.Time.now()),pose=
            pose),blocking=True )

            self.iface['cli_go_waypoint'](way)
        else:
            print self.name + ' Task could not be executed because UAV state = {s}'.format(s=self.uav_state)
            return 'error'

        #wait until state is preempted, ie. until a task execution
        #is requested
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                return 'success'
            r.sleep()

        return 'error'
