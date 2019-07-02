import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

import tasks.ugv_tasks.PickFromPile as PickFromPile
import tasks.ugv_tasks.PlaceObject as PlaceObject

# required message definitions
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon, Point32
from mbzirc_comm_objs.srv import PFPNPlace,PFPNPlaceResponse, AddSharedRegion, AddSharedRegionRequest

# task properties
ResponseType = PFPNPlaceResponse
DataType = PFPNPlace
transitions={'success':'success','error':'error'}

# function to create userdata from a task execution request matching the task
# input keys
def gen_userdata(req):
    userdata = smach.UserData()
    userdata.type = req.type
    userdata.goal_pose = req.goal_pose
    userdata.pile_centroid = req.pile_centroid
    userdata.shared_regions = {}
    for i in range(len(req.shared_regions)):
        userdata.shared_regions[i] = req.shared_regions[i]

    return userdata


# main class
class Task(smach.StateMachine):

    # init
    def __init__(self, name, interface, ugv_ns, z_offset):
        smach.StateMachine.__init__(self,outcomes=['success','error'],
        input_keys = ['shared_regions','type','goal_pose','pile_centroid'],
        output_keys = ['place_real_pose'] )

        self.iface = interface

        #properties. TODO: properties should be part of the Task module and checking if they are present in AgentInterface be done automatically for every task
        properties = ['global_frame', 'agent_frame', 'gripper_frame', 'rb_aabb']
        for prop in properties:
            if prop not in interface.agent_props:
                raise AttributeError('{task} is missing required property {prop} and cannot '\
                'be instantiated.'.format(task=name,prop=prop))

        with self:
            smach.StateMachine.add('Pick_Task', PickFromPile.Task('PFPile_Task',interface, ugv_ns, z_offset), {'success':'Place_Task','error':'error'})
            smach.StateMachine.add('Place_Task', PlaceObject.Task('Place_Task',interface, ugv_ns, z_offset), {'success':'success','error':'error'})
