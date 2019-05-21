import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

import tasks.ugv_tasks.PickObject as PickObject
import tasks.ugv_tasks.PlaceObject as PlaceObject

# required message definitions
from mbzirc_comm_objs.srv import PickNPlace,PickNPlaceResponse

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon, Point32

# task properties
ResponseType = PickNPlaceResponse
DataType = PickNPlace
transitions={'success':'success','error':'error'}

# function to create userdata from a task execution request matching the task
# input keys
def gen_userdata(req):
    userdata = smach.UserData()
    userdata.type = req.type
    userdata.scale = req.scale
    userdata.goal_pose = req.goal_pose
    userdata.obj_pose = req.obj_pose
    userdata.shared_regions = {}
    for i in range(len(req.shared_regions)):
        userdata.shared_regions[i] = req.shared_regions[i]

    return userdata

# main class
class Task(smach.StateMachine):

    # init
    def __init__(self, name, interface, ugv_ns, global_frame, ugv_frame, base_aabb, ws_aabb, gripper_frame, z_offset):
        smach.StateMachine.__init__(self,outcomes=['success','error'],
        input_keys = ['shared_regions','type','scale','goal_pose','obj_pose'])

        with self:
            smach.StateMachine.add('Pick_Task', PickObject.Task('Pick_Task',interface,ugv_ns, global_frame, ugv_frame, base_aabb, ws_aabb, gripper_frame, z_offset), {'success':'Place_Task','error':'error'})
            smach.StateMachine.add('Place_Task', PlaceObject.Task('Place_Task',interface,ugv_ns, global_frame, ugv_frame, base_aabb, ws_aabb, gripper_frame, z_offset), {'success':'success','error':'error'})
