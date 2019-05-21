import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

import tasks.uav_tasks.PickFromPile as PickFromPile
import tasks.uav_tasks.PlaceObject as PlaceObject
import tasks.uav_tasks.GoToWaypoint as GoToWaypoint

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon, Point32

# required message definitions
from mbzirc_comm_objs.srv import PFPNPlace,PFPNPlaceResponse

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
    def __init__(self, name, interface, uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset):
        smach.StateMachine.__init__(self,outcomes=['success','error'],
        input_keys = ['shared_regions','type','goal_pose','pile_centroid'])

        with self:
            smach.StateMachine.add('PFP_Task', PickFromPile.Task('PFP_Task',interface,uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset), {'success':'Go_Task','error':'error'})
            smach.StateMachine.add('Go_Task', GoToWaypoint.Task('go', interface, uav_ns, height, global_frame, uav_frame),
                transitions={'success':'Place_Task','error':'error'}, remapping ={'way_pose':'goal_pose'})
            smach.StateMachine.add('Place_Task', PlaceObject.Task('Place_Task',interface,uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset), {'success':'success','error':'error'})
