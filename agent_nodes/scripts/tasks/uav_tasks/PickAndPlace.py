import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

import tasks.uav_tasks.PickObject as PickObject
import tasks.uav_tasks.PlaceObject as PlaceObject
import tasks.uav_tasks.GoToWaypoint as GoToWaypoint

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
    def __init__(self, name, interface, uav_ns, z_offset):
        smach.StateMachine.__init__(self,outcomes=['success','error'],
        input_keys = ['shared_regions','type','scale','goal_pose','obj_pose'],
        output_keys = ['place_real_pose'])

        self.iface = interface

        #properties. TODO: properties should be part of the Task module and checking if they are present in AgentInterface be done automatically for every task
        properties = ['height', 'global_frame', 'agent_frame', 'gripper_frame']
        for prop in properties:
            if prop not in interface.agent_props:
                raise AttributeError('{task} is missing required property {prop} and cannot '\
                'be instantiated.'.format(task=name,prop=prop))

        self.props = self.iface.agent_props

        with self:
            smach.StateMachine.add('Go_Task', GoToWaypoint.Task('go', interface, uav_ns),
                transitions={'success':'Pick_Task','error':'error'}, remapping ={'way_pose':'obj_pose'})
            smach.StateMachine.add('Pick_Task', PickObject.Task('Pick_Task',interface,uav_ns, z_offset), {'success':'Go_Task2','error':'error'})
            smach.StateMachine.add('Go_Task2', GoToWaypoint.Task('go2', interface, uav_ns),
                transitions={'success':'Place_Task','error':'error'}, remapping ={'way_pose':'goal_pose'})
            smach.StateMachine.add('Place_Task', PlaceObject.Task('Place_Task',interface,uav_ns, z_offset), {'success':'success','error':'error'})