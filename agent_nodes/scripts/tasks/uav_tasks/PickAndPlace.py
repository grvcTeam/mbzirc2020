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
from mbzirc_comm_objs.srv import PickNPlace,PickNPlaceResponse, AddSharedRegion, AddSharedRegionRequest

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
    userdata.shared_regions = req.shared_regions

    ''' --- harcoded for testing ---
    #add region
    rospy.wait_for_service('/add_shared_region')
    try:
        add_reg = rospy.ServiceProxy('/add_shared_region', AddSharedRegion)
        req = AddSharedRegionRequest()
        req.frame_id = 'map'
        req.waiting_points = [Point(8,-2,0)]
        req.region.points = [Point32(8,-2,0),Point32(12,-2,0),Point32(12,2,0),Point32(8,2,0)]
        res = add_reg(req)
        userdata.shared_regions = {res.region_id:req.region}
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    #other keys
    userdata.type = 'brick'
    userdata.goal_pose = Pose(position=Point(0,0,0),orientation=Quaternion(0,0,0,1))
    userdata.scale = Vector3(0.5,0.5,0.5)
    userdata.obj_pose = Pose(position=Point(10,0,0.25),orientation=Quaternion(0,0,0,1))'''

    return userdata

# main class
class Task(smach.StateMachine):

    # init
    def __init__(self, name, interface, uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset):
        smach.StateMachine.__init__(self,outcomes=['success','error'],
        input_keys = ['shared_regions','type','scale','goal_pose','obj_pose'])

        with self:
            smach.StateMachine.add('Go_Task', GoToWaypoint.Task('go', interface, uav_ns, height, global_frame, uav_frame),
                transitions={'success':'Pick_Task','error':'error'}, remapping ={'way_pose':'obj_pose'})
            smach.StateMachine.add('Pick_Task', PickObject.Task('Pick_Task',interface,uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset), {'success':'Go_Task2','error':'error'})
            smach.StateMachine.add('Go_Task2', GoToWaypoint.Task('go2', interface, uav_ns, height, global_frame, uav_frame),
                transitions={'success':'Place_Task','error':'error'}, remapping ={'way_pose':'goal_pose'})
            smach.StateMachine.add('Place_Task', PlaceObject.Task('Place_Task',interface,uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset), {'success':'success','error':'error'})
