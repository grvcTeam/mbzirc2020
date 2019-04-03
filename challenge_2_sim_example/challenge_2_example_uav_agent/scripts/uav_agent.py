#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

from agent_node_help import *
from mbzirc_comm_objs.msg import ObjectDetectionList,  WallBluePrint
from mbzirc_comm_objs.srv import DetectTypes, DetectTypesRequest, SearchForObject, SearchForObjectResponse, BuildWall, BuildWallResponse
from uav_abstraction_layer.srv import GoToWaypoint, GoToWaypointRequest, TakeOff, TakeOffRequest, Land, LandRequest
from geometry_msgs.msg import Pose, PoseStamped, Point, Point32, Quaternion, PolygonStamped, Vector3

from agent_tasks import *

# Task Wrapper. It has input parameters.
class SearchForObjectTaskWrapper(AgentTaskWrapper):

    def __init__(self):
        AgentTaskWrapper.__init__(self, SearchForObjectsTask(),
                transitions={'found':'success','not_found':'success'})

class GoToWaypointTaskWrapper(AgentTaskWrapper):

    def __init__(self):
        AgentTaskWrapper.__init__(self, GoToWaypointTask(),
                transitions={'success':'success','error':'success'})

class PickObjectTaskWrapper(AgentTaskWrapper):

    def __init__(self):
        AgentTaskWrapper.__init__(self, PickObjectTask(),
                transitions={'success':'success','error':'success'})

class PlaceObjectTaskWrapper(AgentTaskWrapper):

    def __init__(self):
        AgentTaskWrapper.__init__(self, PlaceObjectTask(),
                transitions={'success':'success','error':'success'})

class PickAndPlaceObjectTaskWrapper(AgentTaskWrapper):

    def __init__(self):
        AgentTaskWrapper.__init__(self, PickAndPlaceObjectTask(),
                transitions={'success':'success','error':'success'})

class PickFromPileAndPlaceObjectTaskWrapper(AgentTaskWrapper):

    def __init__(self):
        AgentTaskWrapper.__init__(self, PickFromPileAndPlaceObjectTask(),
                transitions={'success':'success','error':'success'})

class BuildWallTaskWrapper(AgentTaskWrapper):

    def __init__(self):
        AgentTaskWrapper.__init__(self, BuildWallTask(),
                transitions={'success':'success','error':'success'})

class UAVAgent():

    #task exec callbacks
    '''def SearchForObject_cb(self, req):
        #build state machine userdata from request
        userdata = smach.UserData()
        userdata.header = req.header
        userdata.search_region = req.search_region
        userdata.object_types = req.object_types
        userdata.stop_after_find  = req.stop_after_find
        userdata.interface = self.agentInterface
        self.tasks_dic['search_for_object'].userdata = userdata

        #request execute task
        self.agentInterface['pub_start_task'].publish(
                StartTask(agent_id=self.agent_id,task_id='search_for_object'))

        return SearchForObjectResponse(success=True)'''

    def GoToWaypoint_cb(self, req):
        shared_region = PolygonStamped()
        shared_region.polygon.points = [Point32(-2,-2,0),Point32(2,-2,0),Point32(2,2,0),Point32(-2,2,0)]

        #build state machine userdata from request
        userdata = smach.UserData()
        userdata.header = req.search_region.header
        userdata.global_frame = 'map'
        userdata.uav_frame = 'uav_1'
        userdata.goal_pose = Pose(position=Point(10,5,5))
        userdata.shared_regions  = {'1': shared_region}
        userdata.interface = self.agentInterface
        self.tasks_dic['go_to_waypoint'].userdata = userdata

        #request execute task
        self.agentInterface['pub_start_task'].publish(
                StartTask(agent_id=self.agent_id,task_id='go_to_waypoint'))

        return SearchForObjectResponse(success=True)

    def PickObject_cb(self, req):
        #build state machine userdata from request
        userdata = smach.UserData()
        userdata.uav_frame = 'uav_1'
        userdata.gripper_frame = 'gripper_link'
        userdata.type = 'brick'
        userdata.scale = Vector3(0.5,0.5,0.5)
        userdata.obj_pose = Pose(position=Point(10,0,0.25),orientation=Quaternion(0,0,0,1))
        userdata.interface = self.agentInterface
        self.tasks_dic['pick_object'].userdata = userdata

        #request execute task
        self.agentInterface['pub_start_task'].publish(
                StartTask(agent_id=self.agent_id,task_id='pick_object'))

        return SearchForObjectResponse(success=True)

    def PlaceObject_cb(self, req):
        #build state machine userdata from request
        userdata = smach.UserData()
        userdata.global_frame = 'map'
        userdata.uav_frame = 'uav_1'
        userdata.gripper_frame = 'gripper_link'
        userdata.type = 'brick'
        userdata.scale = Vector3(0.5,0.5,0.5)
        userdata.goal_pose = Pose(position=Point(0,0,0.25),orientation=Quaternion(0,0,0,1))
        userdata.trans_uav2object = Pose(position=Point(0,0,-1),orientation=Quaternion(0,0,0,1))
        userdata.interface = self.agentInterface
        self.tasks_dic['place_object'].userdata = userdata

        #request execute task
        self.agentInterface['pub_start_task'].publish(
                StartTask(agent_id=self.agent_id,task_id='place_object'))

        return SearchForObjectResponse(success=True)

    def PickAndPlaceObject_cb(self, req):
        #build state machine userdata from request
        userdata = smach.UserData()
        userdata.global_frame = 'map'
        userdata.uav_frame = 'uav_1'
        userdata.gripper_frame = 'gripper_link'
        userdata.type = 'brick'
        userdata.scale = Vector3(0.5,0.5,0.5)
        userdata.goal_pose = Pose(position=Point(0,0,0.25),orientation=Quaternion(0,0,0,1))
        userdata.obj_pose = Pose(position=Point(10,0,0.25),orientation=Quaternion(0,0,0,1))
        userdata.interface = self.agentInterface
        self.tasks_dic['pick_and_place_object'].userdata = userdata

        #request execute task
        self.agentInterface['pub_start_task'].publish(
                StartTask(agent_id=self.agent_id,task_id='pick_and_place_object'))

        return SearchForObjectResponse(success=True)

    def PickFromPileAndPlaceObject_cb(self, req):
        #build state machine userdata from request
        userdata = smach.UserData()
        userdata.global_frame = 'map'
        userdata.uav_frame = 'uav_1'
        userdata.gripper_frame = 'gripper_link'
        userdata.type = 'brick'
        userdata.pile_centroid = Point(0,10,0)
        userdata.goal_pose = Pose(position=Point(0,0,0.25),orientation=Quaternion(0,0,0,1))
        userdata.interface = self.agentInterface
        self.tasks_dic['pick_from_pile_and_place_object'].userdata = userdata

        #request execute task
        self.agentInterface['pub_start_task'].publish(
                StartTask(agent_id=self.agent_id,task_id='pick_from_pile_and_place_object'))

        return SearchForObjectResponse(success=True)

    def BuildWall_cb(self, req):
        userdata = smach.UserData()
        header = Header(frame_id='map',stamp=rospy.Time.now())
        userdata.uav_frame = 'uav_1'
        userdata.gripper_frame = 'gripper_link'
        userdata.red_pile = PoseStamped(header=header,pose=Pose(position=Point(0.6,-9.8,0),orientation=Quaternion(0,0,0,1)))
        userdata.green_pile = PoseStamped(header=header,pose=Pose(position=Point(0.3,10.2,0),orientation=Quaternion(0,0,0,1)))
        userdata.blue_pile = PoseStamped(header=header,pose=Pose(position=Point(-9.4,0.2,0),orientation=Quaternion(0,0,0,1)))
        userdata.orange_pile = PoseStamped(header=header,pose=Pose(position=Point(11.8,0.2,0),orientation=Quaternion(0,0,0,1)))
        userdata.interface = self.agentInterface
        #wall map
        wall =  WallBluePrint()
        wall.wall_frame = PoseStamped(header=header,pose=Pose(position=Point(0,0,0),orientation=Quaternion(0,0,0,1)))
        wall.size_x = 6
        wall.size_y = 1
        wall.size_z = 3
        wall.blueprint = [1, 3, 0, 0, 0, 1, 2, 0, 1, 1, 2, 0, 4, 0, 0, 0, 0, 0]
        userdata.wall_map = wall

        self.tasks_dic['build_wall'].userdata = userdata

        #request execute task
        self.agentInterface['pub_start_task'].publish(
                StartTask(agent_id=self.agent_id,task_id='build_wall'))

        return BuildWallResponse(success=True)

    def __init__(self, agent_id):
        #agent default task and task wrappers
        default_task = HoveringTask()
        self.tasks_dic = {}
        self.tasks_dic['search_for_object'] = SearchForObjectTaskWrapper()
        self.tasks_dic['go_to_waypoint'] = GoToWaypointTaskWrapper()
        self.tasks_dic['pick_object'] = PickObjectTaskWrapper()
        self.tasks_dic['place_object'] = PlaceObjectTaskWrapper()
        self.tasks_dic['pick_and_place_object'] = PickAndPlaceObjectTaskWrapper()
        self.tasks_dic['pick_from_pile_and_place_object'] = PickFromPileAndPlaceObjectTaskWrapper()
        self.tasks_dic['build_wall'] = BuildWallTaskWrapper()
        #agent sm init
        self.agent_id = agent_id
        self.agent_sm = AgentStateMachine(default_task,self.tasks_dic)
        #Agent Interface to be passed to Tasks
        obj_pub = rospy.Publisher('detected_objects', ObjectDetectionList, queue_size=10)
        task_exec_pub = rospy.Publisher('start_task', StartTask, queue_size=10)
        self.agentInterface = {}
        self.agentInterface['pub_obj_det'] = obj_pub
        self.agentInterface['pub_start_task'] = task_exec_pub
        #Task execution request services
        #rospy.Service('uav_search_for_object', SearchForObject, self.SearchForObject_cb)
        rospy.Service('uav_go_to_waypoint', SearchForObject, self.GoToWaypoint_cb)
        rospy.Service('uav_pick_object', SearchForObject, self.PickObject_cb)
        rospy.Service('uav_place_object', SearchForObject, self.PlaceObject_cb)
        rospy.Service('uav_pick_and_place_object', SearchForObject, self.PickAndPlaceObject_cb)
        rospy.Service('uav_pick_from_pile_and_place_object', SearchForObject, self.PickFromPileAndPlaceObject_cb)
        rospy.Service('uav_build_wall', BuildWall, self.BuildWall_cb)

        #Execute agent state machine
        userdata = smach.UserData()
        userdata.height = 2.0
        userdata.interface = self.agentInterface
        self.agent_sm.execute(userdata)

def main():
    rospy.init_node('uav_agent')
    controller = UAVAgent('uav_1')
    rospy.spin()


if __name__ == '__main__':
    main()
