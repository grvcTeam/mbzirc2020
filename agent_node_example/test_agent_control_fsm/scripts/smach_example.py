#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

from agent_node import *
from mbzirc_comm_objs.msg import ObjectDetectionList
from mbzirc_comm_objs.srv import DetectTypes, DetectTypesRequest, SearchForObject, SearchForObjectResponse
from agent_node_example_comm_objects.srv import SearchRegionPath, SearchRegionPathRequest
from uav_abstraction_layer.srv import GoToWaypoint, GoToWaypointRequest, TakeOff, TakeOffRequest, Land, LandRequest
from geometry_msgs.msg import Pose, PoseStamped, Point, Point32, Quaternion, PolygonStamped

from agent_tasks import *

# Task Wrapper. It has input parameters.
class SearchForObjectTaskWrapper(AgentTaskWrapper):

    def __init__(self):
        AgentTaskWrapper.__init__(self, SearchForObjects(),
                transitions={'found':'success','not_found':'success'})

class GoToWaypointTaskWrapper(AgentTaskWrapper):

    def __init__(self):
        AgentTaskWrapper.__init__(self, GoToWaypoint(),
                transitions={'success':'success','error':'success'})

class UAVAgent():

    #task exec callbacks
    def SearchForObject_cb(self, req):
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

        return SearchForObjectResponse(success=True)

    def GoToWaypoint_cb(self, req):
        shared_region = PolygonStamped()
        shared_region.polygon.points = [Point32(-2,-2,0),Point32(2,-2,0),Point32(2,2,0),Point32(-2,2,0)]

        #build state machine userdata from request
        userdata = smach.UserData()
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

    def __init__(self, agent_id):
        #agent default task and task wrappers
        default_task = LandedReadyToTakeOff()
        self.tasks_dic = {}
        self.tasks_dic['search_for_object'] = SearchForObjectTaskWrapper()
        self.tasks_dic['go_to_waypoint'] = GoToWaypointTaskWrapper()
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
        rospy.Service('uav_search_for_object', SearchForObject, self.SearchForObject_cb)
        rospy.Service('uav_go_to_waypoint', SearchForObject, self.GoToWaypoint_cb)

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
