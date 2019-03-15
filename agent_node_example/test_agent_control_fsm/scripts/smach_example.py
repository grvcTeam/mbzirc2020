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
from geometry_msgs.msg import PoseStamped

class LandedReadyToTakeOff(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['preempted','error'],
                input_keys = ['height'])

    def execute(self, userdata):
        self.recall_preempt()
        la_client = rospy.ServiceProxy('/uav_1/ual/land', Land)
        to_client = rospy.ServiceProxy('/uav_1/ual/take_off', TakeOff)
        #land UAVAgent
        la_client(LandRequest(blocking=True))
        #wait until state is preempted, ie. until a task execution
        #is requested
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                to_client(TakeOffRequest(height=userdata.height,blocking=True))
                return 'preempted'
            r.sleep()

        return 'error'

class SearchForObjectsTask(smach.State):

    #Task Cleanup before ending execution
    def on_exit(self,outcome):
        self.obj_det_sub.unregister()
        return outcome

    #Callback from agent topic
    def object_detection_cb(self, msg):
        if self.interface:
            self.interface['pub_obj_det'].publish(msg)
            if msg.objects:
                self.found = True

    def __init__(self):
        smach.State.__init__(self,outcomes=['found','not_found'],
                input_keys = ['header','search_region','object_types','stop_after_find','interface'])

    def execute(self, userdata):
        self.interface = userdata.interface

        #initialize state
        self.found = False

        #get search path
        sp_client = rospy.ServiceProxy('compute_region', SearchRegionPath)
        search_path = sp_client(SearchRegionPathRequest(header=userdata.header,region=userdata.search_region))

        #set up object detection
        od_client = rospy.ServiceProxy('detect_types', DetectTypes)
        od_client(userdata.object_types)

        #subscribe to object detection
        self.obj_det_sub = rospy.Subscriber('fake_objects', ObjectDetectionList, self.object_detection_cb)

        #start searching
        wp_client = rospy.ServiceProxy('/uav_1/ual/go_to_waypoint', GoToWaypoint)
        for waypoint in search_path.path:
            #visit each waypoint and check if objects been not_found
            if self.found and userdata.stop_after_find:
                break

            wp_client(GoToWaypointRequest(waypoint=PoseStamped(header=userdata.header,pose=waypoint),blocking=False ))

        return self.on_exit('found' if self.found else 'not_found')

# Task Wrapper. It has input parameters.
class SearchForObjectTaskWrapper(AgentTaskWrapper):

    def __init__(self):
        AgentTaskWrapper.__init__(self, SearchForObjectsTask(),
                transitions={'found':'success','not_found':'success'})

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

    def __init__(self, agent_id):
        #agent default task and task wrappers
        default_task = LandedReadyToTakeOff()
        self.tasks_dic = {}
        self.tasks_dic['search_for_object'] = SearchForObjectTaskWrapper()
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
