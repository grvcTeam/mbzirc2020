#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

from mbzirc_comm_objs.msg import ObjectDetectionList
from mbzirc_comm_objs.srv import DetectTypes, DetectTypesRequest, SearchRegionPath, SearchRegionPathRequest, SearchForObject, SearchForObjectResponse
from uav_abstraction_layer.srv import GoToWaypoint, GoToWaypointRequest, TakeOff, TakeOffRequest
from geometry_msgs.msg import PoseStamped

# Coordinate and Parametrize Agent Functionalities (Components) to execute a task which can have a finite set of outcomes
# and has input and output parameters
class SearchForObjectsTask(smach.State):

    #Task Cleanup before ending
    def on_exit(self,outcome):
        self.interface = None
        self.obj_det_sub.unregister()
        return outcome

    #Callback from agent topic
    def object_detection_cb(self, msg):
        if self.interface:
            self.interface['pub_obj_det'].publish(msg)
            if msg.objects:
                self.found = True

    def __init__(self):
        input_keys_ = ['header','search_region','object_types','stop_after_find']
        smach.State.__init__(self,outcomes=['found','not_found'], input_keys=input_keys_+['interface'])
        self.interface = None

    def execute(self, userdata):
        #initialize state
        self.interface = userdata.interface
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
        #to_client = rospy.ServiceProxy('/uav_1/ual/take_off', TakeOff)
        #to_client(TakeOffRequest(height=2.0,blocking=True))
        for waypoint in search_path.path:
            #visit each waypoint and check if object been not_found
            if self.found and userdata.stop_after_find:
                break

            wp_client(GoToWaypointRequest(waypoint=PoseStamped(header=userdata.header,pose=waypoint),blocking=True))

        return self.on_exit('found' if self.found else 'not_found')

# Task graph. It has input parameters
class SearchForObjectTaskGraph(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['found','not_found'])

        #add states and transitions
        with self:
            smach.StateMachine.add('Search', SearchForObjectsTask(),
                                   transitions={'found':'found',
                                                'not_found':'not_found'})

class UAVAgent():

    #task graph callbacks
    def SearchForObject_cb(self, req):
        #build state machine userdata from request
        userdata = smach.UserData()
        userdata.header = req.header
        userdata.search_region = req.search_region
        userdata.object_types = req.object_types
        userdata.stop_after_find  = req.stop_after_find
        #agent interface
        userdata.interface = self.agentInterface
        userdata.something = 'cool'

        mission_fsm = SearchForObjectTaskGraph()
        mission_fsm.userdata = userdata
        print 'Entering state machine'
        mission_fsm.execute()

        return SearchForObjectResponse(success=True)

    def __init__(self):
        #Agent Interface to be passed to Tasks, excluding the task graph services
        obj_pub = rospy.Publisher('detected_objects', ObjectDetectionList, queue_size=10)
        self.agentInterface = {'pub_obj_det': obj_pub}
        #Task graph services
        rospy.Service('uav_search_for_object', SearchForObject, self.SearchForObject_cb)

def main():
    rospy.init_node('uav_agent')
    controller = UAVAgent()
    rospy.spin()


if __name__ == '__main__':
    main()
