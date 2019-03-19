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

class SearchForObjects(smach.State):

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

class GoToWaypoint(smach.State):

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
                input_keys = ['global_frame','uav_frame','goal_pose','shared_regions','interface'])

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
