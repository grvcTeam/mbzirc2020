import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

# required message definitions
from mbzirc_comm_objs.msg import ObjectDetectionList
from mbzirc_comm_objs.srv import DetectTypes, DetectTypesRequest, SearchForObject, SearchForObjectResponse
from uav_abstraction_layer.srv import GoToWaypoint, GoToWaypointRequest
from geometry_msgs.msg import Pose, Quaternion, Point

# task properties
ResponseType = SearchForObjectResponse
DataType = SearchForObject
transitions={'found':'success','not_found':'success'}

# function to create userdata from a task execution request matching the task
# input keys
def gen_userdata(req):
    userdata = smach.UserData()
    userdata.search_region = req.search_region.polygon
    userdata.object_types = req.object_types
    userdata.stop_after_find = req.stop_after_find
    return userdata

# main class. TODO: not tested yet
class Task(smach.State):

    #callbacks
    def object_detection_cb(self, msg):
        self.iface['pub_obj_det'].publish(msg)
        if msg.objects:
            self.found = True

    #init
    def __init__(self, name, interface, uav_ns, height, global_frame, uav_frame, aov): #aov = angle of view of the sensor
        smach.State.__init__(self,outcomes=['found','not_found'],
                input_keys = ['search_region','object_types','stop_after_find'])

        #members
        self.found = False
        self.name = name
        self.height = height
        self.global_frame = global_frame
        self.uav_frame = uav_frame
        self.aov = aov

        #interface elements
        interface.add_client('cli_set_obj_types','set_obj_types',DetectTypes)
        interface.add_client('cli_go_waypoint',name,uav_ns+'/'+'go_to_waypoint',
                                GoToWaypoint)
        interface.add_publisher('pub_obj_det',interface.agent_id+'/'+'detected_objects',
                                ObjectDetectionList, 10)
        interface.add_subscriber(self,'sensed_objects',ObjectDetectionList,
                                self.object_detection_cb)

        self.iface = interface

    #main function
    def execute(self, userdata):
        self.found = False

        #compute search path.
        pol = from_geom_msgs_Polygon_to_Shapely_Polygon(userdata.search_region.polygon) #TODO: assuming polygon expressed in global_frame
        try:
            trans_global2uav = lookup_tf_transform(self.global_frame, self.uav_frame, self.iface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print self.name + ' Task could not be executed'
            return 'error'

        pos = from_geom_msgs_Transform_to_Shapely_Point(trans_global2uav)
        path = compute_search_path(self.aov, self.height, pol, pos)

        #set up object detection
        self.iface['cli_set_obj_types'](DetectTypesRequest(types=userdata.object_types))

        #start searching
        for wp in search_path.path:
            #visit each waypoint and check if objects been found
            if self.found and userdata.stop_after_find:
                break

            #TODO: should use the go_to_waypoint task?
            pose = Pose(position=Point(wp[0],wp[1],userdata.height),
            orientation = Quaternion(0,0,0,1))
            self.iface['cli_go_waypoint'](GoToWaypointRequest(waypoint=
            PoseStamped(header=Header(frame_id=userdata.global_frame,stamp =
            rospy.Time.now()),pose=pose),blocking=True ))

        return 'found' if self.found else 'not_found'
