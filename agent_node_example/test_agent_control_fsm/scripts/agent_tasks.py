import roslib
import rospy
import smach
import smach_ros
import tf2_ros

from shapely.geometry import Point
from geom_help import *

from agent_node import *
from mbzirc_comm_objs.msg import ObjectDetectionList
from mbzirc_comm_objs.srv import DetectTypes, DetectTypesRequest, SearchForObject, SearchForObjectResponse, RequestSharedRegion, RequestSharedRegionResponse
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

    #croos shared region
    def cross_region(self, region_id, question):
        #TODO: this probably has to be an action, so the process wait/go can be handled
        res = self.interface['cli_req_enter_shared'](RequestSharedRegion(region_id=region_id,question=question,agent_id=self.agent_id))
        if res.answer = RequestSharedRegion.WAIT:
            self.interface['cli_go_waypoint'](GoToWaypointRequest(waypoint=PoseStamped(header=res.waiting_point.header,pose=res.waiting_point.point),blocking=True ))

        #TODO: this is not complete, must wait until OK, but with the server client it is not possible
        return


    '''shared_regions is a dic 'region_id': polygon'''
    def __init__(self):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['global_frame','uav_frame','goal_pose','shared_regions','interface'])

    def execute(self, userdata):
        #add elements to Interface
        if not 'cli_req_shared' in userdata.interface:
            userdata.interface['cli_req_enter_shared'] = rospy.ServiceProxy('request_enter_shared_region', RequestSharedRegion)
        if not 'tf_buffer' in userdata.interface:
            userdata.interface['tf_buffer'] = tf2_ros.Buffer()
            userdata.interface['tf_listener'] = tf2_ros.TransformListener(tfBuffer)
        if not 'cli_go_waypoint' in userdata.interface:
            userdata.interface['cli_go_waypoint'] = rospy.ServiceProxy('/uav_1/ual/go_to_waypoint', GoToWaypoint)

        self.interface = userdata.interface

        #Get UAV pose
        try:
            trans = tfBuffer.lookup_transform(userdata.global_frame, userdata.uav_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print "transformation " + userdata.global_frame + ' --> ' + userdata.uav_frame \
            + ' could not be retrieved. GoToWaypoint Task could not be executed'
            return

        #Test if the initial or goal pose are inside of a shared region
        uav_point = from_geom_msgs_Transform_to_Shapely_Point(trans.transform)
        goal_point = from_geom_msgs_Pose_to_Shapely_Point(userdata.goal_pose)
        def point_in_region(point,regions):
            for r_id in regions:
                if regions[r_id].contains(point)
                    return r_id
            return ''

        r_id = point_in_region(uav_point,userdata.shared_regions)
        if r_id:
            print "Origin point is in shared region " + r_id + ", requesting exit"
            cross_region(r_id,RequestSharedRegion.FREE_SHARED_REGION)
            print "Exit granted!"

        r_id = point_in_region(goal_point,userdata.shared_regions)
        if r_id:
            print "Goal point is in shared region " + r_id + ", requesting access"
            cross_region(r_id,RequestSharedRegion.FREE_SHARED_REGION)
            print "Access granted!"

        userdata.interface['cli_go_waypoint'](GoToWaypointRequest(waypoint=PoseStamped(header=userdata.header,pose=waypoint),blocking=True ))

        return 'sucess'

class PickObject(smach.State):

    ''''''
    def __init__(self):
        smach.State.__init__(self,outcomes=['sucess','error'],
                input_keys = ['type','scale','pose_stamped'])

    def execute(self, userdata):
        #compute a waypoint from where to approach the object

        #send the way point

        #active magnetic gripper

        #probably should get fresh information on the object pose from object detection

        #send velocity commands until the object is gripped

        #compute pose respect to itself for output_keys

        return 'sucess'
