import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

# message definitions
from mbzirc_comm_objs.srv import RequestSharedRegion, RequestSharedRegionResponse, RequestSharedRegionRequest
from uav_abstraction_layer.srv import GoToWaypoint, GoToWaypointRequest
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point
from std_msgs.msg import Header
from mbzirc_comm_objs.msg import RegionOwnerList

# main class
class Task(smach.State):

    def granted_cb(self, msg):
        for i in range(len(msg.region_owners)):
            if msg.region_owners[i] == self.iface.agent_id:
                self.region_own = i

    def cross_region(self, region_id, question):
        res = self.iface['cli_req_shared'](RequestSharedRegionRequest(region_id=region_id,question=question,agent_id=self.iface.agent_id))
        if res.answer == RequestSharedRegionResponse.WAIT:
            point = res.waiting_point.point
            point.z = self.height #TODO: here assuming res.waiting_point is expressed in the same frame as global_frame
            pose = Pose(position=point,orientation=Quaternion(0,0,0,1))
            self.iface['cli_go_waypoint'](GoToWaypointRequest(waypoint=PoseStamped(header=res.waiting_point.header,pose=pose),blocking=True ))
            r = rospy.Rate(10)
            while not self.region_own == region_id:
                r.sleep()

    def __init__(self, name, interface, uav_ns, height, global_frame, uav_frame):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['way_pose','shared_regions']) #TODO: way pose should be 2D since uav is just allowed to travel in Z=height plane

        #members
        self.name = name
        self.height = height
        self.global_frame = global_frame
        self.uav_frame = uav_frame
        self.region_own = -1

        #interface elements
        interface.add_subscriber(self,'/shared_region_owners',RegionOwnerList,self.granted_cb)
        interface.add_client('cli_req_shared','/request_shared_region',RequestSharedRegion)
        interface.add_client('cli_go_waypoint',uav_ns+'/'+'go_to_waypoint',GoToWaypoint)

        self.iface = interface

    #main function
    def execute(self, userdata):
        #Get UAV pose.
        try:
            trans_global2uav = lookup_tf_transform(self.global_frame, self.uav_frame, self.iface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print self.name + ' Task could not be executed'
            return 'error'

        #Test if the initial or goal pose are inside of a shared region.
        #TODO: not checking if the path intersects a region
        uav_point = from_geom_msgs_Transform_to_Shapely_Point(trans_global2uav.transform)
        goal_point = from_geom_msgs_Pose_to_Shapely_Point(userdata.way_pose)
        def point_in_region(point,regions):
            for r_id in regions:
                if from_geom_msgs_Polygon_to_Shapely_Polygon(regions[r_id]).contains(point):
                    return r_id
            return -1

        r_id = point_in_region(uav_point,userdata.shared_regions)
        if r_id >= 0:
            rospy.loginfo("Origin point is in shared region {r_id}, requesting exit".format(r_id=r_id))
            self.cross_region(r_id,RequestSharedRegionRequest.FREE_SHARED_REGION)
            rospy.loginfo("Exit granted!")

        r_id = point_in_region(goal_point,userdata.shared_regions)
        if r_id >= 0:
            rospy.loginfo("Goal point is in shared region {r_id}, requesting access".format(r_id=r_id))
            self.cross_region(r_id,RequestSharedRegionRequest.RESERVE_SHARED_REGION)
            rospy.loginfo("Access granted!")

        #rospy.loginfo('now, goint to waypoint!')
        pose = Pose()
        pose.orientation = userdata.way_pose.orientation
        pose.position.x = userdata.way_pose.position.x
        pose.position.y = userdata.way_pose.position.y
        pose.position.z = self.height
        self.iface['cli_go_waypoint'](GoToWaypointRequest(waypoint=
        PoseStamped(header=Header(frame_id=self.global_frame,stamp =
        rospy.Time.now()),pose=pose),blocking=True ))

        return 'success'
