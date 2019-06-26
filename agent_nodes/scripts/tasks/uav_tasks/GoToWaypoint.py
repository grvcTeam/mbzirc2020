import roslib
import rospy
import smach
import smach_ros

from shapely.geometry import LineString, Point
from math import sqrt, pow, acos, sin, cos

from utils.geom import *
from utils.agent import *

# message definitions
from mbzirc_comm_objs.srv import RequestSharedRegion, RequestSharedRegionResponse, RequestSharedRegionRequest, GoToWaypoint as GoToWaypointTask, GoToWaypointResponse as GoToWaypointTaskResponse
from uav_abstraction_layer.srv import GoToWaypoint, GoToWaypointRequest
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point, Polygon, Point32
from std_msgs.msg import Header
from mbzirc_comm_objs.msg import RegionOwnerList

# task properties
ResponseType = GoToWaypointTaskResponse
DataType = GoToWaypointTask
transitions={'success':'success','error':'error'}

# function to create userdata from a task execution request matching the task
# input keys
def gen_userdata(req):

    userdata = smach.UserData()
    userdata.way_pose = req.way_pose
    userdata.shared_regions = Polygon()
    userdata.shared_regions.points = [Point32(-2,-2,0),Point32(2,-2,0),Point32(2,2,0),Point32(-2,2,0)]
    return userdata

# main class
class Task(smach.State):

    def granted_cb(self, msg):
        self.region_own = -1
        for i in range(len(msg.region_owners)):
            if msg.region_owners[i] == self.iface.agent_id:
                self.region_own = i

    # move agent from p1 to p2 withouth crossing regions
    def execute_safe_trajectory(self, p1,p2, final_orientation, regions):

        #compute path
        def segment_in_regions(p1,p2,regions):
            s = LineString([p1, p2])
            c = []
            for r_id in regions:
                if from_geom_msgs_Polygon_to_Shapely_Polygon(regions[r_id]).crosses(s):
                    c += [r_id]
            return c

        crosses = segment_in_regions(p1,p2, regions)
        paths = []
        for r_id in crosses:
            l = trajectory_around_region(p1,p2,regions[r_id])
            d = p1.distance(Point(l[0]))
            paths += [(d,l)]

        paths.sort(key = lambda e : e[0],reverse=False)
        p = [p[1] for p in paths]
        p_full = [p1.coords[0]]
        for l in p:
            p_full += l

        p_full += [p2.coords[0]]

        self.follow_trajectory(p_full, final_orientation)


    # sends a list of waypoints
    def follow_trajectory(self, waypoints, final_orientation):
        for i in range(len(waypoints)):
            pose = Pose()
            p = waypoints[i]
            #position:
            pose.position.x = p[0]
            pose.position.y = [1]
            pose.position.z = self.height
            #orientation
            if i == len(waypoints)-1:
                pose.orientation = final_orientation
            else:
                v = [waypoints[i+1][0]-p[0],waypoints[i+1][1]-p[1]]
                n = sqrt(pow(v[0],2)+pow(v[1],2))
                alpha = acos(v[0]/n)
                if v[1] < 0:
                    alpha = -alpha

                pose.orientation = Quaternion(0,0,sin(alpha/2),cos(alpha/2))

            self.iface['cli_go_waypoint'](GoToWaypointRequest(waypoint=
            PoseStamped(header=Header(frame_id=self.global_frame,stamp =
            rospy.Time.now()),pose=pose),blocking=True ))


    def cross_region(self, region_id, question,regions,p1):
        res = self.iface['cli_req_shared'](RequestSharedRegionRequest(region_id=region_id,question=question,agent_id=self.iface.agent_id))
        if res.answer == RequestSharedRegionResponse.WAIT:
            p2 = Point((res.waiting_point.point.x,res.waiting_point.point.y))
            self.execute_safe_trajectory(p1,p2, Quaternion(0,0,0,1), regions)

            rospy.loginfo("REACHED WAITING POINT")
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
            self.cross_region(r_id,RequestSharedRegionRequest.FREE_SHARED_REGION,userdata.shared_regions,uav_point)
            rospy.loginfo("Exit granted!")

        r_id = point_in_region(goal_point,userdata.shared_regions)
        if r_id >= 0:
            rospy.loginfo("Goal point is in shared region {r_id}, requesting access".format(r_id=r_id))
            self.cross_region(r_id,RequestSharedRegionRequest.RESERVE_SHARED_REGION,userdata.shared_regions,uav_point)
            rospy.loginfo("Access granted!")

        #rospy.loginfo('now, goint to waypoint!')
        try:
            trans_global2uav = lookup_tf_transform(self.global_frame, self.uav_frame, self.iface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print self.name + ' Task could not be executed'
            return 'error'

        uav_point = from_geom_msgs_Transform_to_Shapely_Point(trans_global2uav.transform)
        self.execute_safe_trajectory(uav_point, goal_point, userdata.way_pose.orientation, userdata.shared_regions)

        return 'success'
