#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import numpy as np
import PyKDL #Vector, Rotation, Quaternion, Frame

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point, Polygon, Point32
from std_srvs.srv import Empty, EmptyResponse
from mbzirc_comm_objs.srv import PFPNPlace,PFPNPlaceResponse, AddSharedRegion, AddSharedRegionRequest, PFPNPlace, PFPNPlaceRequest, AgentIdle, AgentIdleRequest
from mbzirc_comm_objs.msg import WallBluePrint

def from_geom_msgs_Pose_to_KDL_Frame(transform):
    pos = PyKDL.Vector(transform.position.x,transform.position.y,transform.position.z)
    rot = PyKDL.Rotation.Quaternion(transform.orientation.x, transform.orientation.y, transform.orientation.z, transform.orientation.w)
    return PyKDL.Frame(rot,pos)

def from_KDL_Frame_to_geom_msgs_Pose(transform):
    pos = Point(transform.p.x(),transform.p.y(),transform.p.z())
    r_quat = transform.M.GetQuaternion()
    rot = Quaternion(r_quat[0], r_quat[1], r_quat[2], r_quat[3])
    return Pose(pos,rot)

class node():

    def from_msg_to_matrix(self,msg):
        n_x = msg.size_x
        n_y = msg.size_y
        n_z = msg.size_z

        if len(msg.blueprint) != n_x * n_y * n_z:
            rospy.logerr('wall matrix malformed')
            return np.array()

        mm = []
        for k in  range(n_z):
            m = []
            for j in range(n_y):
                m += [msg.blueprint[(k*n_x*n_y+j*n_x):(k*n_x*n_y+(j+1)*n_x)]]
            mm += [m]

        return np.array(mm)

    def brick_goal_pose(self, length, buffer, i, j, k):
        n_cells = length/0.30
        x = i * (0.30+buffer) + (length + buffer*n_cells) / 2
        y = j * (0.20+buffer) + (0.20+buffer) / 2
        z = k * (0.20+buffer) + (0.20+buffer) / 2

        return PyKDL.Frame(PyKDL.Rotation.Quaternion(0,0,0,1),PyKDL.Vector(x,y,z))

    def send_task(self,req):
        #keep checking until an agent is idle
        r = rospy.Rate(2)
        while 1:
            if self.isidle_1().isIdle:
                print 'iddle!'
                self.pfpnp_1(req)
                break
            elif self.isidle_2().isIdle:
                self.pfpnp_2(req)
                return
            r.sleep()

    def build_wall_cb(self,req):

        #build wall matrix from msg
        wall_matrix = self.from_msg_to_matrix(self.wall_map)
        trans_global2wall = from_geom_msgs_Pose_to_KDL_Frame(self.wall_map.wall_frame.pose)

        #print wall_matrix

        #call pick from pile Tasks in the right order

        for k in range(wall_matrix.shape[0]):
            for j in range(wall_matrix.shape[1]):
                for i in range(wall_matrix.shape[2]):
                    if wall_matrix[k,j,i] == 1: #red brick
                        print '1'
                        trans_wall2brick = self.brick_goal_pose(0.30, 0.01,i,j,k)
                        self.pfpnp_req.goal_pose = from_KDL_Frame_to_geom_msgs_Pose(trans_global2wall * trans_wall2brick)
                        self.pfpnp_req.pile_centroid = self.red_pile.pose.position
                        self.send_task(self.pfpnp_req)
                    elif wall_matrix[k,j,i] == 2: #green brick
                        print '2'
                        trans_wall2brick = self.brick_goal_pose(0.60, 0.01,i,j,k)
                        self.pfpnp_req.goal_pose = from_KDL_Frame_to_geom_msgs_Pose(trans_global2wall * trans_wall2brick)
                        self.pfpnp_req.pile_centroid = self.green_pile.pose.position
                        self.send_task(self.pfpnp_req)
                    elif wall_matrix[k,j,i] == 3: #blue brick
                        print '3'
                        trans_wall2brick = self.brick_goal_pose(1.20, 0.01,i,j,k)
                        self.pfpnp_req.goal_pose = from_KDL_Frame_to_geom_msgs_Pose(trans_global2wall * trans_wall2brick)
                        self.pfpnp_req.pile_centroid = self.blue_pile.pose.position
                        self.send_task(self.pfpnp_req)
                    elif wall_matrix[k,j,i] == 4: #orange brick
                        print '4'
                        trans_wall2brick = self.brick_goal_pose(1.80, 0.01,i,j,k)
                        self.pfpnp_req.goal_pose = from_KDL_Frame_to_geom_msgs_Pose(trans_global2wall * trans_wall2brick)
                        self.pfpnp_req.pile_centroid = self.orange_pile.pose.position
                        self.send_task(self.pfpnp_req)

        return EmptyResponse()

    def __init__(self):

        #clients
        add_reg = rospy.ServiceProxy('/add_shared_region', AddSharedRegion)
        self.pfpnp_1 = rospy.ServiceProxy('/agent_1/uav_1/pfpnp_task', PFPNPlace)
        self.pfpnp_2 = rospy.ServiceProxy('/agent_2/uav_2/pfpnp_task', PFPNPlace)
        self.isidle_1 = rospy.ServiceProxy('/agent_1/uav_1/is_idle', AgentIdle)
        self.isidle_2 = rospy.ServiceProxy('/agent_2/uav_2/is_idle', AgentIdle)


        #server
        rospy.Service('build_wall', Empty, self.build_wall_cb)

        #map information
        header = Header(frame_id='map',stamp=rospy.Time.now())
        self.red_pile = PoseStamped(header=header,pose=Pose(position=Point(0.6,-9.8,0),orientation=Quaternion(0,0,0,1)))
        self.green_pile = PoseStamped(header=header,pose=Pose(position=Point(0.3,10.2,0),orientation=Quaternion(0,0,0,1)))
        self.blue_pile = PoseStamped(header=header,pose=Pose(position=Point(-9.4,0.2,0),orientation=Quaternion(0,0,0,1)))
        self.orange_pile = PoseStamped(header=header,pose=Pose(position=Point(11.8,0.2,0),orientation=Quaternion(0,0,0,1)))

        wall =  WallBluePrint()
        wall.wall_frame = PoseStamped(header=header,pose=Pose(position=Point(0,0,0),orientation=Quaternion(0,0,0,1)))
        wall.size_x = 6
        wall.size_y = 1
        wall.size_z = 2
        wall.blueprint = [1, 3, 0, 0, 0, 1, 2, 0, 1, 1, 2, 0]
        self.wall_map = wall

        #setup shared regions
        shared_regions = {}
        waiting_points = {}

        #central
        p = Polygon()
        p.points = [Point32(-2,-2,0),Point32(2,-2,0),Point32(2,2,0),Point32(-2,2,0)]
        shared_regions[0] = p
        waiting_points[0] = [Point(-2,-2,0)]

        #red pile
        p = Polygon()
        p.points = [Point32(-1.4,-11.8,0),Point32(2.6,-11.8,0),Point32(2.6,-7.8,0),Point32(-1.4,-7.8,0)]
        shared_regions[1] = p
        waiting_points[1] = [Point(2.6,-7.8,0)]

        #green pile
        p = Polygon()
        p.points = [Point32(-1.7,8.2,0),Point32(2.3,8.2,0),Point32(2.3,12.2,0),Point32(-1.7,12.2,0)]
        shared_regions[2] = p
        waiting_points[2] = [Point(-1.7,8.2,0)]

        #blue pile
        p = Polygon()
        p.points = [Point32(-11.4,-1.8,0),Point32(-7.4,-1.8,0),Point32(-7.4,2.2,0),Point32(-11.4,2.2,0)]
        shared_regions[3] = p
        waiting_points[3] = [Point(-7.4,2.2,0)]

        #orange pile
        p = Polygon()
        p.points = [Point32(9.8,-1.8,0),Point32(13.8,-1.8,0),Point32(13.8,2.2,0),Point32(9.8,2.2,0)]
        shared_regions[4] = p
        waiting_points[4] = [Point(9.8,-1.8,0)]


        for id in shared_regions:
            req = AddSharedRegionRequest()
            req.frame_id = 'map'
            req.waiting_points = waiting_points[id]
            req.region = shared_regions[id]
            res = add_reg(req)

        shared_regions =shared_regions

        #pfpnp message
        self.pfpnp_req = PFPNPlaceRequest()
        self.pfpnp_req.type = 'brick'
        #self.pfpnp_req.goal_pose =
        #self.pfpnp_req.pile_centroid =
        self.pfpnp_req.shared_regions = [shared_regions[i] for i in range(5)]


def main():

    rospy.init_node('wall_scheduler')
    n = node()
    rospy.spin()

if __name__ == '__main__':
    main()
