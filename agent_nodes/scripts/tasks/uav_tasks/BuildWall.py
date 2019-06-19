import roslib
import rospy
import smach
import smach_ros

import numpy as np
from shapely.geometry import Point
import PyKDL

from utils.geom import *
from utils.agent import *

import tasks.uav_tasks.PickFromPileAndPlace as PickFromPileAndPlace

# required message definitions
from mbzirc_comm_objs.srv import BuildWall,BuildWallResponse, AddSharedRegion, AddSharedRegionRequest
from mbzirc_comm_objs.msg import WallBluePrint
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point, Polygon, Point32

# task properties
ResponseType = BuildWallResponse
DataType = BuildWall
transitions={'success':'success','error':'error'}

# function to create userdata from a task execution request matching the task
# input keys
def gen_userdata(req):
    # --- harcoded for testing ---
    userdata = smach.UserData()
    header = Header(frame_id='map',stamp=rospy.Time.now())
    userdata.red_pile = PoseStamped(header=header,pose=Pose(position=Point(0.6,-9.8,0),orientation=Quaternion(0,0,0,1)))
    userdata.green_pile = PoseStamped(header=header,pose=Pose(position=Point(0.3,10.2,0),orientation=Quaternion(0,0,0,1)))
    userdata.blue_pile = PoseStamped(header=header,pose=Pose(position=Point(-9.4,0.2,0),orientation=Quaternion(0,0,0,1)))
    userdata.orange_pile = PoseStamped(header=header,pose=Pose(position=Point(11.8,0.2,0),orientation=Quaternion(0,0,0,1)))

    #wall map
    wall =  WallBluePrint()
    wall.wall_frame = PoseStamped(header=header,pose=Pose(position=Point(0,0,0),orientation=Quaternion(0,0,0,1)))
    wall.size_x = 6
    wall.size_y = 1
    wall.size_z = 2
    wall.blueprint = [1, 3, 0, 0, 0, 1, 2, 0, 1, 1, 2, 0]
    userdata.wall_map = wall
    userdata.shared_regions = {}

    return userdata

# main class
class Task(smach.State):

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


    # init
    def __init__(self, name, interface, uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['shared_regions','wall_map','red_pile','green_pile','blue_pile','orange_pile'])

        # members
        self.iface = interface
        self.name = name

        # sub_tasks
        add_sub_task('pfp_task', self, PickFromPileAndPlace, task_args = [uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset])

    # main function
    def execute(self, userdata):

        #build wall matrix from msg
        wall_matrix = self.from_msg_to_matrix(userdata.wall_map)
        trans_global2wall = from_geom_msgs_Pose_to_KDL_Frame(userdata.wall_map.wall_frame.pose)

        #call pick from pile Tasks in the right order
        ud = smach.UserData()
        ud.type = 'brick'
        ud.shared_regions = userdata.shared_regions

        for k in range(wall_matrix.shape[0]):
            for j in range(wall_matrix.shape[1]):
                for i in range(wall_matrix.shape[2]):
                    if wall_matrix[k,j,i] == 1: #red brick
                        trans_wall2brick = self.brick_goal_pose(0.30, 0.01,i,j,k)
                        ud.goal_pose = from_KDL_Frame_to_geom_msgs_Pose(trans_global2wall * trans_wall2brick)
                        ud.pile_centroid = userdata.red_pile.pose.position
                        self.call_task('pfp_task',ud)
                    elif wall_matrix[k,j,i] == 2: #green brick
                        trans_wall2brick = self.brick_goal_pose(0.60, 0.01,i,j,k)
                        ud.goal_pose = from_KDL_Frame_to_geom_msgs_Pose(trans_global2wall * trans_wall2brick)
                        ud.pile_centroid = userdata.green_pile.pose.position
                        self.call_task('pfp_task',ud)
                    elif wall_matrix[k,j,i] == 3: #blue brick
                        trans_wall2brick = self.brick_goal_pose(1.20, 0.01,i,j,k)
                        ud.goal_pose = from_KDL_Frame_to_geom_msgs_Pose(trans_global2wall * trans_wall2brick)
                        ud.pile_centroid = userdata.blue_pile.pose.position
                        self.call_task('pfp_task',ud)
                    elif wall_matrix[k,j,i] == 4: #orange brick
                        trans_wall2brick = self.brick_goal_pose(1.80, 0.01,i,j,k)
                        ud.goal_pose = from_KDL_Frame_to_geom_msgs_Pose(trans_global2wall * trans_wall2brick)
                        ud.pile_centroid = userdata.orange_pile.pose.position
                        self.call_task('pfp_task',ud)

        return 'success'
