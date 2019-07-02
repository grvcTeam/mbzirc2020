import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

import tasks.central_unit_tasks.SearchForBrickPiles as SearchForBrickPiles
import tasks.central_unit_tasks.BuildWall as BuildWall

# required message definitions
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon, Point32, PoseStamped
from mbzirc_comm_objs.srv import SearchAndBuild,SearchAndBuildResponse, AddSharedRegion, AddSharedRegionRequest
from mbzirc_comm_objs.msg import WallBluePrint

# task properties
ResponseType = SearchAndBuildResponse
DataType = SearchAndBuild
transitions={'success':'success','error':'error'}

# function to create userdata from a task execution request matching the task
# input keys
def gen_userdata(req):
    userdata = smach.UserData()

    header = Header(frame_id='map',stamp=rospy.Time.now())
    wall =  WallBluePrint()
    wall.wall_frame = PoseStamped(header=header,pose=Pose(position=Point(0,0,0),orientation=Quaternion(0,0,0,1)))
    wall.size_x = 6
    wall.size_y = 1
    wall.size_z = 2
    wall.blueprint = [1, 3, 0, 0, 0, 1, 2, 0, 1, 1, 2, 0]


    userdata.search_region = Polygon(points=[Point32(-11,-11,0),Point32(11,-11,0),Point32(11,11,0),Point32(-11,11,0)]) #req.search_region
    userdata.wall = wall #req.wall_map


    userdata.piles = {'red_pile': {'type':'brick_pile', 'color':'red', 'scale_x':0.3, 'frame_id':'map','centroid': None, 'aabb': None},
                    'green_pile':{'type':'brick_pile', 'color':'green','scale_x':0.6, 'frame_id':'map','centroid': None, 'aabb': None},
                    'blue_pile':{'type':'brick_pile', 'color':'blue','scale_x':1.2, 'frame_id':'map','centroid': None, 'aabb': None},
                    'orange_pile':{'type':'brick_pile', 'color':'orange','scale_x':1.8, 'frame_id':'map','centroid': None, 'aabb': None}}

    return userdata


# main class
class Task(smach.StateMachine):

    # init
    def __init__(self, name, interface):
        smach.StateMachine.__init__(self,outcomes=['success','error'],
                input_keys = ['search_region', 'wall', 'piles'])

        with self:
            smach.StateMachine.add('search_env', SearchForBrickPiles.Task('search_env',interface), {'success':'build_wall','failure':'build_wall','error':'error'})
            smach.StateMachine.add('build_wall', BuildWall.Task('build_wall',interface), {'success':'success','failure':'success','error':'error'})
