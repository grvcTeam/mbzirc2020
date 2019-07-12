import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

import tasks.central_unit_tasks.SearchForBalloons as SearchForBalloons
import tasks.central_unit_tasks.CatchBalloons as CatchBalloons

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

    userdata.search_region = Polygon(points=[Point32(20,20,0),Point32(50,20,0),Point32(50,50,0),Point32(20,50,0)]) #req.search_region
    userdata.deployment_point = Point(44,7,0)
    userdata.balloons = []

    return userdata


# main class
class Task(smach.StateMachine):

    # init
    def __init__(self, name, interface):
        smach.StateMachine.__init__(self,outcomes=['success','error'],
                input_keys = ['search_region', 'balloons','deployment_point'])

        with self:
            smach.StateMachine.add('search_env', SearchForBalloons.Task('search_env',interface), {'success':'catch_balloons','failure':'catch_balloons','error':'error'})
            smach.StateMachine.add('catch_balloons', CatchBalloons.Task('build_wall',interface), {'success':'success','failure':'success','error':'error'})
