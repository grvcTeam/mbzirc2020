import roslib
import rospy
import smach
import smach_ros
from math import floor, ceil
import numpy as np

from utils.geom import *
from utils.agent import *

import tasks.ugv_tasks.GoToWaypoint as GoToWaypoint

# required message definitions
from geometry_msgs.msg import Pose, Quaternion, Point
from nav_msgs.msg import OccupancyGrid

from std_srvs.srv import SetBool, SetBoolResponse

# task properties
ResponseType = SetBoolResponse
DataType = SetBool
transitions={'success':'success','error':'error'}

# function to create userdata from a task execution request matching the task
# input keys
def gen_userdata(req):

    userdata = smach.UserData()
    pose = Pose()
    pose.orientation = Quaternion(0,0,0,1)
    pose.position = Point(0,9.9,0)
    userdata.shared_regions = []
    userdata.gripper_pose = pose
    return userdata

#aabb format = [xmin, ymin, xmax, ymax] expressed in robot base frame
def get_aabb_range(ws_aabb, rb_aabb, goal_point):
    trans_ws2rb = [rb_aabb[0]-ws_aabb[0],rb_aabb[1]-ws_aabb[1]]

    ws_dim = [ws_aabb[2]-ws_aabb[0], ws_aabb[3]-ws_aabb[1]]
    ws_range = [goal_point[0]-ws_dim[0],goal_point[1]-ws_dim[1],goal_point[0],goal_point[1]] #for the origin of the aabb in global frame
    rb_range = [ws_range[0]+trans_ws2rb[0],ws_range[1]+trans_ws2rb[1],ws_range[2]+trans_ws2rb[0],ws_range[3]+trans_ws2rb[1]]
    return rb_range

def coord2index(o,w,h,r,p,takeFloor):
    f = floor if takeFloor else ceil
    p_i = (p[0]-o[0])/r
    p_j = (p[1]-o[1])/r
    p_i = 0 if p_i < 0 else f(p_i)
    p_j = 0 if p_j < 0 else f(p_j)
    p_i = w-1 if p_i >= w else f(p_i)
    p_j = h-1 if p_j >= h else f(p_j)

    return [int(p_i),int(p_j)]

def check_collision(occ_grid, aabb, threshold):
    ori = [occ_grid.info.origin.position.x,occ_grid.info.origin.position.y]

    #convert aabb coordinates to grid index
    min = coord2index(ori,occ_grid.info.width,occ_grid.info.height,occ_grid.info.resolution,[aabb[0],aabb[1]],True)
    max = coord2index(ori,occ_grid.info.width,occ_grid.info.height,occ_grid.info.resolution,[aabb[2],aabb[3]],False)

    print aabb

    print 'checking collision: ori {ori}, res {res}, w {w}, h {h}, min {min}, max {max}'.format(ori=ori,res=occ_grid.info.resolution,w=occ_grid.info.width,h=occ_grid.info.height,min=min,max=max)

    #check there are no occupied cells in the range
    for i in range(min[0],max[0]+1):
        for j in range(min[1],max[1]+1):
            if occ_grid.data[i+j*occ_grid.info.width] > threshold: #taking unknown as free
                return True

    return False


def get_safe_pose(occ_grid, aabb, range, threshold, x_step, y_step):

    ws_dim = [aabb[2]-aabb[0], aabb[3]-aabb[1]]

    print ws_dim

    print 'x range'
    print np.arange(range[0],range[2],x_step)
    print 'y range'
    print np.arange(range[1],range[3],y_step)

    for x in np.arange(range[0],range[2],x_step): #TODO: take the last one
        for y in np.arange(range[1],range[3],y_step): #TODO: take the last one
            if not check_collision(occ_grid, [x,y,x+ws_dim[0],y+ws_dim[1]], threshold):
                return x,y

    return None,None

# main class
class Task(smach.State):

    def map_cb(self, msg):
        self.occ_grid = msg

    #aabbs are supposed to be expressed in robot frame and  centered in the origin
    def __init__(self, name, interface, ugv_ns, global_frame, ugv_frame, rb_aabb, ws_aabb):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['shared_regions','gripper_pose'],
                io_keys = ['way_pose'])

        #members
        self.name = name
        self.global_frame = global_frame
        self.ugv_frame = ugv_frame
        self.rb_aabb = rb_aabb
        self.ws_aabb  = ws_aabb
        self.occ_grid = None

        #interface elements
        rospy.Subscriber('/map', OccupancyGrid,
                                self.map_cb)

        self.iface = interface

        #sub tasks
        add_sub_task('go_task', self, GoToWaypoint, task_args = [ugv_ns, global_frame, ugv_frame])

    #main function
    def execute(self, userdata):

        #get occ grip map
        if not self.occ_grid:
            print self.name + ' Task could not be executed: no map available'
            return 'error'
        else:
            occ_grid = self.occ_grid

        #compute a waypoint from where to grip the object
        g_point = [userdata.gripper_pose.position.x,userdata.gripper_pose.position.y]
        rb_range = get_aabb_range(self.ws_aabb, self.rb_aabb, g_point)
        min_point = get_safe_pose(occ_grid, self.rb_aabb, rb_range, 0.5, 0.1, 0.1) #TODO threshold, x_step, y_step params

        if min_point[0] == None:
            print self.name + ' Task could not be executed: no valid pose found'
            return 'error'

        pose = Pose()
        pose.orientation = Quaternion(0,0,0,1)
        pose.position = Point(min_point[0]+(self.rb_aabb[2]-self.rb_aabb[0])/2,min_point[1]+(self.rb_aabb[3]-self.rb_aabb[1])/2,0)

        userdata.way_pose = pose
        self.call_task('go_task',userdata)

        return 'success'
