import roslib
import rospy
import smach
import smach_ros
from math import floor, ceil, pi
import numpy as np
import PyKDL

import shapely.geometry
import shapely.affinity

from utils.geom import *
from utils.agent import *

import tasks.ugv_tasks.GoToWaypoint as GoToWaypoint

# required message definitions
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3, Twist
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
    pose.position = Point(0.350020373191,-9.74999731461,0.0999974468085)
    userdata.shared_regions = []
    userdata.obj_pose = pose
    userdata.type = 'brick'
    userdata.scale = Vector3(0.3,0.2,0.2)
    return userdata

# transform a point p coodinates from cartesian to grid index
def coord2index(o,w,h,r,p,takeFloor):
    f = floor if takeFloor else ceil
    p_i = (p[0]-o[0])/r
    p_j = (p[1]-o[1])/r
    p_i = f(p_i)
    p_j = f(p_j)
    p_i = f(p_i)
    p_j = f(p_j)

    return [int(p_i),int(p_j)]

# check if occupied cells in a grid falls inside of an aabb
# aabb: expressed in global frame
def check_collision(occ_grid, aabb, threshold):
    ori = [occ_grid.info.origin.position.x,occ_grid.info.origin.position.y]

    #convert aabb coordinates to grid index
    a = [aabb[0],aabb[1]]
    min = coord2index(ori,occ_grid.info.width,occ_grid.info.height,occ_grid.info.resolution,a,True)
    max = coord2index(ori,occ_grid.info.width,occ_grid.info.height,occ_grid.info.resolution,[aabb[2],aabb[3]],False)

    #print aabb
    #print 'checking collision: ori {ori}, res {res}, w {w}, h {h}, min {min}, max {max}'.format(ori=ori,res=occ_grid.info.resolution,w=occ_grid.info.width,h=occ_grid.info.height,min=min,max=max)
    if min[0] < 0 or min[1] < 0 or max[0] >= occ_grid.info.width or max[1] >= occ_grid.info.height:
        return True

    #check there are no occupied cells in the range
    for i in range(min[0],max[0]+1):
        for j in range(min[1],max[1]+1):
            if occ_grid.data[i+j*occ_grid.info.width] > threshold: #taking unknown as free
                return True

    return False

# compute possible robot grip poses and aabbs
# rb2obj: goal translation vector rb2obj
# rb_aabb: robot base bb in rb frame
def compute_robot_poses(rb2obj, obj_pose, scale, rb_aabb):
    trans_global2obj = from_geom_msgs_Pose_to_KDL_Frame(obj_pose)
    scale_trans = trans_global2obj.M * PyKDL.Vector(scale.x,scale.y,scale.z)
    trans_global2obj.M = PyKDL.Rotation.Quaternion(0,0,0,1)

    rb_p = shapely.geometry.Polygon([(rb_aabb[0],rb_aabb[1]), (rb_aabb[2],rb_aabb[1]), (rb_aabb[2],rb_aabb[3]), (rb_aabb[0],rb_aabb[3])])

    def get_pose(theta):
        pos = PyKDL.Vector(rb2obj[0],rb2obj[1],0)
        rot = PyKDL.Rotation.RotZ(-theta * (pi/180))
        trans_obj2rb = PyKDL.Frame(rot,pos).Inverse()
        trans_global2rb = trans_global2obj*trans_obj2rb

        rb_p_t = transform_Shapely_Polygon_with_KDL_Frame(trans_global2rb,rb_p)
        rb_aabb_t = rb_p_t.bounds

        print theta
        print trans_global2rb
        print rb_aabb_t

        return (trans_global2rb, rb_aabb_t)

    if abs(scale_trans.y()) > abs(scale_trans.x()):
        angles = [0, 180, 90, 270]
    else:
        angles = [90, 270, 0, 180]

    poses = []
    for theta in angles:
        poses += [get_pose(theta)]

    return poses

#takes a rb and work space aabbs in rb frame and computes the range for rb aabb in global frame
#so goal_point is inside of ws aabb. Limited to rb orientation aligned with global axis
#aabb format = [xmin, ymin, xmax, ymax] expressed in robot base frame
def get_aabb_range(ws_aabb, rb_aabb, goal_point):
    trans_ws2rb = [rb_aabb[0]-ws_aabb[0],rb_aabb[1]-ws_aabb[1]]

    ws_dim = [ws_aabb[2]-ws_aabb[0], ws_aabb[3]-ws_aabb[1]]
    ws_range = [goal_point[0]-ws_dim[0],goal_point[1]-ws_dim[1],goal_point[0],goal_point[1]] #for the origin of the aabb in global frame
    rb_range = [ws_range[0]+trans_ws2rb[0],ws_range[1]+trans_ws2rb[1],ws_range[2]+trans_ws2rb[0],ws_range[3]+trans_ws2rb[1]]
    return rb_range

# takes an aabb range and check for a safe one
def get_safe_pose(occ_grid, aabb, range, threshold, x_step, y_step):

    ws_dim = [aabb[2]-aabb[0], aabb[3]-aabb[1]]

    '''print ws_dim
    print 'x range'
    print np.arange(range[0],range[2],x_step)
    print 'y range'
    print np.arange(range[1],range[3],y_step)'''

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
                input_keys = ['shared_regions','obj_pose', 'scale', 'type'],
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

        interface.add_publisher('pub_vel','/mobile_base_controller/cmd_vel',
                                Twist, 10)

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
        rb2obj = (0.748,0.108) # hardcoded for harcoded gripper pose
        poses = compute_robot_poses(rb2obj, userdata.obj_pose, userdata.scale, self.rb_aabb)
        r_pose = None

        for i in range(len(poses)):
            if not check_collision(occ_grid, poses[i][1], 0.5):
                r_pose = poses[i][0]
                break

        print userdata.obj_pose
        print r_pose

        if r_pose == None:
            print self.name + ' Task could not be executed: no valid pose found'
            return 'error'

        userdata.way_pose = from_KDL_Frame_to_geom_msgs_Pose(r_pose)
        self.call_task('go_task',userdata)

        #fine tune pose because of movebase goal tolerance
        def x_rb2obj():
            try:
                trans_global2ugv = lookup_tf_transform(self.global_frame, self.ugv_frame, self.iface['tf_buffer'],5)
                trans_ugv2goal = from_geom_msgs_Transform_to_KDL_Frame(trans_global2ugv.transform).Inverse() * r_pose
                print abs(trans_ugv2goal.p.x())
                return abs(trans_ugv2goal.p.x())
            except Exception as error:
                print repr(error)
                print self.name + ' Task could not be executed'
                return 'error'

        rate = rospy.Rate(10.0)
        print 'approaching!'
        while x_rb2obj() > 0.06:
            self.iface['pub_vel'].publish(Twist(linear=Vector3(0.1,0,0)))
            rate.sleep()


        return 'success'
