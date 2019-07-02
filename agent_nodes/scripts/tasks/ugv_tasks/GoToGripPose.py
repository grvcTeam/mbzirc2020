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

# check if occupied cells falls inside of an aabb
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

# compute possible robot grip poses given an object pose and the corresponding aabbs expressed in global frame
# rb2obj: vector between the requested robot base position and the object center
# rb_aabb: robot base bb in robot base frame
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

# main class
class Task(smach.State):

    def map_cb(self, msg):
        self.occ_grid = msg

    #aabbs are supposed to be expressed in ugv_frame
    #rb_aabb = robot base aabb = robot footprint
    def __init__(self, name, interface, ugv_ns):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['shared_regions','obj_pose', 'scale', 'type'],
                io_keys = ['way_pose'])

        self.iface = interface

        #properties. TODO: properties should be part of the Task module and checking if they are present in AgentInterface be done automatically for every task
        properties = ['global_frame', 'agent_frame', 'rb_aabb']
        for prop in properties:
            if prop not in interface.agent_props:
                raise AttributeError('{task} is missing required property {prop} and cannot '\
                'be instantiated.'.format(task=name,prop=prop))

        self.props = self.iface.agent_props

        #members
        self.name = name
        self.occ_grid = None

        #interface elements
        rospy.Subscriber('/map', OccupancyGrid,
                                self.map_cb)

        interface.add_publisher('pub_vel','/mobile_base_controller/cmd_vel',
                                Twist, 10)

        #sub tasks
        add_sub_task('go_task', self, GoToWaypoint, task_args = [ugv_ns])

    #main function
    def execute(self, userdata):

        #get occ grid map
        if not self.occ_grid:
            print self.name + ' Task could not be executed: no map available'
            return 'error'
        else:
            occ_grid = self.occ_grid

        #compute a waypoint from where to grip the object
        rb2obj = (0.748,0.108) # hardcoded for harcoded gripper pose in PickObject task
        poses = compute_robot_poses(rb2obj, userdata.obj_pose, userdata.scale, self.props['rb_aabb'])
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
                trans_global2ugv = lookup_tf_transform(self.props['global_frame'], self.props['agent_frame'], self.iface['tf_buffer'],5)
                trans_ugv2goal = from_geom_msgs_Transform_to_KDL_Frame(trans_global2ugv.transform).Inverse() * r_pose
                print abs(trans_ugv2goal.p.x())
                return trans_ugv2goal.p.x()
            except Exception as error:
                print repr(error)
                print self.name + ' Task could not be executed'
                return 'error'

        rate = rospy.Rate(10.0)
        print 'approaching!'
        while 1:
            d = x_rb2obj()
            if abs(d) > 0.06:
                v = 0.1 if d > 0 else -0.1
                self.iface['pub_vel'].publish(Twist(linear=Vector3(0.1,0,0)))
            else:
                break
            rate.sleep()


        return 'success'
