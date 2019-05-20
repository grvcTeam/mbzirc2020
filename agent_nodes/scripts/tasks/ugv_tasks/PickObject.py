import roslib
import rospy
import smach
import smach_ros
from math import floor, ceil
import numpy as np

from utils.geom import *
from utils.agent import *

# required message definitions
from mbzirc_comm_objs.msg import GripperAttached
from mbzirc_comm_objs.srv import Magnetize, MagnetizeRequest
from uav_abstraction_layer.srv import GoToWaypoint, GoToWaypointRequest, TakeOff, TakeOffRequest
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point
from nav_msgs import OccupancyGrid

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

    return [p_i,p_j]

def check_collision(occ_grid, aabb, threshold):
    ori = [occ_grid.info.origin.position.x,occ_grid.info.origin.position.y]

    #convert aabb coordinates to grid index
    min = coord2index(ori,occ_grid.info.width,occ_grid.info.height,occ_grid.info.resolution,[aabb[0],aabb[1]],True)
    max = coord2index(ori,occ_grid.info.width,occ_grid.info.height,occ_grid.info.resolution,[aabb[2],aabb[3]],False)

    #check there are no occupied cells in the range
    for i in range(min[0],max[0]+1):
        for j in range(min[1],max[1]+1):
            if occ_grid.data[i+j*ori,occ_grid.info.width] > threshold: #taking unknown as free
                return True

    return False


def get_safe_pose(occ_grid, aabb, range, threshold, x_step, y_step):

    ws_dim = [aabb[2]-aabb[0], aabb[3]-aabb[1]]

    for x in np.arange(range[0],range[2],x_step):
        for y in np.arange(range[1],range[3],y_step):
            if not check_collision(occ_grid, [x,y,x+ws_dim[0],x+ws_dim[1]], threshold):
                return x,y

    return None,None

# main class
class Task(smach.State):

    def attached_cb(self, msg):
        rospy.logdebug('Attached changed!!')
        self.gripper_attached = msg.attached

    #aabbs are supposed to be expressed in robot frame and  centered in the origin
    def __init__(self, name, interface, ugv_ns, global_frame, uav_frame, gripper_frame, z_offset, base_aabb, ws_aabb):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['type','scale','obj_pose'],
                output_keys = ['trans_uav2object'])

        #members
        self.gripper_attached = False
        self.name = name
        self.height = height
        self.global_frame = global_frame
        self.uav_frame = uav_frame
        self.gripper_frame = gripper_frame
        self.z_offset = z_offset

        #interface elements
        interface.add_client('cli_magnetize',ugv_ns+'/'+'magnetize',Magnetize)
        interface.add_client('cli_go_waypoint',ugv_ns+'/'+'go_to_waypoint',
                                GoToWaypoint)
        interface.add_client('cli_take_off',ugv_ns+'/'+'take_off',TakeOff)
        interface.add_publisher('pub_velocity',ugv_ns+'/'+'set_velocity',
                                TwistStamped, 1)
        interface.add_subscriber(self,ugv_ns+'/'+'attached', GripperAttached,
                                self.attached_cb)

        interface.add_subscriber(self,'/map', OccupancyGrid,
                                self.attached_cb)

        self.iface = interface

    #main function
    def execute(self, userdata):
        self.gripper_attached = False

        #TODO: get occ grip map

        #TODO: match requested object pose with object detection information

        #compute a waypoint from where to grip the object
        trans_global2object = from_geom_msgs_Pose_to_KDL_Frame(userdata.obj_pose)
        trans_global2gripper = from_geom_msgs_Pose_to_KDL_Frame(userdata.obj_pose)
        trans_global2gripper.p += PyKDL.Vector(0,0,userdata.scale.z/2 + self.z_offset)
        trans_global2gripper.M = PyKDL.Rotation.Quaternion(0,0,0,1) #TODO: a better solution would align the gripper with the XY projection of the object
        try:
            trans_gripper2uav = lookup_tf_transform(self.gripper_frame, self.uav_frame,self.iface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print self.name + ' Task could not be executed'
            return 'error'

        trans_global2uav = trans_global2gripper * from_geom_msgs_Transform_to_KDL_Frame(trans_gripper2uav.transform)

        rb_range = get_aabb_range(self.ws_aabb, self.rb_aabb, [trans_global2gripper.p.x(),trans_global2gripper.p.y()])
        min_point = get_safe_pose(occ_grid, self.rb_aabb, rb_range, 0.5, 0.1, 0.1) #TODO threshold, x_step, y_step params

        if min_point[0] == None:
            print self.name + ' Task could not be executed'
            return 'error'

        pose = Pose(0)
        pose.orientation = Quaternion(0,0,0,1)
        pose.position = Point(min_point[0]+(self.rb_aabb[2]-self.rb_aabb[0])/2,min_point[1]+(self.rb_aabb[3]-self.rb_aabb[1])/2)

        #send the way point
        



        '''way = GoToWaypointRequest(waypoint=PoseStamped(
        header=Header(frame_id=self.global_frame,stamp=rospy.Time.now()),pose=
        from_KDL_Frame_to_geom_msgs_Pose(trans_global2uav)),blocking=True )

        self.iface['cli_go_waypoint'](way)

        #active magnetic gripper
        self.iface['cli_magnetize'](MagnetizeRequest(magnetize=True ))

        #send velocity commands until the object is gripped
        vel_cmd = TwistStamped()
        vel_cmd.header.frame_id = self.global_frame
        vel_cmd.twist.linear.z = -0.1 #TODO: should be a parameter
        rate = rospy.Rate(10.0)
        trans_uav2global = None
        while not self.gripper_attached:
            vel_cmd.header.stamp = rospy.Time.now()
            self.iface['pub_velocity'].publish(vel_cmd)
            rate.sleep()

        rospy.logdebug('Attached!!')

        #compute object pose respect to itself for output_keys
        if not trans_uav2global:
            try:
                trans_uav2global = lookup_tf_transform(self.uav_frame, self.global_frame,self.iface['tf_buffer'],5)
            except Exception as error:
                print repr(error)
                print self.name + ' Task could not be executed'
                return 'error'

        trans_uav2object = from_geom_msgs_Transform_to_KDL_Frame(trans_uav2global.transform) * trans_global2object
        userdata.trans_uav2object = from_KDL_Frame_to_geom_msgs_Transform(trans_uav2object)

        try:
            trans_global2uav = lookup_tf_transform(self.global_frame, self.uav_frame, self.iface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print self.name + ' Task could not be executed'
            return 'error'

        self.iface['cli_take_off'](TakeOffRequest(height=self.height-trans_global2uav.transform.translation.z,blocking=True))'''

        return 'success'
