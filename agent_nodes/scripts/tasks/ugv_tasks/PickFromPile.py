import roslib
import rospy
import smach
import smach_ros

import shapely.geometry #Point, Polygon
from math import acos, sin, cos
import PyKDL

from utils.geom import *
from utils.agent import *

import tasks.ugv_tasks.PickObject as PickObject
import tasks.ugv_tasks.GoToWaypoint as GoToWaypoint

# required message definitions
from mbzirc_comm_objs.msg import GripperAttached, ObjectDetectionList
from mbzirc_comm_objs.srv import Magnetize, MagnetizeRequest, DetectTypes, DetectTypesRequest
from uav_abstraction_layer.srv import TakeOff, TakeOffRequest #, GoToWaypoint, GoToWaypointRequest
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point, Vector3, Twist

from std_srvs.srv import SetBool, SetBoolResponse


# task properties
ResponseType = SetBoolResponse
DataType = SetBool
transitions={'success':'success','error':'error'}

# function to create userdata from a task execution request matching the task
# input keys
def gen_userdata(req):

    userdata = smach.UserData()
    userdata.shared_regions = []
    userdata.pile_centroid = Point(10,0,0)
    userdata.type = 'brick'
    return userdata

# main class
class Task(smach.State):

    # functions
    def compute_detection_pose(self, pile_centroid, ugv_pos):
        d = 3 # distance to centroid, should be a parameter depending on AOV and pile BB

        c = shapely.geometry.Point(pile_centroid.x,pile_centroid.y)
        r = shapely.geometry.Point(ugv_pos.x,ugv_pos.y)
        circle = c.buffer(d)
        l = shapely.geometry.LineString([r,c])
        int = circle.intersection(l)

        #assuming camera pointing positive Y direction in robot frame base
        y_vec = PyKDL.Vector(int.coords[1][0]-int.coords[0][0],int.coords[1][1]-int.coords[0][1],0)
        y_vec.Normalize()
        #print y_vec
        alpha = acos(y_vec[1])
        if -y_vec[0] < 0:
            alpha = -alpha

        return Pose(position=Point(int.coords[0][0],int.coords[0][1],0),orientation=(Quaternion(0,0,sin(alpha/2),cos(alpha/2))))

    def select_object(self, type, list, ugv_pos):
        if  not list:
            rospy.logwarn('Cannot select object from empty list')
            return None

        try:
            trans_global2camera = lookup_tf_transform(self.global_frame, list[0].header.frame_id,self.iface['tf_buffer'],5)
            trans_global2camera = from_geom_msgs_Transform_to_KDL_Frame(trans_global2camera.transform)
        except Exception as error:
            print repr(error)
            print 'Cannot select object because transform global --> camera is not available'
            return None

        d_list = []
        ugv_v = PyKDL.Vector(ugv_pos.x,ugv_pos.y,0)
        for object in list:
            if object.type == type:
                trans_global2object = trans_global2camera * from_geom_msgs_Pose_to_KDL_Frame(object.pose.pose)
                s = trans_global2object.M * PyKDL.Vector(object.scale.x,object.scale.y,object.scale.z)
                d = (ugv_v - trans_global2object.p).Norm()
                d_list += [(object,d,trans_global2object.p.z(),[-s.x(),s.y(),s.x(),s.y()])]

        d_list.sort(key = lambda x : x[1])

        def l_disjoint(l1,l2):
            return l1[1] < l2[0] or l1[0] > l2[1]

        def bb_int(bb1,bb2):
            return not l_disjoint((bb1[0],bb1[2]),(bb2[0],bb2[2])) and not l_disjoint((bb1[1],bb1[3]),(bb2[1],bb2[3]))

        def hasOn(e,list):
            for e2 in list:
                if bb_int(e[3],e2[3]) and e[2] < e2[2]:
                    return True
            return False

        for e in d_list:
            if not hasOn(e,d_list):
                return e[0]

        return None #never should reach here

    # callbacks
    def object_detection_cb(self, msg):
        if msg.objects:
            self.found = True
            self.objects = msg.objects

    # init
    def __init__(self, name, interface, ugv_ns, global_frame, ugv_frame, base_aabb, ws_aabb, gripper_frame, z_offset):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['shared_regions','type','pile_centroid'],
                output_keys = ['trans_gripper2object'],
                io_keys = ['scale','obj_pose','way_pose'])

        # members
        self.found = False
        self.objects = None
        self.name = name

        self.global_frame = global_frame
        self.ugv_frame = ugv_frame
        self.gripper_frame = gripper_frame
        self.z_offset = z_offset

        # interface elements
        interface.add_subscriber(self,'/'+'sensed_objects',ObjectDetectionList,
                                self.object_detection_cb)

        interface.add_publisher('pub_vel','/mobile_base_controller/cmd_vel',
                                Twist, 10)

        self.iface = interface

        # sub_tasks
        add_sub_task('pick_task', self, PickObject, task_args = [ugv_ns, global_frame, ugv_frame, base_aabb, ws_aabb, gripper_frame, z_offset])
        add_sub_task('go_task', self, GoToWaypoint, task_args = [ugv_ns, global_frame, ugv_frame])

    # main function
    def execute(self, userdata):

        #go to pose to detect bricks
        try:
            trans_global2ugv = lookup_tf_transform(self.global_frame, self.ugv_frame,self.iface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print 'Cannot select object because transform global --> ugv_frame is not available'
            return 'error'

        pose = self.compute_detection_pose(userdata.pile_centroid,trans_global2ugv.transform.translation)

        userdata.way_pose = pose
        self.call_task('go_task',userdata)

        #TODO: setup object detection to detect desired object type

        #raw_input('something to select object')

        #select object
        try:
            trans_global2ugv = lookup_tf_transform(self.global_frame, self.ugv_frame,self.iface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print 'Cannot select object because transform global --> ugv_frame is not available'
            return 'error'

        object = None
        self.found = False
        r = rospy.Rate(10)
        while not rospy.is_shutdown(): #TODO: if there is no object, something should happen...
            if self.found:
                object =  self.select_object(userdata.type,self.objects,trans_global2ugv.transform.translation)
                if(object):
                    break
                else:
                    self.found = False
            r.sleep()

        rospy.logdebug(object)

        #pick object
        userdata.scale = object.scale
        try:
            trans_global2camera = lookup_tf_transform(self.global_frame, object.header.frame_id,self.iface['tf_buffer'],5)
            trans_global2camera = from_geom_msgs_Transform_to_KDL_Frame(trans_global2camera.transform)
        except Exception as error:
            print repr(error)
            print 'Cannot select object because transform global --> camera is not available'
            return 'error'

        trans_global2object = trans_global2camera * from_geom_msgs_Pose_to_KDL_Frame(object.pose.pose)
        userdata.obj_pose = from_KDL_Frame_to_geom_msgs_Pose(trans_global2object)

        #print object.pose.pose
        #print userdata.obj_pose
        #raw_input('something to pick object')

        self.call_task('pick_task',userdata) #TODO: if object can't be picked choose another one

        # move away cause move_base gets stuck
        rate = rospy.Rate(10.0)
        ctr = 0
        while ctr < 20:
            ctr += 1
            self.iface['pub_vel'].publish(Twist(linear=Vector3(-0.5,0,0)))
            rate.sleep()

        rospy.sleep(1.) #wait for stabilization

        return 'success'
