import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

import tasks.uav_tasks.PickObject as PickObject
import tasks.uav_tasks.GoToWaypoint as GoToWaypoint

# required message definitions
from mbzirc_comm_objs.msg import GripperAttached, ObjectDetectionList
from mbzirc_comm_objs.srv import Magnetize, MagnetizeRequest, DetectTypes, DetectTypesRequest
from uav_abstraction_layer.srv import TakeOff, TakeOffRequest #, GoToWaypoint, GoToWaypointRequest
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point

# main class
class Task(smach.State):

    # functions
    def select_object_z_max(self, type, list):
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

        z_max = (None,0.)
        for object in list:
            trans_global2object = trans_global2camera * from_geom_msgs_Pose_to_KDL_Frame(object.pose.pose)
            if object.type == type and trans_global2object.p.z() >= z_max[1]:
                z_max = (object, trans_global2object.p.z())

        return z_max[0]

    # callbacks
    def object_detection_cb(self, msg):
        if msg.objects:
            self.found = True
            self.objects = msg.objects

    # init
    def __init__(self, name, interface, uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['shared_regions','type','pile_centroid'],
                output_keys = ['trans_uav2object'],
                io_keys = ['scale','obj_pose','way_pose'])

        # members
        self.name = name
        self.height = height
        self.global_frame = global_frame
        self.found = False
        self.objects = None

        # interface elements
        #interface.add_client('cli_go_waypoint',uav_ns+'/'+'go_to_waypoint', GoToWaypoint)
        interface.add_subscriber(self,'sensed_objects',ObjectDetectionList,
                                self.object_detection_cb)

        self.iface = interface

        # sub_tasks
        add_sub_task('pick_task', self, PickObject, task_args = [uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset])
        add_sub_task('go_task', self, GoToWaypoint, task_args = [uav_ns, height, global_frame, uav_frame])

    # main function
    def execute(self, userdata):

        #go over pile ceontroid. TODO: 0.5 is because the camera pose...
        header = Header(frame_id=self.global_frame,stamp=rospy.Time.now())
        pose = Pose(position=Point(userdata.pile_centroid.x-0.5,userdata.pile_centroid.y,self.height),
        orientation = Quaternion(0,0,0,1)) #TODO: -0.5 is because of the transformation UAV --> camera_sensor, do this properly
        #self.iface['cli_go_waypoint'](GoToWaypointRequest(waypoint=PoseStamped(header=header,pose=pose),blocking=True ))
        userdata.way_pose = pose
        self.call_task('go_task',userdata)
        rospy.sleep(2.)

        #TODO: setup object detection to detect desired object type

        #select object
        object = None
        self.found = False
        r = rospy.Rate(10)
        while not rospy.is_shutdown(): #TODO: if there is no object, something should happen...
            if self.found:
                object =  self.select_object_z_max(userdata.type,self.objects)
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

        self.call_task('pick_task',userdata)

        return 'success'
