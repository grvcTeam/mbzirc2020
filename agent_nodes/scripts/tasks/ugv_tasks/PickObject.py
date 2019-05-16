import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

# required message definitions
from mbzirc_comm_objs.msg import GripperAttached
from mbzirc_comm_objs.srv import Magnetize, MagnetizeRequest
from uav_abstraction_layer.srv import GoToWaypoint, GoToWaypointRequest, TakeOff, TakeOffRequest
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point

# main class
class Task(smach.State):

    def attached_cb(self, msg):
        rospy.logdebug('Attached changed!!')
        self.gripper_attached = msg.attached

    def __init__(self, name, interface, uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset):
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
        interface.add_client('cli_magnetize',uav_ns+'/'+'magnetize',Magnetize)
        interface.add_client('cli_go_waypoint',uav_ns+'/'+'go_to_waypoint',
                                GoToWaypoint)
        interface.add_client('cli_take_off',uav_ns+'/'+'take_off',TakeOff)
        interface.add_publisher('pub_velocity',uav_ns+'/'+'set_velocity',
                                TwistStamped, 1)
        interface.add_subscriber(self,uav_ns+'/'+'attached', GripperAttached,
                                self.attached_cb)

        self.iface = interface

    #main function
    def execute(self, userdata):
        self.gripper_attached = False

        #TODO: match requested object pose with object detection information

        #compute a waypoint from where to approach the object
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

        #send the way point
        way = GoToWaypointRequest(waypoint=PoseStamped(
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

        self.iface['cli_take_off'](TakeOffRequest(height=self.height-trans_global2uav.transform.translation.z,blocking=True))

        return 'success'
