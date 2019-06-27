import roslib
import rospy
import smach
import smach_ros

from utils.geom import *
from utils.agent import *

# required message definitions
from mbzirc_comm_objs.msg import GripperAttached
from mbzirc_comm_objs.srv import Magnetize, MagnetizeRequest
from uav_abstraction_layer.srv import GoToWaypoint, GoToWaypointRequest
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point

# main class
class Task(smach.State):

    def attached_cb(self, msg):
        rospy.logdebug('Attached changed!!')
        self.gripper_attached = msg.attached

    def __init__(self, name, interface, uav_ns, z_offset):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['type','scale','obj_pose'],
                output_keys = ['trans_uav2object'])

        self.iface = interface

        #properties. TODO: properties should be part of the Task module and checking if they are present in AgentInterface be done automatically for every task
        properties = ['height', 'global_frame', 'agent_frame', 'gripper_frame']
        for prop in properties:
            if prop not in interface.agent_props:
                raise AttributeError('{task} is missing required property {prop} and cannot '\
                'be instantiated.'.format(task=name,prop=prop))

        self.props = self.iface.agent_props

        #members
        self.gripper_attached = False
        self.name = name
        self.z_offset = z_offset

        #interface elements
        interface.add_client('cli_magnetize','magnetize',Magnetize)
        interface.add_client('cli_go_waypoint',uav_ns+'/'+'go_to_waypoint',
                                GoToWaypoint)
        interface.add_publisher('pub_velocity',uav_ns+'/'+'set_velocity',
                                TwistStamped, 1)
        interface.add_subscriber(self,'attached', GripperAttached,
                                self.attached_cb)

        self.iface = interface

    #main function
    def execute(self, userdata):
        self.gripper_attached = False

        #TODO: match requested object pose with object detection information

        #save uav pose for later
        try:
            trans_global2uav_old = lookup_tf_transform(self.props['global_frame'], self.props['agent_frame'],self.iface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print self.name + ' Task could not be executed'
            return 'error'

        #compute a waypoint from where to approach the object
        trans_global2object = from_geom_msgs_Pose_to_KDL_Frame(userdata.obj_pose)
        trans_global2gripper = from_geom_msgs_Pose_to_KDL_Frame(userdata.obj_pose)
        trans_global2gripper.p += PyKDL.Vector(0,0,userdata.scale.z/2 + self.z_offset)
        trans_global2gripper.M = PyKDL.Rotation.Quaternion(0,0,0,1) #TODO: a better solution would align the gripper with the XY projection of the object
        try:
            trans_gripper2uav = lookup_tf_transform(self.props['gripper_frame'], self.props['agent_frame'],self.iface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print self.name + ' Task could not be executed'
            return 'error'

        trans_global2uav = trans_global2gripper * from_geom_msgs_Transform_to_KDL_Frame(trans_gripper2uav.transform)

        #send the way point
        way = GoToWaypointRequest(waypoint=PoseStamped(
        header=Header(frame_id=self.props['global_frame'],stamp=rospy.Time.now()),pose=
        from_KDL_Frame_to_geom_msgs_Pose(trans_global2uav)),blocking=True )

        self.iface['cli_go_waypoint'](way)

        #active magnetic gripper
        self.iface['cli_magnetize'](MagnetizeRequest(magnetize=True ))

        #send velocity commands until the object is gripped
        trans_uav2global = None
        vel_cmd = TwistStamped()
        vel_cmd.header.frame_id = self.props['global_frame']
        vel_cmd.twist.linear.z = -0.1 #TODO: should be a parameter
        rate = rospy.Rate(10.0)
        while not self.gripper_attached:
            vel_cmd.header.stamp = rospy.Time.now()
            self.iface['pub_velocity'].publish(vel_cmd)
            rate.sleep()

        rospy.logdebug('Attached!!')

        #compute object pose respect to itself for output_keys
        if not trans_uav2global:
            try:
                trans_uav2global = lookup_tf_transform(self.props['agent_frame'], self.props['global_frame'],self.iface['tf_buffer'],5)
            except Exception as error:
                print repr(error)
                print self.name + ' Task could not be executed'
                return 'error'

        trans_uav2object = from_geom_msgs_Transform_to_KDL_Frame(trans_uav2global.transform) * trans_global2object
        userdata.trans_uav2object = from_KDL_Frame_to_geom_msgs_Transform(trans_uav2object)

        way = GoToWaypointRequest(waypoint=PoseStamped(
        header=Header(frame_id=self.props['global_frame'],stamp=rospy.Time.now()),pose=
        from_geom_msgs_Transform_to_geom_msgs_Pose(trans_global2uav_old.transform)),blocking=True )

        self.iface['cli_go_waypoint'](way)

        return 'success'
