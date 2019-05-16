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

    def __init__(self, name, interface, uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['type','scale','goal_pose','trans_uav2object'],
                output_keys = ['trans_uav2object'])

        #members
        self.name = name
        self.height = height
        self.global_frame = global_frame
        self.uav_frame = uav_frame
        self.gripper_frame = gripper_frame
        self.z_offset = z_offset

        #interface elements
        interface.add_client('cli_magnetize',uav_ns+'/'+'magnetize',Magnetize)
        interface.add_client('cli_take_off',uav_ns+'/'+'take_off',TakeOff)
        interface.add_client('cli_go_waypoint',uav_ns+'/'+'go_to_waypoint',
                                GoToWaypoint)

        self.iface = interface

    #main function
    def execute(self, userdata):

        #compute uav pose before dropping
        trans_global2object = from_geom_msgs_Pose_to_KDL_Frame(userdata.goal_pose)
        trans_global2object.p += PyKDL.Vector(0,0,userdata.scale.z/2 + self.z_offset)
        trans_global2object.M = PyKDL.Rotation.Quaternion(0,0,0,1) #TODO: ignoring goal pose rotation

        trans_global2uav = trans_global2object * from_geom_msgs_Transform_to_KDL_Frame(userdata.trans_uav2object).Inverse()

        #send waypoint
        header = Header(frame_id=self.global_frame,stamp=rospy.Time.now())
        self.iface['cli_go_waypoint'](GoToWaypointRequest(waypoint=PoseStamped(
        header=header,pose=from_KDL_Frame_to_geom_msgs_Pose(trans_global2uav)),blocking=True ))
        rospy.sleep(2.) #wait for stabilization

        #drop the object
        self.iface['cli_magnetize'](MagnetizeRequest(magnetize=False ))

        try:
            trans_global2uav = lookup_tf_transform(self.global_frame, self.uav_frame, self.iface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print self.name + ' Task could not be executed'
            return 'error'

        self.iface['cli_take_off'](TakeOffRequest(height=self.height-trans_global2uav.transform.translation.z,blocking=True))

        return 'success'
