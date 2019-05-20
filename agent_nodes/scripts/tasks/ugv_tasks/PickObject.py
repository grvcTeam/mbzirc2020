import roslib
import rospy
import smach
import smach_ros
import actionlib
import moveit_commander
import sys
from moveit_commander.exception import MoveItCommanderException
import moveit_msgs

from utils.geom import *
from utils.agent import *

import tasks.ugv_tasks.GoToGripPose as GoToGripPose

# required message definitions
from mbzirc_comm_objs.msg import GripperAttached
from mbzirc_comm_objs.srv import Magnetize, MagnetizeRequest
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point

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
    userdata.obj_pose = pose
    userdata.type = 'brick'
    userdata.scale = None
    return userdata

# main class
class Task(smach.State):

    def attached_cb(self, msg):
        rospy.logdebug('Attached changed!!')
        self.gripper_attached = msg.attached

    #aabbs are supposed to be expressed in robot frame and  centered in the origin
    def __init__(self, name, interface, ugv_ns, global_frame, ugv_frame, base_aabb, ws_aabb, gripper_frame, z_offset):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['shared_regions','type','scale','obj_pose'],
                output_keys = ['trans_gripper2object'],
                io_keys = ['way_pose'])

        #members
        self.gripper_attached = False
        self.name = name

        self.global_frame = global_frame
        self.ugv_frame = ugv_frame
        self.gripper_frame = gripper_frame
        self.z_offset = z_offset

        #interface elements
        #interface.add_client('cli_magnetize',ugv_ns+'/'+'magnetize',Magnetize)
        #interface.add_subscriber(self,ugv_ns+'/'+'attached', GripperAttached,
        #                        self.attached_cb)

        self.iface = interface

        #moveit group is not supported by the interface, so adding it manually for now
        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander("manipulator") #TODO: param
        self.robot = moveit_commander.RobotCommander()

        #sub tasks
        add_sub_task('go_task', self, GoToGripPose, task_args = [ugv_ns, global_frame, ugv_frame, base_aabb, ws_aabb])


    #main function
    def execute(self, userdata):
        self.gripper_attached = False

        #TODO: match requested object pose with object detection information


        #move base to a pose where object can be reached
        self.call_task('go_task',userdata)

        #move gripper to close to object pose
        #print self.group.get_pose_reference_frame()

        self.group.set_goal_position_tolerance(0.01) #TODO: param

        pose_target = userdata.obj_pose
        pose_target.orientation = Quaternion(0.487505048212,0.485644673398,-0.511961062221,0.514182798172)
        pose_target.position.z = pose_target.position.z + self.z_offset

        header = Header(frame_id=self.global_frame,stamp=rospy.Time.now())
        self.group.set_pose_target(PoseStamped(header=header,pose=pose_target))

        plan = self.group.plan()
        self.group.execute(plan)

        #active magnetic gripper
        '''self.iface['cli_magnetize'](MagnetizeRequest(magnetize=True ))

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
                trans_uav2global = lookup_tf_transform(self.ugv_frame, self.global_frame,self.iface['tf_buffer'],5)
            except Exception as error:
                print repr(error)
                print self.name + ' Task could not be executed'
                return 'error'

        trans_gripper2object = from_geom_msgs_Transform_to_KDL_Frame(trans_uav2global.transform) * trans_global2object
        userdata.trans_gripper2object = from_KDL_Frame_to_geom_msgs_Transform(trans_gripper2object)

        try:
            trans_global2uav = lookup_tf_transform(self.global_frame, self.ugv_frame, self.iface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print self.name + ' Task could not be executed'
            return 'error'

        self.iface['cli_take_off'](TakeOffRequest(height=self.height-trans_global2uav.transform.translation.z,blocking=True))'''

        return 'success'
