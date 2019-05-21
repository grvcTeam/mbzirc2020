import roslib
import rospy
import smach
import smach_ros
import actionlib
import moveit_commander
import sys
from moveit_commander.exception import MoveItCommanderException
import moveit_msgs
import copy

from utils.geom import *
from utils.agent import *

import tasks.ugv_tasks.GoToGripPose as GoToGripPose

# required message definitions
from mbzirc_comm_objs.msg import GripperAttached
from mbzirc_comm_objs.srv import Magnetize, MagnetizeRequest
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point, Vector3

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
    pose.position = Point(0,0,0)
    userdata.shared_regions = []
    userdata.goal_pose = pose
    userdata.type = 'brick'
    userdata.scale = Vector3(0.2,0.2,0.2)
    userdata.trans_gripper2object = pose
    return userdata

# main class
class Task(smach.State):

    def arm_vertical(self, inc):

        vals = self.group.get_current_joint_values()
        vals[1] = vals[1] + inc
        try:
          self.group.set_joint_value_target(vals)
        except MoveItCommanderException, e:
          print 'can not set goal pose'
          print e
        finally:
          self.group.go(wait=True)

    def attached_cb(self, msg):
        rospy.logdebug('Attached changed!!')
        self.gripper_attached = msg.attached

    #aabbs are supposed to be expressed in robot frame and  centered in the origin
    def __init__(self, name, interface, ugv_ns, global_frame, ugv_frame, base_aabb, ws_aabb, gripper_frame, z_offset):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['shared_regions','type','scale','goal_pose','trans_gripper2object'],
                output_keys = ['trans_gripper2object'],
                io_keys = ['way_pose', 'obj_pose'])

        #members
        self.gripper_attached = False
        self.name = name

        self.global_frame = global_frame
        self.ugv_frame = ugv_frame
        self.gripper_frame = gripper_frame
        self.z_offset = z_offset

        #interface elements
        interface.add_client('cli_magnetize','/'+'magnetize',Magnetize)
        interface.add_subscriber(self,'/'+'attached', GripperAttached,
                                self.attached_cb)

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
        userdata.obj_pose = userdata.goal_pose
        self.call_task('go_task',userdata)

        #compute gripper pose before dropping
        trans_global2object = from_geom_msgs_Pose_to_KDL_Frame(userdata.goal_pose)
        trans_global2object.p += PyKDL.Vector(0,0,userdata.scale.z/2 + self.z_offset)
        trans_global2gripper = trans_global2object * from_geom_msgs_Transform_to_KDL_Frame(userdata.trans_gripper2object).Inverse()

        #move gripper to drop pose
        #print self.group.get_pose_reference_frame()
        self.group.set_goal_position_tolerance(0.02) #TODO: param
        pose_target = from_KDL_Frame_to_geom_msgs_Pose(trans_global2gripper)
        header = Header(frame_id=self.global_frame,stamp=rospy.Time.now())
        self.group.set_pose_target(PoseStamped(header=header,pose=pose_target))

        plan = self.group.plan()
        self.group.execute(plan)

        # drop the object
        self.iface['cli_magnetize'](MagnetizeRequest(magnetize=False ))

        return 'success'
