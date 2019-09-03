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
from math import pi

from utils.geom import *
from utils.agent import *

import tasks.ugv_tasks.GoToGripPose as GoToGripPose

# required message definitions
from mbzirc_comm_objs.msg import GripperAttached
from mbzirc_comm_objs.srv import Magnetize, MagnetizeRequest
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point, Vector3

# main class
class Task(smach.State):

    def go_to_joint_pose(self, pose):

        try:
          self.group.set_joint_value_target(pose)
        except MoveItCommanderException, e:
          print 'can not set goal pose'
          print e
        finally:
          self.group.go(wait=True)

    def arm_vertical(self, inc):

        try:
          self.group.shift_pose_target(2, inc)
        except MoveItCommanderException, e:
          print 'can not set goal pose'
          print e
        finally:
          self.group.go(wait=True)

    def attached_cb(self, msg):
        rospy.loginfo('Attached changed!!')
        self.gripper_attached = msg.attached

    #aabbs are supposed to be expressed in robot frame and  centered in the origin
    def __init__(self, name, interface, ugv_ns, z_offset):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['shared_regions','type','scale','obj_pose'],
                output_keys = ['trans_gripper2object'],
                io_keys = ['way_pose'])

        self.iface = interface

        #properties. TODO: properties should be part of the Task module and checking if they are present in AgentInterface be done automatically for every task
        properties = ['global_frame', 'agent_frame', 'gripper_frame', 'rb_aabb']
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
        interface.add_client('cli_magnetize','/'+'magnetize',Magnetize)
        interface.add_subscriber(self,'/'+'attached', GripperAttached,
                                self.attached_cb)

        #moveit group is not supported by the interface, so adding it manually for now
        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander("manipulator") #TODO: param
        self.robot = moveit_commander.RobotCommander()

        #sub tasks
        add_sub_task('go_task', self, GoToGripPose, task_args = [ugv_ns])


    #main function
    def execute(self, userdata):
        #TODO: match requested object pose with object detection information


        #move base to a pose where object can be reached
        self.call_task('go_task',userdata)

        #move gripper to close to object pose
        #go to harcoded joint pose
        grip_pose = [-pi/2, -0.008401700396952982, 0.5992523496947246, -2.2012104337559846, -1.598768962404419, -0.00019741267188067013]
        hold_pose = [-pi/2, -0.7091356220415337, 1.2990882056648774, -2.200745601415173, -1.5986056177795644, -0.0003149149102164017]

        self.go_to_joint_pose(grip_pose)

        #print self.group.get_pose_reference_frame()
        '''self.group.set_goal_position_tolerance(0.02) #TODO: param

        pose_target = userdata.obj_pose
        pose_target.orientation = Quaternion(0.487505048212,0.485644673398,-0.511961062221,0.514182798172) #gripper facing downwards
        pose_target.position.z = pose_target.position.z + userdata.scale.z/2 + self.z_offset

        header = Header(frame_id=self.props['global_frame'],stamp=rospy.Time.now())
        self.group.set_pose_target(PoseStamped(header=header,pose=pose_target))

        plan = self.group.plan()
        self.group.execute(plan)'''

        self.gripper_attached = False
        #active magnetic gripper
        self.iface['cli_magnetize'](MagnetizeRequest(magnetize=True ))

        #move gripper vertically until contact
        rate = rospy.Rate(1.0)
        while not self.gripper_attached:
            self.arm_vertical(-0.01)
            rate.sleep()

        #compute object pose respect to itself for output_keys
        try:
            trans_gripper2global = lookup_tf_transform(self.props['gripper_frame'], self.props['global_frame'],self.iface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print self.name + ' Task could not be executed'
            return 'error'

        #moves object to carry position
        '''self.arm_vertical(0.1)
        wpose = self.group.get_current_pose().pose
        wpose.position.z = wpose.position.z + 0.5

        (plan, fraction) = self.group.compute_cartesian_path(
                             [wpose],   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold

        self.group.execute(plan)'''

        self.go_to_joint_pose(hold_pose)

        rospy.logdebug('Attached!!')

        #compute object pose respect to itself for output_keys
        trans_global2object = from_geom_msgs_Pose_to_KDL_Frame(userdata.obj_pose)
        trans_gripper2object = from_geom_msgs_Transform_to_KDL_Frame(trans_gripper2global.transform) * trans_global2object
        userdata.trans_gripper2object = from_KDL_Frame_to_geom_msgs_Transform(trans_gripper2object)

        return 'success'