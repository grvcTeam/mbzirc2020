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


from std_msgs.msg import String

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import brics_actuator.msg
import mcr_arm_motions_ros.planned_motion_utils as utils




# required message definitions
from mbzirc_comm_objs.msg import GripperAttached
from mbzirc_comm_objs.srv import Magnetize, MagnetizeRequest
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point, Vector3

# main class
class Task(smach.State):


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
        self.z_offset = 0

        #interface elements
        interface.add_client('cli_magnetize','/'+'magnetize',Magnetize)
        interface.add_subscriber(self,'/'+'attached', GripperAttached,
                                self.attached_cb)

        self.pubURscript = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
        self.pubBrickPose = rospy.Publisher('/color2detect', String, queue_size=1)
        self.pub_start_pregrasp = rospy.Publisher('/mir_manipulation/pregrasp_planner_pipeline/event_in', String, queue_size=1)


        rospy.Subscriber("/mir_manipulation/pregrasp_planner_pipeline/configuration_out", 
            brics_actuator.msg.JointPositions, self.config_out_cb)

        #moveit group is not supported by the interface, so adding it manually for now
        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander("manipulator") #TODO: param
        self.robot = moveit_commander.RobotCommander()


        self.mb_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.mb_client.wait_for_server()

        #sub tasks
        #add_sub_task('go_task', self, GoToGripPose, task_args = [ugv_ns])

    def config_out_cb(self, msg):

        self.arm_configuration=msg


    def go_to_joint_pose(self, arm_configuration):

        try:
          self.group.set_joint_value_target(arm_configuration)
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

    #main function
    def execute(self, userdata):
        #TODO: match requested object pose with object detection information

        #move base to a pose where object can be reached
        #self.call_task('go_task',userdata)

        goal = MoveBaseGoal()
        goal.target_pose.header = Header(frame_id='base_link',stamp=rospy.Time.now())


        pose=Pose()

        pose.position.x = 3
        pose.position.y = 0
        pose.position.z = 0
        pose.orientation.x =0
        pose.orientation.y =0
        pose.orientation.z =0 
        pose.orientation.w =1

        goal.target_pose.pose = pose

        rospy.loginfo('POSE: {p}'.format(p=pose))

        # self.mb_client.send_goal(goal)
        # wait = self.mb_client.wait_for_result()

        self.pubBrickPose.publish(data='red') # TODO:change this acording to the state machine. available inputs are "red", "orange", "blue", "green"
       # print "depois do go e antes to go to joint pose"

        # comecar o pregrasp sabendo que ele ja esta a receber para o seu /target_pose a pose do brick
        self.pub_start_pregrasp.publish('e_start')

        rospy.sleep(3)

        joint_list = utils.brics_joint_positions_to_list(self.arm_configuration)

        self.go_to_joint_pose(joint_list)

        rospy.sleep(2)


        #move gripper to close to object pose
        #go to harcoded joint pose
        #grip_pose = [-1.7730672995196741, -0.3878215688518067, 1.3928659788714808,-2.5784980068602503, -1.545375706349508, -1.9485176245318812]
        hold_pose = [-1.9231649399288942, -1.7708555307344, 2.020241543371168, -1.792100532449426, -1.54328856236556, -2.102485936227004]
        grip_pose = [-1.7760866324054163,-0.3856819433024903, 1.4052107969867151,  -2.578330179254049, -1.5498421827899378, -1.9483378569232386]

        #self.go_to_joint_pose(grip_pose)

        #rate = rospy.Rate(1)
        #rate.sleep()
        #rate.sleep()
        rospy.sleep(2)



        self.gripper_attached = False

        #active magnetic gripper
        #self.iface['cli_magnetize'](MagnetizeRequest(magnetize=True ))

        rospy.sleep(1)



        #print self.group.get_pose_reference_frame()
        '''self.group.set_goal_position_tolerance(0.02) #TODO: param

        pose_target = userdata.obj_pose
        pose_target.orientation = Quaternion(0.487505048212,0.485644673398,-0.511961062221,0.514182798172) #gripper facing downwards
        pose_target.position.z = pose_target.position.z + userdata.scale.z/2 + self.z_offset

        header = Header(frame_id=self.props['global_frame'],stamp=rospy.Time.now())
        self.group.set_pose_target(PoseStamped(header=header,pose=pose_target))

        plan = self.group.plan()
        self.group.execute(plan)'''

    

        #move gripper vertically until contact
        
        while not self.gripper_attached:
            self.arm_vertical(-0.01)
            rospy.loginfo('down!!')
            #rate.sleep()

            rospy.sleep(1)

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

        self.arm_vertical(0.05)

        rospy.loginfo('Attached!!')

        rospy.sleep(1)

        #self.pubURscript.publish(data='set_payload(2.6)')   # for the 1kg brick
        rospy.sleep(1)
        #self.pubURscript.publish(data='set_payload(2.6)')   # for the 1kg brick


        rospy.sleep(1)

        self.go_to_joint_pose(hold_pose)

        rospy.logdebug('UP!!')

        rospy.sleep(4)

        #compute object pose respect to itself for output_keys
        # trans_global2object = from_geom_msgs_Pose_to_KDL_Frame(userdata.obj_pose)
        # trans_gripper2object = from_geom_msgs_Transform_to_KDL_Frame(trans_gripper2global.transform) * trans_global2object
        # userdata.trans_gripper2object = from_KDL_Frame_to_geom_msgs_Transform(trans_gripper2object)

        return 'success'
