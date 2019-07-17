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


from std_msgs.msg import String


# required message definitions
from mbzirc_comm_objs.msg import GripperAttached
from mbzirc_comm_objs.srv import Magnetize, MagnetizeRequest
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point, Vector3, Twist

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
    def __init__(self, name, interface, ugv_ns, z_offset):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['shared_regions','type','scale','goal_pose','trans_gripper2object'],
                output_keys = ['trans_gripper2object'],
                io_keys = ['way_pose', 'obj_pose'])

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


        self.pubURscript = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)

        

        #moveit group is not supported by the interface, so adding it manually for now
        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander("manipulator") #TODO: param
        self.robot = moveit_commander.RobotCommander()

        #sub tasks
        add_sub_task('go_task', self, GoToGripPose, task_args = [ugv_ns])


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
        '''self.group.set_goal_position_tolerance(0.04) #TODO: param
        pose_target = from_KDL_Frame_to_geom_msgs_Pose(trans_global2gripper)
        header = Header(frame_id=self.props['global_frame'],stamp=rospy.Time.now())
        self.group.set_pose_target(PoseStamped(header=header,pose=pose_target))

        plan = self.group.plan()
        self.group.execute(plan)'''

        grip_pose = [-1.7760866324054163,-0.3856819433024903, 1.4052107969867151,  -2.578330179254049, -1.5498421827899378, -1.9483378569232386]
        self.go_to_joint_pose(grip_pose)

        rospy.sleep(3) #NEEDS SLEEP TO WAIT FOR ARM TO FINISH MOVEMENT
        
        # drop the object
        self.iface['cli_magnetize'](MagnetizeRequest(magnetize=False ))
        rospy.sleep(2) #wait for stabilization

        self.pubURscript.publish(data='set_payload(1.6)')   # back to the installation payload


        # move away cause move_base gets stuck in gazebo
        '''rate = rospy.Rate(10.0)
        ctr = 0
        while ctr < 20:
            ctr += 1
            self.iface['pub_vel'].publish(Twist(linear=Vector3(-0.5,0,0)))
            rate.sleep()

        rospy.sleep(1.) #wait for stabilization'''

        return 'success'
