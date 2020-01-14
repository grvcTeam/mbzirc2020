import roslib
import rospy
import actionlib
import moveit_commander
import sys
import tf
from moveit_commander.exception import MoveItCommanderException
import moveit_msgs
from math import pi
from std_msgs.msg import String, Bool
from plate_detector_class import plate_detector
from geometry_msgs.msg import PoseStamped


from ur5_magnetic_gripper.msg import GripperAttached
from ur5_magnetic_gripper.srv import Magnetize, MagnetizeRequest


# main class
class PickRGB:

  def __init__(self):
    #members
    self.gripper_attached = False
    self.t = tf.TransformListener() 

    self.pubTarget = rospy.Publisher('/armIKpipeline/target_pose', PoseStamped, queue_size=1)
    self.pub_start_pregrasp = rospy.Publisher('/pregrasp_pipeline_event_in', String, queue_size=1)
    self.pub_start_planned_motion = rospy.Publisher('/pregrasp_pipeline_move_arm_planned_motion/event_in', String, queue_size=1)

    #moveit group is not supported by the interface, so adding it manually for now
    moveit_commander.roscpp_initialize(sys.argv)
    self.group = moveit_commander.MoveGroupCommander("manipulator") 
    self.robot = moveit_commander.RobotCommander()
    self.attach = rospy.Subscriber("/attached", GripperAttached, self.attached_cb,queue_size=10)


  def go_to_joint_pose(self, arm_configuration):
    try:
      self.group.set_joint_value_target(arm_configuration)
      self.group.go(wait=True)
    except MoveItCommanderException, e:
      print 'can not set goal pose'
      print e

  def arm_vertical(self, inc):
    try:
      self.group.shift_pose_target(2, inc)
      self.group.go(wait=True)
    except MoveItCommanderException, e:
      print 'can not set move vertically pose'
      print e

  def attached_cb(self, msg):
    rospy.loginfo('Attached changed!!')
    self.gripper_attached = msg.attached

    #main function
  def execute(self):
    hold_pose = [-1.9231649399288942, -1.7708555307344, 2.020241543371168, -1.792100532449426, -1.54328856236556, -2.102485936227004]
    top_view = [-1.5938509146319788, -1.0618065160563965, 0.22519189516176397, -0.2647755903056641, -1.6169903914081019, -1.8908899466144007]

      #INSERIR LASER STUFF
    self.go_to_joint_pose(top_view)
    rospy.sleep(3)
    detector = plate_detector()
    target = detector.execute()
    for i in range(3):
      if target:
        break
      print "plate detector did not find a result. Trying again"
      target = detector.execute()
    if not target:
      print "Aborting plate detector"
      return
    self.pubTarget.publish(target)

    rospy.sleep(0.5)
    self.pub_start_pregrasp.publish('e_start')
    rospy.sleep(1)
    self.pub_start_planned_motion.publish('e_start')
    rospy.sleep(5)
    self.gripper_attached = False
        
    while self.gripper_attached == False: #mudar de attached para wrench
      self.arm_vertical(-0.01)
      rospy.sleep(0.2)
    print "exit while"
    rospy.sleep(1)
# #INSERIR DEMAGNATIZE CB
    self.arm_vertical(0.05)

#     rospy.sleep(1)

    #self.go_to_joint_pose(hold_pose)

    rospy.logdebug('UP!!')
    rospy.sleep(4)
    return 'success'