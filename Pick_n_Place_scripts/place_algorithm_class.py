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
from edge_detector_class import edge_detector
from geometry_msgs.msg import PoseStamped
from mbzirc_comm_objs.srv import Magnetize, MagnetizeRequest
# from laser_brick_detection import Allign


# main class
class Place:

  def __init__(self, side, wall_brick_color, level, over, brick2place_color):
        #members
    self.brick2place_color = brick2place_color
    self.over = over
    self.side = side
    self.wall_brick_color = wall_brick_color
    self.level = level
    self.gripper_attached = True
    self.t = tf.TransformListener() 

    self.pubTarget = rospy.Publisher('/mir_manipulation/pregrasp_planner_pipeline/target_pose', PoseStamped, queue_size=1)
    self.pub_start_pregrasp = rospy.Publisher('/mir_manipulation/pregrasp_planner_pipeline/event_in', String, queue_size=1)
    self.pub_start_planned_motion = rospy.Publisher('/move_arm_planned_motion/event_in', String, queue_size=1)

    self.magnetize = rospy.ServiceProxy('magnetize', Magnetize)
    self.magnetize_request = MagnetizeRequest()
        #moveit group is not supported by the interface, so adding it manually for now
    moveit_commander.roscpp_initialize(sys.argv)
    self.group = moveit_commander.MoveGroupCommander("manipulator") 
    self.robot = moveit_commander.RobotCommander()

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

  def create_pose(self, x, y, z, frame):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(pi/2, pi, pi)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = frame
    return pose

  def compute_place_pose(self, edge_point): #receives edge point in base_link
    if self.brick2place_color == "red":
      offset = 0.15
    elif (self.brick2place_color == "green"):
      offset = 0.3
    elif (self.brick2place_color == "blue"):
      offset = 0.6
    elif (self.brick2place_color == "orange"):
      offset = 0.9
    else:
      return
    if (self.over):
      if self.side == "right":
        y = edge_point.point.y + offset
      if self.side == "left":
        y = edge_point.point.y - offset
      height = self.level*0.2 + 0.2
    if not (self.over):
      if self.side == "right":
        y = edge_point.point.y - offset
      if self.side == "left":
        y = edge_point.point.y + offset
      height = self.level*0.2 
    return self.create_pose(edge_point.point.x, y, height, "/base_link") #VERIFICAR ESTA POSE 


    #main function
  def execute(self):

    hold_pose = [-1.9231649399288942, -1.7708555307344, 2.020241543371168, -1.792100532449426, -1.54328856236556, -2.102485936227004]
    top_view = [-1.5938509146319788, -1.0618065160563965, 0.22519189516176397, -0.2647755903056641, -1.6169903914081019, -1.8908899466144007]

      #INSERIR LASER STUFF
    self.go_to_joint_pose(top_view)
    rospy.sleep(3)

    edge = edge_detector(self.side, self.wall_brick_color, self.level)
    lado = edge.execute()
    for i in range(3):
      if lado:
        break
      print "edge detector did not find a result. Trying again"
      lado = edge.execute()
    if not lado:
      print "Aborting edge detector"
      return
    lado = self.t.transformPoint("/base_link", lado) #if this does not work change to transform Pose

    target = self.compute_place_pose(lado)
    if not target:
      print "Could not compute place pose"
      return
    self.pubTarget.publish(target)

    rospy.sleep(0.5)
    self.pub_start_pregrasp.publish('e_start')
    rospy.sleep(0.5)
    self.pub_start_planned_motion.publish('e_start')
    rospy.sleep(2)
    self.gripper_attached = True
        
    while self.gripper_attached == False: #can wrench be used? it has weight from brick
      self.arm_vertical(-0.01)
      rospy.sleep(0.2)
      rospy.loginfo('down!!')
    rospy.sleep(1)
    while self.gripper_attached = True:
      self.gripper_attached = self.magnetize(self.magnetize_request(magnetize=False))

    self.arm_vertical(0.05)
    rospy.sleep(1)

    # self.go_to_joint_pose(hold_pose)

    while self.gripper_attached = True:
      self.gripper_attached = self.magnetize(self.magnetize_request(magnetize=True))

    rospy.sleep(4)
    return 'success'
