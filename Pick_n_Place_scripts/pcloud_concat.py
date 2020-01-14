import roslib
import rospy
import actionlib
import moveit_commander
import sys
from moveit_commander.exception import MoveItCommanderException
import moveit_msgs
from math import pi
from std_msgs.msg import String, Bool


# main class
class Pcloud:

  def __init__(self):
        #members
    rospy.init_node('pcloud_concatenator', anonymous=True)
    self.gripper_attached = False

    self.pubBrickPose = rospy.Publisher('/color2detect', String, queue_size=1)
    self.pubConcatTrigger = rospy.Publisher('/concatenate_this', Bool, queue_size=1)
    self.pubConcatReset = rospy.Publisher('/concatenate_reset', Bool, queue_size=1)
    self.pub_start_pregrasp = rospy.Publisher('/mir_manipulation/pregrasp_planner_pipeline/event_in', String, queue_size=1)
    self.pub_start_planned_motion = rospy.Publisher('/move_arm_planned_motion/event_in', String, queue_size=1)

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

    #main function
  def execute(self, brick_color):
    hold_pose = [-1.9231649399288942, -1.7708555307344, 2.020241543371168, -1.792100532449426, -1.54328856236556, -2.102485936227004]
    look_around = []
    look_around.append([-1.2538898626910608, -1.866218706170553, 0.9891861120807093, -0.7832563680461426, -1.6210344473468226, -2.1499837080584925])
    look_around.append([-2.4912708441363733, -1.0757158559611817, 0.9895809332477015, -0.4796498578837891, -1.1139004866229456, -2.257519547139303])
    look_around.append([-1.6537531057940882, -0.7855909627727051, 0.19012529054750615, -0.47916968286547856, -1.6044629255877894, -2.257483784352438])
    look_around.append([-0.7324913183795374, -0.5678041738322754, 0.19185334840883428, -0.5627382558635254, -2.242927853261129, -2.257495705281393])
      #INSERIR LASER STUFF

    for i in look_around:
      self.go_to_joint_pose(i)
      rospy.sleep(4)
      self.pubConcatTrigger.publish(data=True)
    self.pubConcatReset.publish(data=True)
    rospy.sleep(1)
    self.pubBrickPose.publish(brick_color) #
    rospy.sleep(0.5)

        # comecar o pregrasp sabendo que ele ja esta a receber para o seu /target_pose a pose do brick
    self.pub_start_pregrasp.publish('e_start')
    rospy.sleep(0.5)
    self.pub_start_planned_motion.publish('e_start')
    rospy.sleep(2)

    self.gripper_attached = False

        #move gripper vertically until contact
        
        # while not self.gripper_attached:
        #     self.arm_vertical(-0.01)
        #     rospy.loginfo('down!!')
        #     #rate.sleep()
    rospy.sleep(1)

    self.arm_vertical(0.05)
    rospy.loginfo('Attached!!')

    rospy.sleep(1)

    self.go_to_joint_pose(hold_pose)

    rospy.logdebug('UP!!')
    rospy.sleep(4)
    return 'success'

def main(args):
  ic = Pcloud()
  try:
    ic.execute("blue")
    #   while not rospy.is_shutdown():
    #     ic.rate.sleep()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
  main(sys.argv)