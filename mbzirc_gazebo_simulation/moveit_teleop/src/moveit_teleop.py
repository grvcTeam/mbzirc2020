#!/usr/bin/env python

import rospy
import actionlib
import moveit_commander
from moveit_commander.exception import MoveItCommanderException

from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from std_srvs.srv import Empty

import sys, tty, termios
from math import sqrt, pow

gripper_pub = None
group = None
valid_pose = [-1.4399860385802867, 0.24422998084221526, -0.5412292105382823, 1.7837733538981624, -0.011756491248733347]
red_pose = [-1.461905992227174, -0.13395470771747142, -0.23128530726500074, 1.0091699755510612, -0.012427445347488408]
valid_pose2 = [1.4399860385802867, 0.24422998084221526, -0.5412292105382823, 1.7837733538981624, -0.011756491248733347]
blue_pose = [1.461905992227174, -0.13395470771747142, -0.23128530726500074, 1.0091699755510612, -0.012427445347488408]
release_pose = [0.0, 0.14450065459268302, -0.5484821737596519, 0.8375268649513847, 0.01997370202354176]

def get_ch():
  fd = sys.stdin.fileno()
  old_settings = termios.tcgetattr(fd)
  try:
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)
  finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
  return ch

###### actions ##############################

def move_to_joint_state(group, state):
  try:
    group.set_joint_value_target(state)
  except MoveItCommanderException, e:
    print 'can not set goal pose'
    print e
  finally:
    group.go(wait=True)

def gripper_action(action):
  print('grasping!')
  msg = GripperCommandGoal()
  if action == 'GRASP':
    msg.command.position = -0.44
  elif action == 'RELEASE':
    msg.command.position = 0.2
  else:
    print 'no action!'
    return

  gripper_pub.send_goal(msg)
  print('grasped!')

####### service callbacks ###################

def pick_red(req):
  #gripper_action('RELEASE')
  move_to_joint_state(group, valid_pose)
  move_to_joint_state(group, red_pose)
  gripper_action('GRASP')

  return {}

def pick_blue(req):
  #gripper_action('RELEASE')
  move_to_joint_state(group, valid_pose2)
  move_to_joint_state(group, blue_pose)
  gripper_action('GRASP')

  return {}

def place_coke(req):
  goal = release_pose
  move_to_joint_state(group, goal)
  gripper_action('RELEASE')

  return {}

###### precomputed acions ###################

def bend_gripper():
  move_joint(group, 3, 1.5708)

def start_draw():
  move_to_joint_state(group, [-1.2, -0.05834763175623525, 0.0547655961036666, 1.5237265739452077, -0.008625295267727928])

def draw_line():
  move_to_joint_state(group, [1.2, -0.05834763175623525, 0.0547655961036666, 1.5237265739452077, -0.008625295267727928])

def draw_fancy():
  move_to_joint_state(group, [-0.5024095526490102, -0.09257661304316134, 0.08441163796190132, 1.1855758067129618, -0.01346512777511144])
  move_to_joint_state(group, [0.5024095526490102, -0.09257661304316134, 0.08441163796190132, 1.1855758067129618, -0.01346512777511144])
  move_to_joint_state(group, [0.70713846512893, -0.05834763175623525, 0.0547655961036666, 1.5237265739452077, -0.008625295267727928])

####### teleop functions ####################

def move_joint(group, index, inc):
  print "moving joint!"
  vals = group.get_current_joint_values()
  if index not in range(len(vals)):
    print 'joint out of index'
    return

  vals[index] = vals[index] + inc
  move_to_joint_state(group, vals)
  print "joint moved!"

#TODO: doesn't work, IK problems.
def move_end(group, key, inc):
  print "moving end!"
  pose_target = group.get_current_pose().pose
  print pose_target.position.x
  print pose_target.position.y
  print pose_target.position.z
  print pose_target.orientation.x
  print pose_target.orientation.y
  print pose_target.orientation.z
  print pose_target.orientation.w

  return

  n = sqrt(pow(pose_target.orientation.x,2)+pow(pose_target.orientation.y,2)+pow(pose_target.orientation.z,2))
  x_norm = pose_target.orientation.x / n
  y_norm = pose_target.orientation.y / n
  z_norm = pose_target.orientation.z / n

  if key == 'x':
    pose_target.position.x = pose_target.position.x + inc
  if key == 'y':
    pose_target.position.y = pose_target.position.y + inc
  if key == 'z':
    pose_target.position.z = pose_target.position.z + inc
  if key == 'X':
    x_norm = x_norm + inc
  if key == 'Y':
    y_norm = y_norm + inc
  if key == 'Z':
    z_norm = z_norm + inc

  n_new = sqrt(pow(x_norm,2)+pow(y_norm,2)+pow(z_norm,2))
  x_norm = pose_target.orientation.x / n_new
  y_norm = pose_target.orientation.y / n_new
  z_norm = pose_target.orientation.z / n_new
  pose_target.orientation.y = x_norm * n
  pose_target.orientation.z = y_norm * n
  pose_target.orientation.w = z_norm * n

  #group.set_pose_target(pose_target)
  move_to_joint_state(group, pose_target)
  print "end moved!"

def joint_teleop(group):
  j_max = len(group.get_current_joint_values())
  j_ind = 0
  inc = 0.1
  key = None
  while key != 'e':
    key = get_ch()
    if key == 'w':
      move_joint(group, j_ind, inc)
    if key == 's':
      move_joint(group, j_ind, -inc)
    if key == 'd':
      j_ind = (j_ind + 1) if (j_ind + 1) < j_max else 0
      print '-----> moving joint  -------> ' + str(j_ind)
    if key == 'a':
      j_ind = (j_ind - 1) if (j_ind - 1) >= 0 else (j_max - 1)
      print '-----> moving joint  -------> ' + str(j_ind)

    if key == 'r':
      gripper_action('RELEASE')
    if key == 'g':
      gripper_action('GRASP')
    if key == 'p':
      print group.get_current_joint_values()
    if key == 'c':
      pick_red('')
    if key == 'x':
      pick_blue('')
    if key == 'v':
      place_coke('')

    '''if key == 'y':
      start_draw()
    if key == 'h':
      draw_line()
    if key == 'n':
      draw_fancy()'''

    if key == 'x':
      move_end(group, 'x', inc)
    if key == 'y':
      move_end(group, 'y', inc)
    if key == 'z':
      move_end(group, 'z', inc)
    if key == 'X':
      move_end(group, 'X', inc)
    if key == 'Y':
      move_end(group, 'Y', inc)
    if key == 'Z':
      move_end(group, 'Z', inc)

    if key == 'i':
      inc = inc - 0.01
      print 'increment changed: ' + str(inc)
    if key == 'o':
      inc = inc + 0.01
      print 'increment changed: ' + str(inc)

###### main ##########################

def moveit_init():
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('moveit_teleop',
                  anonymous=True)

  '''global gripper_pub
  gripper_pub = actionlib.SimpleActionClient('gripper_grasp_posture_controller', GripperCommandAction)
  gripper_pub.wait_for_server()'''
  group = moveit_commander.MoveGroupCommander("manipulator")
  print 'READY'
  return group

def teleop_init():
  global group
  group = moveit_init()
  #bend_gripper()
  joint_teleop(group)
  moveit_commander.roscpp_shutdown()

def server_init():
  global group
  group = moveit_init()
  rospy.Service('moveit_teleop/pick_red', Empty, pick_red)
  rospy.Service('moveit_teleop/pick_blue', Empty, pick_blue)
  rospy.Service('moveit_teleop/place_coke', Empty, place_coke)
  rospy.spin()

if __name__=='__main__':
  try:
    teleop_init()
    #server_init()
  except rospy.ROSInterruptException:
    pass
