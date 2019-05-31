#!/usr/bin/env python

import rospy
import actionlib
import moveit_commander
from moveit_commander.exception import MoveItCommanderException

from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState

import sys, tty, termios
from math import sqrt, pow

import time

####### service callbacks ###################

pub = None
pub2 = None
currentState = None

def current_state_call(data) :
  global currentState
  currentState = data


def get_current_state() :
  global currentState
  currentState = None

  while not currentState:
    time.sleep(0.1) 

  return currentState

def grasp(action) :
  grip = (0.5,0.5,0.5) if action == 'GRASP' else (0.05,0.05,0.05)
  state = get_current_state()
  msg = JointState()
  msg.name = state.name
  msg.position = state.position[:7] +grip

  #print msg

  global pub2
  pub2.publish(msg)

def go_to_state(points):

  '''if len(pos) != 9:
    print 'joint positions array must be of size 9!!'
    return'''

  '''msg = JointState()
  msg.name = ['jaco_arm_0_joint', 'jaco_arm_1_joint', 'jaco_arm_2_joint', 'jaco_arm_3_joint', 'jaco_arm_4_joint', 'jaco_arm_5_joint', 'jaco_finger_joint_0', 'jaco_finger_joint_2', 'jaco_finger_joint_4']
  msg.position = pos

  global pub
  pub.publish(msg)'''

  msg = FollowJointTrajectoryGoal()
  state = get_current_state()
  msg.trajectory.joint_names = state.name[:7]
  points = [state.position[:7]] + points
  for point in points:
    tpoint = JointTrajectoryPoint()
    tpoint.positions = point
    msg.trajectory.points.append(tpoint)

  global pub
  pub.send_goal(msg)
  pub.wait_for_result()

def pick_blue(req):
  points = []
  points.append((0,-1.4510718081844756, -0.4172474211787407, -0.38410237364300137, -2.7470810627595013, 2.1211391964750916, -2.5265898175775403))
  points.append((0,-1.4434113827681108, -0.2998364753579097, -0.41726147490466214, -2.7752690200200822, 2.2291254644445644, -2.5658058238839745)) 
  go_to_state(points)
  print 'I am back!!'
  grasp('GRASP')

  return {}

def place_coke(req):
  points = []
  points.append((0,-1.4510718081844756, -0.4172474211787407, -0.38410237364300137, -2.7470810627595013, 2.1211391964750916, -2.5265898175775403))
  points.append((0,0.4586157525392034, -0.6057810683810996, 0.18768382329765565, -3.0215424718178614, 2.708787954583766, -0.7390761393135916))
  points.append((0,1.301561698705573, 0.3802065120321705, -1.1754006300823114, -2.7911346559094286, 2.1657429587243806, 0.1285928885105374))
  go_to_state(points)
  grasp('UNGRASP')

  return {}


def pick_blue2(req):
  points = []
  points.append((0,-0.18972883077998848, -0.7075334297937916, 0.041427808801247856, -0.20797272254863663, -2.249852310209304, -0.45444002130367167))
  points.append((0,-0.18127232959861406, -0.42787257539668766, 0.055288246050721135, -0.029635979642216093, -2.65119402586706, -0.3778089423414883)) 
  go_to_state(points)
  print 'I am back!!'
  grasp('GRASP')

  return {}

def place_coke2(req):
  points = []
  points.append((0,0.986157525392034, -0.6057810683810996, 0.18768382329765565, -3.0215424718178614, 2.708787954583766, -0.7390761393135916))
  points.append((0,1.301561698705573, 0.3802065120321705, -1.1754006300823114, -2.7911346559094286, 2.1657429587243806, 0.1285928885105374))
  go_to_state(points)
  grasp('UNGRASP')

  return {}

###### main ##########################

def server_init():

  rospy.init_node('moveit_teleop',
                  anonymous=True)

  global pub
  global pub2
  pub2 = rospy.Publisher('/jaco/joint_control', JointState, queue_size=10)
  pub = actionlib.SimpleActionClient('/jaco/joint_trajectory_action', FollowJointTrajectoryAction)
  pub.wait_for_server()

  rospy.Subscriber("/jaco/joint_state", JointState, current_state_call)


  rospy.Service('moveit_teleop/pick_blue', Empty, pick_blue)
  rospy.Service('moveit_teleop/place_coke', Empty, place_coke)
  rospy.Service('moveit_teleop/pick_blue2', Empty, pick_blue2)
  rospy.Service('moveit_teleop/place_coke2', Empty, place_coke2)

  go_to_state([(0,1.301561698705573, 0.3802065120321705, -1.1754006300823114, -2.7911346559094286, 2.1657429587243806, 0.1285928885105374)])

  print 'ready!!'

  rospy.spin()

if __name__=='__main__':
  try:
    server_init()
  except rospy.ROSInterruptException:
    pass