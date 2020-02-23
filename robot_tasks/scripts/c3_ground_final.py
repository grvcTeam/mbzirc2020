#!/usr/bin/env python

# PARAMETER	SPECIFICATION
# Number of UAVs per team	Maximum of 3
# Number of UGVs per team	1
# Arena size	50mx60mx20m
# Tower height	Maximum of 18m
# Tower base and interior specifications	TBD
# Location of simulated fires	Up to 16m in height, inside the arena.
# Environment	Outdoor and indoor
# Mode of operation	Autonomous; manual allowed but penalized
# RTK/DGPS positioning	Allowed but penalized
# Challenge duration	20 Minutes
# Communications	TBD
# Extinguisher types considered	Water carrying container 1-3 liters. Some examples shown in Figure 2.
# Maximum size of UAV	1.2m x 1.2m x 0.5m
# Maximum size of UGV	1.7m x 1.5m x 2m

# import copy
import math
import rospy
import smach
import yaml
import mbzirc_comm_objs.msg as msg

from geometry_msgs.msg import PoseStamped
from tasks.move import TakeOff, GoTo, Land
from tasks.fire import ExtinguishGroundFire, ExtinguishGroundFireSimple
from std_srvs.srv import Trigger, TriggerResponse
from utils.manager import TaskManager
from utils.robot import RobotProxy

# TODO: All these parameters from config!
field_width = 60
field_height = 50
column_count = 4  # 6  # TODO: as a function of fov

def ch3_matrix2pose(Y_index,X_index):
    x0 = -11.0534474738
    y0 = 7.92951858232
    width = 2.44
    depth = 1.22
    x = x0 + X_index*width
    y = y0 + Y_index*depth
    return x, y

class CentralUnit(object):
    def __init__(self):

        # Read parameters
        conf_file = rospy.get_param('~conf_file', 'config/conf_ch3_ground.yaml')
        self.robot_id = rospy.get_param('~uav_id', '6')
        self.height = rospy.get_param('~height', 3.0)
        self.fire_x,self.fire_y = ch3_matrix2pose(rospy.get_param('~fire_i',1),rospy.get_param('~fire_j',1))

        self.robots = {}
        self.robots[self.robot_id] = RobotProxy(self.robot_id)
        self.task_manager = TaskManager(self.robots)

        # Start challenge service
        self.start_challenge_service = rospy.Service('start_c3_ground',Trigger,self.start_challenge)
        self.started = False

    def start_challenge(self, req):
        rospy.loginfo('Starting challenge 3 ground.')
        self.started = True
        return TriggerResponse(True,'')

    # TODO: Could be a smach.State (for all or for every single uav)
    def take_off(self):
        userdata = smach.UserData()
        userdata.height = self.height
        self.task_manager.start_task(self.robot_id, TakeOff(), userdata)
        self.task_manager.wait_for([self.robot_id])

    def extinguish_ground_fire(self):
        userdata = smach.UserData()
        userdata.waypoint = PoseStamped()
        userdata.waypoint.header.frame_id = 'arena'
        userdata.waypoint.pose.position.x = self.fire_x
        userdata.waypoint.pose.position.y = self.fire_y
        userdata.waypoint.pose.position.z = self.height
        userdata.waypoint.pose.orientation.x = 0
        userdata.waypoint.pose.orientation.y = 0
        userdata.waypoint.pose.orientation.z = 0
        userdata.waypoint.pose.orientation.w = 0
        userdata.color = 'fire'
        self.task_manager.start_task(self.robot_id, ExtinguishGroundFireSimple(), userdata)
        self.task_manager.wait_for([self.robot_id])

    def go_home(self):
        # Go to start pose at 6 meters -> Now go to safe pose
        userdata_goto = smach.UserData()
        userdata_goto.waypoint = PoseStamped()
        #userdata_goto.waypoint = self.path[0]
        userdata_goto.waypoint.pose.position.x = -6.0
        userdata_goto.waypoint.pose.position.y = 3.0
        userdata_goto.waypoint.pose.position.z = 6.0
        userdata_goto.waypoint.pose.orientation.x = 0
        userdata_goto.waypoint.pose.orientation.y = 0
        userdata_goto.waypoint.pose.orientation.z = 0
        userdata_goto.waypoint.pose.orientation.w = 0
        self.task_manager.start_task(self.robot_id, GoTo(), userdata_goto)
        self.task_manager.wait_for([self.robot_id])

        # Land
        # userdata_land = smach.UserData()
        # self.task_manager.start_task(self.robot_id, Land(), userdata_land)
        # self.task_manager.wait_for([self.robot_id])

def main():
    rospy.init_node('central_unit_c3_ground_final')

    while rospy.get_rostime() == rospy.Time():
        rospy.logwarn("Waiting for (sim) time to begin!")
        rospy.sleep(1)

    central_unit = CentralUnit()
    rospy.loginfo('Batamanta ready to go.')

    while not rospy.is_shutdown() and not central_unit.started:
        rospy.sleep(1)

    central_unit.take_off()
    central_unit.extinguish_ground_fire()
    central_unit.go_home()

    rospy.loginfo('Finished!')

if __name__ == '__main__':
    main()
