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
import mbzirc_comm_objs.msg as msg

# from geometry_msgs.msg import PoseStamped, Vector3
from geometry_msgs.msg import PoseStamped
from tasks.move import TakeOff, FollowPath
from tasks.fire import ExtinguishFacadeFire
# from tasks.build import PickAndPlace
# from tasks.search import all_piles_are_found
# from utils.translate import color_from_int
from utils.manager import TaskManager
from utils.robot import RobotProxy
from utils.path import generate_uav_paths, set_z
# from utils.wall import get_build_wall_sequence


# TODO: All these parameters from config!
field_width = 60
field_height = 50
column_count = 4  # 6  # TODO: as a function of fov

class CentralUnit(object):
    def __init__(self):
        self.available_robots = ['5']  # Force id to be a string to avoid index confussion  # TODO: auto discovery (and update!)

        self.robots = {}
        for robot_id in self.available_robots:
            self.robots[robot_id] = RobotProxy(robot_id)

        self.task_manager = TaskManager(self.robots)

    # TODO: Move this function to RobotProxy?
    def get_param(self, robot_id, param_name):
        # TODO: Default value in case param_name is not found?
        return rospy.get_param(self.robots[robot_id].url + param_name)

    # TODO: Could be a smach.State (for all or for every single uav)
    def take_off(self):
        for robot_id in self.available_robots:
            # TODO: clear all regions?
            userdata = smach.UserData()
            # userdata.height = self.get_param(robot_id, 'flight_level')  # TODO: Why not directly inside tasks?
            userdata.height = 2.0  # TODO: Why not directly inside tasks?
            self.task_manager.start_task(robot_id, TakeOff(), userdata)
            self.task_manager.wait_for([robot_id])  # Sequential takeoff

    # TODO: Could be a smach.State (for all or for every single uav)
    # def look_for_fire(self):
    #     # TODO: Check this better outside!?
    #     if len(self.fires) > 0:
    #         rospy.logwarn('A fire is found!')
    #         return
    #     robot_paths = {}
    #     point_paths = generate_uav_paths(len(self.available_robots), field_width, field_height, column_count)
    #     for i, robot_id in enumerate(self.available_robots):
    #         robot_path = []
    #         flight_level = self.get_param(robot_id, 'flight_level')
    #         point_path = set_z(point_paths[i], flight_level)
    #         for point in point_path:
    #             waypoint = PoseStamped()
    #             waypoint.header.frame_id = 'arena'
    #             waypoint.pose.position = point
    #             waypoint.pose.orientation.z = 0
    #             waypoint.pose.orientation.w = 1  # TODO: other orientation?
    #             robot_path.append(waypoint)
    #         robot_paths[robot_id] = robot_path

    #     for robot_id in self.available_robots:
    #         userdata = smach.UserData()
    #         userdata.path = robot_paths[robot_id]
    #         self.task_manager.start_task(robot_id, FollowPath(), userdata)

    #     while not len(self.fires) and not rospy.is_shutdown():
    #         # TODO: What happens if fire is NEVER found?
    #         rospy.logwarn('len(self.fires) = {}'.format(len(self.fires)))
    #         rospy.sleep(1.0)

    #     for robot_id in self.available_robots:
    #         rospy.logwarn('preempting {}'.format(robot_id))
    #         self.task_manager.preempt_task(robot_id)

    def extinguish_fire(self):
        userdata = smach.UserData()
        userdata.approximate_pose = PoseStamped()
        userdata.approximate_pose.header.frame_id = 'arena'
        userdata.approximate_pose.pose.position.x = 7.5
        userdata.approximate_pose.pose.position.y = 7.5
        userdata.approximate_pose.pose.position.z = 2.0
        userdata.approximate_pose.pose.orientation.z = 0
        userdata.approximate_pose.pose.orientation.w = 0
        self.task_manager.start_task(self.available_robots[0], ExtinguishFacadeFire(), userdata)


def main():
    rospy.init_node('central_unit_c3')

    while rospy.get_rostime() == rospy.Time():
        rospy.logwarn("Waiting for (sim) time to begin!")
        rospy.sleep(1)

    central_unit = CentralUnit()
    # rospy.sleep(3)

    central_unit.take_off()
    central_unit.extinguish_fire()

    rospy.spin()

if __name__ == '__main__':
    main()
