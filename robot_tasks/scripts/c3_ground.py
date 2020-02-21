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

# from geometry_msgs.msg import PoseStamped, Vector3
from geometry_msgs.msg import PoseStamped
from tasks.move import TakeOff, FollowPath
from tasks.fire import ExtinguishFacadeFire, ExtinguishGroundFire
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

        # Read parameters
        conf_file = rospy.get_param('~conf_file', 'config/conf_ch3_ground.yaml')
        self.robot_id = rospy.get_param('~uav_id', '')
        self.height = rospy.get_param('~height', 3.0)

        with open(conf_file,'r') as file:
            ground_fires_conf = yaml.safe_load(file)

        self.path = []
        if 'path' in ground_fires_conf:
            for waypoint in ground_fires_conf['path']:
                wp = PoseStamped()
                wp.header.frame_id = 'arena'
                wp.pose.position.x = waypoint[0]
                wp.pose.position.y = waypoint[1]
                wp.pose.position.z = self.height
                wp.pose.orientation.x = 0
                wp.pose.orientation.y = 0
                wp.pose.orientation.z = 0
                wp.pose.orientation.w = 0
                self.path.append(wp)
        else:
            rospy.logerr('No path defined in config file')

        self.robots = {}
        self.robots[self.robot_id] = RobotProxy(self.robot_id)
        self.task_manager = TaskManager(self.robots)

    # TODO: Move this function to RobotProxy?
    def get_param(self, robot_id, param_name):
        # TODO: Default value in case param_name is not found?
        return rospy.get_param(self.robots[robot_id].url + param_name)

    # TODO: Could be a smach.State (for all or for every single uav)
    def take_off(self):
        userdata = smach.UserData()
        userdata.height = self.height
        self.task_manager.start_task(self.robot_id, TakeOff(), userdata)
        self.task_manager.wait_for([self.robot_id])
    
    def extinguish_ground_fire(self):
        userdata = smach.UserData()
        userdata.path = self.path
        userdata.color = 'fire_box'
        self.task_manager.start_task(self.robot_id, ExtinguishGroundFire(), userdata)
        self.task_manager.wait_for([self.robot_id])


def main():
    rospy.init_node('central_unit_c3_ground')

    while rospy.get_rostime() == rospy.Time():
        rospy.logwarn("Waiting for (sim) time to begin!")
        rospy.sleep(1)

    central_unit = CentralUnit()
    rospy.sleep(5)

    central_unit.take_off()
    central_unit.extinguish_ground_fire()

    rospy.spin()

if __name__ == '__main__':
    main()
