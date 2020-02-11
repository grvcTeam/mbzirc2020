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

# TODO: Move to robot_proxy? Use own pose? Give a set of possible poses (obstacle avoidance)?
def calculate_extiguish_pose(fire_pose, robot_pose, stream_speed):
    # Supose a pure horizontal (at stream_speed) parabolic shot from robot_pose to fire_pose:
    # TODO: transfor to a common tf (map?, robot?)
    frame_id = 'map'
    g = 9.8  # [m/s^2]
    delta_x = 1  # TODO: fire_pose.position.x - robot_pose.pose.position.x
    delta_y = 0  # TODO: fire_pose.position.y - robot_pose.pose.position.y
    delta_z = -1  # TODO: fire_pose.position.z - robot_pose.pose.position.z
    horizontal_distance = stream_speed * math.sqrt(2.0 * abs(delta_z) / g)
    n_samples = 6
    poses = []
    for i in range(n_samples):
        theta = i * 2*math.pi / n_samples
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = fire_pose.pose.position.x + horizontal_distance * math.cos(theta)
        pose.pose.position.y = fire_pose.pose.position.y + horizontal_distance * math.sin(theta)
        pose.pose.position.z = robot_pose.pose.position.z
        yaw = theta + math.pi
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        poses.append(pose)
    print(poses)

class CentralUnit(object):
    def __init__(self):
        self.available_robots = ['1']  # Force id to be a string to avoid index confussion  # TODO: auto discovery (and update!)

        self.robots = {}
        for robot_id in self.available_robots:
            self.robots[robot_id] = RobotProxy(robot_id)

        self.fires = []
        rospy.Subscriber("estimated_objects", msg.ObjectDetectionList, self.estimation_callback)

        self.task_manager = TaskManager(self.robots)

    # TODO: Move this function to RobotProxy?
    def get_param(self, robot_id, param_name):
        # TODO: Default value in case param_name is not found?
        return rospy.get_param(self.robots[robot_id].url + param_name)

    # TODO: This is repeated!
    def estimation_callback(self, data):
        for estimation in data.objects:
            # TODO: check also scale?
            if estimation.type != ObjectDetection.TYPE_FIRE:
                continue
            pose = PoseStamped()
            pose.header = estimation.header
            pose.pose = estimation.pose.pose
            self.fires.append(pose)

    # TODO: Could be a smach.State (for all or for every single uav)
    def take_off(self):
        for robot_id in self.available_robots:
            # TODO: clear all regions?
            userdata = smach.UserData()
            userdata.height = self.get_param(robot_id, 'flight_level')  # TODO: Why not directly inside tasks?
            self.task_manager.start_task(robot_id, TakeOff(), userdata)
            self.task_manager.wait_for([robot_id])  # Sequential takeoff

    # TODO: Could be a smach.State (for all or for every single uav)
    def look_for_fire(self):
        # TODO: Check this better outside!?
        if len(self.fires) > 0:
            rospy.logwarn('A fire is found!')
            return
        robot_paths = {}
        point_paths = generate_uav_paths(len(self.available_robots), field_width, field_height, column_count)
        for i, robot_id in enumerate(self.available_robots):
            robot_path = []
            flight_level = self.get_param(robot_id, 'flight_level')
            point_path = set_z(point_paths[i], flight_level)
            for point in point_path:
                waypoint = PoseStamped()
                waypoint.header.frame_id = 'arena'
                waypoint.pose.position = point
                waypoint.pose.orientation.z = 0
                waypoint.pose.orientation.w = 1  # TODO: other orientation?
                robot_path.append(waypoint)
            robot_paths[robot_id] = robot_path

        for robot_id in self.available_robots:
            userdata = smach.UserData()
            userdata.path = robot_paths[robot_id]
            self.task_manager.start_task(robot_id, FollowPath(), userdata)

        while not len(self.fires) and not rospy.is_shutdown():
            # TODO: What happens if fire is NEVER found?
            rospy.logwarn('len(self.fires) = {}'.format(len(self.fires)))
            rospy.sleep(1.0)

        for robot_id in self.available_robots:
            rospy.logwarn('preempting {}'.format(robot_id))
            self.task_manager.preempt_task(robot_id)

    # TODO: Could be a smach.State (for all or for every single uav, not so easy!)
    def extinguish_fire(self):
        fires = copy.deepcopy(self.fires)  # Cache piles
        build_wall_sequence = get_build_wall_sequence(wall_blueprint, brick_scales)
        for i, row in enumerate(build_wall_sequence):
            for brick in row:
                print('row[{}] brick = {}'.format(i, brick))
                costs = {}
                while not costs and not rospy.is_shutdown():
                    for robot_id in self.available_robots:
                        if self.task_manager.is_idle(robot_id):
                            costs[robot_id] = self.robots[robot_id].get_cost_to_go_to(piles[brick.color])
                        else:
                            # print('waiting for an idle robot...')
                            rospy.sleep(0.5)
                min_cost_robot_id = min(costs, key = costs.get)
                print('costs: {}, min_cost_id: {}'.format(costs, min_cost_robot_id))

                flight_level = self.get_param(min_cost_robot_id,'flight_level')
                userdata = smach.UserData()
                userdata.above_pile_pose = copy.deepcopy(piles[brick.color])
                userdata.above_pile_pose.pose.position.z = flight_level
                userdata.above_wall_pose = copy.deepcopy(brick.pose)
                userdata.above_wall_pose.pose.position.z = flight_level
                userdata.in_wall_brick_pose = copy.deepcopy(brick.pose)
                self.task_manager.start_task(min_cost_robot_id, PickAndPlace(), userdata)

        # Once arrived here, last pick_and_place task has been allocated
        print('All pick_and_place tasks allocated')
        finished_robots = []
        while not rospy.is_shutdown():
            rospy.sleep(0.5)  # TODO: some sleep here?
            for robot_id in self.available_robots:
                if self.task_manager.is_idle(robot_id) and (robot_id not in finished_robots):
                    finished_robots.append(robot_id)
                    print('now go home, robot [{}]!'.format(robot_id))
                    flight_level = self.get_param(robot_id, 'flight_level')
                    go_home_path = []
                    current_at_flight_level = copy.deepcopy(self.robots[robot_id].pose)
                    current_at_flight_level.pose.position.z = flight_level
                    home_at_flight_level = copy.deepcopy(self.robots[robot_id].home)
                    home_at_flight_level.pose.position.z = flight_level
                    go_home_path.append(current_at_flight_level)
                    go_home_path.append(home_at_flight_level)

                    userdata = smach.UserData()
                    userdata.path = go_home_path
                    self.task_manager.start_task(robot_id, FollowPath(), userdata)

            if set(self.available_robots).issubset(finished_robots):
                print('All done!')
                break

def main():
    rospy.init_node('central_unit_c3')

    while rospy.get_rostime() == rospy.Time():
        rospy.logwarn("Waiting for (sim) time to begin!")
        rospy.sleep(1)

    central_unit = CentralUnit()
    # rospy.sleep(3)

    central_unit.take_off()
    central_unit.look_for_piles() # TODO: if not piles[r, g, b, o], repeat!?
    central_unit.build_wall()

    rospy.spin()

if __name__ == '__main__':
    main()
