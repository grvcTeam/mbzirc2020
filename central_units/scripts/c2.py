#!/usr/bin/env python

# PARAMETER	SPECIFICATION
# Number of UAVs per team	Maximum of 3
# Number of UGVs per team	1
# Arena size	50mx60mx20m
# Brick shapes and material	Rectangular cube, Styrofoam material
# Bricks size (Red, Green, Blue)	Approximately 0.30mx0.20mx0.20m, 0.60mx0.20mx0.20m and1.20x0.20x0.20m
# Bricks size (Orange)	1.80x0.20x0.20 m
# Weight of bricks	O <= 2.0kg , B <= 1.5kg , G <= 1kg , R <= 1kg,
# Brick gripping mechanism	Primarily magnetic, but other gripping mechanisms could be used
# Environment	Outdoor
# Mode of operation	Autonomous; manual allowed but penalized
# RTK/DGPS	Allowed but penalized
# Challenge duration	30 minutes
# Communications	TBD

import math
import copy
import threading
import rospy
import smach
import tf2_ros
import tf2_geometry_msgs
import mbzirc_comm_objs.msg as msg
import mbzirc_comm_objs.srv as srv

from geometry_msgs.msg import PoseStamped, Point, Vector3
from tasks.timing import SleepAndRetry
from tasks.regions import AskForRegionToHover, AskForRegionToMove
from tasks.move import TakeOff, FollowPath
from tasks.wall import PickAndPlace
from tasks.search import all_piles_are_found
from utils.translate import color_from_int
from utils.manager import TaskManager

class RobotProxy(object):
    def __init__(self, robot_id):
        self.id = robot_id
        self.url = 'mbzirc2020_' + self.id + '/'  # TODO: Impose ns: mbzirc2020!?
        # TODO: Unifying robot_model and namespace might be an issue for non homogeneous teams, 
        # but it is somehow forced by the way sensor topics are named in gazebo simulation (mbzirc2020)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pose = PoseStamped()
        rospy.Subscriber(self.url + 'data_feed', msg.RobotDataFeed, self.data_feed_callback)
        # time.sleep(3)  # TODO: allow messages to get in? wait for pose?
        self.home = PoseStamped()

    def set_home(self):
        self.home = copy.deepcopy(self.pose)

    def data_feed_callback(self, data):
        self.pose = data.pose

    # TODO: auto update with changes in self.pose? Not here?
    def build_request_for_go_to_region(self, final_pose, label = 'go_to', radius = 1.0):
        initial_pose = copy.deepcopy(self.pose)
        try:
            if initial_pose.header.frame_id != 'arena':
                initial_pose = self.tf_buffer.transform(initial_pose, 'arena', rospy.Duration(1.0))
            if final_pose.header.frame_id != 'arena':
                final_pose = self.tf_buffer.transform(final_pose, 'arena', rospy.Duration(1.0))
        except:
            rospy.logerr('Failed to transform points to [{}], ignoring!'.format('arena'))

        request = srv.AskForRegionRequest()
        request.agent_id = self.id
        request.label = label
        request.min_corner.header.frame_id = 'arena'
        request.min_corner.point.x = min(initial_pose.pose.position.x - radius, final_pose.pose.position.x - radius)
        request.min_corner.point.y = min(initial_pose.pose.position.y - radius, final_pose.pose.position.y - radius)
        request.min_corner.point.z = min(initial_pose.pose.position.z - radius, final_pose.pose.position.z - radius)
        request.max_corner.header.frame_id = 'arena'
        request.max_corner.point.x = max(initial_pose.pose.position.x + radius, final_pose.pose.position.x + radius)
        request.max_corner.point.y = max(initial_pose.pose.position.y + radius, final_pose.pose.position.y + radius)
        request.max_corner.point.z = max(initial_pose.pose.position.z + radius, final_pose.pose.position.z + radius)

        return request

    def build_request_for_vertical_region(self, center, label = 'vertical', radius = 1.0, z_min = 0, z_max = 25):  # TODO: max_z parameter?
        center_pose = copy.deepcopy(center)
        try:
            if center_pose.header.frame_id != 'arena':
                center_pose = self.tf_buffer.transform(center_pose, 'arena', rospy.Duration(1.0))
        except:
            rospy.logerr('Failed to transform points to [{}], ignoring!'.format('arena'))

        request = srv.AskForRegionRequest()
        request.agent_id = self.id
        request.label = label
        request.min_corner.header.frame_id = 'arena'
        request.min_corner.point.x = center_pose.pose.position.x - radius
        request.min_corner.point.y = center_pose.pose.position.y - radius
        request.min_corner.point.z = z_min
        request.max_corner.header.frame_id = 'arena'
        request.max_corner.point.x = center_pose.pose.position.x + radius
        request.max_corner.point.y = center_pose.pose.position.y + radius
        request.max_corner.point.z = z_max

        return request

    # TODO: Force raw points with no frame_id?
    def get_cost_to_go_to(self, waypoint_in):
        waypoint = copy.deepcopy(waypoint_in)
        # TODO: these try/except inside a function?
        try:
            waypoint = self.tf_buffer.transform(waypoint, self.pose.header.frame_id, rospy.Duration(1.0))  # TODO: check from/to equality
        except:
            rospy.logerr('Failed to transform waypoint from [{}] to [{}]'.format(waypoint.header.frame_id, self.pose.header.frame_id))

        delta_x = waypoint.pose.position.x - self.pose.pose.position.x
        delta_y = waypoint.pose.position.y - self.pose.pose.position.y
        delta_z = waypoint.pose.position.z - self.pose.pose.position.z
        manhattan_distance = abs(delta_x) + abs(delta_y) + abs(delta_z)
        return manhattan_distance

# TODO: All these parameters from config!
field_width = 20  # 60  # TODO: Field is 60 x 50
field_height = 20  # 50  # TODO: Field is 60 x 50
column_count = 4  # 6  # TODO: as a function of fov

brick_scales = {}
# TODO: use enums for colors instead of strings
brick_scales['red'] = Vector3(x = 0.3, y = 0.2, z = 0.2)  # TODO: from config file?
brick_scales['green'] = Vector3(x = 0.6, y = 0.2, z = 0.2)  # TODO: from config file?
brick_scales['blue'] = Vector3(x = 1.2, y = 0.2, z = 0.2)  # TODO: from config file?
brick_scales['orange'] = Vector3(x = 1.8, y = 0.2, z = 0.2)  # TODO: from config file?

# TODO: from especification, assume x-z layout
wall_blueprint = [['red', 'red']]  #, ['green', 'red']]  # , 'blue', 'orange']]  #, ['orange', 'blue', 'green', 'red']]

# TODO: move to path utils
def generate_area_path(width, height, column_count, z = 3.0):
    spacing = 0.5 * width / column_count
    y_min = spacing
    y_max = height - spacing

    path = []
    for i in range(column_count):
        x_column = spacing * (1 + 2*i)
        if i % 2:
            path.append(Point(x = x_column, y = y_max, z = z))
            path.append(Point(x = x_column, y = y_min, z = z))
        else:
            path.append(Point(x = x_column, y = y_min, z = z))
            path.append(Point(x = x_column, y = y_max, z = z))

    return path

# TODO: move to path utils
def print_path(path):
    print('path of lenght {}: ['.format(len(path)))
    for point in path:
        print('[{}, {}, {}]'.format(point.x, point.y, point.z))
    print(']')

# TODO: move to path utils
def generate_uav_paths(uav_count):
    if uav_count <= 0:
        return []

    area_path = generate_area_path(field_width, field_height, column_count)
    point_count = len(area_path)
    delta = int(math.ceil(point_count / float(uav_count)))
    paths = []
    for i in range(uav_count):
        j_min = delta * i
        j_max = delta * (i+1)
        paths.append(area_path[j_min:j_max])
    return paths

# TODO: move to path utils
def set_z(path, z):
    for point in path:
        point.z = z
    return path

# TODO: move to wall utils?
class BrickInWall(object):
    def __init__(self, color, position):
        self.color = color
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'wall'  # Defined by a static tf publisher
        self.pose.pose.position = position
        self.pose.pose.orientation.w = 1  # Assume wall is x-oriented

    def __repr__(self):
        return '[color = {}, pose = [{}: ({},{},{}) ({},{},{},{})]]'.format(self.color, self.pose.header.frame_id, 
                self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z, 
                self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w)

# TODO: move to wall utils?
def get_build_wall_sequence(wall_blueprint):
    buid_wall_sequence = []
    current_z = 0.0
    for brick_row in wall_blueprint:
        current_x = 0.0
        build_row_sequence = []
        for brick_color in brick_row:
            brick_position = Point()
            brick_position.x = current_x + 0.5 * brick_scales[brick_color].x
            brick_position.y = 0.5 * brick_scales[brick_color].y
            brick_position.z = current_z + 0.5 * brick_scales[brick_color].z
            current_x += brick_scales[brick_color].x

            brick_in_wall = BrickInWall(color = brick_color, position = brick_position)
            build_row_sequence.append(brick_in_wall)

        buid_wall_sequence.append(build_row_sequence)
        current_z += brick_scales['red'].z  # As all bricks (should) have the same height
    return buid_wall_sequence

class CentralUnit(object):
    def __init__(self):
        self.available_robots = ['1', '2'] # Force id to be a string to avoid index confussion  # TODO: auto discovery (and update!)

        self.robots = {}
        for robot_id in self.available_robots:
            self.robots[robot_id] = RobotProxy(robot_id)

        self.piles = {}
        rospy.Subscriber("estimated_objects", msg.ObjectDetectionList, self.estimation_callback)

        self.task_manager = TaskManager(self.robots)

    def get_param(self, robot_id, param_name):
        # TODO: Default value in case param_name is not found?
        return rospy.get_param(self.robots[robot_id].url + param_name)

    # TODO: This is repeated in SearchPilesTask
    def estimation_callback(self, data):
        for pile in data.objects:
            # TODO: check type and scale?
            color = color_from_int(pile.color)
            pose = PoseStamped()
            pose.header = pile.header
            pose.pose = pile.pose.pose
            self.piles[color] = pose

    # TODO: Could be a smach.State (for all or for every single uav)
    def take_off(self):
        for robot_id in self.available_robots:
            # TODO: clear all regions?
            userdata = smach.UserData()
            userdata.height = self.get_param(robot_id, 'flight_level')  # TODO: Why not directly inside tasks?
            self.task_manager.start_task(robot_id, TakeOff(), userdata)
            self.task_manager.wait_for([robot_id])  # Sequential takeoff

    # TODO: Could be a smach.State (for all or for every single uav)
    def look_for_piles(self):
        # TODO: Check this better outside!?
        if all_piles_are_found(self.piles):
            rospy.logwarn('All piles are found!')
            return
        robot_paths = {}
        point_paths = generate_uav_paths(len(self.available_robots))
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
            print('sending goal to search_piles server {}'.format(robot_id))
            userdata = smach.UserData()
            userdata.path = robot_paths[robot_id]
            self.task_manager.start_task(robot_id, FollowPath(), userdata)

        while not all_piles_are_found(self.piles) and not rospy.is_shutdown():
            # TODO: What happens if all piles are NEVER found?
            rospy.logwarn('len(self.piles) = {}'.format(len(self.piles)))
            rospy.sleep(1.0)

        for robot_id in self.available_robots:
            rospy.logwarn('preempting {}'.format(robot_id))
            self.task_manager.preempt_task(robot_id)

    # TODO: Could be a smach.State (for all or for every single uav, not so easy!)
    def build_wall(self):
        piles = copy.deepcopy(self.piles)  # Cache piles
        build_wall_sequence = get_build_wall_sequence(wall_blueprint)
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
    rospy.init_node('central_unit_c2')

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
