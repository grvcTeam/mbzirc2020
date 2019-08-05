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

import rospy
# import smach
import actionlib
import mbzirc_comm_objs.msg
from geometry_msgs.msg import PoseStamped, Point, Vector3
from mbzirc_comm_objs.msg import ObjectDetectionList, AgentDataFeed
from mbzirc_comm_objs.srv import GetCostToGoTo
import math
import json

def generate_area_path(width, height, column_count, z = 3.0):
    spacing = 0.5 * width / column_count
    # print('spacing = {}'.format(spacing))
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

def print_path(path):
    print('path of lenght {}: ['.format(len(path)))
    for point in path:
        print('[{}, {}, {}]'.format(point.x, point.y, point.z))
    print(']')

# TODO: All these parameters from config!
field_width = 20  # 60  # TODO: Field is 60 x 50
field_height = 20  # 50  # TODO: Field is 60 x 50
column_count = 4  # 6  # TODO: as a function of fov
def generate_uav_paths(uav_count):
    if uav_count <= 0:
        return []

    area_path = generate_area_path(field_width, field_height, column_count)
    point_count = len(area_path)
    delta = int(math.ceil(point_count / float(uav_count)))
    # print('point_count = {}'.format(point_count))
    # print('delta = {}'.format(delta))
    paths = []
    for i in range(uav_count):
        j_min = delta * i
        j_max = delta * (i+1)
        paths.append(area_path[j_min:j_max])
    return paths

def set_z(path, z):
    for point in path:
        point.z = z
    return path


# TODO: Unifying robot_model and ns might be an issue for non homogeneous teams, 
# but it is somehow forced by the way sensor topics are named in gazebo simulation

piles = {}  # TODO: globals!
def estimation_callback(data):
    global piles
    # pile_list =  ObjectDetectionList()
    for pile in data.objects:
        # TODO: check type and scale?
        properties_dict = {}
        if pile.properties:
            properties_dict = json.loads(pile.properties)
        if 'color' in properties_dict:
            color = properties_dict['color']  # TODO: reused code!
            pose = PoseStamped()
            pose.header = pile.header
            pose.pose = pile.pose.pose
            piles[color] = pose

# TODO: use enums instead of strings
brick_scales = {}
brick_scales['red'] = Vector3(x = 0.3, y = 0.2, z = 0.2)  # TODO: from config file?
brick_scales['green'] = Vector3(x = 0.6, y = 0.2, z = 0.2)  # TODO: from config file?
brick_scales['blue'] = Vector3(x = 1.2, y = 0.2, z = 0.2)  # TODO: from config file?
brick_scales['orange'] = Vector3(x = 1.8, y = 0.2, z = 0.2)  # TODO: from config file?

# TODO: from especification, assume x-z layout
wall_blueprint = [['red', 'green', 'blue', 'orange'], ['orange', 'blue', 'green', 'red']]

class BrickInWall(object):
    def __init__(self, color, position):
        self.color = color
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'wall'  # TODO: define this frame
        self.pose.pose.position = position
        self.pose.pose.orientation.w = 1  # Assume wall is x-oriented

    def __repr__(self):
        return '[color = {}, pose = [{}: ({},{},{}) ({},{},{},{})]]'.format(self.color, self.pose.header.frame_id, 
                self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z, 
                self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w)

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


# def main():
#     buid_wall_sequence = get_build_wall_sequence(wall_blueprint)
#     for i, row in enumerate(buid_wall_sequence):
#         for brick in row:
#             print('row[{}].pose = {}'.format(i, brick))


def main():
    rospy.init_node('cu_agent_c2')
    agents_ns = 'mbzirc2020'  # TODO: As a parameter
    available_uavs = ['1', '2'] # Force id to be a string to avoid index confussion  # TODO: auto discovery (and update!)

    uavs_ns = {}
    for uav_id in available_uavs:
        uavs_ns[uav_id] = agents_ns + '_' + uav_id

    rospy.Subscriber("estimated_objects", ObjectDetectionList, estimation_callback)

    uav_params = {}
    uav_clients = {}  # TODO: is this AgentInterface?
    uav_subscribers = {}  # TODO: is this AgentInterface?
    uav_data_feeds = {}   # TODO: is this AgentInterface?
    def data_feed_callback(data, uav_id):
        uav_data_feeds[uav_id] = data
        print('uav_data_feeds[{}].is_idle = {}'.format(uav_id, data.is_idle))

    for uav_id in available_uavs:
        uav_clients[uav_id] = {}
        uav_subscribers[uav_id] = {}
        uav_clients[uav_id]['follow_path'] = actionlib.SimpleActionClient(uavs_ns[uav_id] + '/task/follow_path', mbzirc_comm_objs.msg.FollowPathAction)
        uav_clients[uav_id]['get_cost_to_go_to'] = rospy.ServiceProxy(uavs_ns[uav_id] + '/get_cost_to_go_to', GetCostToGoTo)
        uav_subscribers[uav_id]['data_feed'] = rospy.Subscriber(uavs_ns[uav_id] + '/data_feed', AgentDataFeed, data_feed_callback, callback_args = uav_id)
        # uav_data_feeds[uav_id] = AgentDataFeed()  # This makes it UNINITIALIZED TODO: needed?

        print('waiting for server {}'.format(uav_id))
        uav_clients[uav_id]['follow_path'].wait_for_server()  # TODO: Timeout!
        rospy.wait_for_service(uavs_ns[uav_id] + '/get_cost_to_go_to')  # TODO: Timeout!

        uav_params[uav_id] = {}
        agent_node_ns = uavs_ns[uav_id] + '/agent_node/'
        uav_params[uav_id]['flight_level'] = rospy.get_param(agent_node_ns + 'flight_level')

    uav_paths = {}
    point_paths = generate_uav_paths(len(available_uavs))
    for i, uav_id in enumerate(available_uavs):
        uav_path = mbzirc_comm_objs.msg.FollowPathGoal()
        flight_level = uav_params[uav_id]['flight_level']
        point_path = set_z(point_paths[i], flight_level)
        # print_path(point_path)
        for point in point_path:
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'arena'  # TODO: other frame_id?
            waypoint.pose.position = point
            waypoint.pose.orientation.z = 0
            waypoint.pose.orientation.w = 1  # TODO: other orientation?
            uav_path.path.append(waypoint)
            # print(waypoint)
        uav_paths[uav_id] = uav_path

    # for uav_id in uav_paths:
    #     print(uav_paths[uav_id])

    for uav_id in available_uavs:
        print('sending goal to server {}'.format(uav_id))
        uav_clients[uav_id]['follow_path'].send_goal(uav_paths[uav_id])

    for uav_id in available_uavs:
        print('waiting result of server {}'.format(uav_id))
        uav_clients[uav_id]['follow_path'].wait_for_result()
        print(uav_clients[uav_id]['follow_path'].get_result())

    rospy.sleep(0.5)  # TODO: some sleep to allow data_feed update
    # print(piles)  # TODO: cache it? if not piles[r, g, b, o], repeat!!
    buid_wall_sequence = get_build_wall_sequence(wall_blueprint)
    for i, row in enumerate(buid_wall_sequence):
        for brick in row:
            print('row[{}] brick = {}'.format(i, brick))
            costs = {}
            for uav_id in available_uavs:
                if uav_data_feeds[uav_id].is_idle:
                    costs[uav_id] = uav_clients[uav_id]['get_cost_to_go_to'](piles[brick.color]).cost
            # for uav_id in costs:
            #     print('uav[{}] cost to go to the {} pile: {}'.format(uav_id, brick.color, costs[uav_id]))
            print(costs)
            if costs:
                min_cost_uav_id = min(costs, key = costs.get)
                print(min_cost_uav_id)
                # uav_clients[min_cost_uav_id]['pick_and_place'].send_goal(pick_there, place_there)
            # TODO: Some sleep here?


if __name__ == '__main__':
    main()


# import roslib
# import rospy
# import smach
# import smach_ros

# from utils.agent import *
# import tasks.central_unit_tasks.Idle
# import tasks.central_unit_tasks.SearchForBrickPiles
# import tasks.central_unit_tasks.CatchBalloons
# import tasks.central_unit_tasks.SearchAndCatch
# import tasks.central_unit_tasks.SearchAndBuild

# def main():

#     rospy.init_node('uav_agent')

#     # params.
#     id = 'central_unit'#rospy.get_param('~agent_id')

#     # instantiate agent structures
#     fsm = AgentStateMachine()
#     iface = AgentInterface(id,fsm,{'type':'central_unit'})

#     # create tasks
#     default_task = tasks.central_unit_tasks.Idle.Task('idle',iface)
#     tasks_dic = {}
#     add_task('search_env', tasks_dic, iface, tasks.central_unit_tasks.SearchForBrickPiles, [])
#     add_task('catch_balloons', tasks_dic, iface, tasks.central_unit_tasks.CatchBalloons, [])
#     add_task('search_and_build', tasks_dic, iface, tasks.central_unit_tasks.SearchAndBuild, [])
#     add_task('search_and_catch', tasks_dic, iface, tasks.central_unit_tasks.SearchAndCatch, [])

#     # initialize state machine
#     d_dic = {'idle': default_task}
#     fsm.initialize(id, d_dic, 'idle', tasks_dic)

#     # execute state machine
#     userdata = smach.UserData()
#     fsm.execute(userdata)

# if __name__ == '__main__':
#     main()
