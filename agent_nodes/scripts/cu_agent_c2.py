#!/usr/bin/env python

import rospy
# import smach
import actionlib
import mbzirc_comm_objs.msg
from geometry_msgs.msg import PoseStamped, Point
import math

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

# def main():
#     paths = generate_uav_paths(2)
#     for path in paths:
#         print_path(path)

def main():
    rospy.init_node('cu_agent_c2')
    available_uavs = [1, 2]  # TODO: auto discovery (and update!)

    uav_clients = {}
    for i in range(1,4):
        if i in available_uavs:
            uav_id = str(i)  # TODO: force id to be a string to avoid index confussion?
            uav_clients[uav_id] = actionlib.SimpleActionClient('/uav_' + uav_id + '/task/follow_path', mbzirc_comm_objs.msg.FollowPathAction)

    for uav_id in uav_clients:
        print('waiting for server {}'.format(uav_id))
        uav_clients[uav_id].wait_for_server()  # TODO: Timeout!

    uav_params = {}
    for uav_id in uav_clients:
        uav_params[uav_id] = {}
        namespace = 'uav_' + uav_id + '/agent_node/'
        uav_params[uav_id]['flight_level'] = rospy.get_param(namespace + 'flight_level')

    uav_paths = {}
    point_paths = generate_uav_paths(len(available_uavs))
    for i, uav_id in enumerate(available_uavs):
        uav_path = mbzirc_comm_objs.msg.FollowPathGoal()
        flight_level = uav_params[str(uav_id)]['flight_level']
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
        uav_paths[str(uav_id)] = uav_path

    # for uav_id in uav_paths:
    #     print(uav_paths[uav_id])

    for uav_id in uav_clients:
        print('sending goal to server {}'.format(uav_id))
        uav_clients[uav_id].send_goal(uav_paths[uav_id])

    for uav_id in uav_clients:
        print('waiting result of server {}'.format(uav_id))
        uav_clients[uav_id].wait_for_result()
        print(uav_clients[uav_id].get_result())

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
