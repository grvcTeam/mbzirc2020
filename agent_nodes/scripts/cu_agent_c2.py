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
import actionlib
import mbzirc_comm_objs.msg
from geometry_msgs.msg import PoseStamped, Point, Vector3
from mbzirc_comm_objs.msg import ObjectDetectionList, AgentDataFeed
from mbzirc_comm_objs.srv import GetCostToGoTo
import math
import json
import copy

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
wall_blueprint = [['red', 'green'], ['green', 'red']]  # , 'blue', 'orange']]  #, ['orange', 'blue', 'green', 'red']]

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

class Agent(object):
    def __init__(self):
        self.available_uavs = ['1', '2'] # Force id to be a string to avoid index confussion  # TODO: auto discovery (and update!)

        # TODO: Unifying robot_model and namespace might be an issue for non homogeneous teams, 
        # but it is somehow forced by the way sensor topics are named in gazebo simulation (mbzirc2020)
        uavs_ns = {}
        for uav_id in self.available_uavs:
            uavs_ns[uav_id] = 'mbzirc2020' + '_' + uav_id

        self.uav_params = {}
        self.uav_clients = {}  # TODO: is this AgentInterface?
        self.uav_subscribers = {}  # TODO: is this AgentInterface?
        self.uav_data_feeds = {}   # TODO: is this AgentInterface?

        for uav_id in self.available_uavs:
            self.uav_clients[uav_id] = {}
            self.uav_subscribers[uav_id] = {}
            self.uav_clients[uav_id]['take_off'] = actionlib.SimpleActionClient(uavs_ns[uav_id] + '/task/take_off', mbzirc_comm_objs.msg.TakeOffAction)
            self.uav_clients[uav_id]['follow_path'] = actionlib.SimpleActionClient(uavs_ns[uav_id] + '/task/follow_path', mbzirc_comm_objs.msg.FollowPathAction)
            self.uav_clients[uav_id]['pick_and_place'] = actionlib.SimpleActionClient(uavs_ns[uav_id] + '/task/pick_and_place', mbzirc_comm_objs.msg.PickAndPlaceAction)
            self.uav_clients[uav_id]['go_home'] = actionlib.SimpleActionClient(uavs_ns[uav_id] + '/task/go_home', mbzirc_comm_objs.msg.GoHomeAction)
            self.uav_clients[uav_id]['get_cost_to_go_to'] = rospy.ServiceProxy(uavs_ns[uav_id] + '/get_cost_to_go_to', GetCostToGoTo)
            self.uav_subscribers[uav_id]['data_feed'] = rospy.Subscriber(uavs_ns[uav_id] + '/data_feed', AgentDataFeed, self.data_feed_callback, callback_args = uav_id)

            print('waiting for servers of agent [{}]'.format(uav_id))
            self.uav_clients[uav_id]['take_off'].wait_for_server()  # TODO: Timeout!
            self.uav_clients[uav_id]['follow_path'].wait_for_server()  # TODO: Timeout!
            self.uav_clients[uav_id]['pick_and_place'].wait_for_server()  # TODO: Timeout!
            self.uav_clients[uav_id]['go_home'].wait_for_server()  # TODO: Timeout!
            rospy.wait_for_service(uavs_ns[uav_id] + '/get_cost_to_go_to')  # TODO: Timeout!

            self.uav_params[uav_id] = {}
            agent_node_ns = uavs_ns[uav_id] + '/agent_node/'
            self.uav_params[uav_id]['flight_level'] = rospy.get_param(agent_node_ns + 'flight_level')  # TODO: Needed here or leave uav alone?

        self.piles = {}
        rospy.Subscriber("estimated_objects", ObjectDetectionList, self.estimation_callback)

    def data_feed_callback(self, data, uav_id):
        self.uav_data_feeds[uav_id] = data

    def estimation_callback(self, data):
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
                self.piles[color] = pose

    # TODO: Could be a smach.State (for all or for every single uav)
    def take_off(self):
        for uav_id in self.available_uavs:
            print('sending goal to take_off server {}'.format(uav_id))
            self.uav_clients[uav_id]['take_off'].send_goal(mbzirc_comm_objs.msg.TakeOffGoal(height = self.uav_params[uav_id]['flight_level']))

        for uav_id in self.available_uavs:
            print('waiting result of take_off server [{}]'.format(uav_id))
            self.uav_clients[uav_id]['take_off'].wait_for_result()
            print(self.uav_clients[uav_id]['take_off'].get_result())

    # TODO: Could be a smach.State (for all or for every single uav)
    def look_for_piles(self):
        uav_paths = {}
        point_paths = generate_uav_paths(len(self.available_uavs))
        for i, uav_id in enumerate(self.available_uavs):
            uav_path = mbzirc_comm_objs.msg.FollowPathGoal()
            flight_level = self.uav_params[uav_id]['flight_level']
            point_path = set_z(point_paths[i], flight_level)
            for point in point_path:
                waypoint = PoseStamped()
                waypoint.header.frame_id = 'arena'
                waypoint.pose.position = point
                waypoint.pose.orientation.z = 0
                waypoint.pose.orientation.w = 1  # TODO: other orientation?
                uav_path.path.append(waypoint)
            uav_paths[uav_id] = uav_path

        for uav_id in self.available_uavs:
            print('sending goal to follow_path server {}'.format(uav_id))
            self.uav_clients[uav_id]['follow_path'].send_goal(uav_paths[uav_id])

        for uav_id in self.available_uavs:
            print('waiting result of follow_path server [{}]'.format(uav_id))
            self.uav_clients[uav_id]['follow_path'].wait_for_result()
            print(self.uav_clients[uav_id]['follow_path'].get_result())

    # TODO: Could be a smach.State (for all or for every single uav, not so easy!)
    def build_wall(self):
        rospy.sleep(0.5)  # TODO: some sleep to allow data_feed update
        piles = copy.deepcopy(self.piles)  # Cache piles
        build_wall_sequence = get_build_wall_sequence(wall_blueprint)
        for i, row in enumerate(build_wall_sequence):
            for brick in row:
                print('row[{}] brick = {}'.format(i, brick))
                costs = {}
                while not costs:
                    for uav_id in self.available_uavs:
                        if self.uav_data_feeds[uav_id].is_idle:
                            costs[uav_id] = self.uav_clients[uav_id]['get_cost_to_go_to'](piles[brick.color]).cost
                        else:
                            rospy.sleep(0.5)
                min_cost_uav_id = min(costs, key = costs.get)
                print('costs: {}, min_cost_id: {}'.format(costs, min_cost_uav_id))
                goal = mbzirc_comm_objs.msg.PickAndPlaceGoal()
                goal.pile_pose = piles[brick.color]
                goal.brick_in_wall_pose = brick.pose
                self.uav_clients[min_cost_uav_id]['pick_and_place'].send_goal(goal)
                rospy.sleep(0.5)  # TODO: some sleep to allow data_feed update
        # Once arrived here, last pick_and_place task has been allocated
        print('All pick_and_place tasks allocated')
        finished_uavs = []
        while True:
            rospy.sleep(0.5)  # TODO: some sleep here?
            for uav_id in self.available_uavs:
                if self.uav_data_feeds[uav_id].is_idle and (uav_id not in finished_uavs):
                    finished_uavs.append(uav_id)
                    print('waiting result of pick_and_place server [{}]'.format(uav_id))
                    self.uav_clients[uav_id]['pick_and_place'].wait_for_result()
                    print(self.uav_clients[uav_id]['pick_and_place'].get_result())
                    print('now go home!')
                    goal = mbzirc_comm_objs.msg.GoHomeGoal()
                    goal.do_land = True
                    self.uav_clients[uav_id]['go_home'].send_goal(goal)
            if set(self.available_uavs).issubset(finished_uavs):
                print('All done!')
                break

        for uav_id in self.available_uavs:
            print('waiting result of go_home server [{}]'.format(uav_id))
            self.uav_clients[uav_id]['go_home'].wait_for_result()
            print(self.uav_clients[uav_id]['go_home'].get_result())

def main():
    rospy.init_node('cu_agent_c2')

    central_agent = Agent()
    # rospy.sleep(3)

    central_agent.take_off()
    central_agent.look_for_piles() # TODO: if not piles[r, g, b, o], repeat! if all found, stop searching?
    central_agent.build_wall()

if __name__ == '__main__':
    main()
