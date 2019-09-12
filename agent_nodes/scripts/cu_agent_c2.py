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
import random
import json
import copy
import time
import rospy
import smach
import smach_ros
import actionlib
import tf2_ros
import tf2_geometry_msgs
import mbzirc_comm_objs.msg
import mbzirc_comm_objs.srv
import ual_action_server.msg

from geometry_msgs.msg import PoseStamped, Point, Vector3

# TODO: uav_agent should not use any implicit centralized information? (params!, region_management!, costs?) as communication is not granted!
class RobotInterface(object):
    def __init__(self, robot_id):
        self.id = robot_id
        self.url = 'mbzirc2020_' + self.id + '/'  # TODO: Impose ns: mbzirc2020!?
        # TODO: Unifying robot_model and namespace might be an issue for non homogeneous teams, 
        # but it is somehow forced by the way sensor topics are named in gazebo simulation (mbzirc2020)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pose = PoseStamped()
        rospy.Subscriber(self.url + 'ual/pose', PoseStamped, self.pose_callback)  # TODO: valid for uav (move to robot_(data)_feed?)
        # time.sleep(3)  # TODO: allow messages to get in? wait for pose?

        self.params = {}
        self.params['flight_level'] = rospy.get_param(self.url + 'flight_level')  # TODO: Needed here or leave uav alone?

        self.home = PoseStamped()

    def set_home(self):
        self.home = copy.deepcopy(self.pose)

    def pose_callback(self, data):
        self.pose = data

    # TODO: auto update with changes in self.pose?
    def build_ask_for_region_request(self, final_pose, radius = 1.0):
        initial_pose = copy.deepcopy(self.pose)
        try:
            if initial_pose.header.frame_id != 'arena':
                initial_pose = self.tf_buffer.transform(initial_pose, 'arena', rospy.Duration(1.0))
            if final_pose.header.frame_id != 'arena':
                final_pose = self.tf_buffer.transform(final_pose, 'arena', rospy.Duration(1.0))
        except:
            rospy.logerr('Failed to transform points to [{}], ignoring!'.format('arena'))

        request = mbzirc_comm_objs.srv.AskForRegionRequest()
        request.agent_id = int(self.id)  # TODO: Make id ALWAYS a string (modify AskForRegion)
        request.min_corner.header.frame_id = 'arena'
        request.min_corner.point.x = min(initial_pose.pose.position.x - radius, final_pose.pose.position.x - radius)
        request.min_corner.point.y = min(initial_pose.pose.position.y - radius, final_pose.pose.position.y - radius)
        request.min_corner.point.z = min(initial_pose.pose.position.z - radius, final_pose.pose.position.z - radius)
        request.max_corner.header.frame_id = 'arena'
        request.max_corner.point.x = max(initial_pose.pose.position.x + radius, final_pose.pose.position.x + radius)
        request.max_corner.point.y = max(initial_pose.pose.position.y + radius, final_pose.pose.position.y + radius)
        request.max_corner.point.z = max(initial_pose.pose.position.z + radius, final_pose.pose.position.z + radius)

        return request

    def build_ask_for_region_request_to_land(self, radius = 1.0):
        final_pose = copy.deepcopy(self.pose)
        final_pose.pose.position.z = 0
        return self.build_ask_for_region_request(final_pose, radius)

    # TODO: Force raw points with no frame_id?
    def get_cost_to_go_to(self, waypoint):
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

# TODO: Get Task out of naming (or invert it!), should all classes be State Machines?
class Sleep(smach.State):
    def __init__(self, duration = 3.0):
        smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'])  # TODO: duration as an input_key?
        self.duration = duration

    def execute(self, userdata):
        rospy.sleep(self.duration)
        return 'succeeded'
        # TODO: aborted?
        # TODO: preempted? Sleep in shorter period chunks to allow preemption?

#TODO: ask for region first? May block others from taking off... Better define a fixed take off sequnce?
class TakeOffTask(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['height'])

        with self:

            def take_off_goal_callback(userdata, default_goal):
                robot.set_home()  # TODO: better do it explicitly BEFORE take off?
                goal = ual_action_server.msg.TakeOffGoal(height = userdata.height)
                return goal

            smach.StateMachine.add('TAKE_OFF', smach_ros.SimpleActionState(robot.url + 'take_off_action', ual_action_server.msg.TakeOffAction,
                                    input_keys = ['height'],
                                    goal_cb = take_off_goal_callback),
                                    transitions = {'succeeded': 'succeeded', 'aborted': 'SLEEP_AND_RETRY'})

            smach.StateMachine.add('SLEEP_AND_RETRY', Sleep(3.0),
                                    transitions = {'succeeded': 'TAKE_OFF'})

class GoToTask(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['waypoint'])

        with self:

            # TODO: decorators?
            def ask_for_region_request_callback(userdata, request):
                ask_for_region_request = robot.build_ask_for_region_request(userdata.waypoint)
                return ask_for_region_request

            def ask_for_region_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('ASK_FOR_REGION_TO_MOVE', smach_ros.ServiceState('ask_for_region', mbzirc_comm_objs.srv.AskForRegion,
                                    input_keys = ['waypoint'],
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'GO_TO', 'aborted': 'SLEEP_AND_RETRY_ASKING'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING', Sleep(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_MOVE'})

            def go_to_goal_callback(userdata, default_goal):
                goal = ual_action_server.msg.GoToGoal(waypoint = userdata.waypoint)
                return goal

            smach.StateMachine.add('GO_TO', smach_ros.SimpleActionState(robot.url + 'go_to_action', ual_action_server.msg.GoToAction,
                                    input_keys = ['waypoint'],
                                    goal_cb = go_to_goal_callback),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_HOVER'})

            smach.StateMachine.add('ASK_FOR_REGION_TO_HOVER', smach_ros.ServiceState('ask_for_region', mbzirc_comm_objs.srv.AskForRegion,
                                    input_keys = ['waypoint'],
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'succeeded'})

class WaypointDispatch(smach.State):
    def __init__(self, robot):  # TODO: pass a GoToTask object instead?
        smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['path'])
        self.go_to_task = GoToTask(robot)

    def execute(self, userdata):
        for waypoint in userdata.path:
            child_userdata = smach.UserData()
            child_userdata.waypoint = waypoint
            self.go_to_task.execute(child_userdata)
        return 'succeeded'
        # TODO: aborted?
        # TODO: preempted?

class FollowPathTask(smach.StateMachine):  # TODO: pass a WaypointDispatch object instead?
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['path'])

        with self:

            smach.StateMachine.add('DISPATCH', WaypointDispatch(robot),
                                    remapping = {'path': 'path'},
                                    transitions = {'succeeded': 'succeeded'})

class PickAndPlaceTask(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['above_pile_pose', 'above_wall_pose', 'in_wall_brick_pose'])

        with self:

            smach.StateMachine.add('GO_TO_PILE', GoToTask(robot),
                                    remapping = {'waypoint': 'above_pile_pose'},
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_PICK'})

            def ask_for_region_request_callback(userdata, request):
                radius = 5.0  # TODO: Tune, assure uav is not going further while pick!
                in_floor_pile_pose = copy.deepcopy(userdata.above_pile_pose)
                in_floor_pile_pose.pose.position.z = 0
                request = robot.build_ask_for_region_request(in_floor_pile_pose, radius)
                return request

            def ask_for_region_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('ASK_FOR_REGION_TO_PICK', smach_ros.ServiceState('ask_for_region', mbzirc_comm_objs.srv.AskForRegion,
                                    input_keys = ['above_pile_pose'],
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'PICK', 'aborted': 'SLEEP_AND_RETRY_ASKING'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING', Sleep(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_PICK'})

            def pick_goal_callback(userdata, default_goal):
                goal = ual_action_server.msg.PickGoal(approximate_pose = userdata.above_pile_pose)
                return goal

            smach.StateMachine.add('PICK', smach_ros.SimpleActionState(robot.url + 'pick_action', ual_action_server.msg.PickAction,
                                    input_keys = ['above_pile_pose'],
                                    goal_cb = pick_goal_callback),
                                    transitions = {'succeeded': 'GO_UP'})

            smach.StateMachine.add('GO_UP', GoToTask(robot),
                                    remapping = {'waypoint': 'above_pile_pose'},
                                    transitions = {'succeeded': 'GO_ABOVE_WALL'})

            smach.StateMachine.add('GO_ABOVE_WALL', GoToTask(robot),
                                    remapping = {'waypoint': 'above_wall_pose'},
                                    transitions = {'succeeded': 'PLACE'})

            def place_goal_callback(userdata, default_goal):
                goal = ual_action_server.msg.PlaceGoal(in_wall_brick_pose = userdata.in_wall_brick_pose)
                return goal

            smach.StateMachine.add('PLACE', smach_ros.SimpleActionState(robot.url + 'place_action', ual_action_server.msg.PlaceAction,
                                    input_keys = ['in_wall_brick_pose'],
                                    goal_cb = place_goal_callback),
                                    transitions = {'succeeded': 'GO_UP_AGAIN'})

            smach.StateMachine.add('GO_UP_AGAIN', GoToTask(robot),
                                    remapping = {'waypoint': 'above_wall_pose'},
                                    transitions = {'succeeded': 'succeeded'})

class LandTask(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'])

        with self:

            def ask_for_region_request_callback(userdata, request):
                radius = 1.0  # TODO: Tune, assure uav is not going further while land!
                request = robot.build_ask_for_region_request_to_land(radius)
                return request

            def ask_for_region_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('ASK_FOR_REGION_TO_LAND', smach_ros.ServiceState('ask_for_region', mbzirc_comm_objs.srv.AskForRegion,
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'LAND', 'aborted': 'SLEEP_AND_RETRY_ASKING'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING', Sleep(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_LAND'})

            # TODO: is this callback needed?
            def land_goal_callback(userdata, default_goal):
                goal = ual_action_server.msg.LandGoal()
                return goal

            smach.StateMachine.add('LAND', smach_ros.SimpleActionState(robot.url + 'land_action', ual_action_server.msg.LandAction,
                                    # input_keys = ['go_home'],  # TODO: bool go_home?
                                    goal_cb = land_goal_callback),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_LANDED'})

            smach.StateMachine.add('ASK_FOR_REGION_LANDED', smach_ros.ServiceState('ask_for_region', mbzirc_comm_objs.srv.AskForRegion,
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'succeeded'})

class Agent(object):
    def __init__(self, robot):
        self.robot = robot
        self.tasks = {}
        self.tasks['take_off'] = TakeOffTask(robot)
        self.tasks['follow_path'] = FollowPathTask(robot)
        self.tasks['pick_and_place'] = PickAndPlaceTask(robot)

        
        self.go_home_action_server = actionlib.SimpleActionServer('task/go_home', GoHomeAction, execute_cb = self.go_home_callback, auto_start = False)  # TODO: change naming from task to agent?
        self.go_home_action_server.start()

        self.land_task = LandTask()

        rospy.Service('get_cost_to_go_to', GetCostToGoTo, self.get_cost_to_go_to)

        self.ual_pose = PoseStamped()
        rospy.Subscriber("ual/pose", PoseStamped, self.ual_pose_callback)

        # TODO: Force these lines to be the lasts in construction to avoid ill data_feed?
        self.feed_publisher = rospy.Publisher('data_feed', AgentDataFeed, queue_size = 1)
        rospy.Timer(rospy.Duration(0.2), self.update_feed_callback)  # TODO: duration?

    def update_feed_callback(self, event):
        data_feed = AgentDataFeed()
        data_feed.is_idle = True
        if self.take_off_task.is_running():
            data_feed.is_idle = False
        if self.follow_path_task.is_running():
            data_feed.is_idle = False
        if self.pick_and_place_task.is_running():
            data_feed.is_idle = False
        # go_home action server reuses follow_path_task
        # TODO: Check all, make it automatic!
        self.feed_publisher.publish(data_feed)

    def go_home_callback(self, goal):
        agent_id = rospy.get_param('~agent_id')
        flight_level = rospy.get_param('~flight_level')  # TODO: Taking it every callback allows parameter changes...
        userdata = smach.UserData()
        userdata.path = []  # TODO: Build path from here to home
        up_here = copy.deepcopy(self.ual_pose)
        up_here.pose.position.z = flight_level
        userdata.path.append(up_here)
        up_home = copy.deepcopy(self.home_pose)
        up_home.pose.position.z = flight_level
        userdata.path.append(up_home)
        print('uav[{}].flight_level = {}'.format(agent_id, flight_level))
        for point in userdata.path:
            print('path point: {}'.format(point))
        outcome = self.follow_path_task.execute(userdata)  # Reuse follow path!
        print('follow_path_task (go_home) output: {}'.format(outcome))
        if goal.do_land:
            userdata = smach.UserData()  # empty
            outcome = self.land_task.execute(userdata)
            print('follow_path_task (go_home) output: {}'.format(outcome))
        self.go_home_action_server.set_succeeded()

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
wall_blueprint = [['red', 'green']]  #, ['green', 'red']]  # , 'blue', 'orange']]  #, ['orange', 'blue', 'green', 'red']]

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

# First sequential model works, TODO: concurrency in uav_id!
class CentralAgent(object):
    def __init__(self):
        self.available_uavs = ['1', '2'] # Force id to be a string to avoid index confussion  # TODO: auto discovery (and update!)

        self.tf_buffer = tf2_ros.Buffer()  # TODO: this will be repated... AgentInterface?
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.robot = {}
        for uav_id in self.available_uavs:
            self.robot[uav_id] = RobotInterface(uav_id)

        self.piles = {}
        rospy.Subscriber("estimated_objects", mbzirc_comm_objs.msg.ObjectDetectionList, self.estimation_callback)

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

            userdata = smach.UserData()
            userdata.height = self.robot[uav_id].params['flight_level']
            # self.home_pose = copy.deepcopy(self.ual_pose)  # TODO: Fetch home_pose!
            take_off_task = TakeOffTask(self.robot[uav_id])
            outcome = take_off_task.execute(userdata)  # TODO: Now it's blocking!
            print('take_off output: {}'.format(outcome))
            # self.take_off_action_server.set_succeeded()

            # self.uav_clients[uav_id]['take_off'].send_goal(ual_action_server.msg.TakeOffGoal(height = self.uav_params[uav_id]['flight_level']))

        # for uav_id in self.available_uavs:
        #     print('waiting result of take_off server [{}]'.format(uav_id))
        #     self.uav_clients[uav_id]['take_off'].wait_for_result()
        #     print(self.uav_clients[uav_id]['take_off'].get_result())

    # TODO: Could be a smach.State (for all or for every single uav)
    def look_for_piles(self):
        uav_paths = {}
        point_paths = generate_uav_paths(len(self.available_uavs))
        for i, uav_id in enumerate(self.available_uavs):
            uav_path = []
            flight_level = self.robot[uav_id].params['flight_level']
            point_path = set_z(point_paths[i], flight_level)
            for point in point_path:
                waypoint = PoseStamped()
                waypoint.header.frame_id = 'arena'
                waypoint.pose.position = point
                waypoint.pose.orientation.z = 0
                waypoint.pose.orientation.w = 1  # TODO: other orientation?
                uav_path.append(waypoint)
            uav_paths[uav_id] = uav_path

        for uav_id in self.available_uavs:
            print('sending goal to follow_path server {}'.format(uav_id))

            userdata = smach.UserData()
            userdata.path = uav_paths[uav_id]
            follow_path_task = FollowPathTask(self.robot[uav_id])
            outcome = follow_path_task.execute(userdata)
            print('follow_path_callback output: {}'.format(outcome))
            # self.follow_path_action_server.set_succeeded()

            # self.uav_clients[uav_id]['follow_path'].send_goal(uav_paths[uav_id])

        # for uav_id in self.available_uavs:
        #     print('waiting result of follow_path server [{}]'.format(uav_id))
        #     self.uav_clients[uav_id]['follow_path'].wait_for_result()
        #     print(self.uav_clients[uav_id]['follow_path'].get_result())

    # TODO: Could be a smach.State (for all or for every single uav, not so easy!)
    def build_wall(self):
        is_idle = {}
        for uav_id in self.available_uavs:
            is_idle[uav_id] = True
        rospy.sleep(0.5)  # TODO: some sleep to allow data_feed update
        piles = copy.deepcopy(self.piles)  # Cache piles
        build_wall_sequence = get_build_wall_sequence(wall_blueprint)
        for i, row in enumerate(build_wall_sequence):
            for brick in row:
                print('row[{}] brick = {}'.format(i, brick))
                costs = {}
                while not costs:
                    for uav_id in self.available_uavs:
                        # if self.uav_data_feeds[uav_id].is_idle:
                        if is_idle[uav_id]:  # TODO: Make it a data feed from agent? Sequential execution does not need it!
                            costs[uav_id] = self.robot[uav_id].get_cost_to_go_to(piles[brick.color])
                        else:
                            rospy.sleep(0.5)
                min_cost_uav_id = min(costs, key = costs.get)
                print('costs: {}, min_cost_id: {}'.format(costs, min_cost_uav_id))
                goal = mbzirc_comm_objs.msg.PickAndPlaceGoal()
                goal.pile_pose = piles[brick.color]
                goal.in_wall_brick_pose = brick.pose

                # flight_level = rospy.get_param('~flight_level')  # TODO: Taking it every callback allows parameter changes...
                flight_level = self.robot[min_cost_uav_id].params['flight_level']
                userdata = smach.UserData()
                userdata.above_pile_pose = copy.deepcopy(goal.pile_pose)
                userdata.above_pile_pose.pose.position.z = flight_level
                userdata.above_wall_pose = copy.deepcopy(goal.in_wall_brick_pose)
                userdata.above_wall_pose.pose.position.z = flight_level
                userdata.in_wall_brick_pose = copy.deepcopy(goal.in_wall_brick_pose)
                pick_and_place_task = PickAndPlaceTask(self.robot[min_cost_uav_id])
                outcome = pick_and_place_task.execute(userdata)
                print('pick_and_place_callback output: {}'.format(outcome))
                # self.pick_and_place_action_server.set_succeeded()

                # self.uav_clients[min_cost_uav_id]['pick_and_place'].send_goal(goal)
                rospy.sleep(0.5)  # TODO: some sleep to allow data_feed update
        # Once arrived here, last pick_and_place task has been allocated
        # TODO: Sequencial implementation may never arrive here (uav blocked at wall)!!!

        print('All pick_and_place tasks allocated')
        finished_uavs = []
        while True:
            rospy.sleep(0.5)  # TODO: some sleep here?
            for uav_id in self.available_uavs:
                if is_idle[uav_id] and (uav_id not in finished_uavs):
                    finished_uavs.append(uav_id)

                    # print('waiting result of pick_and_place server [{}]'.format(uav_id))
                    # self.uav_clients[uav_id]['pick_and_place'].wait_for_result()
                    # print(self.uav_clients[uav_id]['pick_and_place'].get_result())
                    print('now go home!')
                    flight_level = self.robot[uav_id].params['flight_level']
                    go_home_path = []
                    current_at_flight_level = copy.deepcopy(self.robot[uav_id].pose)
                    current_at_flight_level.pose.position.z = flight_level
                    home_at_flight_level = copy.deepcopy(self.robot[uav_id].home)
                    home_at_flight_level.pose.position.z = flight_level
                    go_home_path.append(current_at_flight_level)
                    go_home_path.append(home_at_flight_level)

                    userdata = smach.UserData()
                    userdata.path = go_home_path
                    follow_path_task = FollowPathTask(self.robot[uav_id])
                    outcome = follow_path_task.execute(userdata)
                    print('follow_path_callback output: {}'.format(outcome))

            if set(self.available_uavs).issubset(finished_uavs):
                print('All done!')
                break

        # for uav_id in self.available_uavs:
        #     print('waiting result of go_home server [{}]'.format(uav_id))
        #     self.uav_clients[uav_id]['go_home'].wait_for_result()
        #     print(self.uav_clients[uav_id]['go_home'].get_result())

def main():
    rospy.init_node('cu_agent_c2')

    while rospy.get_rostime() == rospy.Time():
        rospy.logwarn("Waiting for (sim) time to begin!")
        time.sleep(1)

    central_agent = CentralAgent()
    # rospy.sleep(3)

    central_agent.take_off()
    # central_agent.look_for_piles() # TODO: if not piles[r, g, b, o], repeat! if all found, stop searching?
    central_agent.build_wall()

    # TODO(performance): Make it optional, use only in develop stage
    # viewer = smach_ros.IntrospectionServer('viewer', agent.follow_path_task, 'UAV_' + str(agent_id))
    # viewer.start()
    # rospy.spin()
    # viewer.stop()

if __name__ == '__main__':
    main()
