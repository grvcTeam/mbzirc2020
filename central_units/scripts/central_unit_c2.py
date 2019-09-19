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
import threading
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

class ThreadWithReturnValue(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}, Verbose=None):
        threading.Thread.__init__(self, group, target, name, args, kwargs, Verbose)
        self._return = None

    def run(self):
        if self._Thread__target is not None:
            self._return = self._Thread__target(*self._Thread__args, **self._Thread__kwargs)

    # As ONLY __init__() y run() should be overriden
    def get_return_value(self):
        return self._return

class RobotProxy(object):
    def __init__(self, robot_id):
        self.id = robot_id
        self.url = 'mbzirc2020_' + self.id + '/'  # TODO: Impose ns: mbzirc2020!?
        # TODO: Unifying robot_model and namespace might be an issue for non homogeneous teams, 
        # but it is somehow forced by the way sensor topics are named in gazebo simulation (mbzirc2020)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pose = PoseStamped()
        rospy.Subscriber(self.url + 'data_feed', mbzirc_comm_objs.msg.RobotDataFeed, self.data_feed_callback)
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

        request = mbzirc_comm_objs.srv.AskForRegionRequest()
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

        request = mbzirc_comm_objs.srv.AskForRegionRequest()
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

# TODO: Get Task out of naming (or invert it!), should all classes be State Machines?
class SleepAndRetry(smach.State):
    def __init__(self, duration = 3.0, max_retries = None):
        smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'])
        self.duration = duration
        self.max_retries = max_retries
        self.retry_count = 0

    def execute(self, userdata):
        rospy.sleep(self.duration)
        # TODO: Sleep in shorter period chunks to allow faster preemption?
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if not self.max_retries:
            return 'succeeded'
        elif self.retry_count < self.max_retries:
            self.retry_count += 1
            return 'succeeded'
        else:
            return 'aborted'

class AskForRegionToHoverTask(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'])

    def define_for(self, robot):
        with self:

            # TODO: decorators?
            def ask_for_region_request_callback(userdata, request):
                ask_for_region_request = robot.build_request_for_go_to_region(robot.pose, 'hover', 1.0)
                return ask_for_region_request

            def ask_for_region_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('ASK_FOR_REGION', smach_ros.ServiceState('ask_for_region', mbzirc_comm_objs.srv.AskForRegion,
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'succeeded', 'aborted': 'SLEEP_AND_RETRY_ASKING'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING', SleepAndRetry(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION'})
        return self

class AskForRegionToMoveTask(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['waypoint'])

    def define_for(self, robot):
        with self:

            def ask_for_region_request_callback(userdata, request):
                ask_for_region_request = robot.build_request_for_go_to_region(userdata.waypoint, 'go_to', 1.0)
                return ask_for_region_request

            def ask_for_region_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('ASK_FOR_REGION', smach_ros.ServiceState('ask_for_region', mbzirc_comm_objs.srv.AskForRegion,
                                    input_keys = ['waypoint'],
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'succeeded', 'aborted': 'SLEEP_AND_RETRY_ASKING'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING', SleepAndRetry(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION'})
        return self

class TakeOffTask(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['height'])

    def define_for(self, robot):
        with self:

            def take_off_goal_callback(userdata, default_goal):
                robot.set_home()  # TODO: better do it explicitly BEFORE take off?
                goal = ual_action_server.msg.TakeOffGoal(height = userdata.height)
                return goal

            smach.StateMachine.add('TAKE_OFF', smach_ros.SimpleActionState(robot.url + 'take_off_action', ual_action_server.msg.TakeOffAction,
                                    input_keys = ['height'],
                                    goal_cb = take_off_goal_callback),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_HOVER', 'aborted': 'SLEEP_AND_RETRY'})

            smach.StateMachine.add('SLEEP_AND_RETRY', SleepAndRetry(3.0, 5),
                                    transitions = {'succeeded': 'TAKE_OFF', 'aborted': 'aborted'})

            smach.StateMachine.add('ASK_FOR_REGION_TO_HOVER', AskForRegionToHoverTask().define_for(robot),
                                    transitions = {'succeeded': 'succeeded'})

        return self

class FreeRegionsTask(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'])

    def define_for(self, robot, label, free_all = False):
        with self:

            def free_regions_request_callback(userdata, request):
                free_regions_request = mbzirc_comm_objs.srv.FreeRegionsRequest()
                free_regions_request.agent_id = robot.id
                free_regions_request.label = label
                free_regions_request.free_all = free_all
                return free_regions_request

            def free_regions_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('FREE_REGIONS', smach_ros.ServiceState('free_regions', mbzirc_comm_objs.srv.FreeRegions,
                                    request_cb = free_regions_request_callback,
                                    response_cb = free_regions_response_callback),
                                    transitions = {'succeeded': 'succeeded', 'aborted': 'SLEEP_AND_RETRY'})

            smach.StateMachine.add('SLEEP_AND_RETRY', SleepAndRetry(1.0),
                                    transitions = {'succeeded': 'FREE_REGIONS'})
        return self

class GoToTask(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['waypoint'])

    def define_for(self, robot):
        with self:

            smach.StateMachine.add('ASK_FOR_REGION_TO_MOVE', AskForRegionToMoveTask().define_for(robot),
                                    transitions = {'succeeded': 'FREE_LAST_HOVER'})

            smach.StateMachine.add('FREE_LAST_HOVER', FreeRegionsTask().define_for(robot, label = 'hover'),
                                    transitions = {'succeeded': 'GO_TO'})

            def go_to_goal_callback(userdata, default_goal):
                goal = ual_action_server.msg.GoToGoal(waypoint = userdata.waypoint)
                return goal

            smach.StateMachine.add('GO_TO', smach_ros.SimpleActionState(robot.url + 'go_to_action', ual_action_server.msg.GoToAction,
                                    input_keys = ['waypoint'],
                                    goal_cb = go_to_goal_callback),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_HOVER'})

            smach.StateMachine.add('ASK_FOR_REGION_TO_HOVER', AskForRegionToHoverTask().define_for(robot),
                                    transitions = {'succeeded': 'FREE_LAST_MOVE'})

            smach.StateMachine.add('FREE_LAST_MOVE', FreeRegionsTask().define_for(robot, label = 'go_to'),
                                    transitions = {'succeeded': 'succeeded'})

        return self

class FollowPathTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['path'])
        self.go_to_task = None

    def define_for(self, robot):
        self.go_to_task = GoToTask().define_for(robot)
        return self

    def execute(self, userdata):
        for waypoint in userdata.path:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            child_userdata = smach.UserData()
            child_userdata.waypoint = waypoint
            self.go_to_task.execute(child_userdata)
        return 'succeeded'
        # TODO: aborted?

class SearchPilesTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['path'], output_keys = ['piles'])
        self.go_to_task = None
        self.piles = {}
        rospy.Subscriber("estimated_objects", mbzirc_comm_objs.msg.ObjectDetectionList, self.estimation_callback)

    def define_for(self, robot):
        self.go_to_task = GoToTask().define_for(robot)
        return self

    # TODO: This is repeated in central unit
    def estimation_callback(self, data):
        for pile in data.objects:
            # TODO: check type and scale?
            color = color_from_int(pile.color)
            pose = PoseStamped()
            pose.header = pile.header
            pose.pose = pile.pose.pose
            self.piles[color] = pose

    def execute(self, userdata):
        userdata.piles = self.piles
        for waypoint in userdata.path:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            if all_piles_are_found(self.piles):
                return 'succeeded'
            child_userdata = smach.UserData()
            child_userdata.waypoint = waypoint
            self.go_to_task.execute(child_userdata)
        return 'succeeded' if all_piles_are_found(self.piles) else 'aborted'

# class GoHomeTask(smach.StateMachine):  # TODO?

class PickAndPlaceTask(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['above_pile_pose', 'above_wall_pose', 'in_wall_brick_pose'])

    def define_for(self, robot):
        with self:

            def ask_for_region_to_pick_request_callback(userdata, request):
                radius = 2.5  # TODO: Tune, assure uav is not going further while pick!
                z_max = userdata.above_pile_pose.pose.position.z
                request = robot.build_request_for_vertical_region(userdata.above_pile_pose, 'pick', radius, z_max = z_max)
                return request

            def ask_for_region_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('ASK_FOR_REGION_TO_PICK', smach_ros.ServiceState('ask_for_region', mbzirc_comm_objs.srv.AskForRegion,
                                    input_keys = ['above_pile_pose'],
                                    request_cb = ask_for_region_to_pick_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'GO_TO_PILE', 'aborted': 'SLEEP_AND_RETRY_ASKING_TO_PICK'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING_TO_PICK', SleepAndRetry(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_PICK'})

            smach.StateMachine.add('GO_TO_PILE', GoToTask().define_for(robot),
                                    remapping = {'waypoint': 'above_pile_pose'},
                                    transitions = {'succeeded': 'PICK'})

            def pick_goal_callback(userdata, default_goal):
                goal = ual_action_server.msg.PickGoal(approximate_pose = userdata.above_pile_pose)
                return goal

            smach.StateMachine.add('PICK', smach_ros.SimpleActionState(robot.url + 'pick_action', ual_action_server.msg.PickAction,
                                    input_keys = ['above_pile_pose'],
                                    goal_cb = pick_goal_callback),
                                    transitions = {'succeeded': 'GO_UP'})

            smach.StateMachine.add('GO_UP', GoToTask().define_for(robot),
                                    remapping = {'waypoint': 'above_pile_pose'},
                                    transitions = {'succeeded': 'FREE_REGION_TO_PICK'})

            smach.StateMachine.add('FREE_REGION_TO_PICK', FreeRegionsTask().define_for(robot, label = 'pick'),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_PLACE'})

            def ask_for_region_to_place_request_callback(userdata, request):
                radius = 3.0  # TODO: Tune, assure uav is not going further while place!
                z_max = userdata.above_wall_pose.pose.position.z
                request = robot.build_request_for_vertical_region(userdata.above_wall_pose, 'place', radius, z_max = z_max)
                return request

            smach.StateMachine.add('ASK_FOR_REGION_TO_PLACE', smach_ros.ServiceState('ask_for_region', mbzirc_comm_objs.srv.AskForRegion,
                                    input_keys = ['above_wall_pose'],
                                    request_cb = ask_for_region_to_place_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'GO_ABOVE_WALL', 'aborted': 'SLEEP_AND_RETRY_ASKING_TO_PLACE'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING_TO_PLACE', SleepAndRetry(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_PLACE'})

            smach.StateMachine.add('GO_ABOVE_WALL', GoToTask().define_for(robot),
                                    remapping = {'waypoint': 'above_wall_pose'},
                                    transitions = {'succeeded': 'PLACE'})

            def place_goal_callback(userdata, default_goal):
                goal = ual_action_server.msg.PlaceGoal(in_wall_brick_pose = userdata.in_wall_brick_pose)
                return goal

            smach.StateMachine.add('PLACE', smach_ros.SimpleActionState(robot.url + 'place_action', ual_action_server.msg.PlaceAction,
                                    input_keys = ['in_wall_brick_pose'],
                                    goal_cb = place_goal_callback),
                                    transitions = {'succeeded': 'GO_UP_AGAIN'})

            smach.StateMachine.add('GO_UP_AGAIN', GoToTask().define_for(robot),
                                    remapping = {'waypoint': 'above_wall_pose'},
                                    transitions = {'succeeded': 'FREE_REGION_TO_PLACE'})
            
            smach.StateMachine.add('FREE_REGION_TO_PLACE', FreeRegionsTask().define_for(robot, label = 'place'),
                        transitions = {'succeeded': 'succeeded'})

        return self

class LandTask(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'])

    def define_for(self, robot):
        with self:

            def ask_for_region_request_callback(userdata, request):
                radius = 1.0  # TODO: Tune, assure uav is not going further while land!
                z_max = robot.pose.pose.position.z
                request = robot.build_request_for_vertical_region(robot.pose, 'land', radius, z_max = z_max)
                return request

            def ask_for_region_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('ASK_FOR_REGION_TO_LAND', smach_ros.ServiceState('ask_for_region', mbzirc_comm_objs.srv.AskForRegion,
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'LAND', 'aborted': 'SLEEP_AND_RETRY_ASKING'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING', SleepAndRetry(1.0),
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
        return self

class TaskManager(object):
    def __init__(self, robot_interfaces):
        self.robots = robot_interfaces
        self.idle = {}
        self.locks = {}
        self.tasks = {}
        self.threads = {}

        for robot_id in self.robots:
            self.locks[robot_id] = threading.Lock()
            self.idle[robot_id] = threading.Event()
            self.idle[robot_id].set()

        self.manage_thread = threading.Thread(target = self.manage_tasks)
        self.manage_thread.daemon = True
        self.manage_thread.start()

    def manage_tasks(self):
        rate = rospy.Rate(1)  # [Hz]
        while not rospy.is_shutdown():
            for robot_id, thread in self.threads.items():
                self.locks[robot_id].acquire()
                if not self.idle[robot_id].is_set() and not thread.is_alive():
                    thread.join()
                    outcome = thread.get_return_value()
                    rospy.logwarn('task on robot[{}] finished with output: {}'.format(robot_id, outcome))
                    del self.threads[robot_id]  # TODO: needed?
                    del self.tasks[robot_id]    # TODO: needed?
                    self.idle[robot_id].set()
                self.locks[robot_id].release()
            rate.sleep()

    def start_task(self, robot_id, task, userdata):
        # TODO: Force robot_id field inside task instead?
        with self.locks[robot_id]:
            if not self.idle[robot_id].is_set():
                rospy.logerr('robot {} is not idle!'.format(robot_id))
                return False
            self.idle[robot_id].clear()
            self.tasks[robot_id] = task
            self.tasks[robot_id].define_for(self.robots[robot_id])
            self.threads[robot_id] = ThreadWithReturnValue(target = task.execute, args = (userdata,))
            self.threads[robot_id].daemon = True
            self.threads[robot_id].start()
        return True

    def preempt_task(self, robot_id):
        if robot_id not in self.tasks:
            rospy.logerr('Invalid robot_id [{}]'.format(robot_id))
            return False
        rospy.logwarn('Trying to preempt current task on robot[{}]'.format(robot_id))
        self.tasks[robot_id].request_preempt()
        return True

    def is_idle(self, robot_id):
        return self.idle[robot_id].is_set()

    def are_idle(self, id_list):
        for robot_id in id_list:
            if not self.idle[robot_id].is_set():
                return False
        return True

    def wait_for(self, id_list):
        # TODO: Add timeout?
        while not self.are_idle(id_list):
            rospy.sleep(0.2)  # TODO: tune

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

# TODO: move to wall utils?
def all_piles_are_found(piles):
    # TODO: Check not only count, but also size of piles
    return len(piles) >= 4

# TODO: move to object_detection utils?
def color_from_int(color):
    if color == mbzirc_comm_objs.msg.ObjectDetection.COLOR_RED:
        return 'red'
    elif color == mbzirc_comm_objs.msg.ObjectDetection.COLOR_GREEN:
        return 'green'
    elif color == mbzirc_comm_objs.msg.ObjectDetection.COLOR_BLUE:
        return 'blue'
    elif color == mbzirc_comm_objs.msg.ObjectDetection.COLOR_ORANGE:
        return 'orange'
    else:
        return 'unknown'

class CentralUnit(object):
    def __init__(self):
        self.available_robots = ['1', '2'] # Force id to be a string to avoid index confussion  # TODO: auto discovery (and update!)

        self.robots = {}
        for robot_id in self.available_robots:
            self.robots[robot_id] = RobotProxy(robot_id)

        self.piles = {}
        rospy.Subscriber("estimated_objects", mbzirc_comm_objs.msg.ObjectDetectionList, self.estimation_callback)

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
            self.task_manager.start_task(robot_id, TakeOffTask(), userdata)
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
            self.task_manager.start_task(robot_id, FollowPathTask(), userdata)

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
                goal = mbzirc_comm_objs.msg.PickAndPlaceGoal()
                goal.pile_pose = piles[brick.color]
                goal.in_wall_brick_pose = brick.pose

                flight_level = self.get_param(min_cost_robot_id,'flight_level')
                userdata = smach.UserData()
                userdata.above_pile_pose = copy.deepcopy(goal.pile_pose)
                userdata.above_pile_pose.pose.position.z = flight_level
                userdata.above_wall_pose = copy.deepcopy(goal.in_wall_brick_pose)
                userdata.above_wall_pose.pose.position.z = flight_level
                userdata.in_wall_brick_pose = copy.deepcopy(goal.in_wall_brick_pose)
                self.task_manager.start_task(min_cost_robot_id, PickAndPlaceTask(), userdata)

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
                    self.task_manager.start_task(robot_id, FollowPathTask(), userdata)

            if set(self.available_robots).issubset(finished_robots):
                print('All done!')
                break

def main():
    rospy.init_node('central_unit_c2')

    while rospy.get_rostime() == rospy.Time():
        rospy.logwarn("Waiting for (sim) time to begin!")
        time.sleep(1)

    central_unit = CentralUnit()
    # rospy.sleep(3)

    central_unit.take_off()
    central_unit.look_for_piles() # TODO: if not piles[r, g, b, o], repeat!?
    central_unit.build_wall()

    rospy.spin()

if __name__ == '__main__':
    main()