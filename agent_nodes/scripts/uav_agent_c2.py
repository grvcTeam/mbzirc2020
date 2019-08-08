#!/usr/bin/env python

import rospy
import smach
import smach_ros
import copy
import time
import random
import actionlib
import tf2_ros
import tf2_geometry_msgs

from mbzirc_comm_objs.msg import AgentDataFeed
from mbzirc_comm_objs.msg import TakeOffAction, TakeOffGoal
from mbzirc_comm_objs.msg import GoToAction, GoToGoal
from mbzirc_comm_objs.msg import PickAction, PickGoal
from mbzirc_comm_objs.msg import FollowPathAction, FollowPathGoal
from mbzirc_comm_objs.msg import PickAndPlaceAction, PickAndPlaceGoal
from mbzirc_comm_objs.msg import GoHomeAction, GoHomeGoal
from geometry_msgs.msg import PoseStamped
from mbzirc_comm_objs.srv import AskForRegion, AskForRegionRequest, GetCostToGoTo, GetCostToGoToResponse, Magnetize, MagnetizeRequest

class Sleep(smach.State):
    def __init__(self, duration = 3.0):
        smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'])  # TODO: duration as an input_key?
        self.duration = duration

    def execute(self, userdata):
        rospy.sleep(self.duration)
        return 'succeeded'
        # TODO: aborted?
        # TODO: preempted?

class TakeOffTask(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['height'])  # TODO: Use rospy.get_param('~flight_level') instead?

        with self:

            def take_off_goal_callback(userdata, default_goal):
                # TODO: Or from userdata.height?
                flight_level = rospy.get_param('~flight_level')
                goal = TakeOffGoal(height = flight_level)
                return goal

            smach.StateMachine.add('TAKE_OFF', smach_ros.SimpleActionState('take_off_action', TakeOffAction,
                                    input_keys = ['height'],  # TODO: Use rospy.get_param('~flight_level') instead?
                                    goal_cb = take_off_goal_callback),
                                    transitions = {'succeeded': 'succeeded', 'aborted': 'SLEEP_AND_RETRY'})

            smach.StateMachine.add('SLEEP_AND_RETRY', Sleep(3.0),
                                    transitions = {'succeeded': 'TAKE_OFF'})

class GoToTask(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['waypoint'])

        self.tf_buffer = tf2_ros.Buffer()  # TODO: repeated code, AgentInterface?
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        with self:

            # TODO: decorators?
            def ask_for_region_request_callback(userdata, request):
                agent_id = rospy.get_param('~agent_id')
                ask_for_region_request = self.build_ask_for_region_request(agent_id, self.ual_pose, userdata.waypoint)
                return ask_for_region_request

            def ask_for_region_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('ASK_FOR_REGION_TO_MOVE', smach_ros.ServiceState('/ask_for_region', AskForRegion,
                                    input_keys = ['waypoint'],
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'GO_TO', 'aborted': 'SLEEP_AND_RETRY_ASKING'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING', Sleep(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_MOVE'})

            def go_to_goal_callback(userdata, default_goal):
                goal = GoToGoal(waypoint = userdata.waypoint)
                return goal

            smach.StateMachine.add('GO_TO', smach_ros.SimpleActionState('go_to_action', GoToAction,
                                    input_keys = ['waypoint'],
                                    goal_cb = go_to_goal_callback),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_HOVER'})

            smach.StateMachine.add('ASK_FOR_REGION_TO_HOVER', smach_ros.ServiceState('/ask_for_region', AskForRegion,
                                    input_keys = ['waypoint'],
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'succeeded'})

        self.ual_pose = PoseStamped()
        rospy.Subscriber("ual/pose", PoseStamped, self.ual_pose_callback)

    def ual_pose_callback(self, data):
        self.ual_pose = data

    # TODO: almost repeated code
    def build_ask_for_region_request(self, agent_id, initial_pose, final_pose, radius = 1.0):
        try:
            if initial_pose.header.frame_id != 'arena':
                initial_pose = self.tf_buffer.transform(initial_pose, 'arena', rospy.Duration(1.0))
            if final_pose.header.frame_id != 'arena':
                final_pose = self.tf_buffer.transform(final_pose, 'arena', rospy.Duration(1.0))
        except:
            rospy.logerr('Failed to transform points to [{}], ignoring!'.format('arena'))

        request = AskForRegionRequest()
        request.agent_id = agent_id
        request.min_corner.header.frame_id = 'arena'
        request.min_corner.point.x = min(initial_pose.pose.position.x - radius, final_pose.pose.position.x - radius)
        request.min_corner.point.y = min(initial_pose.pose.position.y - radius, final_pose.pose.position.y - radius)
        request.min_corner.point.z = min(initial_pose.pose.position.z - radius, final_pose.pose.position.z - radius)
        request.max_corner.header.frame_id = 'arena'
        request.max_corner.point.x = max(initial_pose.pose.position.x + radius, final_pose.pose.position.x + radius)
        request.max_corner.point.y = max(initial_pose.pose.position.y + radius, final_pose.pose.position.y + radius)
        request.max_corner.point.z = max(initial_pose.pose.position.z + radius, final_pose.pose.position.z + radius)

        return request

class WaypointDispatch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['path'])
        self.go_to_task = GoToTask()

    def execute(self, userdata):
        for waypoint in userdata.path:
            child_userdata = smach.UserData()
            child_userdata.waypoint = waypoint
            self.go_to_task.execute(child_userdata)
        return 'succeeded'
        # TODO: aborted?
        # TODO: preempted?

class FollowPathTask(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['path'])

        with self:

            smach.StateMachine.add('DISPATCH', WaypointDispatch(),
                                    remapping = {'path': 'path'},
                                    transitions = {'succeeded': 'succeeded'})

class PickAndPlaceTask(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['above_pile_pose', 'above_wall_pose', 'brick_in_wall_pose'])

        with self:

            smach.StateMachine.add('GO_TO_PILE', GoToTask(),
                                    remapping = {'waypoint': 'above_pile_pose'},
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_PICK'})

            def ask_for_region_request_callback(userdata, request):
                agent_id = rospy.get_param('~agent_id')
                radius = 5.0  # TODO: Tune, assure uav is not going further while pick!
                request = AskForRegionRequest()
                request.agent_id = agent_id
                request.min_corner.header.frame_id = userdata.above_pile_pose.header.frame_id
                request.min_corner.point.x = userdata.above_pile_pose.pose.position.x - radius
                request.min_corner.point.y = userdata.above_pile_pose.pose.position.y - radius
                request.min_corner.point.z = 0
                request.max_corner.header.frame_id = userdata.above_pile_pose.header.frame_id
                request.max_corner.point.x = userdata.above_pile_pose.pose.position.x + radius
                request.max_corner.point.y = userdata.above_pile_pose.pose.position.y + radius
                request.max_corner.point.z = userdata.above_pile_pose.pose.position.z + radius
                return request

            def ask_for_region_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('ASK_FOR_REGION_TO_PICK', smach_ros.ServiceState('/ask_for_region', AskForRegion,
                                    input_keys = ['above_pile_pose'],
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'PICK', 'aborted': 'SLEEP_AND_RETRY_ASKING'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING', Sleep(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_PICK'})

            def pick_goal_callback(userdata, default_goal):
                goal = PickGoal(approximate_pose = userdata.above_pile_pose)
                return goal

            smach.StateMachine.add('PICK', smach_ros.SimpleActionState('pick_action', PickAction,
                                    input_keys = ['above_pile_pose'],
                                    goal_cb = pick_goal_callback),
                                    transitions = {'succeeded': 'GO_UP'})

            smach.StateMachine.add('GO_UP', GoToTask(),
                                    remapping = {'waypoint': 'above_pile_pose'},
                                    transitions = {'succeeded': 'GO_ABOVE_WALL'})

            smach.StateMachine.add('GO_ABOVE_WALL', GoToTask(),
                                    remapping = {'waypoint': 'above_wall_pose'},
                                    transitions = {'succeeded': 'GO_TO_PLACE'})

            smach.StateMachine.add('GO_TO_PLACE', GoToTask(),
                                    remapping = {'waypoint': 'brick_in_wall_pose'},
                                    transitions = {'succeeded': 'PLACE'})

            def place_request_callback(userdata, request):
                request = MagnetizeRequest()
                request.magnetize = False
                return request

            def place_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('PLACE', smach_ros.ServiceState('magnetize', Magnetize,
                                    request_cb = place_request_callback,
                                    response_cb = place_response_callback),
                                    transitions = {'succeeded': 'GO_UP_AGAIN'})

            smach.StateMachine.add('GO_UP_AGAIN', GoToTask(),
                                    remapping = {'waypoint': 'above_wall_pose'},
                                    transitions = {'succeeded': 'succeeded'})

class Agent(object):

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()  # TODO: this will be repated... AgentInterface?
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.take_off_task = TakeOffTask()
        self.take_off_action_server = actionlib.SimpleActionServer('task/take_off', TakeOffAction, execute_cb = self.take_off_callback, auto_start = False)  # TODO: change naming from task to agent?
        self.take_off_action_server.start()

        self.follow_path_task = FollowPathTask()
        self.follow_path_action_server = actionlib.SimpleActionServer('task/follow_path', FollowPathAction, execute_cb = self.follow_path_callback, auto_start = False)  # TODO: change naming from task to agent?
        self.follow_path_action_server.start()

        self.pick_and_place_task = PickAndPlaceTask()
        self.pick_and_place_action_server = actionlib.SimpleActionServer('task/pick_and_place', PickAndPlaceAction, execute_cb = self.pick_and_place_callback, auto_start = False)  # TODO: change naming from task to agent?
        self.pick_and_place_action_server.start()

        # self.go_home_task = GoHomeTask()  # Reuse follow_path_task
        self.go_home_action_server = actionlib.SimpleActionServer('task/go_home', GoHomeAction, execute_cb = self.go_home_callback, auto_start = False)  # TODO: change naming from task to agent?
        self.go_home_action_server.start()

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

    def ual_pose_callback(self, data):  # TODO: this is repeated code, use AgentInterface?
        self.ual_pose = data

    def take_off_callback(self, goal):
        userdata = smach.UserData()
        userdata.height = goal.height
        self.home_pose = copy.deepcopy(self.ual_pose)  # Fetch home_pose!
        outcome = self.take_off_task.execute(userdata)
        print('take_off_callback output: {}'.format(outcome))
        self.take_off_action_server.set_succeeded()

    def follow_path_callback(self, goal):
        userdata = smach.UserData()
        userdata.path = goal.path
        outcome = self.follow_path_task.execute(userdata)
        print('follow_path_callback output: {}'.format(outcome))
        self.follow_path_action_server.set_succeeded()

    def pick_and_place_callback(self, goal):
        flight_level = rospy.get_param('~flight_level')  # TODO: Taking it every callback allows parameter changes...
        z_offset = 0.4  # TODO: offset in meters between uav and attached brick frames
        userdata = smach.UserData()
        userdata.above_pile_pose = copy.deepcopy(goal.pile_pose)
        userdata.above_pile_pose.pose.position.z = flight_level
        userdata.above_wall_pose = copy.deepcopy(goal.brick_in_wall_pose)
        userdata.above_wall_pose.pose.position.z = flight_level
        userdata.brick_in_wall_pose = copy.deepcopy(goal.brick_in_wall_pose)
        userdata.brick_in_wall_pose.pose.position.z += z_offset
        outcome = self.pick_and_place_task.execute(userdata)
        print('pick_and_place_callback output: {}'.format(outcome))
        self.pick_and_place_action_server.set_succeeded()

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
        print('follow_path_callback (go_home) output: {}'.format(outcome))
        if goal.do_land:  # TODO: Land!
            rospy.logwarn('Landing not implemented yet!')
        self.go_home_action_server.set_succeeded()

    def get_cost_to_go_to(self, req):
        # TODO: these try/except inside a function?
        waypoint = PoseStamped()
        try:
           waypoint = self.tf_buffer.transform(req.waypoint, self.ual_pose.header.frame_id, rospy.Duration(1.0))  # TODO: check from/to equality
        except:
            rospy.logerr('Failed to transform waypoint from [{}] to [{}]'.format(req.waypoint.header.frame_id, self.ual_pose.header.frame_id))

        delta_x = waypoint.pose.position.x - self.ual_pose.pose.position.x
        delta_y = waypoint.pose.position.y - self.ual_pose.pose.position.y
        delta_z = waypoint.pose.position.z - self.ual_pose.pose.position.z
        manhattan_distance = abs(delta_x) + abs(delta_y) + abs(delta_z)
        return GetCostToGoToResponse(cost = manhattan_distance)

def main():
    rospy.init_node('uav_agent')

    while rospy.get_rostime() == rospy.Time():
        rospy.logwarn("Waiting for (sim) time to begin!")
        time.sleep(1)

    agent = Agent()
    # agent_id = rospy.get_param('~agent_id')
    # flight_level = rospy.get_param('~flight_level')

    # TODO(performance): Make it optional, use only in develop stage
    # viewer = smach_ros.IntrospectionServer('viewer', agent.follow_path_task, 'UAV_' + str(agent_id))
    # viewer.start()
    rospy.spin()
    # viewer.stop()

if __name__ == '__main__':
    main()


# import roslib
# import rospy
# import smach
# import smach_ros

# from utils.agent import *
# import tasks.uav_tasks.Hovering
# import tasks.uav_tasks.PickAndPlace
# import tasks.uav_tasks.PickFromPileAndPlace
# import tasks.uav_tasks.BuildWall
# import tasks.uav_tasks.SearchForObjects
# import tasks.uav_tasks.GoToWaypoint

# def main():

#     rospy.init_node('uav_agent')

#     # params.
#     id = rospy.get_param('~agent_id')
#     uav_ns = rospy.get_param('~uav_ns')
#     z_offset = rospy.get_param('~z_offset')
#     height = rospy.get_param('~height')
#     global_frame = rospy.get_param('~global_frame')
#     uav_frame = rospy.get_param('~uav_frame')
#     gripper_frame = rospy.get_param('~gripper_frame')
#     aov = rospy.get_param('~aov')

#     # instantiate agent structures
#     agent_props = {}
#     agent_props['type'] = 'UAV'
#     agent_props['global_frame'] = global_frame
#     agent_props['agent_frame'] = uav_frame
#     agent_props['gripper_frame'] = gripper_frame
#     agent_props['height'] = height
#     agent_props['aov'] = aov


#     fsm = AgentStateMachine()
#     iface = AgentInterface(id,fsm,agent_props)

#     # create tasks
#     default_task = tasks.uav_tasks.Hovering.Task('hovering',iface,uav_ns,)
#     tasks_dic = {}
#     add_task('pfpnp_task', tasks_dic, iface, tasks.uav_tasks.PickFromPileAndPlace, [uav_ns, z_offset])
#     add_task('pnp_task', tasks_dic, iface, tasks.uav_tasks.PickAndPlace, [uav_ns, z_offset])
#     add_task('build_wall', tasks_dic, iface, tasks.uav_tasks.BuildWall, [uav_ns, z_offset])
#     add_task('search_region', tasks_dic, iface, tasks.uav_tasks.SearchForObjects, [uav_ns])
#     add_task('go_waypoint', tasks_dic, iface, tasks.uav_tasks.GoToWaypoint, [uav_ns])

#     # initialize state machine
#     d_dic = {'hovering': default_task}
#     fsm.initialize(id, d_dic, 'hovering', tasks_dic)

#     # TODO(performance): Make it optional, use only in develop stage
#     viewer = smach_ros.IntrospectionServer('viewer', fsm, id.upper())
#     viewer.start()

#     # execute state machine
#     userdata = smach.UserData()
#     fsm.execute(userdata)

#     # Wait for ctrl-c to stop the application
#     # rospy.spin()
#     # viewer.stop()

# if __name__ == '__main__':
#     main()
