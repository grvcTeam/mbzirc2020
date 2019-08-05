#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
import random
import actionlib
import tf2_ros
import tf2_geometry_msgs

from mbzirc_comm_objs.msg import AgentDataFeed
from mbzirc_comm_objs.msg import HoverAction, HoverGoal
from mbzirc_comm_objs.msg import GoToAction, GoToGoal
from mbzirc_comm_objs.msg import FollowPathAction, FollowPathGoal
from geometry_msgs.msg import PoseStamped
from mbzirc_comm_objs.srv import AskForRegion, AskForRegionRequest, GetCostToGoTo, GetCostToGoToResponse

def build_ask_for_region_request(agent_id, initial_pose, final_pose, radius = 1.0):
    if initial_pose.header.frame_id != 'arena':
        raise ValueError('frame_id = {} not expected'.format(initial_pose.header.frame_id))  # TODO: transform?
    if final_pose.header.frame_id != 'arena':
        raise ValueError('frame_id = {} not expected'.format(final_pose.header.frame_id))  # TODO: transform?

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

class Sleep(smach.State):
    def __init__(self, duration = 3.0):
        smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'])  # TODO: duration as an input_key?
        self.duration = duration

    def execute(self, userdata):
        rospy.sleep(self.duration)
        return 'succeeded'
        # TODO: aborted?
        # TODO: preempted?

class GoToTask(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['waypoint'])

        with self:

            # TODO: move outside! Rename action to takeoff!
            def hover_goal_callback(userdata, default_goal):
                flight_level = rospy.get_param('~flight_level')
                goal = HoverGoal(height = flight_level)
                return goal
            # TODO: move outside! Rename action to takeoff!
            smach.StateMachine.add('HOVER', smach_ros.SimpleActionState('hover_action', HoverAction,
                                    input_keys = ['waypoint'],
                                    output_keys = ['waypoint'],
                                    goal_cb = hover_goal_callback),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_MOVE', 'aborted': 'SLEEP_AND_RETRY_HOVER'})

            smach.StateMachine.add('SLEEP_AND_RETRY_HOVER', Sleep(3.0),
                                    transitions = {'succeeded': 'HOVER'})

            # TODO: decorators?
            def ask_for_region_request_callback(userdata, request):
                agent_id = rospy.get_param('~agent_id')
                ask_for_region_request = build_ask_for_region_request(agent_id, self.ual_pose, userdata.waypoint)
                return ask_for_region_request

            def ask_for_region_response_callback(userdata, response):
                if response.success:
                    return 'succeeded'
                else:
                    return 'aborted'

            smach.StateMachine.add('ASK_FOR_REGION_TO_MOVE', smach_ros.ServiceState('/ask_for_region', AskForRegion,
                                    input_keys = ['waypoint'],
                                    output_keys = ['waypoint'],
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
                                    remapping = {'path': 'path', 'waypoint': 'waypoint'},
                                    transitions = {'succeeded': 'succeeded'})

            # smach.StateMachine.add('GO_TO_TASK', GoToTask(), 
            #                         remapping = {'waypoint': 'waypoint'},
            #                         transitions = {'succeeded': 'succeeded', 'aborted': 'SLEEP'}
            # )

            # smach.StateMachine.add('SLEEP', Sleep(1.0),
            #                         # remapping = {'waypoint': 'waypoint'},
            #                         transitions = {'succeeded': 'GO_TO_TASK'})

    # def execute(self, userdata):
    #     for waypoint in userdata.path:
    #         child_userdata = smach.UserData()
    #         child_userdata.waypoint = waypoint
    #         self.execute(child_userdata)

class Agent(object):

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()  # TODO: this will be repated... AgentInterface?
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.follow_path_task = FollowPathTask()
        self.follow_path_action_server = actionlib.SimpleActionServer('task/follow_path', FollowPathAction, execute_cb = self.follow_path_callback, auto_start = False)  # TODO: change naming from task to agent?
        self.follow_path_action_server.start()

        rospy.Service('get_cost_to_go_to', GetCostToGoTo, self.get_cost_to_go_to)

        self.ual_pose = PoseStamped()
        rospy.Subscriber("ual/pose", PoseStamped, self.ual_pose_callback)

        # TODO: Force these lines to be the lasts in construction to avoid ill data_feed?
        self.feed_publisher = rospy.Publisher('data_feed', AgentDataFeed, queue_size = 1)
        rospy.Timer(rospy.Duration(0.2), self.update_feed_callback)  # TODO: duration?

    def update_feed_callback(self, event):
        data_feed = AgentDataFeed()
        data_feed.is_idle = True
        if self.follow_path_task.is_running():
            data_feed.is_idle = False
        # TODO: Check all other tasks!
        self.feed_publisher.publish(data_feed)

    def ual_pose_callback(self, data):  # TODO: this is repeated code, use AgentInterface?
        self.ual_pose = data

    def follow_path_callback(self, goal):
        userdata = smach.UserData()
        userdata.path = goal.path
        outcome = self.follow_path_task.execute(userdata)
        print('follow_path_callback output: {}'.format(outcome))
        self.follow_path_action_server.set_succeeded()

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
