import rospy
import smach
import smach_ros
import mbzirc_comm_objs.msg
import mbzirc_comm_objs.srv as srv

from timing import SleepAndRetry
from move import GoTo
from regions import FreeRegions


class PickAndPlace(smach.StateMachine):
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

            smach.StateMachine.add('ASK_FOR_REGION_TO_PICK', smach_ros.ServiceState('ask_for_region', srv.AskForRegion,
                                    input_keys = ['above_pile_pose'],
                                    request_cb = ask_for_region_to_pick_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'GO_TO_PILE', 'aborted': 'SLEEP_AND_RETRY_ASKING_TO_PICK'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING_TO_PICK', SleepAndRetry(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_PICK'})

            smach.StateMachine.add('GO_TO_PILE', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'above_pile_pose'},
                                    transitions = {'succeeded': 'PICK'})

            def pick_goal_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.PickGoal(approximate_pose = userdata.above_pile_pose)
                return goal

            smach.StateMachine.add('PICK', smach_ros.SimpleActionState(robot.url + 'pick_action', mbzirc_comm_objs.msg.PickAction,
                                    input_keys = ['above_pile_pose'],
                                    goal_cb = pick_goal_callback),
                                    transitions = {'succeeded': 'GO_UP'})

            smach.StateMachine.add('GO_UP', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'above_pile_pose'},
                                    transitions = {'succeeded': 'FREE_REGION_TO_PICK'})

            smach.StateMachine.add('FREE_REGION_TO_PICK', FreeRegions().define_for(robot, label = 'pick'),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_PLACE'})

            def ask_for_region_to_place_request_callback(userdata, request):
                radius = 3.0  # TODO: Tune, assure uav is not going further while place!
                z_max = userdata.above_wall_pose.pose.position.z
                request = robot.build_request_for_vertical_region(userdata.above_wall_pose, 'place', radius, z_max = z_max)
                return request

            smach.StateMachine.add('ASK_FOR_REGION_TO_PLACE', smach_ros.ServiceState('ask_for_region', srv.AskForRegion,
                                    input_keys = ['above_wall_pose'],
                                    request_cb = ask_for_region_to_place_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'GO_ABOVE_WALL', 'aborted': 'SLEEP_AND_RETRY_ASKING_TO_PLACE'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING_TO_PLACE', SleepAndRetry(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_PLACE'})

            smach.StateMachine.add('GO_ABOVE_WALL', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'above_wall_pose'},
                                    transitions = {'succeeded': 'PLACE'})

            def place_goal_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.PlaceGoal(in_wall_brick_pose = userdata.in_wall_brick_pose)
                return goal

            smach.StateMachine.add('PLACE', smach_ros.SimpleActionState(robot.url + 'place_action', mbzirc_comm_objs.msg.PlaceAction,
                                    input_keys = ['in_wall_brick_pose'],
                                    goal_cb = place_goal_callback),
                                    transitions = {'succeeded': 'GO_UP_AGAIN'})

            smach.StateMachine.add('GO_UP_AGAIN', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'above_wall_pose'},
                                    transitions = {'succeeded': 'FREE_REGION_TO_PLACE'})
            
            smach.StateMachine.add('FREE_REGION_TO_PLACE', FreeRegions().define_for(robot, label = 'place'),
                        transitions = {'succeeded': 'succeeded'})

        return self

