import rospy
import smach
import smach_ros
import mbzirc_comm_objs.msg
import mbzirc_comm_objs.srv as srv

from std_srvs.srv import SetBool, SetBoolRequest
from timing import SleepAndRetry
from move import GoTo


class Pick(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['color', 'waiting_pose', 'above_pile_pose'])

    def define_for(self, robot):
        with self:

            def lock_piles_area_request_callback(userdata, request):
                request = SetBoolRequest()
                request.data = True
                return request

            def lock_piles_area_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('LOCK_PILES_AREA', smach_ros.ServiceState('lock_pick_region', SetBool,
                                    request_cb = lock_piles_area_request_callback,
                                    response_cb = lock_piles_area_response_callback),
                                    transitions = {'succeeded': 'GO_TO_PILES', 'aborted': 'SLEEP_AND_RETRY_LOCK_PILES_AREA'})

            smach.StateMachine.add('SLEEP_AND_RETRY_LOCK_PILES_AREA', SleepAndRetry(3.0),
                                    transitions = {'succeeded': 'LOCK_PILES_AREA'})

            smach.StateMachine.add('GO_TO_PILES', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'above_pile_pose'},
                                    transitions = {'succeeded': 'FREE_WAITING_SPOT'})

            def free_waiting_spot_request_callback(userdata, request):
                request = SetBoolRequest()
                request.data = False
                return request

            def free_waiting_spot_response_callback(userdata, response):
                return 'succeeded'

            smach.StateMachine.add('FREE_WAITING_SPOT', smach_ros.ServiceState('lock_wait_region', SetBool,
                                    request_cb = free_waiting_spot_request_callback,
                                    response_cb = free_waiting_spot_response_callback),
                                    transitions = {'succeeded': 'PICK_ACTION'})

            def pick_action_goal_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.PickGoal(color = userdata.color)
                return goal

            smach.StateMachine.add('PICK_ACTION', smach_ros.SimpleActionState(robot.url + 'pick_action', mbzirc_comm_objs.msg.PickAction,
                                    input_keys = ['color'],
                                    goal_cb = pick_action_goal_callback),
                                    transitions = {'succeeded': 'succeeded', 'aborted': 'BACK_TO_WAITING_AREA'})

            smach.StateMachine.add('BACK_TO_WAITING_AREA', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'waiting_pose'},
                                    transitions = {'succeeded': 'FREE_PILES'})

            def free_piles_request_callback(userdata, request):
                request = SetBoolRequest()
                request.data = False
                return request

            def free_piles_response_callback(userdata, response):
                return 'succeeded'

            smach.StateMachine.add('FREE_PILES', smach_ros.ServiceState('lock_pick_region', SetBool,
                                    request_cb = free_piles_request_callback,
                                    response_cb = free_piles_response_callback),
                                    transitions = {'succeeded': 'aborted'})

        return self


class Place(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['waiting_pose', 'segment_to_the_left_pose', 'segment_offset'])

    def define_for(self, robot):
        with self:

            def lock_wall_area_request_callback(userdata, request):
                request = SetBoolRequest()
                request.data = True
                return request

            def lock_wall_area_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('LOCK_WALL_AREA', smach_ros.ServiceState('lock_place_region', SetBool,
                                    request_cb = lock_wall_area_request_callback,
                                    response_cb = lock_wall_area_response_callback),
                                    transitions = {'succeeded': 'GO_SEGMENT_TO_THE_LEFT', 'aborted': 'SLEEP_AND_RETRY_LOCK_WALL_AREA'})

            smach.StateMachine.add('SLEEP_AND_RETRY_LOCK_WALL_AREA', SleepAndRetry(1.0),
                                    transitions = {'succeeded': 'LOCK_WALL_AREA'})

            smach.StateMachine.add('GO_SEGMENT_TO_THE_LEFT', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'segment_to_the_left_pose'},
                                    transitions = {'succeeded': 'FREE_PILES'})

            def free_piles_request_callback(userdata, request):
                request = SetBoolRequest()
                request.data = False
                return request

            def free_piles_response_callback(userdata, response):
                return 'succeeded'

            smach.StateMachine.add('FREE_PILES', smach_ros.ServiceState('lock_pick_region', SetBool,
                                    request_cb = free_piles_request_callback,
                                    response_cb = free_piles_response_callback),
                                    transitions = {'succeeded': 'PLACE_ACTION'})

            def place_action_goal_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.PlaceGoal(wall_y_offset = userdata.segment_offset)
                return goal

            smach.StateMachine.add('PLACE_ACTION', smach_ros.SimpleActionState(robot.url + 'place_action', mbzirc_comm_objs.msg.PlaceAction,
                                    input_keys = ['segment_offset'],
                                    goal_cb = place_action_goal_callback),
                                    transitions = {'succeeded': 'BACK_TO_WAITING_AREA_SUCCEEDED', 'aborted': 'BACK_TO_WAITING_AREA_FAILED'})

            smach.StateMachine.add('BACK_TO_WAITING_AREA_SUCCEEDED', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'waiting_pose'},
                                    transitions = {'succeeded': 'FREE_WALL_SUCCEEDED'})

            def free_wall_request_callback(userdata, request):
                request = SetBoolRequest()
                request.data = False
                return request

            def free_wall_response_callback(userdata, response):
                return 'succeeded'

            smach.StateMachine.add('FREE_WALL_SUCCEEDED', smach_ros.ServiceState('lock_place_region', SetBool,
                                    request_cb = free_wall_request_callback,
                                    response_cb = free_wall_response_callback),
                        transitions = {'succeeded': 'succeeded'})

            smach.StateMachine.add('BACK_TO_WAITING_AREA_FAILED', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'waiting_pose'},
                                    transitions = {'succeeded': 'FREE_WALL_FAILED'})

            smach.StateMachine.add('FREE_WALL_FAILED', smach_ros.ServiceState('lock_place_region', SetBool,
                                    request_cb = free_wall_request_callback,
                                    response_cb = free_wall_response_callback),
                        transitions = {'succeeded': 'aborted'})

        return self
