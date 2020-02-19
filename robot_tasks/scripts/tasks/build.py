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

            # smach.StateMachine.add('GO_TO_WAITING_AREA', GoTo().define_for(robot),
            #                         remapping = {'waypoint': 'waiting_pose'},
            #                         transitions = {'succeeded': 'ASK_FOR_REGION_TO_PICK'})

            def ask_for_region_to_pick_request_callback(userdata, request):
                request = SetBoolRequest()
                request.data = True
                return request

            def ask_for_region_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('ASK_FOR_REGION_TO_PICK', smach_ros.ServiceState('lock_pick_region', SetBool,
                                    request_cb = ask_for_region_to_pick_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'GO_TO_PILE', 'aborted': 'SLEEP_AND_RETRY_ASKING_TO_PICK'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING_TO_PICK', SleepAndRetry(3.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_PICK'})

            smach.StateMachine.add('GO_TO_PILE', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'above_pile_pose'},
                                    transitions = {'succeeded': 'PICK'})

            def pick_goal_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.PickGoal(color = userdata.color)
                return goal

            smach.StateMachine.add('PICK', smach_ros.SimpleActionState(robot.url + 'pick_action', mbzirc_comm_objs.msg.PickAction,
                                    input_keys = ['color'],
                                    goal_cb = pick_goal_callback),
                                    transitions = {'succeeded': 'GO_UP'})

            smach.StateMachine.add('GO_UP', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'above_pile_pose'},
                                    transitions = {'succeeded': 'FREE_REGION_TO_PICK'})

            # TODO: BACK_TO_WAITING_POSE

            def free_pick_region_request_callback(userdata, request):
                request = SetBoolRequest()
                request.data = False
                return request

            def free_pick_region_response_callback(userdata, response):
                return 'succeeded'

            smach.StateMachine.add('FREE_REGION_TO_PICK', smach_ros.ServiceState('lock_pick_region', SetBool,
                                    request_cb = free_pick_region_request_callback,
                                    response_cb = free_pick_region_response_callback),
                                    transitions = {'succeeded': 'succeeded'})

        return self


class Place(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['waiting_pose', 'segment_to_the_left_pose', 'segment_offset'])

    def define_for(self, robot):
        with self:

            # smach.StateMachine.add('GO_TO_WAITING_AREA', GoTo().define_for(robot),
            #                         remapping = {'waypoint': 'waiting_pose'},
            #                         transitions = {'succeeded': 'ASK_FOR_REGION_TO_PLACE'})

            def ask_for_region_to_place_request_callback(userdata, request):
                request = SetBoolRequest()
                request.data = True
                return request

            def ask_for_region_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('ASK_FOR_REGION_TO_PLACE', smach_ros.ServiceState('lock_place_region', SetBool,
                                    request_cb = ask_for_region_to_place_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'GO_SEGMENT_TO_THE_LEFT', 'aborted': 'SLEEP_AND_RETRY_ASKING_TO_PLACE'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING_TO_PLACE', SleepAndRetry(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_PLACE'})

            smach.StateMachine.add('GO_SEGMENT_TO_THE_LEFT', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'segment_to_the_left_pose'},
                                    transitions = {'succeeded': 'PLACE'})

            def place_goal_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.PlaceGoal(in_wall_brick_pose = userdata.segment_offset)
                return goal

            smach.StateMachine.add('PLACE', smach_ros.SimpleActionState(robot.url + 'place_action', mbzirc_comm_objs.msg.PlaceAction,
                                    input_keys = ['segment_offset'],
                                    goal_cb = place_goal_callback),
                                    transitions = {'succeeded': 'GO_UP'})

            smach.StateMachine.add('GO_UP', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'segment_to_the_left_pose'},
                                    transitions = {'succeeded': 'FREE_REGION_TO_PLACE'})

            # TODO: BACK_TO_WAITING_POSE

            def free_place_region_request_callback(userdata, request):
                request = SetBoolRequest()
                request.data = False
                return request

            def free_place_region_response_callback(userdata, response):
                return 'succeeded'

            smach.StateMachine.add('FREE_REGION_TO_PLACE', smach_ros.ServiceState('lock_place_region', SetBool,
                                    request_cb = free_place_region_request_callback,
                                    response_cb = free_place_region_response_callback),
                        transitions = {'succeeded': 'succeeded'})

        return self
