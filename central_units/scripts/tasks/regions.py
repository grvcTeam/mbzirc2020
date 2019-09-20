import rospy
import smach
import smach_ros
import mbzirc_comm_objs.srv as srv

from timing import SleepAndRetry


class AskForRegionToHover(smach.StateMachine):
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

            smach.StateMachine.add('ASK_FOR_REGION', smach_ros.ServiceState('ask_for_region', srv.AskForRegion,
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'succeeded', 'aborted': 'SLEEP_AND_RETRY_ASKING'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING', SleepAndRetry(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION'})
        return self


class AskForRegionToMove(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['waypoint'])

    def define_for(self, robot):
        with self:

            def ask_for_region_request_callback(userdata, request):
                ask_for_region_request = robot.build_request_for_go_to_region(userdata.waypoint, 'go_to', 1.0)
                return ask_for_region_request

            def ask_for_region_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('ASK_FOR_REGION', smach_ros.ServiceState('ask_for_region', srv.AskForRegion,
                                    input_keys = ['waypoint'],
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'succeeded', 'aborted': 'SLEEP_AND_RETRY_ASKING'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING', SleepAndRetry(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION'})
        return self


class FreeRegions(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'])

    def define_for(self, robot, label, free_all = False):
        with self:

            def free_regions_request_callback(userdata, request):
                free_regions_request = srv.FreeRegionsRequest()
                free_regions_request.agent_id = robot.id
                free_regions_request.label = label
                free_regions_request.free_all = free_all
                return free_regions_request

            def free_regions_response_callback(userdata, response):
                return 'succeeded' if response.success else 'aborted'

            smach.StateMachine.add('FREE_REGIONS', smach_ros.ServiceState('free_regions', srv.FreeRegions,
                                    request_cb = free_regions_request_callback,
                                    response_cb = free_regions_response_callback),
                                    transitions = {'succeeded': 'succeeded', 'aborted': 'SLEEP_AND_RETRY'})

            smach.StateMachine.add('SLEEP_AND_RETRY', SleepAndRetry(1.0),
                                    transitions = {'succeeded': 'FREE_REGIONS'})
        return self
