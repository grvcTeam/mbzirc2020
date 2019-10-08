import rospy
import smach
import smach_ros
import mbzirc_comm_objs.msg
import mbzirc_comm_objs.srv as srv

from timing import SleepAndRetry
from regions import AskForRegionToHover, AskForRegionToMove, FreeRegions


class TakeOff(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['height'])

    def define_for(self, robot):
        with self:

            def take_off_goal_callback(userdata, default_goal):
                robot.set_home()  # TODO: better do it explicitly BEFORE take off?
                goal = mbzirc_comm_objs.msg.TakeOffGoal(height = userdata.height)
                return goal

            smach.StateMachine.add('TAKE_OFF', smach_ros.SimpleActionState(robot.url + 'take_off_action', mbzirc_comm_objs.msg.TakeOffAction,
                                    input_keys = ['height'],
                                    goal_cb = take_off_goal_callback),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_HOVER', 'aborted': 'SLEEP_AND_RETRY'})

            smach.StateMachine.add('SLEEP_AND_RETRY', SleepAndRetry(3.0, 5),
                                    transitions = {'succeeded': 'TAKE_OFF', 'aborted': 'aborted'})

            smach.StateMachine.add('ASK_FOR_REGION_TO_HOVER', AskForRegionToHover().define_for(robot),
                                    transitions = {'succeeded': 'succeeded'})

        return self


class GoTo(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['waypoint'])

    def define_for(self, robot):
        with self:

            smach.StateMachine.add('ASK_FOR_REGION_TO_MOVE', AskForRegionToMove().define_for(robot),
                                    transitions = {'succeeded': 'FREE_LAST_HOVER'})

            smach.StateMachine.add('FREE_LAST_HOVER', FreeRegions().define_for(robot, label = 'hover'),
                                    transitions = {'succeeded': 'GO_TO'})

            def go_to_goal_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.GoToGoal(waypoint = userdata.waypoint)
                return goal

            smach.StateMachine.add('GO_TO', smach_ros.SimpleActionState(robot.url + 'go_to_action', mbzirc_comm_objs.msg.GoToAction,
                                    input_keys = ['waypoint'],
                                    goal_cb = go_to_goal_callback),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_HOVER'})

            smach.StateMachine.add('ASK_FOR_REGION_TO_HOVER', AskForRegionToHover().define_for(robot),
                                    transitions = {'succeeded': 'FREE_LAST_MOVE'})

            smach.StateMachine.add('FREE_LAST_MOVE', FreeRegions().define_for(robot, label = 'go_to'),
                                    transitions = {'succeeded': 'succeeded'})

        return self


class FollowPath(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['path'])
        self.go_to_task = None

    def define_for(self, robot):
        self.go_to_task = GoTo().define_for(robot)
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


class Land(smach.StateMachine):
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

            smach.StateMachine.add('ASK_FOR_REGION_TO_LAND', smach_ros.ServiceState('ask_for_region', srv.AskForRegion,
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'LAND', 'aborted': 'SLEEP_AND_RETRY_ASKING'})

            smach.StateMachine.add('SLEEP_AND_RETRY_ASKING', SleepAndRetry(1.0),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_TO_LAND'})

            # TODO: is this callback needed?
            def land_goal_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.LandGoal()
                return goal

            smach.StateMachine.add('LAND', smach_ros.SimpleActionState(robot.url + 'land_action', mbzirc_comm_objs.msg.LandAction,
                                    # input_keys = ['go_home'],  # TODO: bool go_home?
                                    goal_cb = land_goal_callback),
                                    transitions = {'succeeded': 'ASK_FOR_REGION_LANDED'})

            smach.StateMachine.add('ASK_FOR_REGION_LANDED', smach_ros.ServiceState('ask_for_region', srv.AskForRegion,
                                    request_cb = ask_for_region_request_callback,
                                    response_cb = ask_for_region_response_callback),
                                    transitions = {'succeeded': 'succeeded'})
        return self


# class GoHome(smach.StateMachine):  # TODO?
