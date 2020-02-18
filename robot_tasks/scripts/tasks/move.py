import rospy
import smach
import smach_ros
import mbzirc_comm_objs.msg
import mbzirc_comm_objs.srv as srv

from timing import SleepAndRetry


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
                                    transitions = {'succeeded': 'succeeded', 'aborted': 'SLEEP_AND_RETRY'})

            smach.StateMachine.add('SLEEP_AND_RETRY', SleepAndRetry(3.0, 5),
                                    transitions = {'succeeded': 'TAKE_OFF', 'aborted': 'aborted'})

        return self


class GoTo(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['waypoint'])

    def define_for(self, robot):
        with self:

            def go_to_goal_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.GoToGoal(waypoint = userdata.waypoint)
                return goal

            smach.StateMachine.add('GO_TO', smach_ros.SimpleActionState(robot.url + 'go_to_action', mbzirc_comm_objs.msg.GoToAction,
                                    input_keys = ['waypoint'],
                                    goal_cb = go_to_goal_callback),
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

            # TODO: is this callback needed?
            def land_goal_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.LandGoal()
                return goal

            smach.StateMachine.add('LAND', smach_ros.SimpleActionState(robot.url + 'land_action', mbzirc_comm_objs.msg.LandAction,
                                    # input_keys = ['go_home'],  # TODO: bool go_home?
                                    goal_cb = land_goal_callback),
                                    transitions = {'succeeded': 'succeeded'})
        return self


# class GoHome(smach.StateMachine):  # TODO?
