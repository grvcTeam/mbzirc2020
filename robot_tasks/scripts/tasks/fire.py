import rospy
import smach
import smach_ros
import mbzirc_comm_objs.msg
import mbzirc_comm_objs.srv as srv

# from timing import SleepAndRetry
from move import GoTo
# from regions import FreeRegions


class ExtinguishFacadeFire(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['approximate_pose'])

    def define_for(self, robot):
        with self:

            # TODO: Move in some kind of roundabout arount the building
            smach.StateMachine.add('GO_TO_FACADE_FIRE', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'approximate_pose'},
                                    transitions = {'succeeded': 'EXTINGUISH'})

            def extinguish_facade_fire_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.ExtinguishFacadeFireGoal()
                return goal

            smach.StateMachine.add('EXTINGUISH', smach_ros.SimpleActionState(robot.url + 'extinguish_facade_fire_action', mbzirc_comm_objs.msg.ExtinguishFacadeFireAction,
                                    input_keys = ['above_pile_pose'],
                                    goal_cb = extinguish_facade_fire_callback),
                                    transitions = {'succeeded': 'succeeded'})

        return self
