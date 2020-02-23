import rospy
import smach
import smach_ros
import mbzirc_comm_objs.msg
import mbzirc_comm_objs.srv as srv

# from timing import SleepAndRetry
from move import GoTo

class GoToFacadeFire(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['fires_file', 'fire_id'])

    def define_for(self, robot):
        with self:

            def go_to_facade_fire_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.GoToFacadeFireGoal()
                goal.fires_file = userdata.fires_file
                goal.fire_id = userdata.fire_id
                return goal

            smach.StateMachine.add('GO_TO_FACADE_FIRE', smach_ros.SimpleActionState(robot.url + 'go_to_facade_fire_action', mbzirc_comm_objs.msg.GoToFacadeFireAction,
                                    input_keys = ['fires_file', 'fire_id'],
                                    goal_cb = go_to_facade_fire_callback),
                                    transitions = {'succeeded': 'succeeded'})

        return self
class ExtinguishFacadeFire(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'])

    def define_for(self, robot):
        with self:

            # TODO: Move in some kind of roundabout arount the building
            # smach.StateMachine.add('GO_TO_FACADE_FIRE', GoTo().define_for(robot),
            #                         remapping = {'waypoint': 'approximate_pose'},
            #                         transitions = {'succeeded': 'EXTINGUISH'})

            def extinguish_facade_fire_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.ExtinguishFacadeFireGoal()
                return goal

            smach.StateMachine.add('EXTINGUISH', smach_ros.SimpleActionState(robot.url + 'extinguish_facade_fire_action', mbzirc_comm_objs.msg.ExtinguishFacadeFireAction,
                                    goal_cb = extinguish_facade_fire_callback),
                                    transitions = {'succeeded': 'succeeded'})

        return self

class ExtinguishGroundFire(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['path', 'color'])

    def define_for(self, robot):
        with self:

            def look_for_ground_fires_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.LookForGroundFiresGoal(path = userdata.path)
                return goal

            smach.StateMachine.add('LOOK_FOR_GROUND_FIRES', smach_ros.SimpleActionState(robot.url + 'look_for_ground_fires_action', mbzirc_comm_objs.msg.LookForGroundFiresAction,
                                    input_keys = ['path'],
                                    goal_cb = look_for_ground_fires_callback),
                                    transitions = {'succeeded': 'EXTINGUISH'})

            def extinguish_ground_fire_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.ExtinguishGroundFireGoal(color = userdata.color)
                return goal

            smach.StateMachine.add('EXTINGUISH', smach_ros.SimpleActionState(robot.url + 'extinguish_ground_fire_action', mbzirc_comm_objs.msg.ExtinguishGroundFireAction,
                                    input_keys = ['color'],
                                    goal_cb = extinguish_ground_fire_callback),
                                    transitions = {'succeeded': 'succeeded'})

        return self

class ExtinguishGroundFireSimple(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['waypoint', 'color'])

    def define_for(self, robot):
        with self:

            smach.StateMachine.add('GO_TO_FIRE', GoTo().define_for(robot),
                                    remapping = {'waypoint': 'waypoint'},
                                    transitions = {'succeeded': 'EXTINGUISH'})

            def extinguish_ground_fire_callback(userdata, default_goal):
                goal = mbzirc_comm_objs.msg.ExtinguishGroundFireGoal(color = userdata.color)
                return goal

            smach.StateMachine.add('EXTINGUISH', smach_ros.SimpleActionState(robot.url + 'extinguish_ground_fire_action', mbzirc_comm_objs.msg.ExtinguishGroundFireAction,
                                    input_keys = ['color'],
                                    goal_cb = extinguish_ground_fire_callback),
                                    transitions = {'succeeded': 'succeeded'})

        return self