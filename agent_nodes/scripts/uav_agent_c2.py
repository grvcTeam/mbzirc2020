#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time

from mbzirc_comm_objs.msg import HoverAction, HoverGoal
from mbzirc_comm_objs.msg import GoToAction, GoToGoal
from mbzirc_comm_objs.msg import FollowPathAction, FollowPathGoal
from geometry_msgs.msg import PoseStamped
# from smach_tutorials.msg import TestAction, TestGoal
# from actionlib import *
# from actionlib_msgs.msg import *


def main():
    rospy.init_node('uav_agent')

    while rospy.get_rostime() == rospy.Time():
        rospy.logwarn("Waiting for (sim) time to begin!")
        time.sleep(1)

    # Create a SMACH state machine
    fsm = smach.StateMachine(outcomes = ['succeeded', 'aborted', 'preempted'])

    # Open the container
    with fsm:
        # Add states to the container

        # Add a simple action state. This will use an empty, default goal
        # As seen in TestServer above, an empty goal will always return with
        # GoalStatus.SUCCEEDED, causing this simple action state to return
        # the outcome 'succeeded'
        # smach.StateMachine.add('GOAL_DEFAULT',
        #                        smach_ros.SimpleActionState('test_action', TestAction),
        #                        {'succeeded':'GOAL_STATIC'})

        # Add another simple action state. This will give a goal
        # that should abort the action state when it is received, so we
        # map 'aborted' for this state onto 'succeeded' for the state machine.
        # smach.StateMachine.add('GOAL_STATIC',
        #                        smach_ros.SimpleActionState('test_action', TestAction,
        #                                                goal = TestGoal(goal=1)),
        #                        {'aborted':'GOAL_CB'})
        smach.StateMachine.add('HOVER',
                               smach_ros.SimpleActionState('hover_action', HoverAction,
                                                       goal = HoverGoal(height = 2.0)),  # TODO: height from param?
                               {'succeeded':'GO_TO'})

        wp = PoseStamped()
        wp.pose.position.x = 1
        wp.pose.position.y = 1
        wp.pose.position.z = 1
        # wp.pose.orientation.w = 1

        smach.StateMachine.add('GO_TO',
                               smach_ros.SimpleActionState('go_to_action', GoToAction,
                                                       goal = GoToGoal(waypoint = wp)),
                               {'succeeded':'FOLLOW_PATH'})

        smach.StateMachine.add('FOLLOW_PATH',
                               smach_ros.SimpleActionState('follow_path_action', FollowPathAction,
                                                       goal = FollowPathGoal(path = [PoseStamped(), wp, PoseStamped()])),
                               {'succeeded':'succeeded'})


        # smach.StateMachine.add('PICK',
        #                        smach_ros.SimpleActionState('ual_action', UALAction,
        #                                                goal = UALGoal(command=UALGoal.PICK)),
        #                        {'succeeded':'succeeded'})


        # Add another simple action state. This will give a goal
        # that should abort the action state when it is received, so we
        # map 'aborted' for this state onto 'succeeded' for the state machine.
        # def goal_callback(userdata, default_goal):
        #     goal = TestGoal()
        #     goal.goal = 2
        #     return goal

        # smach.StateMachine.add('GOAL_CB',
        #                        smach_ros.SimpleActionState('test_action', TestAction,
        #                                                goal_cb = goal_callback),
        #                        {'aborted':'succeeded'})

        # For more examples on how to set goals and process results, see 
        # executive_smach/smach_ros/tests/smach_actionlib.py

    # Execute SMACH plan
    outcome = fsm.execute()

    rospy.signal_shutdown('All done.')


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
