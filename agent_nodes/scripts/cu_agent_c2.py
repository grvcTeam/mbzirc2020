#!/usr/bin/env python

import rospy
# import smach
import actionlib
import mbzirc_comm_objs.msg
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':

    rospy.init_node('cu_agent_c2')
    available_uavs = [1, 2]  # TODO: auto discovery (and update!)

    uav_client = {}
    for i in range(1,4):
        if i in available_uavs:
            uav_id = str(i)
            uav_client[uav_id] = actionlib.SimpleActionClient('/uav_' + uav_id + '/task/follow_path', mbzirc_comm_objs.msg.FollowPathAction)

    for uav_id in uav_client:
        print('waiting for server {}'.format(uav_id))
        uav_client[uav_id].wait_for_server()

    flight_level = 2.0
    goal = mbzirc_comm_objs.msg.FollowPathGoal()
    for i in range(3):
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'map'
        waypoint.pose.position.x = i*10
        waypoint.pose.position.y = 1
        waypoint.pose.position.z = flight_level
        waypoint.pose.orientation.z = 0
        waypoint.pose.orientation.w = 1
        goal.path.append(waypoint)

    for uav_id in uav_client:
        print('sending goal to server {}'.format(uav_id))
        uav_client[uav_id].send_goal(goal)

    for uav_id in uav_client:
        print('waiting result of server {}'.format(uav_id))
        uav_client[uav_id].wait_for_result()
        print(uav_client[uav_id].get_result())


# import roslib
# import rospy
# import smach
# import smach_ros

# from utils.agent import *
# import tasks.central_unit_tasks.Idle
# import tasks.central_unit_tasks.SearchForBrickPiles
# import tasks.central_unit_tasks.CatchBalloons
# import tasks.central_unit_tasks.SearchAndCatch
# import tasks.central_unit_tasks.SearchAndBuild

# def main():

#     rospy.init_node('uav_agent')

#     # params.
#     id = 'central_unit'#rospy.get_param('~agent_id')

#     # instantiate agent structures
#     fsm = AgentStateMachine()
#     iface = AgentInterface(id,fsm,{'type':'central_unit'})

#     # create tasks
#     default_task = tasks.central_unit_tasks.Idle.Task('idle',iface)
#     tasks_dic = {}
#     add_task('search_env', tasks_dic, iface, tasks.central_unit_tasks.SearchForBrickPiles, [])
#     add_task('catch_balloons', tasks_dic, iface, tasks.central_unit_tasks.CatchBalloons, [])
#     add_task('search_and_build', tasks_dic, iface, tasks.central_unit_tasks.SearchAndBuild, [])
#     add_task('search_and_catch', tasks_dic, iface, tasks.central_unit_tasks.SearchAndCatch, [])

#     # initialize state machine
#     d_dic = {'idle': default_task}
#     fsm.initialize(id, d_dic, 'idle', tasks_dic)

#     # execute state machine
#     userdata = smach.UserData()
#     fsm.execute(userdata)

# if __name__ == '__main__':
#     main()
