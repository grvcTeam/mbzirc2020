#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

from utils.agent import *
import tasks.central_unit_tasks.Idle
import tasks.central_unit_tasks.SearchForBrickPiles
import tasks.central_unit_tasks.CatchBalloons
import tasks.central_unit_tasks.SearchAndCatch
import tasks.central_unit_tasks.SearchAndBuild

def main():

    rospy.init_node('uav_agent')

    # params.
    id = 'central_unit'#rospy.get_param('~agent_id')

    # instantiate agent structures
    fsm = AgentStateMachine()
    iface = AgentInterface(id,fsm,{'type':'central_unit'})

    # create tasks
    default_task = tasks.central_unit_tasks.Idle.Task('idle',iface)
    tasks_dic = {}
    add_task('search_env', tasks_dic, iface, tasks.central_unit_tasks.SearchForBrickPiles, [])
    add_task('catch_balloons', tasks_dic, iface, tasks.central_unit_tasks.CatchBalloons, [])
    add_task('search_and_build', tasks_dic, iface, tasks.central_unit_tasks.SearchAndBuild, [])
    add_task('search_and_catch', tasks_dic, iface, tasks.central_unit_tasks.SearchAndCatch, [])

    # initialize state machine
    d_dic = {'idle': default_task}
    fsm.initialize(id, d_dic, 'idle', tasks_dic)

    # execute state machine
    userdata = smach.UserData()
    fsm.execute(userdata)

if __name__ == '__main__':
    main()
