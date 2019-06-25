#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

from utils.agent import *
import tasks.central_unit_tasks.Idle
import tasks.central_unit_tasks.SearchEnvironment
import tasks.central_unit_tasks.BuildWall
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
    add_task('search_env', tasks_dic, iface, tasks.central_unit_tasks.SearchEnvironment, [])
    add_task('build_wall', tasks_dic, iface, tasks.central_unit_tasks.BuildWall, [])
    add_task('search_and_build', tasks_dic, iface, tasks.central_unit_tasks.SearchAndBuild, [])

    # initialize state machine
    d_dic = {'idle': default_task}
    fsm.initialize(id, d_dic, 'idle', tasks_dic)

    # execute state machine
    userdata = smach.UserData()
    fsm.execute(userdata)

if __name__ == '__main__':
    main()
