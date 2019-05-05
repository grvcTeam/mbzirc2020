#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

from utils.agent import *
import tasks.test_tasks.DummyTask
import tasks.test_tasks.TestTask

def main():

    rospy.init_node('test_agent')

    # instantiate agent structures
    id = 'test_agent'
    fsm = AgentStateMachine()
    iface = AgentInterface(id,fsm)

    # create tasks
    default_task = tasks.test_tasks.DummyTask.Task('dummy',iface)
    tasks_dic = {}
    add_task('test_task', tasks_dic, iface,  tasks.test_tasks.TestTask)
    add_task('test_task2', tasks_dic, iface,  tasks.test_tasks.TestTask)

    # initialize state machine
    fsm.initialize(id, default_task, tasks_dic)

    # execute state machine
    userdata = smach.UserData()
    fsm.execute(userdata)

if __name__ == '__main__':
    main()
