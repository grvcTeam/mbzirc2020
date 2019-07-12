#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

import threading

from utils.agent import *
import tasks.ugv_tasks.Idle
import tasks.ugv_tasks.PickAndPlace
import tasks.ugv_tasks.PickFromPileAndPlace
import tasks.ugv_tasks.GoToWaypoint

def main():

    rospy.init_node('ugv_agent')

    # params.
    id = rospy.get_param('~agent_id')
    ugv_ns = rospy.get_param('~ugv_ns')
    z_offset = rospy.get_param('~z_offset')
    global_frame = rospy.get_param('~global_frame')
    ugv_frame = rospy.get_param('~ugv_frame')
    gripper_frame = rospy.get_param('~gripper_frame')

    base_aabb = [-0.32, -0.34, 0.37, 0.34]

    # instantiate agent structures
    agent_props = {}
    agent_props['type'] = 'UGV'
    agent_props['global_frame'] = global_frame
    agent_props['agent_frame'] = ugv_frame
    agent_props['gripper_frame'] = gripper_frame
    agent_props['rb_aabb'] = base_aabb

    fsm = AgentStateMachine()
    iface = AgentInterface(id,fsm,agent_props)

    # create tasks
    default_task = tasks.ugv_tasks.Idle.Task('idle',iface)
    tasks_dic = {}
    add_task('pplace_task', tasks_dic, iface, tasks.ugv_tasks.PickAndPlace, [ugv_ns, z_offset])
    add_task('pfpnp_task', tasks_dic, iface, tasks.ugv_tasks.PickFromPileAndPlace, [ugv_ns, z_offset])
    add_task('go_waypoint', tasks_dic, iface, tasks.ugv_tasks.GoToWaypoint, [ugv_ns])

    # initialize state machine
    d_dic = {'idle': default_task}
    fsm.initialize(id, d_dic, 'idle', tasks_dic)

    # execute state machine
    userdata = smach.UserData()
    #fsm.execute(userdata)

    th = threading.Thread(target=fsm.execute, args=[userdata,])
    th.start()

    rospy.spin()

if __name__ == '__main__':
    main()
