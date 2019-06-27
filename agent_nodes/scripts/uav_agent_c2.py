#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

from utils.agent import *
import tasks.uav_tasks.Hovering
import tasks.uav_tasks.PickAndPlace
import tasks.uav_tasks.PickFromPileAndPlace
import tasks.uav_tasks.BuildWall
import tasks.uav_tasks.SearchForObjects
import tasks.uav_tasks.GoToWaypoint

def main():

    rospy.init_node('uav_agent')

    # params.
    id = rospy.get_param('~agent_id')
    uav_ns = rospy.get_param('~uav_ns')
    z_offset = rospy.get_param('~z_offset')
    height = rospy.get_param('~height')
    global_frame = rospy.get_param('~global_frame')
    uav_frame = rospy.get_param('~uav_frame')
    gripper_frame = rospy.get_param('~gripper_frame')
    aov = rospy.get_param('~aov')

    # instantiate agent structures
    agent_props = {}
    agent_props['type'] = 'UAV'
    agent_props['global_frame'] = global_frame
    agent_props['agent_frame'] = uav_frame
    agent_props['gripper_frame'] = gripper_frame
    agent_props['height'] = height
    agent_props['aov'] = aov


    fsm = AgentStateMachine()
    iface = AgentInterface(id,fsm,agent_props)

    # create tasks
    default_task = tasks.uav_tasks.Hovering.Task('hovering',iface,uav_ns, height, global_frame, uav_frame)
    tasks_dic = {}
    add_task('pfpnp_task', tasks_dic, iface, tasks.uav_tasks.PickFromPileAndPlace, [uav_ns, z_offset])
    add_task('pnp_task', tasks_dic, iface, tasks.uav_tasks.PickAndPlace, [uav_ns, z_offset])
    add_task('build_wall', tasks_dic, iface, tasks.uav_tasks.BuildWall, [uav_ns, z_offset])
    add_task('search_region', tasks_dic, iface, tasks.uav_tasks.SearchForObjects, [uav_ns])
    add_task('go_waypoint', tasks_dic, iface, tasks.uav_tasks.GoToWaypoint, [uav_ns])

    # initialize state machine
    d_dic = {'hovering': default_task}
    fsm.initialize(id, d_dic, 'hovering', tasks_dic)

    # execute state machine
    userdata = smach.UserData()
    fsm.execute(userdata)

if __name__ == '__main__':
    main()
