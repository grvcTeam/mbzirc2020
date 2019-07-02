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

    # instantiate agent structures
    fsm = AgentStateMachine()
    iface = AgentInterface(id,fsm)

    # create tasks
    default_task = tasks.uav_tasks.Hovering.Task('hovering',iface,uav_ns, height, global_frame, uav_frame)
    tasks_dic = {}
    add_task('pfpnp_task', tasks_dic, iface, tasks.uav_tasks.PickFromPileAndPlace, [uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset])
    add_task('pnp_task', tasks_dic, iface, tasks.uav_tasks.PickAndPlace, [uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset])
    add_task('build_wall', tasks_dic, iface, tasks.uav_tasks.BuildWall, [uav_ns, height, global_frame, uav_frame, gripper_frame, z_offset])

    # initialize state machine
    fsm.initialize(id, default_task, tasks_dic)

    # TODO(performance): Make it optional, use only in develop stage
    viewer = smach_ros.IntrospectionServer('viewer', fsm, id.upper())
    viewer.start()

    # execute state machine
    userdata = smach.UserData()
    userdata.height = 2.0
    fsm.execute(userdata)

    # Wait for ctrl-c to stop the application
    # rospy.spin()
    # viewer.stop()

if __name__ == '__main__':
    main()
