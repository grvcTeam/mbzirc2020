#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

from utils.agent import *
import tasks.ugv_tasks.Idle
import tasks.ugv_tasks.GoToGripPose
import tasks.ugv_tasks.PickObject

def main():

    rospy.init_node('ugv_agent')

    # params.
    id = rospy.get_param('~agent_id')
    ugv_ns = rospy.get_param('~ugv_ns')
    z_offset = rospy.get_param('~z_offset')
    global_frame = rospy.get_param('~global_frame')
    ugv_frame = rospy.get_param('~ugv_frame')
    gripper_frame = rospy.get_param('~gripper_frame')

    base_aabb = [-0.32, -0.34, 0.32, 0.34]
    ws_aabb = [-0.32, -0.49, 0.32, 0.49]

    # instantiate agent structures
    fsm = AgentStateMachine()
    iface = AgentInterface(id,fsm)

    # create tasks
    default_task = tasks.ugv_tasks.Idle.Task('idle',iface)
    tasks_dic = {}
    add_task('gogrip_task', tasks_dic, iface, tasks.ugv_tasks.GoToGripPose, [ugv_ns, global_frame, ugv_frame, base_aabb, ws_aabb])
    add_task('pick_task', tasks_dic, iface, tasks.ugv_tasks.PickObject, [ugv_ns, global_frame, ugv_frame, base_aabb, ws_aabb, gripper_frame, z_offset])

    # initialize state machine
    fsm.initialize(id, default_task, tasks_dic)

    # execute state machine
    userdata = smach.UserData()
    fsm.execute(userdata)

if __name__ == '__main__':
    main()
