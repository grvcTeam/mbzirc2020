#! /usr/bin/env python
import time
import rospy
import smach
# import tf2_ros
# import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped


# PARAMETER	SPECIFICATION
# Number of UAVs per team	Maximum of 3
# Number of UGVs per team	1
# Arena size	50mx60mx20m
# Brick shapes and material	Rectangular cube, Styrofoam material
# Bricks size (Red, Green, Blue)	Approximately 0.30mx0.20mx0.20m, 0.60mx0.20mx0.20m and1.20x0.20x0.20m
# Bricks size (Orange)	1.80x0.20x0.20 m
# Weight of bricks	O <= 2.0kg , B <= 1.5kg , G <= 1kg , R <= 1kg,
# Brick gripping mechanism	Primarily magnetic, but other gripping mechanisms could be used
# Environment	Outdoor
# Mode of operation	Autonomous; manual allowed but penalized
# RTK/DGPS	Allowed but penalized
# Challenge duration	30 minutes
# Communications	TBD

class GoToTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['waypoint'])

    def execute(self, userdata):
        print('going to: {}'.format(userdata.waypoint))
        return 'succeeded'

class PickAndPlaceTask(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'], input_keys = ['pile_pose', 'brick_in_wall_pose'])
        self.register_input_keys(['above_pile_pose', 'above_brick_in_wall_pose'])

        with self:

            smach.StateMachine.add('GO_TO_PILE', GoToTask(),
                                    remapping = {'waypoint': 'pile_pose'},
                                    transitions = {'succeeded': 'GO_TO_WALL'})

            smach.StateMachine.add('GO_TO_WALL', GoToTask(),
                                    remapping = {'waypoint': 'brick_in_wall_pose'},
                                    transitions = {'succeeded': 'succeeded'})

def main():
    pick_and_place_task = PickAndPlaceTask()
    userdata = smach.UserData()
    userdata.pile_pose = PoseStamped()
    userdata.brick_in_wall_pose = PoseStamped()
    outcome = pick_and_place_task.execute(userdata)
    print('pick_and_place_callback output: {}'.format(outcome))

if __name__ == '__main__':
    main()
