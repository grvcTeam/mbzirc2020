#!/usr/bin/env python
"""
This component moves a robotic manipulator, in a planned manner, to a specified
joint configuration using MoveIt!.

**Input(s):**
  * `target_configuration`: The joint configuration to which the manipulator will
  be moved.

**Parameter(s):**
  * `move_group`: MoveIt! interface.
  * `arm`: Name of the group to move.
  * `loop_rate`: Node cycle rate (in hz).

"""
#-*- encoding: utf-8 -*-

import rospy
import actionlib
import moveit_commander
import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
import brics_actuator.msg


class PlannedMotion(object):
    """
    Components that move the arm in a planned motion.

    """
    def __init__(self):
        # Params
        self.event = None
        self.target_configuration = None

        # MoveIt! interface
        move_group = rospy.get_param('~move_group', None)
        print move_group + "planned"

        assert move_group is not None, "Move group must be specified."

        # Wait for MoveIt!
        client = actionlib.SimpleActionClient(move_group, moveit_msgs.msg.MoveGroupAction)
        rospy.loginfo("Waiting for '{0}' server".format(move_group))
        client.wait_for_server()
        rospy.loginfo("Found server '{0}'".format(move_group))

        # Name of the group to move
        arm = rospy.get_param('~arm', None)
        assert arm is not None, "The group to be moved must be specified (e.g. arm)."

        # Set up MoveIt!
        self.arm = moveit_commander.MoveGroupCommander(arm)

        self.scene = moveit_commander.PlanningSceneInterface()

        # Whether MoveIt! should wait for the arm to be stopped.
        self.wait_for_motion = rospy.get_param('~wait_for_motion', True)

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)

        self.base_position_pub = rospy.Publisher("/move_base_simple/goal", geometry_msgs.msg.PoseStamped, queue_size=1)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~target_configuration", brics_actuator.msg.JointPositions,
            self.target_configuration_cb
        )

    def target_configuration_cb(self, msg):
        """
        Obtains the joint configuration where the arm will be moved.

        """
        self.target_configuration = msg

    def event_in_cb(self, msg):
        """
        Starts a planned motion based on the specified arm position.

        """
        self.event = msg.data

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """

        if self.event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        elif self.target_configuration:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.arm.stop()
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            move_status = self.move_arm(self.target_configuration, self.wait_for_motion)

            rospy.sleep(1)





            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = "ee_link"
            box_pose.pose.position.y = 0.1
            box_pose.pose.orientation.w = 1.0
            box_name = "box"
            eef_link = "ee_link"
            self.scene.add_box(box_name, box_pose, size=(0.4, 0.2, 0.6))

            rospy.sleep(2)

            self.scene.attach_box(eef_link, box_name)    #falta acrescentar os touching links que servem para que o Moveit ignore collisions entre
                                                         # o attached object e estes links

            ########### REMOVE BOX FROM PLANNING SCENE ######################
            # self.scene.remove_attached_object(eef_link, name=box_name)
            # rospy.sleep(1)
            # self.scene.remove_world_object(box_name)

            

            if move_status:
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

        self.reset_component_data()
        return 'INIT'

    def move_arm(self, joint_configuration, wait=True):
        """
        Moves the arm to a specified joint configuration.

        :param joint_configuration: The target joint configuration.
        :type joint_configuration: brics_actuator.msg.JointPositions

        :param wait: Wait for the execution of the trajectory to complete,
        :type wait: bool

        :return: False if the target configuration cannot be achieved.
        :rtype: bool

        """
        print "plannnnnnnnnnnnnnnnnnnnneeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeedddddddddddddddddd"
        print joint_configuration



        joint_list = self.brics_joint_positions_to_list(joint_configuration)

        if len(joint_list) > 6:
            joint_list=joint_list[2:]

            
        print joint_list




        # base_position_pose = geometry_msgs.msg.PoseStamped()

        # base_position_pose.header.stamp = rospy.Time.now()
        # base_position_pose.header.frame_id = 'base_link'

        # base_position_pose.pose.position.x= joint_list[0]
        # base_position_pose.pose.position.y= joint_list[1]
        # base_position_pose.pose.position.z= 0

        # base_position_pose.pose.orientation.x= 0 
        # base_position_pose.pose.orientation.y= 0
        # base_position_pose.pose.orientation.z= 0
        # base_position_pose.pose.orientation.w= 1


        # self.base_position_pub.publish(base_position_pose)

        #rospy.sleep(joint_list[1]*5)

        #joint_list[0]=0
        #joint_list[1]=0


        self.arm.set_joint_value_target(joint_list)
        status = self.arm.go(wait=wait)

        return status

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None
        self.target_configuration = None

    def brics_joint_positions_to_list(self, joint_configuration):

        return [joint.value for joint in joint_configuration.positions]



def main():
    rospy.init_node("planned_motion", anonymous=True)
    planned_motion = PlannedMotion()
    planned_motion.start()
