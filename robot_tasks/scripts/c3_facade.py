#!/usr/bin/env python

# PARAMETER	SPECIFICATION
# Number of UAVs per team	Maximum of 3
# Number of UGVs per team	1
# Arena size	50mx60mx20m
# Tower height	Maximum of 18m
# Tower base and interior specifications	TBD
# Location of simulated fires	Up to 16m in height, inside the arena.
# Environment	Outdoor and indoor
# Mode of operation	Autonomous; manual allowed but penalized
# RTK/DGPS positioning	Allowed but penalized
# Challenge duration	20 Minutes
# Communications	TBD
# Extinguisher types considered	Water carrying container 1-3 liters. Some examples shown in Figure 2.
# Maximum size of UAV	1.2m x 1.2m x 0.5m
# Maximum size of UGV	1.7m x 1.5m x 2m

import time
import math
import rospy
import rospkg
import smach
import yaml
import mbzirc_comm_objs.msg as msg

from mbzirc_comm_objs.srv import CheckFire
from geometry_msgs.msg import PoseStamped
from tasks.move import TakeOff, GoTo, FollowPath
from tasks.fire import ExtinguishFacadeFire, GoToFacadeFire
from utils.manager import TaskManager
from utils.robot import RobotProxy
from std_srvs.srv import Trigger, TriggerResponse

# Start service
started = False
def start_challenge(req):
    rospy.loginfo('Starting challenge 3 facade.')
    global started
    started = True
    return TriggerResponse(True,'')

def main():
    rospy.init_node('c3_facade')
    robot_id = str(rospy.get_param('~uav_id', '2'))
    facade = rospy.get_param('~facade', 'left')
    pre_fire_id = rospy.get_param('~fire_id', '')

    while rospy.get_rostime() == rospy.Time():
        rospy.logwarn("Waiting for (sim) time to begin!")
        rospy.sleep(1)

    global started
    file_name = 'fire_{}.yaml'.format(facade)
    fires_dir = rospkg.RosPack().get_path('fire_extinguisher') + '/fires/'
    file_url = fires_dir + file_name
    yaml_file = open(file_url, 'r')
    rospy.loginfo('Fires loaded from: %s', file_url)
    fires_yaml = yaml.load(yaml_file)
    # TODO: Check file consistency

    robot = {}
    robot[robot_id] = RobotProxy(robot_id)
    task_manager = TaskManager(robot)
    start_challenge_service = rospy.Service('start_c3_facade_{}'.format(robot_id),Trigger,start_challenge)

    # Get safe path from file
    safe_path = []
    if 'path' in fires_yaml:
        for waypoint in fires_yaml['path']:
            wp = PoseStamped()
            wp.header.frame_id = 'arena'
            wp.pose.position.x = waypoint['pose'][0]
            wp.pose.position.y = waypoint['pose'][1]
            wp.pose.position.z = waypoint['pose'][2]
            wp.pose.orientation.x = waypoint['pose'][3]
            wp.pose.orientation.y = waypoint['pose'][4]
            wp.pose.orientation.z = waypoint['pose'][5]
            wp.pose.orientation.w = waypoint['pose'][6]
            safe_path.append(wp)
    else:
        rospy.logerr('No path in file, exiting facade fire')
        return

    rospy.loginfo('Don Pin-Pon {} ready to go.'.format(robot_id))

    # Wait for start service
    while not rospy.is_shutdown() and not started:
        rospy.sleep(1)

    userdata = smach.UserData()
    userdata.height = safe_path[0].pose.position.z
    task_manager.start_task(robot_id, TakeOff(), userdata)
    task_manager.wait_for([robot_id])

    # TODO: Goto safe facade approaching position
    userdata_safe_path = smach.UserData()
    userdata_safe_path.path = safe_path
    task_manager.start_task(robot_id, FollowPath(), userdata_safe_path)
    task_manager.wait_for([robot_id])

    # Go directly to pre_fire_id
    if pre_fire_id:
        userdata_pre_fire = smach.UserData()
        userdata_pre_fire.fires_file = 'fire_' + facade + '.yaml'
        userdata_pre_fire.fire_id = pre_fire_id
        task_manager.start_task(robot_id, GoToFacadeFire(), userdata_pre_fire)
        task_manager.wait_for([robot_id])
        task_manager.start_task(robot_id, ExtinguishFacadeFire(), smach.UserData())
        task_manager.wait_for([robot_id])
        return

    # TODO: Auto order by z?
    for fire in fires_yaml['fires']:
        userdata = smach.UserData()
        # userdata.waypoint = PoseStamped() 
        # userdata.waypoint.header.frame_id = fire['pose_frame']
        # userdata.waypoint.pose.position.x = fire['pose'][0]
        # userdata.waypoint.pose.position.y = fire['pose'][1]
        # userdata.waypoint.pose.position.z = fire['pose'][2]
        # userdata.waypoint.pose.orientation.x = fire['pose'][3]
        # userdata.waypoint.pose.orientation.y = fire['pose'][4]
        # userdata.waypoint.pose.orientation.z = fire['pose'][5]
        # userdata.waypoint.pose.orientation.w = fire['pose'][6]
        # rospy.loginfo('robot {} going to:\n {}\n'.format(robot_id, userdata.waypoint))
        userdata.fires_file = file_name
        userdata.fire_id = fire['id']
        task_manager.start_task(robot_id, GoToFacadeFire(), userdata)
        task_manager.wait_for([robot_id])

        time.sleep(3)
        fire_detected = False  # TODO: Bypass?
        # ask_for_fire_url = 'thermal_detection/fire_detected'
        ask_for_fire_url = '/mbzirc2020_' + robot_id + '/thermal_detection/fire_detected'
        rospy.loginfo('robot {} waiting for {}...'.format(robot_id, ask_for_fire_url))
        # rospy.wait_for_service(ask_for_fire_url)  # TODO: Wait?
        rospy.loginfo('robot {} finished waiting!'.format(robot_id))
        # try:
        #     ask_for_fire = rospy.ServiceProxy(ask_for_fire_url, CheckFire)
        #     response = ask_for_fire()
        #     fire_detected = response.fire_detected
        # except rospy.ServiceException, e:
        #     rospy.logerr("Service call failed: {}".format(e))

        if fire_detected:
            rospy.loginfo('Extinguishing fire: {}'.format(fire['id']))
            task_manager.start_task(robot_id, ExtinguishFacadeFire(), smach.UserData())
            task_manager.wait_for([robot_id])
            # Back to approximate_pose
            # task_manager.start_task(robot_id, GoTo(), userdata)
            # task_manager.wait_for([robot_id])
            break

    rospy.loginfo('robot {} finished!\n'.format(robot_id))
    rospy.spin()

if __name__ == '__main__':
    main()
