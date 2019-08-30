#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose, Point

import re
from threading import Lock


class Ch1ScoreManager:
    def __init__(self):
        rospy.init_node('ch1_score_manager')

        rospy.loginfo('[Ch1-Score-Manager] Starting ch1_score_manager...')

        self.drones_number = rospy.get_param('~drones_number', 2)
        self.balloons_number = rospy.get_param('~balloons_number', 6)
        self.drones_radio = rospy.get_param('~drones_radio', 0.4)
        self.balloons_radio = rospy.get_param('~balloon_radio', 0.3)
        self.checker_rate = rospy.get_param('~checker_rate', 50.0)
        safety_factor = rospy.get_param('~safety_factor', 0.95)

        self.balloon_link_name_pattern = re.compile("^balloon_\d{1,}::balloon$")
        self.drone_link_name_pattern = re.compile("^dji_f550_c1_\d{1,}::base_link$")

        self.balloons_model_names = [None]*self.balloons_number
        self.balloons_positions = [None]*self.balloons_number
        self.balloons_positions_mutex = Lock()

        self.balloons_measured = 0
        self.gazebo_models = 0
        self.balloons_index = [None]*self.balloons_number

        self.drones_model_names = [None]*self.drones_number
        self.drones_link_index = [None]*self.drones_number
        self.drones_positions = [None]*self.drones_number
        self.drones_positions_mutex = Lock()

        self.distance_threshold2 = (
            (self.balloons_radio + self.drones_radio) * safety_factor)**2
        self.checker_configured = False

        self.score_pub = rospy.Publisher(
            '/score_manager/score_balloons', Int32, queue_size=1)
        self.score = 0
        self.score_mutex = Lock()
        rospy.Timer(rospy.Duration(1.0/5.0), self.publish_score)

        self.gz_links_state_sub = rospy.Subscriber(
            '/gazebo/link_states', LinkStates, self.links_state_cb)
        rospy.wait_for_service('/gazebo/delete_model')
        self.gz_delete_model = rospy.ServiceProxy(
            '/gazebo/delete_model', DeleteModel)

        rospy.Timer(rospy.Duration(1.0/self.checker_rate), self.check_distance)

    def links_state_cb(self, msg):
        index = 0
        if self.balloons_measured < self.balloons_number or not self.gazebo_models == len(msg.name):
            self.gazebo_models = len(msg.name)
            for name in msg.name:
                if self.balloon_link_name_pattern.match(name):
                    balloon_name = name.split('::')[0]
                    balloon_number = int(balloon_name.split('_')[1])

                    with self.balloons_positions_mutex:
                        self.balloons_index[balloon_number] = index
                        self.balloons_model_names[balloon_number] = balloon_name
                        self.balloons_positions[balloon_number] = msg.pose[index].position
        
                    self.balloons_measured += 1
                    rospy.logdebug('[Ch1-Score-Manager] Adding balloon link name: ' + name)

                elif self.drone_link_name_pattern.match(name):
                    drone_name = name.split('::')[0]  
                    drone_number = int(drone_name.split('_')[-1]) -1

                    with self.drones_positions_mutex:
                        self.drones_model_names[drone_number] = drone_name
                        self.drones_link_index[drone_number] = index
                        drone_position = msg.pose[index].position
                        self.drones_positions[drone_number] = drone_position

                    rospy.logdebug('[Ch1-Score-Manager] Adding drone link name: ' + name)
                index += 1
            return

        with self.balloons_positions_mutex:
            for i in range(len(self.balloons_index)):
                if(self.balloons_index[i] == None):
                    continue
                balloon_position = msg.pose[self.balloons_index[i]].position
                self.balloons_positions[i] = balloon_position

        with self.drones_positions_mutex:
            for i in range(len(self.drones_link_index)):
                if(self.drones_link_index[i] == None):
                    continue
                drone_position = msg.pose[self.drones_link_index[i]].position
                self.drones_positions[i] = drone_position

    def check_distance(self, event):
        balloons_to_remove = set()
        with self.balloons_positions_mutex:
            with self.drones_positions_mutex:
                for i in range(len(self.balloons_index)):
                    if self.balloons_index[i] == None:
                        continue
                    
                    balloon_position = self.balloons_positions[i]
                    
                    for j in range(len(self.drones_positions)):
                        if (self.drones_positions[j] == None):
                            continue
                        
                        distance2 = ((balloon_position.x - self.drones_positions[j].x) ** 2) + \
                                    ((balloon_position.y - self.drones_positions[j].y) ** 2) + \
                                    ((balloon_position.z - self.drones_positions[j].z) ** 2)

                        if distance2 <= self.distance_threshold2:
                            balloons_to_remove.add(i)
                            rospy.loginfo('[Ch1-Score-Manager] Drone: [%s] collides with balloon [%s]' %
                                          (self.drones_model_names[j], self.balloons_model_names[i]))

                for balloon in balloons_to_remove:
                    rospy.loginfo('[Ch1-Score-Manager] Removing balloon [%s]' % self.balloons_model_names[balloon])
                    
                    try:
                        response = self.gz_delete_model( self.balloons_model_names[balloon])
                        if response.success:
                            rospy.loginfo('[Ch1-Score-Manager] Success!')
                            with self.score_mutex:
                                self.score += 1
                            self.balloons_model_names[balloon] = None
                            self.balloons_index[balloon] = None
                            self.balloons_measured -= 1
                            self.balloons_number -= 1
                        else:
                            rospy.logwarn(
                                '[Ch1-Score-Manager] Error removing balloon')
                    except rospy.ServiceException, e:
                        rospy.logerr(
                            '[Ch1-Score-Manager] Error calling removing model service')

    def publish_score(self, event):
        with self.score_mutex:
            self.score_pub.publish(self.score)


if __name__ == '__main__':
    Ch1ScoreManager()
    rospy.spin()
