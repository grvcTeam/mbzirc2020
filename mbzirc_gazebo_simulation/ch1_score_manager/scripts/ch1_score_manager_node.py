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

        self.balloons_model_names = list()
        self.balloons_link_index = list()
        self.balloons_positions = list()
        self.balloons_positions_mutex = Lock()

        self.drones_model_names = list()
        self.drones_link_index = list()
        self.drones_positions = list()
        self.drones_positions_mutex = Lock()

        self.distance_threshold2 = ((self.balloons_radio + self.drones_radio) * safety_factor)**2
        self.checker_configured = False

        self.score_pub = rospy.Publisher('/score_manager/score_balloons', Int32, queue_size=1)
        self.score = 0
        self.score_mutex = Lock()
        rospy.Timer(rospy.Duration(1.0/5.0), self.publish_score)

        self.gz_links_state_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.links_state_cb)
        rospy.wait_for_service('/gazebo/delete_model')
        self.gz_delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    def links_state_cb(self, msg):
        if not self.checker_configured:
            self.balloons_model_names = list()
            self.balloons_link_index = list()
            self.drones_model_names = list()
            self.drones_link_index = list()
            index = 0
            for name in msg.name:
                if self.balloon_link_name_pattern.match(name):
                    self.balloons_model_names.append(name.split('::')[0])
                    self.balloons_link_index.append(index)
                    rospy.logdebug('[Ch1-Score-Manager] Adding balloon link name: ' + name)
                elif self.drone_link_name_pattern.match(name):
                    self.drones_model_names.append(name.split('::')[0])
                    self.drones_link_index.append(index)
                    rospy.logdebug('[Ch1-Score-Manager] Adding drone link name: ' + name)
                index += 1
            if len(
                    self.balloons_model_names) == self.balloons_number and len(
                    self.drones_model_names) == self.drones_number:
                rospy.logdebug('[Ch1-Score-Manager] Balloons indexs: ' + str(self.balloons_link_index))
                rospy.logdebug('[Ch1-Score-Manager] Balloons models: ' + str(self.balloons_model_names))
                rospy.logdebug('[Ch1-Score-Manager] Drones indexs: ' + str(self.drones_link_index))
                rospy.logdebug('[Ch1-Score-Manager] Drones models: ' + str(self.drones_model_names))

                self.checker_configured = True

                with self.balloons_positions_mutex:
                    self.balloons_positions = list()
                    for index in self.balloons_link_index:
                        balloon_position = msg.pose[index].position
                        self.balloons_positions.append(balloon_position)

                with self.drones_positions_mutex:
                    self.drones_positions = list()
                    for index in self.drones_link_index:
                        drone_position = msg.pose[index].position
                        self.drones_positions.append(drone_position)

                rospy.loginfo('[Ch1-Score-Manager] Starting checker timer')
                rospy.Timer(rospy.Duration(1.0/self.checker_rate), self.check_distance)
            return

        with self.balloons_positions_mutex:
            self.balloons_positions = list()
            for index in self.balloons_link_index:
                try:
                    balloon_position = msg.pose[index].position
                    self.balloons_positions.append(balloon_position)
                except IndexError:
                    continue

        with self.drones_positions_mutex:
            self.drones_positions = list()
            for index in self.drones_link_index:
                try:
                    drone_position = msg.pose[index].position
                    self.drones_positions.append(drone_position)
                except IndexError:
                    continue

    def check_distance(self, event):
        balloons_to_remove = set()
        with self.balloons_positions_mutex:
            with self.drones_positions_mutex:
                for i in range(len(self.balloons_positions)):
                    balloon_position = self.balloons_positions[i]
                    for j in range(self.drones_number):
                        drone_position = self.drones_positions[j]
                        distance2 = ((balloon_position.x - drone_position.x) **
                                     2)+((balloon_position.y - drone_position.y)**2)+((balloon_position.z - drone_position.z)**2)
                        if distance2 <= self.distance_threshold2:
                            balloons_to_remove.add(i)
                            rospy.loginfo('[Ch1-Score-Manager] Drone: [%s] collides with balloon [%s]' %
                                          (self.drones_model_names[j], self.balloons_model_names[i]))
            for balloon in balloons_to_remove:
                rospy.loginfo('[Ch1-Score-Manager] Removing balloon [%s]' % self.balloons_model_names[balloon])
                try:
                    response = self.gz_delete_model(self.balloons_model_names[balloon])
                    if response.success:
                        rospy.loginfo('[Ch1-Score-Manager] Success!')
                        with self.score_mutex:
                            self.score += 1
                        self.balloons_model_names.pop(balloon)
                        self.balloons_link_index.pop(balloon)
                    else:
                        rospy.logwarn('[Ch1-Score-Manager] Error removing balloon')
                except rospy.ServiceException, e:
                    rospy.logerr('[Ch1-Score-Manager] Error calling removing model service')

    def publish_score(self, event):
        with self.score_mutex:
            self.score_pub.publish(self.score)


if __name__ == '__main__':
    Ch1ScoreManager()
    rospy.spin()
