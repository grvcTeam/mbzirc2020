#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger

def main():
    rospy.init_node('c3_starter')

    ground_id = rospy.get_param('~ground_id', '0')
    start_c3_ground_srv = rospy.ServiceProxy('start_c3_ground',Trigger)
    facade_1_id = rospy.get_param('~facade_1_id', '0')
    start_c3_facade_srv_1 = rospy.ServiceProxy('start_c3_facade_'+str(facade_1_id),Trigger)
    facade_2_id = rospy.get_param('~facade_2_id', '0')
    start_c3_facade_srv_2 = rospy.ServiceProxy('start_c3_facade_'+str(facade_2_id),Trigger)

    # Wait for key input to start C3
    raw_input(" Press ENTER to start C3 ")

    if int(ground_id) != 0:
        try:
            rospy.loginfo('Starting C3 ground {}'.format(ground_id))
            res = start_c3_ground_srv()
            if res:
                rospy.loginfo('C3 ground running!')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.sleep(30)

    if int(facade_1_id) != 0:
        try:
            rospy.loginfo('Starting C3 facade {}'.format(facade_1_id))
            res = start_c3_facade_srv_1()
            if res:
                rospy.loginfo('C3 facade {} running!'.format(facade_1_id))
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.sleep(30)

    if int(facade_2_id) != 0:
        try:
            rospy.loginfo('Starting C3 facade {}'.format(facade_2_id))
            res = start_c3_facade_srv_2()
            if res:
                rospy.loginfo('C3 facade {} running!'.format(facade_2_id))
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == '__main__':
    main()
