#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger

def main():
    rospy.init_node('start_c3')

    start_c3_ground_srv = rospy.ServiceProxy('start_c3_ground',Trigger)
    start_c3_facade_srv_2 = rospy.ServiceProxy('start_c3_facade_2',Trigger)
    start_c3_facade_srv_4 = rospy.ServiceProxy('start_c3_facade_4',Trigger)

    try:
        res = start_c3_ground_srv()
        if res:
            rospy.loginfo('C3 ground running!')
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    rospy.sleep(30)
    try:        
        res = start_c3_facade_srv_2()
        if res:
            rospy.loginfo('C3 facade 2 running!')
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    rospy.sleep(30)
    try:
        res = start_c3_facade_srv_4()
        if res:
            rospy.loginfo('C3 facade 4 running!')
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    main()
