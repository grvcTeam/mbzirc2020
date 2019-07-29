#!/usr/bin/env python
import rospy
from mbzirc_comm_objs.srv import AskForRegion, AskForRegionRequest
from geometry_msgs.msg import PointStamped
import random
import time

if __name__ == '__main__':
    rospy.init_node('shared_regions_manager_test', anonymous = True)

    ask_for_region_url = 'ask_for_region'
    rospy.wait_for_service(ask_for_region_url)
    ask_for_region = rospy.ServiceProxy(ask_for_region_url, AskForRegion)

    count = 0
    max_count = 5
    while not rospy.is_shutdown():

        x = random.random() * 3.0
        y = random.random() * 3.0
        z = random.random() * 3.0

        request = AskForRegionRequest()
        request.agent_id = count
        request.min_corner = PointStamped()
        request.min_corner.header.frame_id = 'map'
        request.min_corner.point.x = x
        request.min_corner.point.y = y
        request.min_corner.point.z = z
        request.max_corner = PointStamped()
        request.max_corner.header.frame_id = 'map'
        request.max_corner.point.x = x + 1.0
        request.max_corner.point.y = y + 1.0
        request.max_corner.point.z = z + 1.0

        print(request)
        response = ask_for_region(request)
        print(response)

        time.sleep(3)
        count = (count + 1) % max_count
