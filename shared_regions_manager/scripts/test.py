#!/usr/bin/env python
import rospy
from mbzirc_comm_objs.srv import AddSharedRegion, AddSharedRegionRequest
from geometry_msgs.msg import PointStamped
import random
import time

if __name__ == '__main__':
    rospy.init_node('shared_regions_manager_test', anonymous = True)

    add_shared_region_url = 'add_shared_region'
    rospy.wait_for_service(add_shared_region_url)
    add_shared_regions = rospy.ServiceProxy(add_shared_region_url, AddSharedRegion)

    count = 0
    max_count = 5
    while not rospy.is_shutdown():

        x = random.random() * 3.0
        y = random.random() * 3.0
        z = random.random() * 3.0

        add_request = AddSharedRegionRequest()
        add_request.agent_id = count
        add_request.min_corner = PointStamped()
        add_request.min_corner.header.frame_id = 'map'
        add_request.min_corner.point.x = x
        add_request.min_corner.point.y = y
        add_request.min_corner.point.z = z
        add_request.max_corner = PointStamped()
        add_request.max_corner.header.frame_id = 'map'
        add_request.max_corner.point.x = x + 1.0
        add_request.max_corner.point.y = y + 1.0
        add_request.max_corner.point.z = z + 1.0

        print(add_request)
        add_response = add_shared_regions(add_request)
        print(add_response)

        time.sleep(3)
        count = (count + 1) % max_count
