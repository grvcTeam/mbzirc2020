#!/usr/bin/env python
import rospy
from mbzirc_comm_objs.srv import AddSharedRegion, AddSharedRegionRequest, RemoveSharedRegion, RemoveSharedRegionRequest
from geometry_msgs.msg import PointStamped
import time

if __name__ == '__main__':
    rospy.init_node('shared_regions_manager_test', anonymous = True)

    add_shared_region_url = 'add_shared_region'
    remove_shared_region_url = 'remove_shared_region'

    rospy.wait_for_service(add_shared_region_url)
    rospy.wait_for_service(remove_shared_region_url)

    add_shared_regions = rospy.ServiceProxy(add_shared_region_url, AddSharedRegion)
    remove_shared_regions = rospy.ServiceProxy(remove_shared_region_url, RemoveSharedRegion)

    while not rospy.is_shutdown():

        add_request = AddSharedRegionRequest()
        add_request.min_corner = PointStamped()
        add_request.min_corner.header.frame_id = 'map'
        add_request.min_corner.point.x = 0
        add_request.min_corner.point.y = 0
        add_request.min_corner.point.z = 0
        add_request.max_corner = PointStamped()
        add_request.max_corner.header.frame_id = 'map'
        add_request.max_corner.point.x = 1
        add_request.max_corner.point.y = 2
        add_request.max_corner.point.z = 3

        add_response = add_shared_regions(add_request)
        print(add_response)
        time.sleep(3)

        if (add_response.success):
            remove_request = RemoveSharedRegionRequest()
            remove_request.region_id = add_response.region_id
            remove_response = remove_shared_regions(remove_request)
            time.sleep(3)

    rospy.spin()
