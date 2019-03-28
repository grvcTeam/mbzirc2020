#!/usr/bin/env python
import rospy
from mbzirc_comm_objs.srv import RequestSharedRegion, RequestSharedRegionResponse

def handle_request_cb(req):
    res = RequestSharedRegionResponse()
    res.answer = RequestSharedRegionResponse.OK
    return res

def shared_region_admin():
    srv = rospy.Service('request_shared_region', RequestSharedRegion, handle_request_cb)
    rospy.init_node('shared_region_admin', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    try:
        shared_region_admin()
    except rospy.ROSInterruptException:
        pass
