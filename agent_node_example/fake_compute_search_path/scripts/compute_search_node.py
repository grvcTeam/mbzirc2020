#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from agent_node_example_comm_objects.srv import SearchRegionPath, SearchRegionPathResponse

def compute_path_cb(req):
    res = SearchRegionPathResponse()
    pose1 = Pose()
    pose2 = Pose()
    pose1.position.x = 2
    pose1.position.y = 2
    pose1.position.z = 5
    pose2.position.x = 5
    pose2.position.y = 5
    pose2.position.z = 2
    res.path = [pose1,pose2]
    return res

def compute_path():
    srv = rospy.Service('compute_region', SearchRegionPath, compute_path_cb)
    rospy.init_node('compute_search_region', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    try:
        compute_path()
    except rospy.ROSInterruptException:
        pass
