#!/usr/bin/env python

import Queue
from mbzirc_comm_objs.msg import RegionOwnerList
from mbzirc_comm_objs.srv import RequestSharedRegion, RequestSharedRegionResponse, RequestSharedRegionRequest, AddSharedRegion, AddSharedRegionResponse, RemoveSharedRegion, RemoveSharedRegionResponse
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

from multimaster_msgs_fkie.msg import MasterState
from multimaster_msgs_fkie.srv import DiscoverMasters

import rospy
import rosservice
import re

class ConnectedAgentsInfo():

    def get_agents(self):

        s_list = rosservice.get_service_list()
        a_tasks = {}
        for service in s_list:
            if '/task' in service:
                e = service.find('/task')
                s = s1 = 0
                while s1 >= 0:
                    s1 = service.find('/',s,e)
                    s = s1+1 if s1 >= 0 else s

                a_name = service[s:e]
                a_tasks[a_name] = [service] if a_name not in a_tasks else a_tasks[a_name] + [service]

        return a_tasks


    def get_agents_cb(self, req):

        req.res = self.get_agents()

    def graph_change_cb(self, msg):

        self.pub.publish(self.get_agents())

    def __init__(self):

        self.use_multi_master = rospy.get_param('~use_multi_master')

        if self.use_multi_master
            rospy.Subscriber('/master_discovery/changes', MasterState, self.graph_change_cb)

        self.pub = rospy.Publisher('changes', AgentsList, queue_size=1)
        rospy.Service('agent_list', GetAgentList, self.get_agents_cb)

        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('shared_regions_manager', anonymous=True)
        ConnectedAgentsInfo()
    except rospy.ROSInterruptException:
        pass
