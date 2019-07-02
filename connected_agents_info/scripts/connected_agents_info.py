#!/usr/bin/env python

from multimaster_msgs_fkie.msg import MasterState
from mbzirc_comm_objs.srv import GetJson, GetJsonResponse

import rospy
import json
import rosservice
from std_msgs.msg import String

class ConnectedAgentsInfo():

    def get_agents(self):

        s_list = rosservice.get_service_list()
        a_tasks = {}
        for service in s_list:
            if '/task' in service:
                print service
                e = service.find('/task')
                s = s1 = 0
                '''while s1 >= 0:
                    s1 = service.find('/',s,e)
                    s = s1+1 if s1 >= 0 else s'''

                a_name = service[s:e]
                s = (rosservice.get_service_type(service), service)
                a_tasks[a_name] = [s] if a_name not in a_tasks else a_tasks[a_name] + [s]

        return a_tasks

    def get_agents_cb(self, req):

        return  GetJsonResponse(jsonStr=json.dumps(self.get_agents()))

    def graph_change_cb(self, msg):

        self.pub.publish(String(data=self.get_agents()))

    def __init__(self):
        #
        self.pub = rospy.Publisher('changes', String, queue_size=1)
        rospy.Service('agent_list', GetJson, self.get_agents_cb) # json string with the format {'agent_id': [(task_service_type,task_service_address), ...], ...}
        self.sub = rospy.Subscriber('/master_discovery/changes', MasterState, self.graph_change_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('shared_regions_manager', anonymous=True)
        ConnectedAgentsInfo()
    except rospy.ROSInterruptException:
        pass