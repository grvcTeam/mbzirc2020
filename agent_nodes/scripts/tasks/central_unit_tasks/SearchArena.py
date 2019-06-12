import roslib
import rospy
import smach
import smach_ros
import json
import sys

from utils.geom import *
from utils.agent import *

# required message definitions
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from std_msgs.msg import String
from mbzirc_comm_objs.srv import GetJson, GetJsonRequest, SearchForObject, SearchForObjectRequest, AgentIdle, AgentIdleRequest

# task properties
ResponseType = SetBoolResponse
DataType = SetBool
transitions={'success':'success','error':'error'}

# function to create userdata from a task execution request matching the task
# input keys
def gen_userdata(req):

    userdata = smach.UserData()
    userdata.isWrite = req.data
    return userdata

# Task. At initialization it adds required elements to the AgentInterface
# At execution it uses these elements to coordinate components
class Task(smach.State):

    def all_found(self):
        for item in self.items:
            if self.items[item] == None:
                return False
        return True

    def object_detection_cb(self, msg):
        # check if any of the objects can be one of the searched items
        for item in self.items:
            if self.items[item] == None:
                self.items[item] = self.find_item(self.items_d[item], msg)

    def find_item(self, item, msg):
        pass

    #returns the bounding box for all centroid extruded by object scale vector length
    def objects2group(list):
        #find group aabb
        xmin = ymin = sys.float_info.max
        xmax = ymax = - sys.float_info.max

        for object in list:
            p = list.pose.pose.position
            xmax = p.x if p.x > xmax else xmax
            xmin = p.x if p.x < xmin else xmin
            ymax = p.y if p.y > ymax else ymax
            ymin = p.y if p.y < ymin else ymin

        


    # find aabb larger size and divides it in n.
    def divide_regions(self, aabb, n):
        x_l = aabb[2] - aabb[0]
        y_l = aabb[3] - aabb[1]

        def poly(x0,y0,x1,y1):
            return shapely.geometry.Polygon([(x0,y0),(x1,y0),(x1,y1),(x0,y1)])

        regions = []
        if x_l > y_l:
            segs = [aabb[0]+i*x_l/n for i in range(n)]+[aabb[2]]
            for i in range(len(segs)):
                regions += [poly(segs[i],aabb[1],segs[i+1],aabb[3])]
        else:
            segs = [aabb[1]+j*y_l/n for j in range(n)]+[aabb[3]]
            for i in range(len(segs)):
                regions += [poly(aabb[0],segs[i],aabb[2],segs[i+1])]

        return regions


    #init
    def __init__(self, name, interface):
        smach.State.__init__(self,outcomes=['success','error','failure'],
                input_keys = ['search_region', 'items']) #items = {'name': {'prop':value,...},...}

        #members
        self.is_searching = False
        self.name = name

        #interface elements
        self.iface = interface

    # main function
    def execute(self, userdata):
        # get available agents
        a_dic = json.loads(self.iface['agent_list']().jsonStr)
        uav_dic = []
        for a in a_dic:
            #get agent properties
            c = rospy.ServiceProxy('/{agent_id}/agent_props}'.format(agent_id=a), GetJson)
            props = json.loads(c().jsonStr)
            search_address = [e[1] for e in a_dic[a] if e[0] == 'mbzirc_comm_objs/SearchForObject']
            # add to list
            if 'type' in props and props['type'] == 'UAV' and search_address:
                uav_dic[a] = search_address[0]

        # divide arena in number or agents regions.
        region = from_geom_msgs_Polygon_to_Shapely_Polygon(userdata.search_region.polygon)
        aabb = region.bounds
        sub_regions = self.divide_regions(aabb,len(uav_dic.keys()))

        #items. Harcoded, should be taken from key
        self.items_d = {'red_pile':{'type':'brick','scale':(0.3,0.2,0.2),'is_group':True},
                        'blue_pile':{'type':'brick','scale':(0.6,0.2,0.2),'is_group':True},
                        'green_pile':{'type':'brick','scale':(1.2,0.2,0.2),'is_group':True},
                        'orange_pile':{'type':'brick','scale':(1.8,0.2,0.2),'is_group':True}}

        self.items = {}
        for item in self.items_d:
            self.items[item] = None

        # send search tasks and wait
        for a in uav_dic:
            req = SearchForObjectRequest()
            req.object_types = ['brick']
            req.stop_after_find = False
            req.search_region = PolygonStamped()

            c = rospy.ServiceProxy(uav_dic[a], SearchForObject)
            c(req)
            self.iface.add_subscriber(self,'/{agent_id}/detected_objects}'.format(agent_id=a),ObjectDetectionList,
                                    self.object_detection_cb)

            uav_dic[a] = rospy.ServiceProxy('/{agent_id}/is_idle'.format(agent_id=a), AgentIdle)

        r = rospy.Rate(1)
        while not self.all_found():
            #check if agents are idle
            all_idle = True
            for a in uav_dic:
                if not uav_dic[a]().isIdle:
                    all_idle = False

            if all_idle and not self.all_found():
                return 'failure'
            r.sleep()

        return 'success'
