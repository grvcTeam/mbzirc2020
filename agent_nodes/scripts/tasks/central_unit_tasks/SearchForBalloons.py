import roslib
import rospy
import smach
import smach_ros
import json
import sys
from math import sqrt, pow

from utils.geom import *
from utils.agent import *
import shapely.geometry
import threading

from math import isnan

# required message definitions
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from std_msgs.msg import String
from mbzirc_comm_objs.msg import ObjectDetectionList
from mbzirc_comm_objs.srv import GetJson, GetJsonRequest, SearchForObject, SearchForObjectRequest, AgentIdle, AgentIdleRequest, SearchForBrickPiles, SearchForBrickPilesResponse, SetAgentProp, SetAgentPropRequest
from geometry_msgs.msg import Polygon, Point32,  PolygonStamped

# Task. At initialization it adds required elements to the AgentInterface
# At execution it uses these elements to coordinate components
class Task(smach.State):

    # object detection callback. Compose object detection information from all agents
    def object_detection_cb(self, msg):

        for object in msg.objects:
            #if object.type == 'balloon': #TODO: hue object detection not inferring type
            self.add_balloon(object)

    def add_balloon(self,object):

        def bb_from_obj(obj):
            return [obj.pose.pose.position.x-obj.scale.x/2,obj.pose.pose.position.y-obj.scale.y/2,obj.pose.pose.position.x+obj.scale.x/2,obj.pose.pose.position.y+obj.scale.y/2]

        isIn = False
        o_bb = bb_from_obj(object)

        for v in o_bb:
            if isnan(v): #TODO: hue object detection returning nan for position or scale
                return

        for b in self.balloons:
            b_bb = bb_from_obj(b)
            print 'bb1: {b}'.format(b= o_bb)
            print 'bb2: {b}'.format(b= b_bb)
            print self.bb_do_intersect(o_bb,b_bb)
            if self.bb_do_intersect(o_bb,b_bb):
                isIn = True
                break

        if not isIn:
            print 'added balloon'
            self.balloons += [object]

    # test if two bbs intersects. Also returns true if one contains the other.
    def bb_do_intersect(self,bb1,bb2):

        def in_seg(s1,s2,p):
            return s1 <= p and s2 >= p

        def inter_seg(s11,s12,s21,s22):
            return in_seg(s11,s12,s21) or in_seg(s11,s12,s22) or in_seg(s21,s22,s11)

        #print '{b1} I {b2} = {i}'.format(b1=bb1,b2=bb2,i=inter_seg(bb1[0],bb1[2],bb2[0],bb2[2]) and inter_seg(bb1[1],bb1[3],bb2[1],bb2[3]))

        return inter_seg(bb1[0],bb1[2],bb2[0],bb2[2]) and inter_seg(bb1[1],bb1[3],bb2[1],bb2[3])

    # returns the bb of two bbs
    def bb_union(self,bb1,bb2):
        return [min(bb1[0],bb2[0]),min(bb1[1],bb2[1]),max(bb1[2],bb2[2]),max(bb1[3],bb2[3])]

    # find aabb larger size and divides it in n sub-bbs.
    def divide_regions(self, aabb, n):
        x_l = aabb[2] - aabb[0]
        y_l = aabb[3] - aabb[1]

        def poly(x0,y0,x1,y1):
            return Polygon(points=[Point32(x0,y0,0),Point32(x1,y0,0),Point32(x1,y1,0),Point32(x0,y1,0)])

        regions = []
        if x_l > y_l:
            segs = [aabb[0]+i*x_l/n for i in range(n)]+[aabb[2]]
            for i in range(len(segs)-1):
                regions += [poly(segs[i],aabb[1],segs[i+1],aabb[3])]
        else:
            segs = [aabb[1]+j*y_l/n for j in range(n)]+[aabb[3]]
            for i in range(len(segs)-1):
                regions += [poly(aabb[0],segs[i],aabb[2],segs[i+1])]

        return regions


    #init
    def __init__(self, name, interface):
        smach.State.__init__(self,outcomes=['success','error','failure'],
                input_keys = ['search_region'],
                io_keys = ['balloons']) #piles = {'name': {'prop':value,...},...}

        #members
        self.name = name
        self.balloons = []

        #interface elements
        self.iface = interface

    # main function
    def execute(self, userdata):
        # get available agents
        a_dic = json.loads(self.iface['agent_list']().jsonStr)
        uav_dic = {}
        for a in a_dic:
            #get agent properties
            c = rospy.ServiceProxy('{agent_id}/agent_props'.format(agent_id=a), GetJson)
            props = json.loads(c().jsonStr)
            search_address = [e[1] for e in a_dic[a] if e[0] == 'mbzirc_comm_objs/SearchForObject']
            # add to list
            if 'type' in props and props['type'] == 'UAV' and search_address:
                uav_dic[a] = search_address[0]

        #print uav_dic

        # divide arena in number or agents regions.
        region = from_geom_msgs_Polygon_to_Shapely_Polygon(userdata.search_region)
        aabb = region.bounds
        sub_regions = self.divide_regions(aabb,len(uav_dic.keys()))

        #print sub_regions


        # set different height (altitude) values for each UAV.
        h = 7
        for uav in uav_dic:
            c = rospy.ServiceProxy('{agent_id}/set_agent_props'.format(agent_id=uav), SetAgentProp)
            c(jsonStr=json.dumps({'height':h}))
            h -= 1 # works because there are 3 UAVs maximum

        # send search tasks and wait
        n = 0
        for a in uav_dic:
            req = SearchForObjectRequest()
            req.object_types = ['balloon']
            req.stop_after_find = False
            req.search_region = PolygonStamped()
            req.search_region.header.frame_id = "map"
            req.search_region.header.stamp = rospy.Time.now()
            req.search_region.polygon = sub_regions[n]
            req.z_plane = 4 # TODO: this is the maximum height at which balloons can be
            n += 1

            c = rospy.ServiceProxy(uav_dic[a], SearchForObject)
            t = threading.Thread(target=c, args=[req,])
            t.start()
            self.iface.add_subscriber(self,'{agent_id}/detected_objects'.format(agent_id=a),ObjectDetectionList,
                                    self.object_detection_cb)

            uav_dic[a] = rospy.ServiceProxy('{agent_id}/is_idle'.format(agent_id=a), AgentIdle)

        r = rospy.Rate(1)
        while 1:
            #check if agents are idle
            all_idle = True
            for a in uav_dic:
                if not uav_dic[a]().isIdle:
                    all_idle = False

            if all_idle:
                break
            r.sleep()

        userdata.balloons = self.balloons

        print userdata.balloons

        res =  'success'
        return res
