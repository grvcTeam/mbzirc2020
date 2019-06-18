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

# required message definitions
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from std_msgs.msg import String
from mbzirc_comm_objs.msg import ObjectDetectionList
from mbzirc_comm_objs.srv import GetJson, GetJsonRequest, SearchForObject, SearchForObjectRequest, AgentIdle, AgentIdleRequest
from geometry_msgs.msg import Polygon, Point32,  PolygonStamped

# task properties
ResponseType = SetBoolResponse
DataType = SetBool
transitions={'success':'success','failure':'success','error':'error'}

# function to create userdata from a task execution request matching the task
# input keys
def gen_userdata(req):

    userdata = smach.UserData()
    userdata.search_region = Polygon(points=[Point32(-15,-15,0),Point32(15,-15,0),Point32(15,15,0),Point32(-15,15,0)])
    userdata.items = None
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
        '''if self.all_found():
            return'''

        # group objects by similar objects. TODO: Hardcoded for bricks and piles
        groups = self.objects2group(msg.objects)

        # check if any of the objects can be one of the searched items. TODO: in can take partially detected pile as item
        for item in self.items:
            bb = self.find_item(self.items_d[item], groups)
            if bb:
                cam_frame = msg.objects[0].header.frame_id
                try:
                    trans_global2camera = lookup_tf_transform('map', cam_frame, self.iface['tf_buffer'],5,2,msg.objects[0].header.stamp)
                    trans_global2camera = from_geom_msgs_Transform_to_KDL_Frame(trans_global2camera.transform)
                except Exception as error:
                    print repr(error)
                    print self.name + ' Task could not be executed'
                    return 'error'

                bb_p = shapely.geometry.Polygon([(bb[0],bb[1]), (bb[2],bb[1]), (bb[2],bb[3]), (bb[0],bb[3])])
                bb_p = transform_Shapely_Polygon_with_KDL_Frame(trans_global2camera,bb_p)
                bb = bb_p.bounds
                #print 'bb: {b}'.format(b=bb)

                if self.items[item] and self.bb_do_intersect(bb,self.items[item]):
                    bb = self.bb_union(bb,self.items[item])
                    print 'item updated {item}!!'.format(item=bb)

                self.items[item] = bb

    def bb_do_intersect(self,bb1,bb2):
        def in_seg(s1,s2,p):
            return s1 <= p and s2 >= p

        def inter_seg(s11,s12,s21,s22):
            return in_seg(s11,s12,s21) or in_seg(s11,s12,s22)

        print '{b1} I {b2} = {i}'.format(b1=bb1,b2=bb2,i=inter_seg(bb1[0],bb1[2],bb2[0],bb2[2]) and inter_seg(bb1[1],bb1[3],bb2[1],bb2[3]))

        return inter_seg(bb1[0],bb1[2],bb2[0],bb2[2]) and inter_seg(bb1[1],bb1[3],bb2[1],bb2[3])

    def bb_union(self,bb1,bb2):
        return [min(bb1[0],bb2[0]),min(bb1[1],bb2[1]),max(bb1[2],bb2[2]),max(bb1[3],bb2[3])]

    def find_item(self, item, groups):
        for scale in groups:
            if item == scale:
                return groups[scale]
        return None

    #returns the bounding box for all centroids extruded by object scale vector length
    def objects2group(self, list):
        if not list:
            return 0,0,0,0

        #find groups aabb
        groups = {}

        for object in list:
            #find group to which object belongs. TODO: harcoded for bricks detected with fake camera
            if object.scale.x not in groups:
                xmin = ymin = sys.float_info.max
                xmax = ymax = - sys.float_info.max
                groups[object.scale.x] = [xmin,ymin,xmax,ymax]

            #update bb
            p = object.pose.pose.position
            groups[object.scale.x][2] = p.x if p.x > groups[object.scale.x][2] else groups[object.scale.x][2]
            groups[object.scale.x][0] = p.x if p.x < groups[object.scale.x][0] else groups[object.scale.x][0]
            groups[object.scale.x][3] = p.y if p.y > groups[object.scale.x][3] else groups[object.scale.x][3]
            groups[object.scale.x][1] = p.y if p.y < groups[object.scale.x][1] else groups[object.scale.x][1]

        #inflates aabbs
        for scale in groups:
            groups[scale] = [groups[scale][0]-scale/2,groups[scale][1]-scale/2,groups[scale][2]+scale/2,groups[scale][3]+scale/2]

        return groups

    # find aabb larger size and divides it in n.
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

        #items. Harcoded, should be taken from key
        self.items_d = {'red_pile':0.3,
                        'blue_pile':0.6,
                        'green_pile':1.2,
                        'orange_pile':1.8}

        self.items = {}
        for item in self.items_d:
            self.items[item] = None

        # send search tasks and wait
        n = 0
        for a in uav_dic:
            req = SearchForObjectRequest()
            req.object_types = ['brick']
            req.stop_after_find = False
            req.search_region = PolygonStamped()
            req.search_region.header.frame_id = "map"
            req.search_region.header.stamp = rospy.Time.now()
            req.search_region.polygon = sub_regions[n]
            n += 1

            c = rospy.ServiceProxy(uav_dic[a], SearchForObject)
            c(req)
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

        res =  'success' if self.all_found() else 'failure'
        return res
