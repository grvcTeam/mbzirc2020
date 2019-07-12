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

# required message definitions
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from std_msgs.msg import String
from mbzirc_comm_objs.msg import ObjectDetectionList
from mbzirc_comm_objs.srv import GetJson, GetJsonRequest, SearchForObject, SearchForObjectRequest, AgentIdle, AgentIdleRequest, SearchForBrickPiles, SearchForBrickPilesResponse, SetAgentProp, SetAgentPropRequest
from geometry_msgs.msg import Polygon, Point32,  PolygonStamped

# task properties
ResponseType = SearchForBrickPilesResponse
DataType = SearchForBrickPiles
transitions={'success':'success','failure':'success','error':'error'}

# function to create userdata from a task execution request matching the task
# input keys
def gen_userdata(req):

    userdata = smach.UserData()
    userdata.search_region = req.search_region
    userdata.piles = req.piles

    '''userdata.search_region = Polygon(points=[Point32(-11,-11,0),Point32(11,-11,0),Point32(11,11,0),Point32(-11,11,0)])
    userdata.piles = {'red_pile': {'type':'brick_pile', 'color':'red', 'scale_x':0.3, 'frame_id':'map','centroid': None, 'aabb': None},
                    'green_pile':{'type':'brick_pile', 'color':'green','scale_x':0.6, 'frame_id':'map','centroid': None, 'aabb': None},
                    'blue_pile':{'type':'brick_pile', 'color':'blue','scale_x':1.2, 'frame_id':'map','centroid': None, 'aabb': None},
                    'orange_pile':{'type':'brick_pile', 'color':'orange','scale_x':1.8, 'frame_id':'map','centroid': None, 'aabb': None}}'''
    return userdata

# Task. At initialization it adds required elements to the AgentInterface
# At execution it uses these elements to coordinate components
class Task(smach.State):

    # check if all the piles requested in input_keys have been identified
    def all_found(self):
        for pile in self.piles_d:
            if self.piles_d[pile]['aabb'] == None:
                return False
        return True

    # object detection callback. Compose object detection information from all agents
    def object_detection_cb(self, msg):
        '''if self.all_found():
            return'''

        # group objects by type.
        if self.useColor:
            piles = self.bricks2pile_color(msg.objects)
            prop = 'color'
        else:
            piles = self.bricks2pile_scale(msg.objects)
            prop = 'scale_x'

        # check if any of the detected group of objects is part of one of the requested piles
        # and updates its bounding box
        for pile in self.piles_d:
            bb = self.find_pile(self.piles_d[pile][prop], piles)
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

                if self.piles_d[pile]['aabb'] and self.bb_do_intersect(bb,self.piles_d[pile]['aabb']):
                    bb = self.bb_union(bb,self.piles_d[pile]['aabb'])
                    #print 'pile updated {pile}!!'.format(pile=bb)

                self.piles_d[pile]['aabb'] = bb


    def find_pile(self, value, piles):
        for prop in piles:
            if value == prop:
                return piles[prop]
        return None

    # groups objects in the list according to a property. The chosen property is the object color
    # since it is different for each brick type. This would work just if the object msg has property color.
    def bricks2pile_color(self, list):
        if not list:
            return 0,0,0,0

        #find piles aabb
        piles = {}

        for object in list:
            #find group to which the object belongs.
            props = json.loads(object.properties)
            if 'color' not in props:
                continue

            if props['color'] not in piles:
                xmin = ymin = sys.float_info.max
                xmax = ymax = - sys.float_info.max
                piles[props['color']] = [xmin,ymin,xmax,ymax]

            #update bb
            p = object.pose.pose.position
            piles[props['color']][2] = p.x if p.x > piles[props['color']][2] else piles[props['color']][2]
            piles[props['color']][0] = p.x if p.x < piles[props['color']][0] else piles[props['color']][0]
            piles[props['color']][3] = p.y if p.y > piles[props['color']][3] else piles[props['color']][3]
            piles[props['color']][1] = p.y if p.y < piles[props['color']][1] else piles[props['color']][1]

        #inflates aabbs
        inf_val = 0.5
        for color in piles:
            piles[color] = [piles[color][0]-inf_val/2,piles[color][1]-inf_val/2,piles[color][2]+inf_val/2,piles[color][3]+inf_val/2]

        return piles

    # groups objects in the list according to a property. The chosen property is the object scale in x axis
    # since it is different for each brick type. This would work just for the fake camera plugin.
    def bricks2pile_scale(self, list):
        if not list:
            return 0,0,0,0

        #find piles aabb
        piles = {}

        for object in list:
            #find group to which the object belongs.
            if object.scale.x not in piles:
                xmin = ymin = sys.float_info.max
                xmax = ymax = - sys.float_info.max
                piles[object.scale.x] = [xmin,ymin,xmax,ymax]

            #update bb
            p = object.pose.pose.position
            piles[object.scale.x][2] = p.x if p.x > piles[object.scale.x][2] else piles[object.scale.x][2]
            piles[object.scale.x][0] = p.x if p.x < piles[object.scale.x][0] else piles[object.scale.x][0]
            piles[object.scale.x][3] = p.y if p.y > piles[object.scale.x][3] else piles[object.scale.x][3]
            piles[object.scale.x][1] = p.y if p.y < piles[object.scale.x][1] else piles[object.scale.x][1]

        #inflates aabbs
        for scale in piles:
            piles[scale] = [piles[scale][0]-scale/2,piles[scale][1]-scale/2,piles[scale][2]+scale/2,piles[scale][3]+scale/2]

        return piles

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
                io_keys = ['piles']) #piles = {'name': {'prop':value,...},...}

        #members
        self.useColor = True
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

        #piles.
        self.piles_d = userdata.piles

        # set different height (altitude) values for each UAV.
        h = 5
        for uav in uav_dic:
            c = rospy.ServiceProxy('{agent_id}/set_agent_props'.format(agent_id=uav), SetAgentProp)
            c(jsonStr=json.dumps({'height':h}))
            h -= 1 # works because there are 3 UAVs maximum

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
            req.z_plane = 0
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

        for pile in self.piles_d:
            if self.piles_d[pile]['aabb']:
                bb = self.piles_d[pile]['aabb']
                self.piles_d[pile]['centroid'] = ((bb[2]+bb[0])/2,(bb[3]+bb[1])/2)

        print userdata.piles

        res =  'success' if self.all_found() else 'failure'
        return res
