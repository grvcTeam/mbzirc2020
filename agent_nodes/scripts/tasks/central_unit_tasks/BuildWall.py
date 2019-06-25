import roslib
import rospy
import smach
import smach_ros
import json
import sys
from math import sqrt, pow
import threading
from pydispatch import dispatcher
import Queue

from utils.geom import *
from utils.agent import *
import shapely.geometry

# required message definitions
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from std_msgs.msg import String, Header
from mbzirc_comm_objs.msg import ObjectDetectionList
from mbzirc_comm_objs.srv import GetJson, GetJsonRequest, SearchForObject, SearchForObjectRequest, AgentIdle, AgentIdleRequest
from geometry_msgs.msg import Polygon, Point32,  PolygonStamped, Point, Pose, PoseStamped, Quaternion

from mbzirc_comm_objs.srv import PFPNPlace, BuildWall, BuildWallResponse, AddSharedRegion, AddSharedRegionRequest, GoToWaypoint, GoToWaypointRequest
from mbzirc_comm_objs.msg import WallBluePrint

# task properties
ResponseType = BuildWallResponse
DataType = BuildWall
transitions={'success':'success','failure':'success','error':'error'}

# function to create userdata from a task execution request matching the task
# input keys
def gen_userdata(req):

    #wall def
    header = Header(frame_id='map',stamp=rospy.Time.now())
    wall =  WallBluePrint()
    wall.wall_frame = PoseStamped(header=header,pose=Pose(position=Point(0,0,0),orientation=Quaternion(0,0,0,1)))
    wall.size_x = 6
    wall.size_y = 1
    wall.size_z = 2
    wall.blueprint = [1, 3, 0, 0, 0, 1, 2, 0, 1, 1, 2, 0]

    userdata = smach.UserData()
    userdata.wall = wall
    userdata.items = {'red_pile': {'scale_x': 0.3, 'frame_id': 'map', 'aabb': [-0.2605240219463596, -10.149997164259327, 0.498081876014302, -9.599997164259069],
    'centroid': (0.1187789270339712, -9.874997164259199), 'type': 'brick_pile'},
    'blue_pile': {'scale_x': 1.2, 'frame_id': 'map', 'aabb': [-10.592243186737846, -0.6302098295615275, -8.02393939651886, 0.8499999988991274],
    'centroid': (-9.308091291628354, 0.10989508466879994), 'type': 'brick_pile'},
    'orange_pile': {'scale_x': 1.8, 'frame_id': 'map', 'aabb': [9.00026646361924, -0.9013597683516157, 12.790655918822848, 1.223507096940815],
    'centroid': (10.895461191221045, 0.16107366429459968), 'type': 'brick_pile'},
    'green_pile': {'scale_x': 0.6, 'frame_id': 'map', 'aabb': [-0.29030306291454, 9.700002165326108, 1.0949409841728985, 10.550002165326285],
    'centroid': (0.40231896062917927, 10.125002165326197), 'type': 'brick_pile'}}
    return userdata

# Task. At initialization it adds required elements to the AgentInterface
# At execution it uses these elements to coordinate components
class Task2Dispatch(object):

    states = ['not_sent','active','completed']

    def __init__(self, id, taskType):
        self.id = id
        self.type = taskType
        self.agent_id = None
        self.srv_address = None
        self.parameters = {}
        self.state = self.states[0]
        self.constraints = [] #Tasks which must be completed before this one can be sent
        self.constrains = [] #Tasks this one is constraining

    def __repr__(self):
        cstr = [task.id for task in self.constraints]
        ccstr = [task.id for task in self.constrains]
        return 'id: {id}, constraints: {c}, constrains: {cc}'.format(id=self.id, c= cstr, cc = ccstr)


class Task(smach.State):

    def brick_goal_pose(self, length, buffer, i, j, k):
        n_cells = length/0.30
        x = i * (0.30+buffer) + (length + buffer*n_cells) / 2
        y = j * (0.20+buffer) + (0.20+buffer) / 2
        z = k * (0.20+buffer) + (0.20+buffer) / 2

        return PyKDL.Frame(PyKDL.Rotation.Quaternion(0,0,0,1),PyKDL.Vector(x,y,z))

    # returns a matrix representing the wall blueprint from a msg
    def from_msg_to_matrix(self,msg):
        n_x = msg.size_x
        n_y = msg.size_y
        n_z = msg.size_z

        if len(msg.blueprint) != n_x * n_y * n_z:
            rospy.logerr('wall matrix malformed')
            return np.array()

        mm = []
        for k in  range(n_z):
            m = []
            for j in range(n_y):
                m += [msg.blueprint[(k*n_x*n_y+j*n_x):(k*n_x*n_y+(j+1)*n_x)]]
            mm += [m]

        return mm

    def build_task_matrix(self, wall_map):

        #build wall matrix from msg
        wall_matrix = self.from_msg_to_matrix(wall_map)
        trans_global2wall = from_geom_msgs_Pose_to_KDL_Frame(wall_map.wall_frame.pose)

        #build tasks from matrix
        counter = 0
        task_dic = {}
        for k in range(len(wall_matrix)):
            for j in range(len(wall_matrix[k])):
                l = []
                for i in range(len(wall_matrix[k][j])):
                    if wall_matrix[k][j][i] == 1: #red brick
                        #print '1'
                        x_l = 0.30
                        centroid = self.items['red_pile']['centroid']
                    elif wall_matrix[k][j][i] == 2: #green brick
                        #print '2'
                        x_l = 0.60
                        centroid = self.items['green_pile']['centroid']
                    elif wall_matrix[k][j][i] == 3: #blue brick
                        #print '3'
                        x_l = 1.20
                        centroid =self.items['blue_pile']['centroid']
                    elif wall_matrix[k][j][i] == 4: #orange brick
                        #print '4'
                        x_l = 1.80
                        centroid = self.items['orange_pile']['centroid']
                    else:
                        continue

                    params = {'type': 'brick', 'shared_regions':self.shared_regions}
                    params['pile_centroid'] = Point(centroid[0],centroid[1],0)
                    trans_wall2brick = self.brick_goal_pose(x_l, 0.01,i,j,k)
                    params['goal_pose'] = from_KDL_Frame_to_geom_msgs_Pose(trans_global2wall * trans_wall2brick)
                    task = Task2Dispatch('task_{n}'.format(n=counter),PFPNPlace)
                    counter += 1
                    task.parameters = params

                    task_dic[task.id]=task
                    l += [(trans_wall2brick.p.x()-x_l/2,trans_wall2brick.p.x()+x_l/2,task)]

                wall_matrix[k][j] = l

        return wall_matrix, task_dic

    def set_brick_constraints(self, wall_matrix):

        def in_seg(s1,s2,p):
            return s1 <= p and s2 >= p

        def inter_seg(s11,s12,s21,s22):
            return in_seg(s11,s12,s21) or in_seg(s11,s12,s22) or in_seg(s21,s22,s11)

        # a brick constraints those
        for k in range(1,len(wall_matrix)):
            for j in range(len(wall_matrix[k])):
                for i in range(len(wall_matrix[k][j])):
                    b_top = wall_matrix[k][j][i]
                    for i2 in range(len(wall_matrix[k-1][j])):
                        b_bottom = wall_matrix[k-1][j][i2]
                        if inter_seg(b_top[0],b_top[1],b_bottom[0],b_bottom[1]):
                            b_top[2].constraints += [b_bottom[2]]

    # send a task calling the right service and blocks until execution is completed.
    def dispatch_task(self, task_info):

        if not task_info.srv_address:
            return None

        rospy.wait_for_service(task_info.srv_address, 5)
    	client = rospy.ServiceProxy(task_info.srv_address,task_info.type)
    	try:
            res = client(**task_info.parameters)
            task_info.state = Task2Dispatch.states[2]
            task_info.outcome = res.outcome

    	except rospy.ServiceException, e:
    		print "Service call failed: %s"%e
    		return None


        # sends agent out of the wall shared region. TODO: this is dirty hack
        rospy.wait_for_service(task_info.agent_id+'/task/go_waypoint', 5)
    	client = rospy.ServiceProxy(task_info.srv_address,GoToWaypoint)
        client(global_frame='map',shared_regions=self.shared_regions, way_pose=Pose(position=Point(3,0,0),orientation=Quaternion(0,0,0,1)))
        ################3

        print 'completed task {t} with outcome {o}'.format(t=task_info.id, o = res.outcome)
        dispatcher.send( signal='task_completed', task_id = task_info.id, outcome=res.outcome, sender=self )

        return res

    # updates front. TODO: could trigger actions depending on outcome
    def completed_cb(self, task_id, outcome):
        print 'updating front'
        for task in self.tasks[task_id].constrains:
            task.constraints.remove(self.tasks[task_id])
            if not task.constraints:
                self.front.put(task)

    def get_picknplace_agents(self):
        # get available agents
        a_dic = json.loads(self.iface['agent_list']().jsonStr)
        pfpnp_dic = {}
        for a in a_dic:
            # get agent properties
            c = rospy.ServiceProxy('{agent_id}/agent_props'.format(agent_id=a), GetJson)
            props = json.loads(c().jsonStr)
            pfpnp_address = [e[1] for e in a_dic[a] if e[0] == 'mbzirc_comm_objs/PFPNPlace']
            # add to list
            if pfpnp_address:
                pfpnp_dic[a] = pfpnp_address[0]

        return pfpnp_dic

    def get_idle_agents(self, a_dic):
        idle_l = []
        for a in a_dic:
            c = rospy.ServiceProxy('{agent_id}/is_idle'.format(agent_id=a), AgentIdle)
            if c().isIdle:
                idle_l += [a]

        return idle_l

    # first dispatch strategy: get agents able to do pick and place and dipatch to idle
    def dispatch2idle(self):

        a_dic = self.get_picknplace_agents()

        r = rospy.Rate(0.5)
        while 1:
            if not self.front.empty():
                idles = self.get_idle_agents(a_dic)
                if idles:
                    a = idles[0]
                    # dispatch in thread
                    t = self.front.get()
                    print 'sending task: {t}'.format(t=t)
                    t.state = Task2Dispatch.states[1]
                    t.agent_id = a
                    t.srv_address = a_dic[a]
                    th = threading.Thread(target=self.dispatch_task, args=[t,])
                    th.start()
            r.sleep()

    #init
    def __init__(self, name, interface):
        smach.State.__init__(self,outcomes=['success','error','failure'],
                input_keys = ['items','wall']) #items = {'name': {'prop':value,...},...}

        #members
        self.name = name

        #interface elements
        self.iface = interface

        dispatcher.connect(self.completed_cb, signal='task_completed', sender=self )

    # main function
    def execute(self, userdata):

        # check items are present
        if not 'red_pile' in userdata.items or not 'green_pile' in userdata.items or not 'blue_pile' in userdata.items or not 'orange_pile' in userdata.items:
            rospy.loginfo('task {t} could not be executed'.format(t=self.name))
            return 'failure'

        self.items = userdata.items

        # build and add shared regions
        add_reg = rospy.ServiceProxy('/add_shared_region', AddSharedRegion)

        def poly_from_aabb(bb):
            p = Polygon()
            p.points = [Point32(bb[0],bb[1],0),Point32(bb[2],bb[1],0),Point32(bb[2],bb[3],0),Point32(bb[0],bb[3],0)]
            return p

        p = Polygon()
        p.points = [Point32(-2,-2,0),Point32(2,-2,0),Point32(2,2,0),Point32(-2,2,0)] #TODO: the central shared region should be computed from wall blueprint
        shared_regions = [p]

        for pile in self.items:
            if self.items[pile]['type'] == 'brick_pile':
                p = poly_from_aabb(self.items[pile]['aabb'])
                shared_regions += [p]

        self.shared_regions = shared_regions

        for p in shared_regions:
            req = AddSharedRegionRequest()
            req.frame_id = 'map'
            req.waiting_points = p.points
            req.region = p
            res = add_reg(req)

        #build list of tasks and set constraints
        wall_matrix, task_dic =  self.build_task_matrix(userdata.wall)
        self.set_brick_constraints(wall_matrix)
        self.tasks = task_dic

        for task_id in task_dic:
            for task_id2 in task_dic:
                if task_dic[task_id] in task_dic[task_id2].constraints:
                    task_dic[task_id].constrains += [task_dic[task_id2]]

        front = Queue.Queue()
        for j in range(len(wall_matrix[0])):
            for i in range(len(wall_matrix[0][j])):
                front.put(wall_matrix[0][j][i][2])

        self.front = front

        #print self.tasks

        ########################################################################

        self.dispatch2idle()
        return 'error'
