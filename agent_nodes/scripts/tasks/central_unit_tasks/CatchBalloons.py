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
from geometry_msgs.msg import Polygon, Point32,  PolygonStamped, Point, Pose, PoseStamped, Quaternion, Vector3

from mbzirc_comm_objs.srv import PickNPlace, BuildWall, BuildWallResponse, AddSharedRegion, AddSharedRegionRequest, GoToWaypoint, GoToWaypointRequest
from mbzirc_comm_objs.msg import WallBluePrint, ObjectDetection

from shapely.geometry import LineString, Point as ShapelyPoint

# task properties
ResponseType = BuildWallResponse
DataType = BuildWall
transitions={'success':'success','failure':'success','error':'error'}

# function to create userdata from a task execution request matching the task
# input keys
def gen_userdata(req):

    userdata = smach.UserData()
    userdata.deployment_point = Point(44,7,0)

    obj = ObjectDetection()
    obj.pose.pose.position.x = 10
    obj.pose.pose.position.y = 10
    obj.pose.pose.position.z = 2.35
    obj.scale.x = 0.28
    obj.scale.y = 0.21
    obj.scale.z = 0.2

    userdata.balloons = [obj]
    return userdata

# helper class to contain each of the pick and place task required to build
# the wall including its parameters
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

    # print
    def __repr__(self):
        cstr = [task.id for task in self.constraints]
        ccstr = [task.id for task in self.constrains]
        return 'id: {id}, constraints: {c}, constrains: {cc}'.format(id=self.id, c= cstr, cc = ccstr)

# Task. At initialization it adds required elements to the AgentInterface
# At execution it uses these elements to coordinate components
class Task(smach.State):

    # Compute required pick and place tasks from wall msg
    def compute_tasks_from_balloons(self, balloons):

        # order balloons by distance to deploy area
        d_p = ShapelyPoint((self.deployment_point.x,self.deployment_point.y))
        b_list = []
        for obj in balloons:
            b_list += [(d_p.distance(ShapelyPoint(obj.pose.pose.position.x,obj.pose.pose.position.y)),obj)]

        b_list.sort(key = lambda e : e[0],reverse=False)
        b_list = [p[1] for p in b_list]

        #build tasks
        counter = 0
        t_list = []
        for o in b_list:
            params = {'type': 'balloon', 'shared_regions':self.shared_regions}
            params['scale'] = Vector3(0.2,0.2,0.2)
            params['obj_pose'] = o.pose.pose
            params['goal_pose'] = Pose(position= self.deployment_point, orientation = Quaternion(0,0,0,1))
            task = Task2Dispatch('task_{n}'.format(n=counter),PickNPlace)
            counter += 1
            task.parameters = params
            t_list += [task]

        return t_list

    # send a task to an agent calling the right service and blocks until execution is completed.
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


        # sends the agent out of the wall shared region after completing the pick and place
        # TODO: dirty solution, not even checking if the agent can execute a go to waypoint task
        #rospy.wait_for_service(task_info.agent_id+'/task/go_waypoint', 5)
    	#client = rospy.ServiceProxy(task_info.agent_id+'/task/go_waypoint',GoToWaypoint)
        #client(global_frame='map',shared_regions=self.shared_regions, way_pose=Pose(position=Point(3,0,0),orientation=Quaternion(0,0,0,1)))
        ################

        print 'completed task {t} with outcome {o}'.format(t=task_info.id, o = res.outcome)
        dispatcher.send( signal='task_completed', task_id = task_info.id, outcome=res.outcome, sender=self )

        return res

    def uncompleted_tasks(self):
        n = 0
        for t in self.tasks:
            if self.tasks[t].state != Task2Dispatch.states[2]:
                n += 1

        return n

    # retrieve available agents which can perform a pick and place task
    def get_picknplace_agents(self):
        # get available agents
        a_dic = json.loads(self.iface['agent_list']().jsonStr)
        pnp_dic = {}
        h = 7
        for a in a_dic:
            # get agent properties
            c = rospy.ServiceProxy('{agent_id}/agent_props'.format(agent_id=a), GetJson)
            props = json.loads(c().jsonStr)
            pnp_address = [e[1] for e in a_dic[a] if e[0] == 'mbzirc_comm_objs/PickNPlace']
            # add to list
            if pnp_address:
                pnp_dic[a] = pnp_address[0]
            # set different height to UAVs
            if 'type' in props and props['type'] == 'UAV':
                c = rospy.ServiceProxy('{agent_id}/set_agent_props'.format(agent_id=a), SetAgentProp)
                c(jsonStr=json.dumps({'height':h}))
                h -= 1 # works because there are 3 UAVs maximum


        return pnp_dic

    # retrieve agents from a_dic which are currently idle
    def get_idle_agents(self, a_dic):
        idle_l = []
        for a in a_dic:
            c = rospy.ServiceProxy('{agent_id}/is_idle'.format(agent_id=a), AgentIdle)
            if c().isIdle:
                idle_l += [a]

        return idle_l

    # FIRST dispatch strategy: get agents able to do pick and place and dipatch to idles
    def dispatch2idle(self):
        # retrieve available agents
        a_dic = self.get_picknplace_agents()

        # dispatch tasks
        r = rospy.Rate(0.5)
        while len(self.task_list):
            idles = self.get_idle_agents(a_dic)
            if idles:
                a = idles[0]
                # dispatch in thread
                t = self.task_list[0]
                print 'sending task: {t}'.format(t=t)
                t.state = Task2Dispatch.states[1]
                t.agent_id = a
                t.srv_address = a_dic[a]
                th = threading.Thread(target=self.dispatch_task, args=[t,])
                th.start()
                self.task_list = self.task_list[1:]
            r.sleep()

    #init
    def __init__(self, name, interface):
        smach.State.__init__(self,outcomes=['success','error','failure'],
                input_keys = ['balloons','deployment_point']) #piles = {'name': {'prop':value,...},...}

        #members
        self.name = name

        #interface elements
        self.iface = interface

    # main function
    def execute(self, userdata):

        self.deployment_point = userdata.deployment_point

        # compute and add shared regions
        add_reg = rospy.ServiceProxy('/add_shared_region', AddSharedRegion)

        p = Polygon()
        p_x = userdata.deployment_point.x
        p_y = userdata.deployment_point.y
        bb = [p_x-1,p_y-1,p_x+1,p_y+1]
        p.points = [Point32(bb[0],bb[1],0),Point32(bb[2],bb[1],0),Point32(bb[2],bb[3],0),Point32(bb[0],bb[3],0)]
        shared_regions = [p]

        self.shared_regions = shared_regions

        for p in shared_regions:
            req = AddSharedRegionRequest()
            req.frame_id = 'map'
            req.waiting_points = p.points
            req.region = p
            res = add_reg(req)

        # compute list of tasks
        self.task_list =  self.compute_tasks_from_balloons(userdata.balloons)

        #print self.tasks

        ########################################################################
        # dispatch tasks to agents until completion
        self.dispatch2idle() # NOTE: Other dispatch strategies (eg. assing pile to agent) can be implemented in a different functions and replaced here
        return 'success'
