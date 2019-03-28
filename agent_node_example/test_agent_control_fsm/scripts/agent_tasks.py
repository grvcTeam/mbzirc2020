import roslib
import rospy
import smach
import smach_ros
import tf2_ros
import numpy as np
import PyKDL

from shapely.geometry import Point
from geom_help import *

from agent_node import *
from mbzirc_comm_objs.msg import ObjectDetectionList, GripperAttached, WallBluePrint
from mbzirc_comm_objs.srv import DetectTypes, DetectTypesRequest, SearchForObject, SearchForObjectResponse, RequestSharedRegion, RequestSharedRegionResponse, RequestSharedRegionRequest, Magnetize, MagnetizeRequest
from agent_node_example_comm_objects.srv import SearchRegionPath, SearchRegionPathRequest
from uav_abstraction_layer.srv import GoToWaypoint, GoToWaypointRequest, TakeOff, TakeOffRequest, Land, LandRequest
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion
from std_msgs.msg import Header

class LandedReadyToTakeOffTask(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['preempted','error'],
                input_keys = ['height'])

    def execute(self, userdata):
        self.recall_preempt()
        la_client = rospy.ServiceProxy('/uav_1/ual/land', Land)
        to_client = rospy.ServiceProxy('/uav_1/ual/take_off', TakeOff)
        #land UAVAgent
        la_client(LandRequest(blocking=True))
        #wait until state is preempted, ie. until a task execution
        #is requested
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                to_client(TakeOffRequest(height=userdata.height,blocking=True))
                return 'preempted'
            r.sleep()

        return 'error'

class HoveringTask(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['preempted','error'],
                input_keys = ['height'])

    def execute(self, userdata):
        self.recall_preempt()
        to_client = rospy.ServiceProxy('/uav_1/ual/take_off', TakeOff)
        to_client(TakeOffRequest(height=userdata.height,blocking=True))
        #wait until state is preempted, ie. until a task execution
        #is requested
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                return 'preempted'
            r.sleep()

        return 'error'

class SearchForObjectsTask(smach.State):
    #Task Cleanup before ending execution
    def on_exit(self,outcome):
        self.obj_det_sub.unregister()
        return outcome

    #Callback from agent topic
    def object_detection_cb(self, msg):
        if self.interface:
            self.interface['pub_obj_det'].publish(msg)
            if msg.objects:
                self.found = True

    def __init__(self):
        smach.State.__init__(self,outcomes=['found','not_found'],
                input_keys = ['global_frame','search_region','object_types','stop_after_find','interface'])

    def execute(self, userdata):
        self.interface = userdata.interface

        #initialize state
        self.found = False

        #get search path
        sp_client = rospy.ServiceProxy('compute_region', SearchRegionPath)
        search_path = sp_client(SearchRegionPathRequest(header=
        Header(frame_id=userdata.global_frame,stamp = rospy.Time.now()),region=userdata.search_region))

        #set up object detection
        od_client = rospy.ServiceProxy('detect_types', DetectTypes)
        od_client(userdata.object_types)

        #subscribe to object detection
        self.obj_det_sub = rospy.Subscriber('/ual_1/sensed_objects', ObjectDetectionList, self.object_detection_cb)

        #start searching
        wp_client = rospy.ServiceProxy('/uav_1/ual/go_to_waypoint', GoToWaypoint)
        for waypoint in search_path.path:
            #visit each waypoint and check if objects been not_found
            if self.found and userdata.stop_after_find:
                break

            #TODO: should use the go_to_waypoint task
            wp_client(GoToWaypointRequest(waypoint=PoseStamped(header=Header(frame_id=
            userdata.global_frame,stamp = rospy.Time.now()),pose=waypoint),blocking=False ))

        return self.on_exit('found' if self.found else 'not_found')

class GoToWaypointTask(smach.State):

    #croos shared region
    def cross_region(self, region_id, question):
        #TODO: this probably has to be an action, so the process wait/go can be handled
        res = self.interface['cli_req_enter_shared'](RequestSharedRegionRequest(region_id=region_id,question=question,agent_id=self.agent_id))
        if res.answer == RequestSharedRegionResponse.WAIT:
            self.interface['cli_go_waypoint'](GoToWaypointRequest(waypoint=PoseStamped(header=res.waiting_point.header,pose=res.waiting_point.point),blocking=True ))

        #TODO: this is not complete, must wait until OK, but with the server client it is not possible
        return


    '''shared_regions is a dic 'region_id': polygon'''
    def __init__(self):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['global_frame','uav_frame','goal_pose','shared_regions','interface'])
        self.agent_id = 'uav_1'

    def execute(self, userdata):
        #add elements to Interface
        if not 'cli_req_shared' in userdata.interface:
            userdata.interface['cli_req_enter_shared'] = rospy.ServiceProxy('request_shared_region', RequestSharedRegion)
        if not 'tf_buffer' in userdata.interface:
            userdata.interface['tf_buffer'] = tf2_ros.Buffer()
            userdata.interface['tf_listener'] = tf2_ros.TransformListener(userdata.interface['tf_buffer'])
        if not 'cli_go_waypoint' in userdata.interface:
            userdata.interface['cli_go_waypoint'] = rospy.ServiceProxy('/uav_1/ual/go_to_waypoint', GoToWaypoint)

        self.interface = userdata.interface

        #Get UAV pose.
        try:
            trans_global2uav = lookup_tf_transform(userdata.global_frame, userdata.uav_frame,userdata.interface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print 'GoToWaypoint Task could not be executed'
            return 'error'

        #Test if the initial or goal pose are inside of a shared region
        uav_point = from_geom_msgs_Transform_to_Shapely_Point(trans_global2uav.transform)
        goal_point = from_geom_msgs_Pose_to_Shapely_Point(userdata.goal_pose)
        def point_in_region(point,regions):
            for r_id in regions:
                if from_geom_msgs_Polygon_to_Shapely_Polygon(regions[r_id].polygon).contains(point):
                    return r_id
            return ''

        r_id = point_in_region(uav_point,userdata.shared_regions)
        if r_id:
            print "Origin point is in shared region " + r_id + ", requesting exit"
            self.cross_region(r_id,RequestSharedRegionRequest.FREE_SHARED_REGION)
            print "Exit granted!"

        r_id = point_in_region(goal_point,userdata.shared_regions)
        if r_id:
            print "Goal point is in shared region " + r_id + ", requesting access"
            self.cross_region(r_id,RequestSharedRegionRequest.RESERVE_SHARED_REGION)
            print "Access granted!"

        print 'now, go to waypoint'
        userdata.interface['cli_go_waypoint'](GoToWaypointRequest(waypoint=
        PoseStamped(header=Header(frame_id=userdata.global_frame,stamp =
        rospy.Time.now()),pose=userdata.goal_pose),blocking=True ))

        return 'success'

class PickObjectTask(smach.State):

    def attached_cb(self, msg):
        print 'Attached changed!!'
        self.gripper_attached = msg.attached

    ''''''
    def __init__(self):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['global_frame','uav_frame','gripper_frame','type','scale','obj_pose','interface'],
                output_keys = ['trans_uav2object'])

        self.gripper_attached = False

    def execute(self, userdata):
        if not 'cli_go_waypoint' in userdata.interface:
            userdata.interface['cli_go_waypoint'] = rospy.ServiceProxy('/uav_1/ual/go_to_waypoint', GoToWaypoint)
        if not 'cli_magnetize' in userdata.interface:
            userdata.interface['cli_magnetize'] = rospy.ServiceProxy('/ual_1/magnetize', Magnetize)
        userdata.interface['sub_gripper_attached'] = rospy.Subscriber('/ual_1/attached', GripperAttached, self.attached_cb)
        if not 'pub_velocity' in userdata.interface:
            userdata.interface['pub_velocity'] = rospy.Publisher('uav_1/ual/set_velocity', TwistStamped, queue_size=1)
        if not 'tf_buffer' in userdata.interface:
            userdata.interface['tf_buffer'] = tf2_ros.Buffer()
            userdata.interface['tf_listener'] = tf2_ros.TransformListener(userdata.interface['tf_buffer'])

        #TODO: match requested object with object detection information
        print userdata.obj_pose

        #compute a waypoint from where to approach the object
        z_separation = 0.5
        trans_global2object = from_geom_msgs_Pose_to_KDL_Frame(userdata.obj_pose)
        trans_global2gripper = from_geom_msgs_Pose_to_KDL_Frame(userdata.obj_pose)
        trans_global2gripper.p += PyKDL.Vector(0,0,userdata.scale.z/2 + z_separation)
        trans_global2gripper.M = PyKDL.Rotation.Quaternion(0,0,0,1) #TODO: a better solution would aligned the gripper with the XY projection of the object
        try:
            trans_gripper2uav = lookup_tf_transform(userdata.gripper_frame, userdata.uav_frame,userdata.interface['tf_buffer'],5)
        except Exception as error:
            print repr(error)
            print 'PickObject Task could not be executed'
            return 'error'

        trans_global2uav = trans_global2gripper * from_geom_msgs_Transform_to_KDL_Frame(trans_gripper2uav.transform)

        #send the way point

        way = GoToWaypointRequest(waypoint=PoseStamped(
        header=Header(frame_id=userdata.global_frame,stamp=rospy.Time.now()),pose=
        from_KDL_Frame_to_geom_msgs_Pose(trans_global2uav)),blocking=True )

        print way

        print 'Write something to approach!'

        #raw_input()

        userdata.interface['cli_go_waypoint'](way)

        #active magnetic gripper
        userdata.interface['cli_magnetize'](MagnetizeRequest(magnetize=True ))

        #send velocity commands until the object is gripped
        vel_cmd = TwistStamped()
        vel_cmd.header.frame_id = userdata.global_frame
        vel_cmd.twist.linear.z = -0.1
        rate = rospy.Rate(10.0)
        trans_uav2global = None
        while not self.gripper_attached:
            '''try:
                trans_uav2global = lookup_tf_transform(userdata.uav_frame, userdata.global_frame,userdata.interface['tf_buffer'])
            except Exception:
                pass'''
            vel_cmd.header.stamp = rospy.Time.now()
            userdata.interface['pub_velocity'].publish(vel_cmd)
            rate.sleep()

        #vel_cmd.twist.linear.z = 3
        #vel_cmd.header.stamp = rospy.Time.now()
        #userdata.interface['pub_velocity'].publish(vel_cmd)
        print 'Attached!!'
        #compute pose respect to itself for output_keys
        if not trans_uav2global:
            print 'no trans'
            try:
                trans_uav2global = lookup_tf_transform(userdata.uav_frame, userdata.global_frame,userdata.interface['tf_buffer'],5)
            except Exception as error:
                print repr(error)
                print 'PickObject Task could not be executed'
                return 'error'

        trans_uav2object = from_geom_msgs_Transform_to_KDL_Frame(trans_uav2global.transform) * trans_global2object #TODO: Z value does not seems to be right
        print trans_uav2global.transform
        print trans_global2object
        print trans_uav2object
        userdata.trans_uav2object = from_KDL_Frame_to_geom_msgs_Transform(trans_uav2object)
        userdata.interface['sub_gripper_attached'].unregister()
        to_client = rospy.ServiceProxy('/uav_1/ual/take_off', TakeOff)
        to_client(TakeOffRequest(height=2,blocking=True))
        print 'Write something to pick!'

        #raw_input()

        return 'success'

class PlaceObjectTask(smach.State):

    ''''''
    def __init__(self):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['global_frame','uav_frame','gripper_frame','type','scale','goal_pose','trans_uav2object','interface'])

    def execute(self, userdata):
        if not 'cli_go_waypoint' in userdata.interface:
            userdata.interface['cli_go_waypoint'] = rospy.ServiceProxy('/uav_1/ual/go_to_waypoint', GoToWaypoint)
        if not 'cli_magnetize' in userdata.interface:
            userdata.interface['cli_magnetize'] = rospy.ServiceProxy('/ual_1/magnetize', Magnetize)
        if not 'tf_buffer' in userdata.interface:
            userdata.interface['tf_buffer'] = tf2_ros.Buffer()
            userdata.interface['tf_listener'] = tf2_ros.TransformListener(userdata.interface['tf_buffer'])

        #compute uav pose before dropping
        z_drop = 0.3
        trans_global2object = from_geom_msgs_Pose_to_KDL_Frame(userdata.goal_pose)
        trans_global2object.p += PyKDL.Vector(0,0,userdata.scale.z/2 + z_drop)
        trans_global2object.M = PyKDL.Rotation.Quaternion(0,0,0,1) #TODO: ignoring goal pose rotation

        print userdata.goal_pose
        print userdata.trans_uav2object

        trans_global2uav = trans_global2object * from_geom_msgs_Transform_to_KDL_Frame(userdata.trans_uav2object).Inverse()

        #send waypoint
        header = Header(frame_id=userdata.global_frame,stamp=rospy.Time.now())
        userdata.interface['cli_go_waypoint'](GoToWaypointRequest(waypoint=PoseStamped(
        header=header,pose=from_KDL_Frame_to_geom_msgs_Pose(trans_global2uav)),blocking=True ))
        rospy.sleep(2.)

        print 'Write something to place!'

        #raw_input()

        #drop the object
        userdata.interface['cli_magnetize'](MagnetizeRequest(magnetize=False ))

        return 'success'

class PickFromPileTask(smach.State):

    def select_object_z_max(self, type, list):
        if  not list:
            print 'Cannot select object from empty list'
            return None

        try:
            trans_global2camera = lookup_tf_transform(self.userdata.global_frame, list[0].header.frame_id,self.userdata.interface['tf_buffer'],5)
            trans_global2camera = from_geom_msgs_Transform_to_KDL_Frame(trans_global2camera.transform)
        except Exception as error:
            print repr(error)
            print 'Cannot select object because transform global --> camera is not available'
            return None

        z_max = (None,0.)
        for object in list:
            trans_global2object = trans_global2camera * from_geom_msgs_Pose_to_KDL_Frame(object.pose.pose)
            if object.type == type and trans_global2object.p.z() >= z_max[1]:
                z_max = (object, trans_global2object.p.z())

        return z_max[0]

    def object_detection_cb(self, msg):
        self.userdata.interface['pub_obj_det'].publish(msg)
        if msg.objects:
            self.found = True
            self.objects = msg.objects

    def __init__(self):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['global_frame','uav_frame','gripper_frame','type','pile_centroid','interface','scale','obj_pose'],
                output_keys = ['trans_uav2object','scale','obj_pose'])

        self.found = False
        self.objects = None

    def execute(self, userdata):
        if not 'cli_go_waypoint' in userdata.interface:
            userdata.interface['cli_go_waypoint'] = rospy.ServiceProxy('/uav_1/ual/go_to_waypoint', GoToWaypoint)
        if not 'obj_det_sub' in userdata.interface:
            userdata.interface['obj_det_sub'] = rospy.Subscriber('/ual_1/sensed_objects', ObjectDetectionList, self.object_detection_cb)
        if not 'tf_buffer' in userdata.interface:
            userdata.interface['tf_buffer'] = tf2_ros.Buffer()
            userdata.interface['tf_listener'] = tf2_ros.TransformListener(userdata.interface['tf_buffer'])

        self.userdata = userdata

        #go over pile ceontroid
        header = Header(frame_id=userdata.global_frame,stamp=rospy.Time.now())
        pose = Pose(position=Point(userdata.pile_centroid.x,userdata.pile_centroid.y,3),
        orientation = Quaternion(0,0,0,1))
        userdata.interface['cli_go_waypoint'](GoToWaypointRequest(waypoint=PoseStamped(header=header,pose=pose),blocking=True ))
        rospy.sleep(2.)
        #select object
        object = None
        self.found = False
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.found:
                object =  self.select_object_z_max(userdata.type,self.objects)
                if(object):
                    break
                else:
                    self.found = False
                r.sleep()

        #pick object
        print object

        userdata.scale = object.scale
        try:
            trans_global2camera = lookup_tf_transform(userdata.global_frame, object.header.frame_id,userdata.interface['tf_buffer'],5)
            trans_global2camera = from_geom_msgs_Transform_to_KDL_Frame(trans_global2camera.transform)
        except Exception as error:
            print repr(error)
            print 'Cannot select object because transform global --> camera is not available'
            return 'error'

        trans_global2object = trans_global2camera * from_geom_msgs_Pose_to_KDL_Frame(object.pose.pose)
        userdata.obj_pose = from_KDL_Frame_to_geom_msgs_Pose(trans_global2object)

        pick_task = PickObjectTask()
        pick_task.execute(userdata)
        #print userdata.trans_uav2object
        #userdata.trans_uav2object = None
        return 'success'

class PickAndPlaceObjectTask(smach.StateMachine):

    '''Parameters:
        - task: agent task to be wrapped. Can be a state or a state machine.
        - transitions: dic containing the transitions from the wrapped task outcomes
          to 'success' and 'error' '''
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['success','error'],
        input_keys = ['global_frame','uav_frame','gripper_frame','type','scale','goal_pose','obj_pose','interface'])#, input_keys=list(task._input_keys | set(['interface'])))


        with self:
            smach.StateMachine.add('Pick_Task', PickObjectTask(), {'success':'Place_Task','error':'success'})
            smach.StateMachine.add('Place_Task', PlaceObjectTask(), {'success':'success','error':'success'})

class PickFromPileAndPlaceObjectTask(smach.StateMachine):

    '''Parameters:
        - task: agent task to be wrapped. Can be a state or a state machine.
        - transitions: dic containing the transitions from the wrapped task outcomes
          to 'success' and 'error' '''
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['success','error'],
        input_keys = ['global_frame','uav_frame','gripper_frame','type','goal_pose','scale','obj_pose','pile_centroid','interface'])#, input_keys=list(task._input_keys | set(['interface'])))


        with self:
            smach.StateMachine.add('Pick_Task', PickFromPileTask(), {'success':'Place_Task','error':'success'})
            smach.StateMachine.add('Place_Task', PlaceObjectTask(), {'success':'success','error':'success'})

class BuildWallTask(smach.State):

    def from_msg_to_matrix(self,msg):
        n_x = msg.size_x
        n_y = msg.size_y
        n_z = msg.size_z

        if len(msg.blueprint) != n_x * n_y * n_z:
            print 'wall matrix malformed'
            return np.array()

        mm = []
        for k in  range(n_z):
            m = []
            for j in range(n_y):
                m += [msg.blueprint[(k*n_x*n_y+j*n_x):(k*n_x*n_y+(j+1)*n_x)]]
            mm += [m]

        return np.array(mm)

    def brick_goal_pose(self, length, i, j, k):
        x = i * 0.30 + length / 2
        y = j * 0.20 + 0.20 / 2
        z = k * 0.20 + 0.20 / 2

        return PyKDL.Frame(PyKDL.Rotation.Quaternion(0,0,0,1),PyKDL.Vector(x,y,z))

    def __init__(self):
        smach.State.__init__(self,outcomes=['success','error'],
                input_keys = ['wall_map','red_pile','green_pile','blue_pile','orange_pile','uav_frame','gripper_frame','interface'])

    def execute(self, userdata):

        #build wall matrix from msg
        wall_matrix = self.from_msg_to_matrix(userdata.wall_map)
        trans_global2wall = from_geom_msgs_Pose_to_KDL_Frame(userdata.wall_map.wall_frame.pose)

        #call pick from pile Tasks in the right order
        p_n_p_task = PickFromPileAndPlaceObjectTask()
        ud = smach.UserData()
        ud.global_frame = userdata.wall_map.wall_frame.header.frame_id
        ud.uav_frame = userdata.uav_frame
        ud.gripper_frame = userdata.gripper_frame
        ud.type = 'brick'
        ud.interface = userdata.interface

        for k in range(wall_matrix.shape[0]):
            for j in range(wall_matrix.shape[1]):
                for i in range(wall_matrix.shape[2]):
                    if wall_matrix[k,j,i] == 1: #red brick
                        trans_wall2brick = self.brick_goal_pose(0.30,i,j,k)
                        ud.goal_pose = from_KDL_Frame_to_geom_msgs_Pose(trans_global2wall * trans_wall2brick)
                        ud.pile_centroid = userdata.red_pile.pose.position
                        p_n_p_task.execute(ud)
                    elif wall_matrix[k,j,i] == 2: #green brick
                        trans_wall2brick = self.brick_goal_pose(0.60,i,j,k)
                        ud.goal_pose = from_KDL_Frame_to_geom_msgs_Pose(trans_global2wall * trans_wall2brick)
                        ud.pile_centroid = userdata.green_pile.pose.position
                        p_n_p_task.execute(ud)
                    elif wall_matrix[k,j,i] == 3: #blue brick
                        trans_wall2brick = self.brick_goal_pose(1.20,i,j,k)
                        ud.goal_pose = from_KDL_Frame_to_geom_msgs_Pose(trans_global2wall * trans_wall2brick)
                        ud.pile_centroid = userdata.blue_pile.pose.position
                        p_n_p_task.execute(ud)
                    elif wall_matrix[k,j,i] == 4: #orange brick
                        trans_wall2brick = self.brick_goal_pose(1.80,i,j,k)
                        ud.goal_pose = from_KDL_Frame_to_geom_msgs_Pose(trans_global2wall * trans_wall2brick)
                        ud.pile_centroid = userdata.orange_pile.pose.position
                        p_n_p_task.execute(ud)

        return 'success'
