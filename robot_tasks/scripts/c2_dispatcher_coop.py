#!/usr/bin/env python

# PARAMETER	SPECIFICATION
# Number of UAVs per team	Maximum of 3
# Number of UGVs per team	1
# Arena size	50mx60mx20m
# Brick shapes and material	Rectangular cube, Styrofoam material
# Bricks size (Red, Green, Blue)	Approximately 0.30mx0.20mx0.20m, 0.60mx0.20mx0.20m and 1.20x0.20x0.20m
# Bricks size (Orange)	1.80x0.20x0.20 m
# Weight of bricks	O <= 2.0kg , B <= 1.5kg , G <= 1kg , R <= 1kg,
# Brick gripping mechanism	Primarily magnetic, but other gripping mechanisms could be used
# Environment	Outdoor
# Mode of operation	Autonomous; manual allowed but penalized
# RTK/DGPS	Allowed but penalized
# Challenge duration	30 minutes
# Communications	TBD

##### Behavior #####

# Area is partitioned between the UAVs to search for piles and walls. 
# UAVs get pick&place tasks for their wall. We can create a queue of bricks to place in the wall, extracted from the given pattern. 
# Assign bricks in their order in the layer (per rows). After, completing all segments available. Then, go to the second layer.  
# Each UAV can travel at a different altitude to avoid conflicts (except for in the search task). 
# UAV pile and wall will be shared resources. 
# If we do not find piles or walls in the search task, we retry. 
# If we fail picking a brick, we retry. 
# If a brick falls down while picking or placing, we discard it and go to another one. 

# If there is no communication at all with the Ground Station, use a single UAV with CentralUnit onboard.

import copy
import rospy
import smach
import yaml
from math import pi,fabs,sin,cos,sqrt,atan2
# from enum import Enum, unique
import mbzirc_comm_objs.msg as msg
import mbzirc_comm_objs.srv as srv
import tf2_ros
import tf.transformations

from geometry_msgs.msg import PoseStamped, Vector3, Quaternion, TransformStamped
from std_srvs.srv import SetBool
from tasks.move import TakeOff, FollowPath, GoTo
from tasks.build import Pick, Place
from utils.manager import TaskManager
from utils.robot import RobotProxy
from utils.path import generate_uav_paths, set_z, predefined_uav_paths
from utils.wall import *

brick_scales = {}
brick_scales[msg.ObjectDetection.COLOR_RED]    = Vector3(x = 0.2, y = 0.3, z = 0.2)
brick_scales[msg.ObjectDetection.COLOR_GREEN]  = Vector3(x = 0.2, y = 0.6, z = 0.2)
brick_scales[msg.ObjectDetection.COLOR_BLUE]   = Vector3(x = 0.2, y = 1.2, z = 0.2)
brick_scales[msg.ObjectDetection.COLOR_ORANGE] = Vector3(x = 0.2, y = 1.8, z = 0.2)

# TODO: Use always int instead of string!?
def color_int_to_string(int_color):
    if int_color == msg.ObjectDetection.COLOR_RED:
        return 'red'
    elif int_color == msg.ObjectDetection.COLOR_GREEN:
        return 'green'
    elif int_color == msg.ObjectDetection.COLOR_BLUE:
        return 'blue'
    elif int_color == msg.ObjectDetection.COLOR_ORANGE:
        return 'orange'
    else:
        return 'unexpected'

# @unique
# class RobotState(Enum):
#     UNASSIGNED = 0
#     PICKING = 1
#     PLACING = 2
#     WAITING = 3
STATE_UNASSIGNED = 0
STATE_PICKING = 1
STATE_PLACING = 2
STATE_WAITING = 3

class CentralUnit(object):
    def __init__(self):

        # Read parameters
        conf_file = rospy.get_param('~conf_file', 'config/conf.yaml')
        wall_file = rospy.get_param('~wall_file', 'config/uav_wall.txt')
        init_task = rospy.get_param('~init_task', 0)
        
        self.n_segments = 4
        self.n_layers = 2 
        self.wall_pattern = parse_wall(wall_file, n_segments=self.n_segments, n_layers=self.n_layers, n_bricks=7)
        self.brick_task_list = get_brick_task_list(self.wall_pattern, brick_scales, init_task)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        with open(conf_file,'r') as file:
            arena_conf = yaml.safe_load(file)

        if 'arena' in arena_conf and 'x_min' in arena_conf['arena']:
            self.field_width = arena_conf['arena']['x_max'] - arena_conf['arena']['x_min']
            self.field_height = arena_conf['arena']['y_max'] - arena_conf['arena']['y_min'] 
        else:
            rospy.logerr('No arena limits specified in conf file')

        if 'uav_pile_zone' in arena_conf and 'x_min' in arena_conf['uav_pile_zone']:
            self.uav_pile_zone = arena_conf['uav_pile_zone']
        else:
            rospy.logwarn('No UAV pile zone specified in conf file')
            self.uav_pile_zone = None

        if 'uav_wall_zone' in arena_conf and 'x_min' in arena_conf['uav_wall_zone']:
            self.uav_wall_zone = arena_conf['uav_wall_zone']
        else:
            rospy.logwarn('No UAV wall zone specified in conf file')
            self.uav_wall_zone = None

        if 'ugv_pile_zone' in arena_conf and 'x_min' in arena_conf['ugv_pile_zone']:
            self.ugv_pile_zone = arena_conf['ugv_pile_zone']
        else:
            rospy.logwarn('No UGV pile zone specified in conf file')
            self.ugv_pile_zone = None

        if 'ugv_wall_zone' in arena_conf and 'x_min' in arena_conf['ugv_wall_zone']:
            self.ugv_wall_zone = arena_conf['ugv_wall_zone']
        else:
            rospy.logwarn('No UGV wall zone specified in conf file')
            self.ugv_wall_zone = None

        self.column_count = rospy.get_param('~column_count', 4)
        self.available_robots = rospy.get_param('~uav_ids', [])

        if len(self.available_robots) == 0:
            rospy.logerr('No available UAVs for Dispatcher')
        
        self.robots = {}
        self.robot_states = {}
        for robot_id in self.available_robots:
            self.robots[robot_id] = RobotProxy(robot_id)
            self.robot_states[robot_id] = STATE_UNASSIGNED
        self.assigned_brick_task = {}

        self.flight_levels = {}
        for robot_id in self.available_robots:
            self.flight_levels[robot_id] = rospy.get_param(self.robots[robot_id].url + 'flight_level', 5.0)

        self.waiting_pose = []
        self.piles = []

        self.uav_piles = {}
        self.ugv_piles = {}

        self.ugv_wall = None
        self.uav_walls = {}
        self.wall_segment_ids = []

        self.objects_locked = False
        
        rospy.Subscriber("estimated_objects", msg.ObjectList, self.estimation_callback)

        self.task_manager = TaskManager(self.robots)

    def estimation_callback(self, data):

        if not self.objects_locked:
            if len(data.objects) >= 0 and data.objects[0].type == msg.Object.TYPE_PILE:
                self.piles = copy.deepcopy(data.objects)

            elif len(data.objects) >= 0 and data.objects[0].type == msg.Object.TYPE_WALL:
                self.ugv_wall = None
                self.uav_walls = {}
            
                for obj in data.objects:
                    pose = PoseStamped()
                    pose.header = obj.header
                    pose.pose = obj.pose.pose
                   
                    if obj.type == msg.Object.TYPE_WALL and obj.sub_type == msg.Object.SUBTYPE_UAV:
                        self.uav_walls[obj.id] = pose
                    
                    elif obj.type == msg.Object.TYPE_WALL and obj.sub_type == msg.Object.SUBTYPE_UGV:
                        # If several UGV walls, we keep the most recent one
                        self.ugv_wall = pose

    def take_off(self):
        for robot_id in self.available_robots:
            userdata = smach.UserData()
            #userdata.height = self.flight_levels[robot_id]
            userdata.height = 7.0 #TODO: take off altitude fixed    
            self.task_manager.start_task(robot_id, TakeOff(), userdata)
            self.task_manager.wait_for([robot_id])  # Sequential takeoff for safety reasons

    def lock_objects(self):
        self.objects_locked = True

    def unlock_objects(self):
        self.objects_locked = False

    def save_objects_file(self):
        filename = 'saved_objects.yaml'
        config_url = rospkg.RosPack().get_path('mbzirc_launchers') + '/config/' + filename
        with open(config_url, 'w') as config:

            yaml_dict_piles = {}

            for color,centroid in self.uav_piles:
                orientation = [centroid.pose.orientation.x, centroid.pose.orientation.y, centroid.pose.orientation.z, centroid.pose.orientation.w]
                (_,_,yaw) = tf.transformations.euler_from_quaternion(orientation)

                yaml_dict_piles[color] = {'x': centroid.pose.position.x,'y': centroid.pose.position.y,'yaw': yaw}

            yaml.dump({'uav_piles': yaml_dict_piles}, config)

            yaml_dict_walls = {}

            for i,wall_id in enumerate(self.wall_segment_ids):
                centroid = self.uav_walls[wall_id]
                orientation = [centroid.pose.orientation.x, centroid.pose.orientation.y, centroid.pose.orientation.z, centroid.pose.orientation.w]
                (_,_,yaw) = tf.transformations.euler_from_quaternion(orientation)

                yaml_dict_walls[i] = {'x': centroid.pose.position.x,'y': centroid.pose.position.y,'yaw': yaw}

            yaml.dump({'uav_walls': yaml_dict_walls}, config)


    def load_objects_file(self):
        filename = 'saved_objects.yaml'
        config_url = rospkg.RosPack().get_path('mbzirc_launchers') + '/config/' + filename
    
        try:
            with open(config_url, 'r') as config:
                recovered_objects = yaml.safe_load(config)
            
            self.uav_piles = recovered_objects['uav_piles']
            self.uav_walls = recovered_objects['uav_walls']
            self.wall_segment_ids = recovered_objects['uav_segment_ids']
        
            return True

        except IOError:
        
            rospy.logwarn('No file with objects found')
            return False

    def look_for_objects(self):
        for robot_id in self.available_robots:
            service_url = 'mbzirc2020_' + robot_id + '/set_types'
            rospy.wait_for_service(service_url)
            detect_types = srv.DetectTypesRequest()
            detect_types.types = []
            detect_types.command = srv.DetectTypesRequest.COMMAND_DETECT_ALL
            detect_types.visualize = True
            try:
                set_types = rospy.ServiceProxy(service_url, srv.DetectTypes)
                response = set_types(detect_types)
                if not response.success:
                    rospy.logerr("Service call failed!")
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: {}".format(e))

            # Activate also L
            service_url = 'mbzirc2020_' + robot_id + '/ugv_wall_detector/enable'
            
            #rospy.wait_for_service(service_url)
            # try:
            #     enable_detector = rospy.ServiceProxy(service_url, SetBool)
            #     response = enable_detector(True)
            #     if not response.success:
            #         rospy.logerr("Service call failed!")
            # except rospy.ServiceException, e:
            #     rospy.logerr("Service call failed: {}".format(e))
    
        robot_paths = {}
        #point_paths = generate_uav_paths(len(self.available_robots), self.field_width, self.field_height, self.column_count)
        point_paths = predefined_uav_paths()

        for i, robot_id in enumerate(self.available_robots):
            robot_path = []
            #flight_level = self.flight_levels[robot_id]
            #point_path = set_z(point_paths[i], flight_level)
            for point in point_paths[i]:
                waypoint = PoseStamped()
                waypoint.header.frame_id = 'arena'
                waypoint.pose.position = point
                robot_path.append(waypoint)
            robot_paths[robot_id] = robot_path

        #TODO: remove from second UAV's path the points at low altitude. Descend with an additional action

        for robot_id in self.available_robots:
            print('sending goal to search_objects server {}'.format(robot_id))
            userdata = smach.UserData()
            userdata.path = robot_paths[robot_id]
            self.task_manager.start_task(robot_id, FollowPath(), userdata)


        while not rospy.is_shutdown() and not self.task_manager.are_idle(self.available_robots) and not (all_piles_are_found(self.uav_piles,self.ugv_piles) and all_walls_are_found(self.wall_segment_ids,self.ugv_wall)):
            self.cluster_piles()
            self.cluster_wall_segments()
            rospy.logwarn('len(self.uav_piles) = {}'.format(len(self.uav_piles)))
            rospy.logwarn('len(self.ugv_piles) = {}'.format(len(self.ugv_piles)))
            rospy.logwarn('len(self.wall_segment_ids) = {}'.format(len(self.wall_segment_ids)))
            rospy.sleep(1.0)


        for robot_id in self.available_robots:
            if not self.task_manager.is_idle(robot_id):
                rospy.logwarn('preempting {}'.format(robot_id))
                self.task_manager.preempt_task(robot_id)
                self.task_manager.wait_for([robot_id])

            # Deactivate L detector
            # service_url = 'mbzirc2020_' + robot_id + '/ugv_wall_detector/enable'
            
            # rospy.wait_for_service(service_url)
            # try:
            #     enable_detector = rospy.ServiceProxy(service_url, SetBool)
            #     response = enable_detector(False)
            #     if not response.success:
            #         rospy.logerr("Service call failed!")
            # except rospy.ServiceException, e:
            #     rospy.logerr("Service call failed: {}".format(e))
        

    def compute_waiting_poses(self, wall_position, pile_position):
        center = [(wall_position.x+pile_position.x)/2.0, (wall_position.y+pile_position.y)/2.0]
        wall_to_pile = [pile_position.x-wall_position.x, pile_position.y-wall_position.y]
        yaw = atan2(wall_to_pile[1], wall_to_pile[0])

        # Waiting poses: [UAV1-pile, UAV2-pile, UAV1-wall, UAV2-wall]
        d = 5.0
        self.waiting_pose = {
            self.available_robots[0]: [
                [center[0]+d*cos(pi/4.0+yaw),center[1]+d*sin(pi/4.0+yaw)],
                [center[0]+d*cos(3*pi/4.0+yaw),center[1]+d*sin(3*pi/4.0+yaw)]
            ],
            self.available_robots[1]: [
                [center[0]+d*cos(-pi/4.0+yaw),center[1]+d*sin(-pi/4.0+yaw)],
                [center[0]+d*cos(-3*pi/4.0+yaw),center[1]+d*sin(-3*pi/4.0+yaw)]
            ]
        }

    def cluster_piles(self):

        dist_th = 7.0 

        if len(self.piles) == 0:
            rospy.logerr('No piles in cache')

        else:
            piles_idx = range(len(self.piles))
            
            # clusters: list. Each cluster dict with 'centroid'->[x,y], 'color'->{id,area} 
            clusters = []

            while(len(piles_idx) > 0):

                # {color, id, area, x, y}
                pile_info = {
                    'color': self.piles[piles_idx[0]].color, 
                    'id': self.piles[piles_idx[0]].id,
                    'area': self.piles[piles_idx[0]].scale.x*self.piles[piles_idx[0]].scale.y, 
                    'x': self.piles[piles_idx[0]].pose.pose.position.x, 
                    'y': self.piles[piles_idx[0]].pose.pose.position.y
                }

                # Init cluster with first pile available
                cluster = {}
                cluster['centroid'] = [pile_info['x'], pile_info['y'] ]
                cluster[pile_info['color']] = {'id': pile_info['id'], 'area': pile_info['area'] }

                piles_idx[0] = -1

                for i,id in enumerate(piles_idx):
                    
                    if id != -1:

                        pile_info = {
                            'color': self.piles[id].color, 
                            'id': self.piles[id].id, 
                            'area': self.piles[id].scale.x*self.piles[id].scale.y, 
                            'x': self.piles[id].pose.pose.position.x, 
                            'y': self.piles[id].pose.pose.position.y
                        }

                        dist = sqrt( (pile_info['x']-cluster['centroid'][0])**2 + (pile_info['y']-cluster['centroid'][1])**2 ) 
                        if dist <= dist_th:
                            
                            if pile_info['color'] in cluster and pile_info['area'] > cluster[pile_info['color']]['area']:

                                cluster['centroid'] = [ (cluster['centroid'][0]+pile_info['x'])/2.0 , (cluster['centroid'][1]+pile_info['y'])/2.0 ]                       
                                cluster[pile_info['color']]['id'] = pile_info['id']
                                cluster[pile_info['color']]['area'] = pile_info['area']

                            # Delete pile
                            piles_idx[i] = -1
                    
                piles_idx = [id for id in piles_idx if id != -1]

                clusters.append(cluster)

            # Take clusters with more colors 
        
            pile_layout = [
                msg.ObjectDetection.COLOR_ORANGE, 
                msg.ObjectDetection.COLOR_BLUE, 
                msg.ObjectDetection.COLOR_GREEN,
                msg.ObjectDetection.COLOR_RED
            ]

            first_bigger = {}
            second_bigger = {}
            for cluster in clusters:
                if len(cluster) > len(first_bigger):
                    second_bigger = first_bigger
                    first_bigger = cluster
                elif len(cluster) > len(second_bigger):
                    second_bigger = cluster

            if len(first_bigger) == 0 or len(second_bigger) == 0:
                rospy.logwarn("No enough clusters!")
                self.uav_piles = {}
                self.ugv_piles = {}
                return

            if first_bigger['centroid'][0] > second_bigger['centroid'][0]:
                uav_cluster = first_bigger
                ugv_cluster = second_bigger
            else:
                uav_cluster = second_bigger
                ugv_cluster = first_bigger

            # uav_cluster = {}
            # for cluster in clusters:
            #     if is_within_limits(self.uav_pile_zone,cluster['centroid'][0],cluster['centroid'][1]):
            #         if len(cluster) > len(uav_cluster):
            #             uav_cluster = cluster
            
            if(len(uav_cluster) > 0 ):

                for key in uav_cluster.keys():
                    if key != 'centroid':
                        self.uav_piles[key] = PoseStamped()
                        for pile in self.piles:
                            if pile.id == uav_cluster[key]['id']:
                                self.uav_piles[key].header = pile.header
                                self.uav_piles[key].pose = pile.pose.pose

                # Fill in not found piles with cluster centroid and any of the orientations
                any_header = self.uav_piles[self.uav_piles.keys()[0]].header
                any_orientation = self.uav_piles[self.uav_piles.keys()[0]].pose.orientation

                for color in pile_layout:
                    if color not in self.uav_piles:
                        rospy.logwarn('UAV pile {} inserted automatically'.format(color))
                        self.uav_piles[color] = PoseStamped()
                        self.uav_piles[color].header = any_header 
                        self.uav_piles[color].pose.position.x = uav_cluster['centroid'][0]
                        self.uav_piles[color].pose.position.y = uav_cluster['centroid'][1]
                        self.uav_piles[color].pose.position.z = 0.0
                        self.uav_piles[color].pose.orientation = copy.deepcopy(any_orientation)
            else: 
                self.uav_piles = {}
                rospy.logwarn('UAV piles not found')

            # ugv_cluster = {}
            # for cluster in clusters:
            #     if is_within_limits(self.ugv_pile_zone,cluster['centroid'][0],cluster['centroid'][1]):
            #         if len(cluster) > len(ugv_cluster):
            #             ugv_cluster = cluster
            
            if(len(ugv_cluster) > 0 ):

                for key in ugv_cluster.keys():
                    if key != 'centroid':
                        self.ugv_piles[key] = PoseStamped()
                        for pile in self.piles:
                            if pile.id == ugv_cluster[key]['id']:
                                self.ugv_piles[key].header = pile.header
                                self.ugv_piles[key].pose = pile.pose.pose

                # Fill in not found piles with cluster centroid and any of the orientations
                any_header = self.ugv_piles[self.ugv_piles.keys()[0]].header
                any_orientation = self.ugv_piles[self.ugv_piles.keys()[0]].pose.orientation

                for color in pile_layout:
                    if color not in self.ugv_piles:
                        rospy.logwarn('UGV pile {} inserted automatically'.format(color))
                        self.ugv_piles[color] = PoseStamped()
                        self.ugv_piles[color].header = any_header 
                        self.ugv_piles[color].pose.position.x = ugv_cluster['centroid'][0]
                        self.ugv_piles[color].pose.position.y = ugv_cluster['centroid'][1]
                        self.ugv_piles[color].pose.position.z = 0.0
                        self.ugv_piles[color].pose.orientation = copy.deepcopy(any_orientation)

            else: 
                self.ugv_piles = {}
                rospy.logwarn('UGV piles not found')


    def clusters_connected(self, top_segment, bottom_segment):

        result = False

        wall_size = 4
        dist_error = 1.5
        angle_error = pi/6

        # Compute distance
        max_dist = sqrt(2*((wall_size/2)**2)) + dist_error

        dist = sqrt((top_segment.pose.position.x - bottom_segment.pose.position.x)**2 + (top_segment.pose.position.y - bottom_segment.pose.position.y)**2)

        # Compute angle between segments
        top_quaternion = [top_segment.pose.orientation.x, top_segment.pose.orientation.y, top_segment.pose.orientation.z, top_segment.pose.orientation.w]
        bottom_quaternion = [bottom_segment.pose.orientation.x, bottom_segment.pose.orientation.y, bottom_segment.pose.orientation.z, bottom_segment.pose.orientation.w]
        (roll_t,pitch_t,yaw_t) = tf.transformations.euler_from_quaternion(top_quaternion)
        (roll_b,pitch_b,yaw_b) = tf.transformations.euler_from_quaternion(bottom_quaternion)
        
        angle = yaw_t - yaw_b

        if angle >= pi:
            angle = angle - 2*pi
        elif angle <= -2*pi:
            angle = angle + 2*pi
    
        # Compute local vector from top to bottom, to check if bottom is above top
        bottom_top = [bottom_segment.pose.position.x - top_segment.pose.position.x, bottom_segment.pose.position.y - top_segment.pose.position.y]
        bottom_top = [bottom_top[0]*cos(yaw_t) + bottom_top[1]*sin(yaw_t), -bottom_top[0]*sin(yaw_t) + bottom_top[1]*cos(yaw_t)]

        if dist <= max_dist and pi/2-angle_error <= fabs(angle) and fabs(angle) <= pi/2+angle_error and bottom_top[1] > 0:
            result = True

        return result

    def cluster_wall_segments(self):
    
        changed = True

        clusters = []
        for id in self.uav_walls.keys():
            clusters.append([id])

        while changed:
            changed = False
            nclusters = len(clusters)

            for i in range(nclusters):
                for j in range(i+1,nclusters):

                    top_segment = self.uav_walls[clusters[i][-1]]
                    bottom_segment = self.uav_walls[clusters[j][0]]

                    if self.clusters_connected(top_segment,bottom_segment):
                        clusters[i].extend(clusters[j])
                        changed = True
                        del clusters[j]

                    else:
                        top_segment = self.uav_walls[clusters[j][-1]]
                        bottom_segment = self.uav_walls[clusters[i][0]]

                        if self.clusters_connected(top_segment,bottom_segment):
                            clusters[j].extend(clusters[i])  
                            clusters[i] = clusters[j]
                            changed = True
                            del clusters[j]

                    if changed:
                        break
                
                if changed:
                    break
 
        found = False
        self.wall_segment_ids = []
        for cluster in clusters:
            if len(cluster) == self.n_segments:
                self.wall_segment_ids = cluster[:]
                found = True
                rospy.loginfo('Cluster of walls with ids {}'.format(self.wall_segment_ids))
                break

        return found

    def publishWallTfs(self):
        wall_transforms = []
        for i in range(4):
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "arena"
            transform.child_frame_id = "uav_wall_" + str(i)
            transform.transform.translation = self.uav_walls[self.wall_segment_ids[i]].pose.position
            transform.transform.rotation = self.uav_walls[self.wall_segment_ids[i]].pose.orientation
            wall_transforms.append(transform)

        self.broadcaster.sendTransform(wall_transforms)

    def reallignWalls(self):
        # Force left to right order
        dx = self.uav_walls[self.wall_segment_ids[1]].pose.position.x - self.uav_walls[self.wall_segment_ids[0]].pose.position.x
        if dx < 0:
            self.wall_segment_ids.reverse() # Reorder

        # Allign walls
        for i in range(3):
            orig_id = i
            dest_id = i+1
            if i == 4:
                orig_id = i
                dest_id = i-1

            dx = self.uav_walls[self.wall_segment_ids[dest_id]].pose.position.x - self.uav_walls[self.wall_segment_ids[orig_id]].pose.position.x
            dy = self.uav_walls[self.wall_segment_ids[dest_id]].pose.position.y - self.uav_walls[self.wall_segment_ids[orig_id]].pose.position.y
            d_orig = dx*dx + dy*dy

            wall_quaternion = [
                self.uav_walls[self.wall_segment_ids[orig_id]].pose.orientation.x,
                self.uav_walls[self.wall_segment_ids[orig_id]].pose.orientation.y,
                self.uav_walls[self.wall_segment_ids[orig_id]].pose.orientation.z,
                self.uav_walls[self.wall_segment_ids[orig_id]].pose.orientation.w
            ]
            (_,_,wall_yaw) = tf.transformations.euler_from_quaternion(wall_quaternion)
            dx_moved = dx + 2 * sin(wall_yaw)
            dy_moved = dy + 2 * cos(wall_yaw)
            d_moved = dx_moved*dx_moved + dy_moved*dy_moved

            if (d_moved > d_orig and i < 3) or (d_moved < d_orig and i == 3):
                # The next wall is in the other direction, change orientation by 180 degrees
                new_quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,wall_yaw+pi)
                self.uav_walls[self.wall_segment_ids[orig_id]].pose.orientation = Quaternion(*new_quaternion)

    # Return False if wall building is aborted
    def build_wall(self):

        # Reallign walls and publish static wall tfs
        self.reallignWalls()
        self.publishWallTfs()

        # Take central position from walls and piles and compute waiting positions
        p1 = self.uav_walls[self.wall_segment_ids[1]]
        p2 = self.uav_walls[self.wall_segment_ids[2]]
        p_wall = Vector3(x=(p1.pose.position.x + p2.pose.position.x)/2.0, y=(p1.pose.position.y + p2.pose.position.y)/2.0, z=0.0)

        pose_pile = self.uav_piles[msg.ObjectDetection.COLOR_GREEN].pose.position

        #self.compute_waiting_poses(p_wall, pose_pile)
        # Magic waiting poses
        waiting_poses = []
        waiting_poses.append([0.0, 6.0, 4.0])
        waiting_poses.append([0.0, 15.0, 6.0])

        # Send robots to waiting positions
        for i,robot_id in enumerate(self.available_robots):
            userdata = smach.UserData()
            userdata.waypoint = PoseStamped()
            userdata.waypoint.header.frame_id = 'arena'
            userdata.waypoint.pose.position.x = waiting_poses[i][0]
            userdata.waypoint.pose.position.y = waiting_poses[i][1]
            userdata.waypoint.pose.position.z = waiting_poses[i][2]
            userdata.waypoint.pose.orientation.x = 0
            userdata.waypoint.pose.orientation.y = 0
            userdata.waypoint.pose.orientation.z = 0
            userdata.waypoint.pose.orientation.w = 1
            self.task_manager.start_task(robot_id, GoTo(), userdata)
            self.task_manager.wait_for([robot_id])

        while not rospy.is_shutdown():

            for robot_id in self.available_robots:

                if self.robot_states[robot_id] == STATE_UNASSIGNED:
                    if any(state == STATE_WAITING for state in self.robot_states.itervalues()):
                        self.robot_states[robot_id] = STATE_WAITING
                        continue
                    for brick_task in self.brick_task_list:
                        if brick_task.state == 'TODO':
                            brick_task.state = 'DOING'
                            self.assigned_brick_task[robot_id] = brick_task
                            userdata = smach.UserData()
                            userdata.color = color_int_to_string(self.assigned_brick_task[robot_id].color)
                            userdata.waiting_pose = PoseStamped() 
                            userdata.waiting_pose.header.frame_id = 'arena'
                            userdata.waiting_pose.pose.position.x = waiting_poses[0][0] #self.waiting_pose[robot_id][0][0]
                            userdata.waiting_pose.pose.position.y = waiting_poses[0][1] #self.waiting_pose[robot_id][0][1]
                            userdata.waiting_pose.pose.position.z = waiting_poses[0][2] # TODO: magic!
                            userdata.waiting_pose.pose.orientation.x = 0
                            userdata.waiting_pose.pose.orientation.y = 0
                            userdata.waiting_pose.pose.orientation.z = 0
                            userdata.waiting_pose.pose.orientation.w = 1
                            userdata.above_pile_pose = copy.deepcopy(self.uav_piles[self.assigned_brick_task[robot_id].color])
                            userdata.above_pile_pose.pose.position.z = 4.0  # TODO: magic!
                            userdata.above_pile_pose.pose.orientation.x = 0
                            userdata.above_pile_pose.pose.orientation.y = 0
                            userdata.above_pile_pose.pose.orientation.z = 0
                            userdata.above_pile_pose.pose.orientation.w = 1
                            self.task_manager.start_task(robot_id, Pick(), userdata)
                            self.robot_states[robot_id] = STATE_PICKING
                            rospy.loginfo('robot {} going to pick {}'.format(robot_id, self.assigned_brick_task[robot_id].color))
                            break

                elif self.robot_states[robot_id] == STATE_PICKING:
                    if self.task_manager.is_idle(robot_id):
                        if self.task_manager.outcomes[robot_id] == 'succeeded':
                            # Place
                            userdata = smach.UserData()
                            userdata.waiting_pose = PoseStamped()
                            userdata.waiting_pose.header.frame_id = 'arena'
                            userdata.waiting_pose.pose.position.x = waiting_poses[1][0] #self.waiting_pose[robot_id][1][0]
                            userdata.waiting_pose.pose.position.y = waiting_poses[1][1] #self.waiting_pose[robot_id][1][1]
                            userdata.waiting_pose.pose.position.z = waiting_poses[1][2] # TODO: magic!
                            userdata.waiting_pose.pose.orientation.x = 0
                            userdata.waiting_pose.pose.orientation.y = 0
                            userdata.waiting_pose.pose.orientation.z = 0
                            userdata.waiting_pose.pose.orientation.w = 1
                            userdata.segment_to_the_left_pose = getSegmentToTheLeftPose(self.assigned_brick_task[robot_id])
                            userdata.segment_to_the_left_pose.pose.position.z = 2.5  # TODO: magic!
                            userdata.segment_offset = abs(self.assigned_brick_task[robot_id].position)
                            self.task_manager.start_task(robot_id, Place(), userdata)
                            self.robot_states[robot_id] = STATE_PLACING
                            rospy.loginfo('robot {} going to place {}'.format(robot_id, self.assigned_brick_task[robot_id].color))

                        else:
                            self.assigned_brick_task[robot_id].state = 'TODO'
                            self.robot_states[robot_id] = STATE_WAITING
                            # TODO: if pick fails due to not brick found, removed object from Estimator, remove locally and abort
                            # self.uav_piles[brick.color] = [] 
                            # call Estimator service to remove object self.uav_pile_ids[brick.color] 
                            # rospy.logwarn("Missing pile, aborting wall building")

                elif self.robot_states[robot_id] == STATE_PLACING:
                    if self.task_manager.is_idle(robot_id):
                        if self.task_manager.outcomes[robot_id] == 'succeeded':
                            self.assigned_brick_task[robot_id].state = 'DONE'

                            # Save in file actual state of tasks
                            #save_brick_task_list(self.brick_task_list)
                            self.robot_states[robot_id] = STATE_UNASSIGNED
                            rospy.loginfo('robot {} finished task!'.format(robot_id))

                        else:
                            self.assigned_brick_task[robot_id].state = 'TODO'
                            self.robot_states[robot_id] = STATE_WAITING
                            # TODO: if place fails due to not wall found, removed object from Estimator, remove locally and abort
                            # del self.uav_walls[ self.wall_segment_ids[segment] ] 
                            # call Estimator service to remove object self.wall_segment_ids[segment]
                            # rospy.logwarn("Missing wall, aborting wall building")

                elif self.robot_states[robot_id] == STATE_WAITING:
                    if all(state == STATE_WAITING for state in self.robot_states.itervalues()):
                        rospy.logwarn('All uavs are waiting!')
                        return False

            rospy.sleep(0.5)

            if all(brick_task == 'DONE' for brick_task in self.brick_task_list):
                rospy.loginfo('All tasks done!')
                return True


def main():
    rospy.init_node('central_unit_c2')

    while rospy.get_rostime() == rospy.Time():
        rospy.logwarn("Waiting for (sim) time to begin!")
        rospy.sleep(1)

    central_unit = CentralUnit()
    central_unit.take_off()

    finished = False
    while not finished and not rospy.is_shutdown():

        if(not central_unit.load_objects_file()):

            central_unit.unlock_objects()
            rospy.sleep(3)

            while (not uav_piles_are_found(central_unit.uav_piles) or not uav_walls_are_found(central_unit.wall_segment_ids)) and not rospy.is_shutdown(): 
                central_unit.look_for_objects()

            central_unit.lock_objects()
            #central_unit.save_objects_file()
        
        rospy.loginfo('Building wall!')
        finished = central_unit.build_wall()

    rospy.spin()

if __name__ == '__main__':
    main()
