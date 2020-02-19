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
import tf2_py as tf2
import tf.transformations

from geometry_msgs.msg import PoseStamped, Vector3
from tasks.move import TakeOff, FollowPath
from tasks.build import Pick, Place
from utils.manager import TaskManager
from utils.robot import RobotProxy
from utils.path import generate_uav_paths, set_z
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
        
        self.n_segments = 4
        self.n_layers = 2 
        self.wall_pattern = parse_wall(wall_file, n_segments=self.n_segments, n_layers=self.n_layers, n_bricks=7)
        self.brick_task_list = get_brick_task_list(self.wall_pattern, brick_scales)
        # save_brick_task_list(self.brick_task_list)  # TODO: Only if START
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        with open(conf_file,'r') as file:
            arena_conf = yaml.safe_load(file)

        if 'arena' in arena_conf and 'x_min' in arena_conf['arena']:
            self.field_width = arena_conf['arena']['x_max'] - arena_conf['arena']['x_min']
            self.field_height = arena_conf['arena']['y_max'] - arena_conf['arena']['y_min'] 
        else:
            rospy.logerr('No arena limits specified in conf file')

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
                   
                    elif obj.type == msg.Object.TYPE_WALL and obj.sub_type == msg.Object.SUBTYPE_UAV:
                        self.uav_walls[obj.id] = pose
                    
                    elif obj.type == msg.Object.TYPE_WALL and obj.sub_type == msg.Object.SUBTYPE_UGV:
                        # If several UGV walls, we keep the most recent one
                        self.ugv_wall = pose


    def take_off(self):
        for robot_id in self.available_robots:
            userdata = smach.UserData()
            userdata.height = self.flight_levels[robot_id]  
            self.task_manager.start_task(robot_id, TakeOff(), userdata)
            self.task_manager.wait_for([robot_id])  # Sequential takeoff for safety reasons

    def lock_objects(self):
        self.objects_locked = True

    def unlock_objects(self):
        self.objects_locked = False

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
        # TODO: Activate also L and wall detector! And deactivate at the end!?

        robot_paths = {}
        point_paths = generate_uav_paths(len(self.available_robots), self.field_width, self.field_height, self.column_count)
        for i, robot_id in enumerate(self.available_robots):
            robot_path = []
            flight_level = self.flight_levels[robot_id]
            point_path = set_z(point_paths[i], flight_level)
            for point in point_path:
                waypoint = PoseStamped()
                waypoint.header.frame_id = 'arena'
                waypoint.pose.position = point
                robot_path.append(waypoint)
            robot_paths[robot_id] = robot_path

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
            rospy.sleep(1.0)

        for robot_id in self.available_robots:
            rospy.logwarn('preempting {}'.format(robot_id))
            self.task_manager.preempt_task(robot_id)
        
    def compute_waiting_poses(self, wall_position, pile_position):
        center = [(wall_position.x+pile_position.x)/2.0, (wall_position.y+pile_position.y)/2.0]
        wall_to_pile = [pile_position.x-wall_position.x, pile_position.y-wall_position.y]
        yaw = atan2(wall_to_pile.y, wall_to_pile.x)

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
            rospy.logerr('No piles in cached')

        else:
            piles_idx = range(len(self.piles))
            
            # clusters: list. Each cluster dict with 'centroid'->[x,y], 'color'->{id,area} 
            clusters = []

            while(len(piles_idx) > 0):

                # {color, id, area, x, y}
                pile_info = {
                    'color': self.piles[piles_idx[0]].color, 
                    'id': self.pile[piles_idx[0]].id,
                    'area': self.pile[piles_idx[0]].scale[0]*self.pile[piles_idx[0]].scale[1], 
                    'x': self.pile[piles_idx[0]].pose.pose.position.x, 
                    'y': self.pile[piles_idx[0]].pose.pose.position.y
                }

                # Init cluster with first pile available
                cluster = {}
                cluster['centroid'] = [pile_info['x'], pile_info['y'] ]
                cluster[pile_info['color']] = {'id': pile_info['id'], 'area': pile_info['area'] ]

                del piles_idx[0]

                for i in range(len(piles_idx)):

                    pile_info = {
                        'color': self.piles[piles_idx[i]].color, 
                        'id': self.pile[piles_idx[i]].id, 
                        'area': self.pile[piles_idx[i]].scale[0]*self.pile[piles_idx[i]].scale[1], 
                        'x': self.pile[piles_idx[i]].pose.pose.position.x, 
                        'y': self.pile[piles_idx[i]].pose.pose.position.y
                    }

                    dist = sqrt( (pile_info['x']-cluster['centroid'][0])**2 + (pile_info['y']-cluster['centroid'][1])**2 ) 
                    if dist <= dist_th:
                        
                        if pile_info['color'] in cluster and pile_info['area'] > cluster[pile_info['color']]['area']:

                            cluster['centroid'] = [ (cluster['centroid'][0]+pile_info['x'])/2.0 , (cluster['centroid'][1]+pile_info['y'])/2.0 ]                       
                            cluster[pile_info['color']]['id'] = pile_info['id']
                            cluster[pile_info['color']]['area'] = pile_info['area']

                        # Delete pile
                        del piles_idx[i]
                
                clusters.append(cluster)

            #TODO Read pile/wall area

            uav_cluster = {}
            for cluster in clusters:
                if cluster['centroid'][0] > 30:
                    if len(cluster) > len(uav_cluster):
                        uav_cluster = cluster

            if(len(uav_cluster) > 0 ):
                for key in uav_cluster.keys():
                    if key != 'centroid':
                        self.uav_piles[key] = PoseStamped()
                        for pile in self.piles:
                            if pile.id == self.uav_cluster[key][id]:
                                self.uav_piles[key].header = pile.header
                                self.uav_piles[key].pose = pile.pose.pose
            else: 
                self.uav_piles = {}

            ugv_cluster = {}
            for cluster in clusters:
                if cluster['centroid'][0] < 30:
                    if len(cluster) > len(ugv_cluster):
                        ugv_cluster = cluster
            
            if(len(ugv_cluster) > 0 ):
                for key in ugv_cluster.keys():
                    if key != 'centroid':
                        self.ugv_piles[key] = PoseStamped()
                        for pile in self.piles:
                            if pile.id == self.ugv_cluster[key][id]:
                                self.ugv_piles[key].header = pile.header
                                self.ugv_piles[key].pose = pile.pose.pose
            else: 
                self.ugv_piles = {}

            #TODO: fill in automatically piles if any of them was found?
            

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
                break

        return found

    def publishWallTfs(self):
        wall_transforms = []
        for i in range(4):
            wall_transforms[i] = geometry_msgs.msg.TransformStamped()

            wall_transforms[i].header.stamp = rospy.Time.now()
            wall_transforms[i].header.frame_id = "arena"
            wall_transforms[i].child_frame_id = "uav_wall_" + str(i)
            wall_transforms[i].transform.translation = self.uav_walls[self.wall_segment_ids[i]].pose.position
            wall_transforms[i].transform.rotation = self.uav_walls[self.wall_segment_ids[i]].pose.orientation

        self.broadcaster.sendTransform(wall_transforms)

    # Return False if wall building is aborted
    def build_wall(self):

        # Take central position from walls and piles and compute waiting positions
        p1 = self.uav_walls[self.wall_segment_ids[1]]
        p2 = self.uav_walls[self.wall_segment_ids[2]]
        p_wall = Vector3(x=(p1.pose.position.x + p2.pose.position.x)/2.0, y=(p1.pose.position.y + p2.pose.position.y)/2.0, z=0.0)
       
        pose_pile = self.uav_piles[msg.ObjectDetection.COLOR_GREEN].pose.position

        self.compute_waiting_poses(p_wall, pose_pile)

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
                            userdata.waiting_pose.pose.position.x = self.piles_waiting_pose[robot_id][0][0]
                            userdata.waiting_pose.pose.position.y = self.piles_waiting_pose[robot_id][0][1]
                            userdata.above_pile_pose = copy.deepcopy(self.uav_piles[self.assigned_brick_task[robot_id].color])
                            userdata.above_pile_pose.pose.position.z = 5.0
                            self.task_manager.start_task(robot_id, Pick(), userdata)
                            self.robot_states[robot_id] = STATE_PICKING
                            rospy.loginfo('robot {} going to pick {}'.format(robot_id, self.assigned_brick_task[robot_id].color))
                            break

                elif self.robot_states[robot_id] == STATE_PICKING:
                    if self.task_manager.is_idle(robot_id):
                        if self.task_manager.outcomes[robot_id] == 'succeeded':
                            # TODO: Place
                            userdata = smach.UserData()
                            userdata.waiting_pose.header.frame_id = 'arena'
                            userdata.waiting_pose.pose.position.x = self.piles_waiting_pose[robot_id][1][0]
                            userdata.waiting_pose.pose.position.y = self.piles_waiting_pose[robot_id][1][1]
                            userdata.segment_to_the_left_pose = getSegmentToTheLeftPose(self.assigned_brick_task[robot_id])
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
                            # TODO: save brick_task_list
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

        central_unit.unlock_objects()
        rospy.sleep(3)

        while not uav_piles_are_found(central_unit.uav_piles) and not uav_walls_are_found(central_unit.wall_segment_ids) and not rospy.is_shutdown(): 
            central_unit.look_for_objects()

        central_unit.lock_objects()
        
        finished = central_unit.build_wall()

    rospy.spin()

if __name__ == '__main__':
    main()
