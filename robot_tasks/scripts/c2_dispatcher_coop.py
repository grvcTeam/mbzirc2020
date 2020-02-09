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
import mbzirc_comm_objs.msg as msg

from geometry_msgs.msg import PoseStamped, Vector3
from tasks.move import TakeOff, FollowPath
from tasks.build import PickAndPlace
from utils.translate import color_from_int
from utils.manager import TaskManager
from utils.robot import RobotProxy
from utils.path import generate_uav_paths, set_z
from utils.wall import all_piles_are_found, all_walls_are_found, get_build_wall_sequence


field_width  = 20  # TODO: read them from file 
field_height = 20  # TODO: should be 50
column_count = 4  # 6  # TODO: as a function of fov

brick_scales = {}
# TODO: use enums (or just r/g/b/o) for colors instead of long strings?
brick_scales['red']    = Vector3(x = 0.3, y = 0.2, z = 0.2)
brick_scales['green']  = Vector3(x = 0.6, y = 0.2, z = 0.2)
brick_scales['blue']   = Vector3(x = 1.2, y = 0.2, z = 0.2)
brick_scales['orange'] = Vector3(x = 1.8, y = 0.2, z = 0.2)

# TODO: from especification, assume x-z layout
wall_blueprint = {1: [['red', 'green'], ['green', 'red']], 2: [['red', 'green'], ['green', 'red']], 3: [['red', 'green'], ['green', 'red']], 4: [['orange', 'orange'], ['orange', 'orange']] } 


class CentralUnit(object):
    def __init__(self):
        self.available_robots = ['1', '2', '3'] # Force id to be a string to avoid index confussion  # TODO: auto discovery (and update!)

        self.robots = {}
        for robot_id in self.available_robots:
            self.robots[robot_id] = RobotProxy(robot_id)

        self.ugv_piles = {}
        self.ugv_wall = None
        self.uav_piles = {}
        self.uav_walls = {}
        self.objects_locked = False
        
        rospy.Subscriber("estimated_objects", msg.ObjectList, self.estimation_callback)

        self.task_manager = TaskManager(self.robots)

    def get_param(self, robot_id, param_name):
        # TODO: Default value in case param_name is not found?
        return rospy.get_param(self.robots[robot_id].url + param_name)

    def estimation_callback(self, data):

        if not objects_locked:

            for obj in data.objects:
                # TODO: check scale?
                color = color_from_int(pile.color)
                pose = PoseStamped()
                pose.header = obj.header
                pose.pose = obj.pose.pose

                if obj.type == msg.Object.TYPE_PILE and obj.sub_type == msg.Object.SUBTYPE_UAV:
                    self.uav_piles[color] = pose

                elif obj.type == msg.Object.TYPE_PILE and obj.sub_type == msg.Object.SUBTYPE_UGV:
                    self.ugv_piles[color] = pose
                
                elif obj.type == msg.Object.TYPE_WALL and obj.sub_type == msg.Object.SUBTYPE_UAV:
                    self.uav_walls[obj.id] = pose
                
                elif obj.type == msg.Object.TYPE_WALL and obj.sub_type == msg.Object.SUBTYPE_UGV:
                    self.ugv_wall = pose


    def take_off(self):
        for robot_id in self.available_robots:
            # TODO: clear all regions?
            userdata = smach.UserData()
            userdata.height = self.get_param(robot_id, 'flight_level')  # TODO: Why not directly inside tasks?
            self.task_manager.start_task(robot_id, TakeOff(), userdata)
            self.task_manager.wait_for([robot_id])  # Sequential takeoff for safety reasons

    def lock_objects(self):
        objects_locked = True

    def look_for_objects(self):
        robot_paths = {}
        point_paths = generate_uav_paths(len(self.available_robots), field_width, field_height, column_count)
        for i, robot_id in enumerate(self.available_robots):
            robot_path = []
            flight_level = self.get_param(robot_id, 'flight_level')
            point_path = set_z(point_paths[i], flight_level)
            for point in point_path:
                waypoint = PoseStamped()
                waypoint.header.frame_id = 'arena'
                waypoint.pose.position = point
                waypoint.pose.orientation.z = 0
                waypoint.pose.orientation.w = 1  # TODO: other orientation?
                robot_path.append(waypoint)
            robot_paths[robot_id] = robot_path

        for robot_id in self.available_robots:
            print('sending goal to search_objects server {}'.format(robot_id))
            userdata = smach.UserData()
            userdata.path = robot_paths[robot_id]
            self.task_manager.start_task(robot_id, FollowPath(), userdata)

        while not all_piles_are_found(self.uav_piles,self.ugv_piles) and not all_walls_are_found(self.uav_walls,self.ugv_wall) and not rospy.is_shutdown():
            # TODO: What happens if all piles are NEVER found?
            rospy.logwarn('len(self.uav_piles) = {}'.format(len(self.uav_piles)))
            rospy.logwarn('len(self.ugv_piles) = {}'.format(len(self.ugv_piles)))
            rospy.sleep(1.0)

        for robot_id in self.available_robots:
            rospy.logwarn('preempting {}'.format(robot_id))
            self.task_manager.preempt_task(robot_id)

    def build_wall(self):
        piles = copy.deepcopy(self.uav_piles)  # Cache piles
        
        build_wall_sequence = {}
        for wall_id,wall_pose in self.uav_walls.items():
            build_wall_sequence[wall_id] = get_build_wall_sequence(wall_blueprint[wall_id], brick_scales, wall_pose)

        for wall_id in build_wall_sequence.keys():

            for i, row in enumerate(build_wall_sequence[wall_id]):
                for brick in row:
                    print('row[{}] brick = {}'.format(i, brick))
                    costs = {}
                    while not costs and not rospy.is_shutdown():
                        for robot_id in self.available_robots:
                            if self.task_manager.is_idle(robot_id):
                                costs[robot_id] = self.robots[robot_id].get_cost_to_go_to(piles[brick.color])
                            else:
                                # print('waiting for an idle robot...')
                                rospy.sleep(0.5)
                    min_cost_robot_id = min(costs, key = costs.get)
                    print('costs: {}, min_cost_id: {}'.format(costs, min_cost_robot_id))

                    flight_level = self.get_param(min_cost_robot_id,'flight_level')
                    userdata = smach.UserData()
                    userdata.above_pile_pose = copy.deepcopy(piles[brick.color])
                    userdata.above_pile_pose.pose.position.z = flight_level
                    userdata.above_wall_pose = copy.deepcopy(brick.pose)
                    userdata.above_wall_pose.pose.position.z = flight_level
                    userdata.in_wall_brick_pose = copy.deepcopy(brick.pose)
                    self.task_manager.start_task(min_cost_robot_id, PickAndPlace(), userdata)

        # Once arrived here, last pick_and_place task has been allocated
        print('All pick_and_place tasks allocated')
        finished_robots = []
        while not rospy.is_shutdown():
            rospy.sleep(0.5)  # TODO: some sleep here?
            for robot_id in self.available_robots:
                if self.task_manager.is_idle(robot_id) and (robot_id not in finished_robots):
                    finished_robots.append(robot_id)
                    print('now go home, robot [{}]!'.format(robot_id))
                    flight_level = self.get_param(robot_id, 'flight_level')
                    go_home_path = []
                    current_at_flight_level = copy.deepcopy(self.robots[robot_id].pose)
                    current_at_flight_level.pose.position.z = flight_level
                    home_at_flight_level = copy.deepcopy(self.robots[robot_id].home)
                    home_at_flight_level.pose.position.z = flight_level
                    go_home_path.append(current_at_flight_level)
                    go_home_path.append(home_at_flight_level)

                    userdata = smach.UserData()
                    userdata.path = go_home_path
                    self.task_manager.start_task(robot_id, FollowPath(), userdata)

            if set(self.available_robots).issubset(finished_robots):
                print('All done!')
                break

def main():
    rospy.init_node('central_unit_c2')

    while rospy.get_rostime() == rospy.Time():
        rospy.logwarn("Waiting for (sim) time to begin!")
        rospy.sleep(1)

    central_unit = CentralUnit()
    # rospy.sleep(3)

    central_unit.take_off()
    while not uav_piles_are_found(central_unit.uav_piles) and not uav_walls_are_found(central_unit.uav_walls): 
        central_unit.look_for_objects()
        #TODO: fill in automatically piles if any of them was found?

    central_unit.lock_objects()
    # TODO: is uav_walls modified inside the function?
    order_wall_segments(self.uav_walls)
    
    central_unit.build_wall()

    rospy.spin()

if __name__ == '__main__':
    main()
