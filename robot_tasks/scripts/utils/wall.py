import copy
import rospkg
import yaml
from geometry_msgs.msg import PoseStamped, Point
from mbzirc_comm_objs.msg import ObjectDetection as ObjectDetection
from utils.translate import int_from_color
import tf2_py as tf2

def all_piles_are_found(uav_piles, ugv_piles):
    return len(uav_piles) >= 4 and len(ugv_piles) >= 4 

def all_walls_are_found(uav_wall, ugv_wall):
    return len(uav_wall) >= 4 and ugv_wall != None 

def uav_piles_are_found(uav_piles):
    return len(uav_piles) >= 4 

def uav_walls_are_found(uav_wall):
    return len(uav_wall) >= 4

def parse_wall(wall_file, n_segments, n_layers, n_bricks):

    wall_pattern = {}
    for segment in range(n_segments):
        wall_pattern[segment] = []

    with open(wall_file, 'r') as f:
        
        lines = f.readlines()

        for line in lines:

            bricks = line.split()

            for segment in range(n_segments - 1):

                colors = []

                for i in range(segment*n_bricks,segment*n_bricks+n_bricks):
                    colors.append(int_from_color(bricks[i]))

                wall_pattern[segment].insert(0,colors)

    for layer in range(n_layers):
        wall_pattern[n_segments-1].append([ObjectDetection.COLOR_ORANGE,ObjectDetection.COLOR_ORANGE])

    return wall_pattern

class BrickInWall(object):
    def __init__(self, color, position, wall_pose):

        q = tf2.Quaternion()
        tf2.fromMsg(wall_pose.pose.orientation,q)
        t = tf2.Vector3(wall_pose.pose.position.x,wall_pose.pose.position.y,wall_pose.pose.position.z)
        brick_wall = tf2.Vector3(position.x,position.y,position.z)

        arenaToWall = tf2.Transform(q, t)
        brick_arena = arenaToWall.inverse()*brick_wall

        self.color = color
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'arena' 
        self.pose.pose.position = Point(brick_arena[0],brick_arena[1],brick_arena[2])
        self.pose.pose.orientation = wall_pose.pose.orientation


    def __repr__(self):
        return '[color = {}, pose = [{}: ({},{},{}) ({},{},{},{})]]'.format(self.color, self.pose.header.frame_id, 
                self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z, 
                self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w)


def get_build_wall_sequence(wall_blueprint, brick_scales, wall_pose, offset):

    buid_wall_sequence = []
    current_z = 0.0
    for brick_row in wall_blueprint:
        current_y = 0.0
        build_row_sequence = []
        for brick_color in brick_row:
            brick_position = Point()
            brick_position.x = 0.5 * brick_scales[brick_color].x
            brick_position.y = current_y + 0.5 * brick_scales[brick_color].y
            brick_position.z = current_z + 0.5 * brick_scales[brick_color].z
            current_y += brick_scales[brick_color].y + offset

            brick_in_wall = BrickInWall(color = brick_color, position = brick_position, wall_pose = wall_pose)
            build_row_sequence.append(brick_in_wall)

        buid_wall_sequence.append(build_row_sequence)
        current_z += brick_scales[ObjectDetection.COLOR_RED].z  # As all bricks (should) have the same height
    return buid_wall_sequence

class BrickTask(object):
    def __init__(self):
        self.color = ObjectDetection.COLOR_UNKNOWN
        self.segment = 0
        self.layer = 0
        self.position = 0.0
        self.state = 'UNINITIALIZED'

    def __repr__(self):
        return "%s(color=%r, segment=%r, layer=%r, position=%r, state=%r)" % (
            self.__class__.__name__, self.color, self.segment, self.layer, self.position, self.state)


def get_brick_task_list(wall_pattern, brick_scales):
    brick_task_list = []
    for layer_index in range(2):
        current_y = [-2.0, -2.0, -2.0]
        for brick_index in range(7):
            for segment_index in range(3):  # orange is last!
                brick = BrickTask()
                brick.color = wall_pattern[segment_index][layer_index][brick_index]
                brick.segment = segment_index
                brick.layer = layer_index
                brick.position = current_y[segment_index] + 0.5 * brick_scales[brick.color].y
                brick.state = 'TODO'
                brick_task_list.append(copy.deepcopy(brick))
                current_y[segment_index] = brick.position + 0.5 * brick_scales[brick.color].y + 0.065

    for layer_index in range(2):
        current_y = -2.0
        for brick_index in range(2):
            brick = BrickTask()
            brick.color = wall_pattern[3][layer_index][brick_index]
            brick.segment = 3
            brick.layer = layer_index
            brick.position = current_y + 0.5 * brick_scales[brick.color].y
            brick.state = 'TODO'
            brick_task_list.append(copy.deepcopy(brick))
            current_y = brick.position + 0.5 * brick_scales[brick.color].y + 0.4

    return brick_task_list

def save_brick_task_list(brick_task_list):
    brick_task_config_filename = 'saved_brick_task_list.yaml'
    brick_task_config_url = rospkg.RosPack().get_path('mbzirc_launchers') + '/config/' + brick_task_config_filename
    with open(brick_task_config_url, 'w') as config:
        yaml.dump({'brick_task_list': brick_task_list}, config)

def load_brick_test_list():
    brick_task_config_filename = 'saved_brick_task_list.yaml'
    brick_task_config_url = rospkg.RosPack().get_path('mbzirc_launchers') + '/config/' + brick_task_config_filename
    with open(brick_task_config_url, 'r') as config:
        recovered_task_list = yaml.load(config)['brick_task_list']

    return recovered_task_list
