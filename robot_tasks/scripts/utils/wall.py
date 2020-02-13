from geometry_msgs.msg import PoseStamped, Point
from mbzirc_comm_objs.msg import ObjectDetection as ObjectDetection
from utils import int_from_color
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

    with open(wall_file, “r”) as f:
        
        lines = f.readlines()

        for line in lines:

            bricks = line.split()

            for segment in range(n_segments - 1):

                colors = []

                for i in range(segment*n_bricks:segment*n_bricks+n_bricks):
                    colors.append(int_from_color(bricks[i]))

                wall_pattern[segment].insert(0,colors)

    for layer in range(n_layers):
        wall_patter[n_segments-1].append([ObjectDetection.COLOR_ORANGE,ObjectDetection.COLOR_ORANGE])

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
