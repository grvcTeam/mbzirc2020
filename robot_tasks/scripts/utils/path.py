import math
from geometry_msgs.msg import Point


def generate_area_path(width, height, column_count, z = 3.0):
    spacing = 0.5 * width / column_count
    y_min = spacing
    y_max = height - spacing

    path = []
    for i in range(column_count):
        x_column = spacing * (1 + 2*i)
        if i % 2:
            path.append(Point(x = x_column, y = y_max, z = z))
            path.append(Point(x = x_column, y = y_min, z = z))
        else:
            path.append(Point(x = x_column, y = y_min, z = z))
            path.append(Point(x = x_column, y = y_max, z = z))

    return path


def print_path(path):
    print('path of lenght {}: ['.format(len(path)))
    for point in path:
        print('[{}, {}, {}]'.format(point.x, point.y, point.z))
    print(']')


def generate_uav_paths(uav_count, field_width, field_height, column_count):
    if uav_count <= 0:
        return []

    area_path = generate_area_path(field_width, field_height, column_count)
    point_count = len(area_path)
    delta = int(math.ceil(point_count / float(uav_count)))
    paths = []
    for i in range(uav_count):
        j_min = delta * i
        j_max = delta * (i+1)
        paths.append(area_path[j_min:j_max])
    return paths

def predefined_uav_paths():
    paths = []

    left_uav = [Point(x=0, y=14, z=7), Point(x=-20, y=14, z=7), Point(x=-20, y=19, z=7), Point(x=-20, y=19, z=0.6), Point(x=-2, y=19, z=0.6), Point(x=-2, y=19, z=4)]

    right_uav = [Point(x=8, y=0, z=7), Point(x=8, y=7, z=7), Point(x=24, y=7, z=7), Point(x=24, y=21, z=7), Point(x=2, y=21, z=7)]

    paths.append(left_uav)
    paths.append(right_uav)

    return paths

def set_z(path, z):
    for point in path:
        point.z = z
    return path
