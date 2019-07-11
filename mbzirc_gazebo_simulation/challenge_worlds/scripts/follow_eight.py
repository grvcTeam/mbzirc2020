#!/usr/bin/env python
import math
import numpy as np
import matplotlib.pyplot as plt
import time
import rospy
from uav_abstraction_layer.srv import TakeOff, GoToWaypoint, Land
from geometry_msgs.msg import PoseStamped

# Generate a lemniscate with 4n+1 points and lenght l
def generate_lemniscate(n, l):
    if n < 0:
        raise ValueError('Parameter n (number of points, 4n+1) is negative')
    if l < 0:
        raise ValueError('Parameter l (length) is negative')
    if n == 0:
        return np.array([[0, 0]])

    dtheta = 0.25*np.pi / float(n)
    a2 = 0.125*l*l
    # print('dtheta = {}, a^2 = {}'.format(dtheta, a2))

    # Calculate 1st quadrant
    points_q1 = [[0, 0]]
    for i in range(n):
        theta = 0.25*np.pi - (i+1)*dtheta
        r = math.sqrt(2*a2 * math.cos(2.0*theta))
        # print('theta = {}, r = {}'.format(theta, r))
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        # print('x = {}, y = {}'.format(x, y))
        points_q1.append([x, y])

    # Exploit x-symmetry for 4th quadrant
    points_q4 = []
    for p in reversed(points_q1):
        x = +p[0]
        y = -p[1]
        points_q4.append([x,y])

    # Exploit y-symmetry for 2nd+3rd quadrant
    points_q14 = points_q1 + points_q4[1:]
    points_q23 = []
    for p in points_q14:
        x = -p[0]
        y = +p[1]
        points_q23.append([x, y])

    # Concatenate all quadrants
    points = points_q14 + points_q23[1:]
    return np.array(points)

# Resample path with at least d-distant points
def resample_required_distance(path, d):
    new_path = []
    for a, b in zip(path, path[1:]):
        ab = b - a
        delta = np.linalg.norm(ab)
        if delta > d:
            n = math.ceil(delta/d)
            for i in range(int(n)):
                new_point = a + i*(1.0/n)*ab
                # print(new_point)
                new_path.append(new_point)
        else:
            new_path.append(a)

    new_path.append(path[-1])
    return np.array(new_path)

# Rotate path in x-y plane
def rotate_2d(path, theta):
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)
    r_post = np.array([[cos_t, sin_t], [-sin_t, cos_t]])

    new_path = []
    for point in path:
        new_point = np.dot(point, r_post)
        new_path.append(new_point)
        # print(new_point)

    return np.array(new_path)

# Translate path in x-y plane
def translate_2d(path, x, y):
    new_path = []
    for point in path:
        new_point = point + [x, y]
        new_path.append(new_point)

    return np.array(new_path)

# Follow an eight-shaped path
if __name__ == "__main__":

    rospy.init_node('follow_eight')

    eight_n      = 10   # []:    path will have at least 4n+1 points
    eight_length = 75.0 # [m]:   longitudinal length of path
    max_delta    = 0.1  # [m]:   max distance between path points
    eight_yaw    = 0.1  # [rad]: longitudinal axis yaw
    x_offset     = 50.0 # [m]:   x offset of eight center
    y_offset     = 30.0 # [m]:   y offset of eight center
    z_flight     = 10.0 # [m]:   z for the path
    v_set        = 8.0  # [m/s]  set velocity for path following
    # TODO: tilt angle, rotation sense

    lemniscate = generate_lemniscate(eight_n, eight_length)
    points = translate_2d(resample_required_distance(lemniscate, max_delta), x_offset, y_offset)
    points = rotate_2d(points, eight_yaw)
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    # print(points)
    plt.plot(x, y, 'ro')
    plt.axis('equal')
    plt.show()

    go_to_waypoint_url = 'ual/go_to_waypoint'
    rospy.wait_for_service(go_to_waypoint_url)
    go_to_waypoint = rospy.ServiceProxy(go_to_waypoint_url, GoToWaypoint)
    pose_pub = rospy.Publisher('ual/set_pose', PoseStamped, queue_size=1)

    waypoint = PoseStamped()
    waypoint.header.frame_id = 'map'  # TODO: other?
    waypoint.pose.position.x = points[0][0]
    waypoint.pose.position.y = points[0][1]
    waypoint.pose.position.z = z_flight
    waypoint.pose.orientation.x = 0
    waypoint.pose.orientation.y = 0
    waypoint.pose.orientation.z = 0
    waypoint.pose.orientation.w = 1  # TODO: Other?
    print('Going to initial position...')
    go_to_waypoint(waypoint, True)

    loop_count = 0
    while not rospy.is_shutdown():
        loop_count += 1
        print('Loop: {}'.format(loop_count))
    
        for a, b in zip(points, points[1:]):
            if rospy.is_shutdown():
                break
            ab = b - a
            delta = np.linalg.norm(ab)
            dt = delta / v_set
            waypoint.pose.position.x = b[0]
            waypoint.pose.position.y = b[1]
            pose_pub.publish(waypoint)
            # print(b)
            time.sleep(dt)

# TODO: Simpler eight?
# x = a*sin(t)
# y = a*sin(t)*cos(t)
