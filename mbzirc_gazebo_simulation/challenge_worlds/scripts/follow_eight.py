#!/usr/bin/env python
import math
import numpy as np
import matplotlib.pyplot as plt
import time
import rospy
import tf
from uav_abstraction_layer.srv import TakeOff, GoToWaypoint, Land
from uav_abstraction_layer.msg import State
from geometry_msgs.msg import PoseStamped, PointStamped

current_state = State()
def state_callback(state):
    global current_state
    if state != current_state:
        current_state = state

def generate_lemniscate(eight_length, tolerance, step_size ):

        # Generate lemniscate with small step size
        a=eight_length/2
        X=np.arange(-a,a,tolerance/10)
        Y=X*np.cos(np.arcsin(X/a))
        
        i = 0
        
        x=[]
        y=[]
        
        x.append(X[i])
        y.append(Y[i])

        while(True):
    
            inc = i + 1
            if(inc>len(X)-1):
                break
                
            d = np.sqrt((X[inc]-X[i])**2 + (Y[inc]-Y[i])**2)
            
            # Find points with the predefined step_size in between
            while(not (d > step_size)):
                inc = inc+1
                if(inc>len(X)-1):
                    inc = inc-1
                    break
                d = np.sqrt((X[inc]-X[i])**2 + (Y[inc]-Y[i])**2)

            i=inc
            x.append(X[i])
            y.append(Y[i])
        
        x.extend(x[::-1]) # X[::-1] returns a new list with X reversed 
        y.extend(y)
        rospy.loginfo( "There are {} points in the trajectory".format(len(x)) )
        rospy.loginfo( "Distance between points {}".format(step_size) )
        
        return x, y

# Follow an eight-shaped path
def main():

    rospy.init_node('follow_eight')

    
    
    pose_rate    = 15   # Pose topic desired rate
    pose_t       = 1.0/pose_rate # [s] Time between publicatios
    eight_length = 75.0 # [m]:   longitudinal length of path
    v_set        = 4.0  # [m/s]  set velocity for path following

    
    step_size    = (1.0/pose_rate) * v_set 
    tolerance    = step_size/10
    # TODO: rotation sense

    X, Y = generate_lemniscate(eight_length, tolerance, step_size)


    eight_frame_id = 'eight_path'
    take_off_url = 'ual/take_off'
    go_to_waypoint_url = 'ual/go_to_waypoint'
    rospy.wait_for_service(take_off_url)
    rospy.wait_for_service(go_to_waypoint_url)
    take_off = rospy.ServiceProxy(take_off_url, TakeOff)
    go_to_waypoint = rospy.ServiceProxy(go_to_waypoint_url, GoToWaypoint)
    pose_pub = rospy.Publisher('ual/set_pose', PoseStamped, queue_size=1)
    rospy.Subscriber('ual/state', State, state_callback)
    tf_listener = tf.TransformListener()
    tf_listener.waitForTransform('map', eight_frame_id, rospy.Time(), rospy.Duration(5))

    while current_state.state != State.LANDED_ARMED and not rospy.is_shutdown():
        rospy.loginfo("Waiting for LANDED_ARMED")
        time.sleep(1.0)
    take_off(2.0, True)

    position = PointStamped()
    position.header.frame_id = eight_frame_id
    position.point.x = X[0]
    position.point.y = Y[0]
    position.point.z = 0

    waypoint = PoseStamped()
    waypoint.header.frame_id = 'map'
    waypoint.pose.position = tf_listener.transformPoint('map', position).point
    waypoint.pose.orientation.x = 0
    waypoint.pose.orientation.y = 0
    waypoint.pose.orientation.z = 0
    waypoint.pose.orientation.w = 1  # TODO: Other?
    print('Going to initial position...')
    go_to_waypoint(waypoint, True)
    print('Following eight path...')

    loop_count = 0
    while not rospy.is_shutdown():
        loop_count += 1
        print('Loop: {}'.format(loop_count))
    
        for i in range(len(X)):

            if rospy.is_shutdown():
                break

            position.point.x = X[i]
            position.point.y = Y[i]
            waypoint.pose.position = tf_listener.transformPoint('map', position).point
            pose_pub.publish(waypoint)
            
            time.sleep(pose_t)

if __name__ == "__main__":
    main()
