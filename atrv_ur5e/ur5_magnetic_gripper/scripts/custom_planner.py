#!/usr/bin/env python

import roslib
import rospy
import tf

import geometry_msgs.msg
import nav_msgs.msg

class node():

    def __init__(self):


        self.pub = rospy.Publisher('attached', GripperAttached, queue_size=1)
        self.pub2 = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)

        self.footprint_sub = rospy.Subscriber('/move_base/global_costmap/footprint', geometry_msgs.PolygonStamped, self.footprint_cb)
        self.costmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', nav_msgs.OccupancyGrid, self.costmap_cb)

        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', geometry_msgs.PolygonStamped, self.goal_cb)
        self.start_sub = rospy.Subscriber('/odometry/filtered', nav_msgs.Odometry, self.start_cb)

        print "dasfasdfasdfasdfasdf"






    def footprint_cb(self,msg):

        self.footprint=msg

    def costmap_cb(self,req):    
    
    def goal_cb(self,req):

    def start_cb(self,req):





    goal_x = goal.pose.position.x;
    goal_y = goal.pose.position.y;
    start_x = start.pose.position.x;
    start_y = start.pose.position.y;

    diff_x = goal_x - start_x;
    diff_y = goal_y - start_y;
    diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

    target_x = goal_x;
    target_y = goal_y;
    target_yaw = goal_yaw;

    done = false;
    scale = 1.0;
    dScale = 0.01;


    def magnetize_cb(self,req):

        if not req.magnetize:
            v = 24
            self.pub2.publish(data='set_tool_voltage({v}) '.format(v=v))
            rospy.sleep(1)

            while self.force<0:
                rospy.sleep(1)
                self.pub2.publish(data='set_tool_voltage({v}) '.format(v=v))
                
            else:
                return MagnetizeResponse(success=True)   

        if req.magnetize:
            v = 0
            self.pub2.publish(data='set_tool_voltage({v}) '.format(v=v))
            rospy.sleep(3)
            self.pub2.publish(data='set_tool_voltage({v}) '.format(v=v))  #SOLUTION NOT ROBUST AT ALL

            return MagnetizeResponse(success=True)


def main():

    rospy.init_node('ur5_magnetic_gripper')
    n = node()
    rospy.spin()

if __name__ == '__main__':
    main()
