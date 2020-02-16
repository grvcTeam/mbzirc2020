#!/usr/bin/env python
# license removed for brevity
import rospy
from mbzirc_comm_objs.msg import Object,ObjectList,ObjectDetection
import std_msgs.msg
import geometry_msgs.msg
import tf_conversions
import tf2_ros
from math import pi,sqrt

def tester():
    topic = 'estimated_objects'
    pub = rospy.Publisher(topic, ObjectList, queue_size=1)
    rospy.init_node('tester', anonymous=True)

    while not rospy.is_shutdown():
            
        key = raw_input('Press s to send a detection: ')

        if(key == "s"):

            object_list = ObjectList()
            object_list.stamp = rospy.Time.now()

            x_ini = 10
            y_ini = 10
            a = 2.0

            yaws = [-pi/4.0, pi/4.0, 0.0, -pi/4.0, pi/4.0, -pi/2.0, -pi/4.0]
            x_offs = [a*sqrt(2)/2.0, -a*sqrt(2)/2.0, -4, -a*sqrt(2)/2.0, -a*sqrt(2)/2.0, -6, -a*sqrt(2)/2.0]
            y_offs = [10*a*sqrt(2)/2.0, a*sqrt(2)/2.0, 10, 3*a*sqrt(2)/2.0, 5*a*sqrt(2)/2.0, 12, 7*a*sqrt(2)/2.0]
    

            for i in range(7):

                obj = Object()
                obj.header = std_msgs.msg.Header()
                obj.header.stamp = rospy.Time.now()
                obj.header.frame_id = 'arena'

                obj.type = Object.TYPE_WALL
                obj.sub_type = Object.SUBTYPE_UAV
                obj.color = ObjectDetection.COLOR_UNKNOWN
                obj.id = i+1

                obj.scale = geometry_msgs.msg.Vector3(0.4,4.0,1.7)

                obj.pose = geometry_msgs.msg.PoseWithCovariance()
                obj.pose.pose.position.x = x_ini + x_offs[i]  
                obj.pose.pose.position.y = y_ini + y_offs[i]
                obj.pose.pose.position.z = 1.6
    
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaws[i])
                obj.pose.pose.orientation.x = q[0]
                obj.pose.pose.orientation.y = q[1]
                obj.pose.pose.orientation.z = q[2]
                obj.pose.pose.orientation.w = q[3]

                object_list.objects.append(obj)

            pub.publish(object_list)
                
        
if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass