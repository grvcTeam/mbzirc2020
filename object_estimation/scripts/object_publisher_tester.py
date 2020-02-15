#!/usr/bin/env python
# license removed for brevity
import rospy
from mbzirc_comm_objs.msg import Object,ObjectList,ObjectDetection
import std_msgs.msg
import geometry_msgs.msg
import tf_conversions
import tf2_ros

def tester():
    topic = 'estimated_objects'
    pub = rospy.Publisher(topic, ObjectList, queue_size=1)
    rospy.init_node('tester', anonymous=True)

    while not rospy.is_shutdown():
            
        key = raw_input('Press s to send a detection: ')

        if(key == "s"):

            object_list = ObjectList()
            object_list.stamp = rospy.Time.now()

            for i in range(7):

                obj = Object()
                obj.header = std_msgs.msg.Header()
                obj.header.stamp = rospy.Time.now()
                obj.header.frame_id = 'arena'

                obj.type = Object.TYPE_WALL
                obj.sub_type = Object.SUBTYPE_UAV
                obj.color = ObjectDetection.COLOR_UNKNOWN

                obj.scale = geometry_msgs.msg.Vector3(0.4,4.0,1.7)

                object_list.objects.append(obj)


            obj.id 
            
            obj.pose


            pub.publish(detection_list)

            
            
            detection.pose = geometry_msgs.msg.PoseWithCovariance()
            detection.pose.pose.position.x = 25.0
            detection.pose.pose.position.y = 20.0
            detection.pose.pose.position.z = 0.0
            yaw = 0.0
  
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
            detection.pose.pose.orientation.x = q[0]
            detection.pose.pose.orientation.y = q[1]
            detection.pose.pose.orientation.z = q[2]
            detection.pose.pose.orientation.w = q[3]
                
            detection.pose.covariance[0] = 1.0
            detection.pose.covariance[7] = 1.0
            detection.pose.covariance[14] = 1.0
                            
        
if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass