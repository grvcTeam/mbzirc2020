#!/usr/bin/env python
# license removed for brevity
import rospy
from mbzirc_comm_objs.msg import ObjectDetection,ObjectDetectionList
import std_msgs.msg
import geometry_msgs.msg
import tf_conversions
import tf2_ros

def tester(agent_id='1'):
    topic = 'mbzirc2020_' + agent_id + '/sensed_objects'
    pub = rospy.Publisher(topic, ObjectDetectionList, queue_size=1)
    rospy.init_node('tester', anonymous=True)

    while not rospy.is_shutdown():
            
        key = raw_input('Press s to send a detection: ')

        if(key == "s"):

            detection_list = ObjectDetectionList()
            detection_list.agent_id = agent_id
            detection_list.stamp = rospy.Time.now()

            detection = ObjectDetection()
            detection.header = std_msgs.msg.Header()
            detection.header.stamp = rospy.Time.now()
            detection.header.frame_id = 'arena'

            detection.type = ObjectDetection.TYPE_UCHANNEL
            detection.color = ObjectDetection.COLOR_UNKNOWN
            detection.scale = geometry_msgs.msg.Vector3(0.4,4,2.0)
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
                

            detection_list.objects.append(detection)
            pub.publish(detection_list)

if __name__ == '__main__':
    try:
        tester('1')
    except rospy.ROSInterruptException:
        pass