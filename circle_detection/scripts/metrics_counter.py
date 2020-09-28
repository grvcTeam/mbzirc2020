#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from mbzirc_comm_objs.msg import ObjectDetection, ObjectDetectionList

class MetricsCounter(object):

    def __init__(self):
        self.frame_count = 0
        self.detection_list_count = 0
        self.circle_count = [          0,           0,           0,           0,                   0]
        self.circle_label = ["no circles", "1 circle", "2 circles", "3 circles", "4 or more circles"]
        self.img_subscriber = rospy.Subscriber("circle_detection/detected_circles_image", Image, self.frame_callback)
        self.detect_subscriber = rospy.Subscriber("sensed_objects", ObjectDetectionList, self.detection_callback)

    def frame_callback(self, msg):
        self.frame_count += 1
        rospy.loginfo("New frame received, frame_count = {}".format(self.frame_count))

    def detection_callback(self, msg):
        self.detection_list_count += 1
        rospy.loginfo("New detection list received, detection_list_count = {}".format(self.detection_list_count))
        current_circle_count = 0
        for obj in msg.objects:
            if obj.type == ObjectDetection.TYPE_HOLE:
                current_circle_count += 1
            else:
                rospy.loginfo("Something is detected, but it is not a hole, it is a: [{}]".format(obj.type))
        circle_count_index = 0
        circle_count_index = current_circle_count if current_circle_count < 4 else 4
        self.circle_count[circle_count_index] += 1
        rospy.loginfo("Found: {}\t Update: {}".format(self.circle_label[circle_count_index], self.circle_count))

def main():
    rospy.init_node('metrics_counter', anonymous=True)
    metrics_counter = MetricsCounter()
    rospy.spin()

if __name__ == '__main__':
    main()
