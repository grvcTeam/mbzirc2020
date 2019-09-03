#!/usr/bin/env python
import json
import copy
import argparse
import rospy
from std_msgs.msg import ColorRGBA
from mbzirc_comm_objs.msg import ObjectDetectionList
from visualization_msgs.msg import Marker, MarkerArray

# TODO: namespacing?
marker_array_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size = 1)

def get_color(properties_dict, alpha = 1.0):
    color_out = ColorRGBA()
    color_out.a = alpha
    color_out.r = 0.5  # Default is gray
    color_out.g = 0.5
    color_out.b = 0.5
    if 'color' in properties_dict:
        color_in = properties_dict['color']
        if color_in == 'red':
            color_out.r = 1.0
            color_out.g = 0.0
            color_out.b = 0.0
        elif color_in == 'green':
            color_out.r = 0.0
            color_out.g = 1.0
            color_out.b = 0.0
        elif color_in == 'blue':
            color_out.r = 0.0
            color_out.g = 0.0
            color_out.b = 1.0
        elif color_in == 'orange':
            color_out.r = 1.0
            color_out.g = 0.65
            color_out.b = 0.0
    return color_out

def sensed_objects_callback(data, ns):
    marker_array = MarkerArray()
    for id, sensed in enumerate(data.objects):
        type_marker = Marker()
        type_marker.header = sensed.header
        type_marker.ns = ns
        type_marker.lifetime = rospy.Duration(1.0)
        type_marker.id = 10 * id
        type_marker.type = Marker.TEXT_VIEW_FACING
        type_marker.action = Marker.ADD
        type_marker.pose = copy.deepcopy(sensed.pose.pose)
        type_marker.pose.position.z += 0.15  # A little bit up
        type_marker.scale.z = 0.1
        type_marker.color.r = 1.0
        type_marker.color.g = 1.0
        type_marker.color.b = 1.0
        type_marker.color.a = 1.0
        type_marker.text = sensed.type  # TODO: Add id?
        marker_array.markers.append(type_marker)

        pose_marker = Marker()
        pose_marker.header = sensed.header
        pose_marker.ns = ns
        pose_marker.lifetime = rospy.Duration(1.0)
        pose_marker.id = 10 * id + 1
        pose_marker.type = Marker.SPHERE
        pose_marker.action = Marker.ADD
        pose_marker.pose = sensed.pose.pose
        pose_marker.scale.x = sensed.pose.covariance[0]
        pose_marker.scale.y = sensed.pose.covariance[7]
        pose_marker.scale.z = sensed.pose.covariance[14]
        pose_marker.color.r = 1.0
        pose_marker.color.g = 1.0
        pose_marker.color.b = 1.0
        pose_marker.color.a = 1.0
        marker_array.markers.append(pose_marker)

        properties_dict = {}
        if sensed.properties:
            properties_dict = json.loads(sensed.properties)

        scale_marker = Marker()
        scale_marker.header = sensed.header
        scale_marker.ns = ns
        scale_marker.lifetime = rospy.Duration(1.0)
        scale_marker.id = 10 * id + 2
        scale_marker.type = Marker.CUBE
        scale_marker.action = Marker.ADD
        scale_marker.pose = sensed.pose.pose
        scale_marker.scale = sensed.scale
        scale_marker.color = get_color(properties_dict, 0.5)
        marker_array.markers.append(scale_marker)

        marker_array_pub.publish(marker_array)

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description = 'Visualize object detection, publishing visualization markers for rviz')
    parser.add_argument('-topic', type = str, default = 'sensed_objects',
                        help = 'name of the topic that publishes the ObjectDetectionList to visualize')
    args, unknown = parser.parse_known_args()
    # utils.check_unknown_args(unknown)

    rospy.init_node('visualize_object_detection', anonymous = True)

    rospy.Subscriber(args.topic, ObjectDetectionList, sensed_objects_callback, queue_size = 1, callback_args = args.topic)
    rospy.spin()

if __name__ == '__main__':
    main()
