#!/usr/bin/env python
import rospy
from mbzirc_comm_objs.srv import AskForRegion, AskForRegionResponse
# from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PointStamped, Pose, Vector3
from visualization_msgs.msg import Marker, MarkerArray

class Interval(object):
    def __init__(self, min_value, max_value):
        if min_value >= max_value:
            raise ValueError('min_value({}) >= max_value ({})'.format(min_value, max_value))
        self.min_value = min_value
        self.max_value = max_value

    def overlaps_with(self, other):
        return (self.min_value <= other.max_value) and (self.max_value >= other.min_value)

    def mid_point(self):
        return (self.min_value + self.max_value) / 2.0

    def width(self):
        return abs(self.max_value - self.min_value)

class Region(object):
    def __init__(self, min_corner, max_corner):
        self.frame_id = 'map'

        if min_corner.header.frame_id != self.frame_id:
            raise ValueError('frame_id = {} not expected'.format(min_corner.header.frame_id))  # TODO: transform?
            
        if max_corner.header.frame_id != self.frame_id:
            raise ValueError('frame_id = {} not expected'.format(max_corner.header.frame_id))  # TODO: transform?

        self.x_interval = Interval(min_corner.point.x, max_corner.point.x)
        self.y_interval = Interval(min_corner.point.y, max_corner.point.y)
        self.z_interval = Interval(min_corner.point.z, max_corner.point.z)

    def overlaps_with(self, other):
        return self.x_interval.overlaps_with(other.x_interval) and self.y_interval.overlaps_with(other.y_interval) and self.z_interval.overlaps_with(other.z_interval)

    def get_pose(self):
        pose = Pose()
        pose.position.x = self.x_interval.mid_point()
        pose.position.y = self.y_interval.mid_point()
        pose.position.z = self.z_interval.mid_point()
        pose.orientation.w = 1.0
        return pose

    def get_scale(self):
        scale = Vector3()
        scale.x = self.x_interval.width()
        scale.y = self.y_interval.width()
        scale.z = self.z_interval.width()
        return scale

class SharedRegionsManager():

    def __init__(self):
        self.regions = {}
        self.marker_duration = rospy.Duration(1.0)
        rospy.Service('ask_for_region', AskForRegion, self.ask_for_region_callback)
        self.marker_array_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size = 1)  # TODO: namespacing?
        rospy.Timer(self.marker_duration, self.publish_marker_callback)

    def ask_for_region_callback(self, req):
        proposed_region = Region(req.min_corner, req.max_corner)
        for owner_id, region in self.regions.items():
            if region.overlaps_with(proposed_region) and owner_id != req.agent_id:
                rospy.logwarn("Proposed region by agent {} overlaps with agent {}".format(req.agent_id, owner_id))
                return AskForRegionResponse(success = False)
        self.regions[req.agent_id] = proposed_region
        return AskForRegionResponse(success = True)

    def publish_marker_callback(self, event):
        marker_array = MarkerArray()
        for owner_id, region in self.regions.items():
            id_marker = Marker()
            id_marker.header.frame_id = region.frame_id
            id_marker.header.stamp = rospy.Time.now()
            id_marker.ns = 'region_owner'
            id_marker.lifetime = self.marker_duration
            id_marker.id = owner_id
            id_marker.type = Marker.TEXT_VIEW_FACING
            id_marker.action = Marker.ADD
            id_marker.pose = region.get_pose()
            id_marker.scale.z = 0.5
            id_marker.color.r = 1.0
            id_marker.color.g = 1.0
            id_marker.color.b = 1.0
            id_marker.color.a = 1.0
            id_marker.text = str(owner_id)
            marker_array.markers.append(id_marker)

            region_marker = Marker()
            region_marker.header.frame_id = region.frame_id
            region_marker.header.stamp = rospy.Time.now()
            region_marker.ns = 'region_dimensions'
            region_marker.lifetime = self.marker_duration
            region_marker.id = owner_id
            region_marker.type = Marker.CUBE
            region_marker.action = Marker.ADD
            region_marker.pose = region.get_pose()
            region_marker.scale = region.get_scale()
            region_marker.color.r = 0.8  # TODO: color from owner_id
            region_marker.color.g = 0.2
            region_marker.color.b = 0.2
            region_marker.color.a = 0.5
            marker_array.markers.append(region_marker)

        self.marker_array_pub.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('shared_regions_manager', anonymous = True)
    SharedRegionsManager()
    rospy.spin()
