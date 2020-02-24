#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Bool

class SimpleRegionsManager():

    def __init__(self):
        self.pick_region_status = Bool()
        self.place_region_status = Bool()
        self.wait_region_status = Bool()
        self.after_place_region_status = Bool()
        self.pick_region_status.data = False
        self.place_region_status.data = False
        self.wait_region_status.data = False
        self.after_place_region_status.data = False
        rospy.Service('lock_pick_region', SetBool, self.lock_pick_region_callback)
        rospy.Service('lock_place_region', SetBool, self.lock_place_region_callback)
        rospy.Service('lock_wait_region', SetBool, self.lock_wait_region_callback)
        rospy.Service('lock_after_place_region', SetBool, self.lock_after_place_region_callback)

        self.status_duration = rospy.Duration(1.0)
        self.pick_region_status_pub = rospy.Publisher('pick_region_status', Bool, queue_size = 1)
        self.place_region_status_pub = rospy.Publisher('place_region_status', Bool, queue_size = 1)
        self.wait_region_status_pub = rospy.Publisher('wait_region_status', Bool, queue_size = 1)
        self.after_place_region_status_pub = rospy.Publisher('after_place_region_status', Bool, queue_size = 1)
        rospy.Timer(self.status_duration, self.publish_status_callback)

    def lock_pick_region_callback(self, req):
        success_response = False
        if req.data: # Lock request
            if not self.pick_region_status.data: # Is free?
                self.pick_region_status.data = True
                success_response = True
        else: # Unlock request
            if self.pick_region_status.data: # Is locked?
                self.pick_region_status.data = False
                success_response = True

        return SetBoolResponse(success = success_response)

    def lock_place_region_callback(self, req):
        success_response = False
        if req.data: # Lock request
            if not self.place_region_status.data: # Is free?
                self.place_region_status.data = True
                success_response = True
        else: # Unlock request
            if self.place_region_status.data: # Is locked?
                self.place_region_status.data = False
                success_response = True

        return SetBoolResponse(success = success_response)

    def lock_wait_region_callback(self, req):
        success_response = False
        if req.data: # Lock request
            if not self.wait_region_status.data: # Is free?
                self.wait_region_status.data = True
                success_response = True
        else: # Unlock request
            if self.wait_region_status.data: # Is locked?
                self.wait_region_status.data = False
                success_response = True

        return SetBoolResponse(success = success_response)

    def lock_after_place_region_callback(self, req):
        success_response = False
        if req.data: # Lock request
            if not self.after_place_region_status.data: # Is free?
                self.after_place_region_status.data = True
                success_response = True
        else: # Unlock request
            if self.after_place_region_status.data: # Is locked?
                self.after_place_region_status.data = False
                success_response = True

        return SetBoolResponse(success = success_response)

    def publish_status_callback(self, event):
        self.pick_region_status_pub.publish(self.pick_region_status)
        self.place_region_status_pub.publish(self.place_region_status)
        self.wait_region_status_pub.publish(self.wait_region_status)
        self.after_place_region_status_pub.publish(self.after_place_region_status)

if __name__ == '__main__':
    rospy.init_node('simple_regions_manager', anonymous = True)
    SimpleRegionsManager()
    rospy.spin()
