#!/usr/bin/env python
import rospy
from mbzirc_comm_objs.msg import ObjectDetection, ObjectDetectionList
from mbzirc_comm_objs.srv import DetectTypes, DetectTypesResponse
from random import randint

global object_types

def detect_types_cb(req):
    global object_types
    object_types = req.types
    return DetectTypesResponse()

def object_detection():
    global object_types
    object_types = []
    pub = rospy.Publisher('fake_objects', ObjectDetectionList, queue_size=10)
    srv = rospy.Service('detect_types', DetectTypes, detect_types_cb)
    rospy.init_node('fake_oject_detection', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    ctr = 0
    msg = ObjectDetectionList()
    obj = ObjectDetection()
    while not rospy.is_shutdown():
        ctr += 1
        if ctr == 50:
            ctr = 0
            if len(object_types) and randint(0,1):
                obj.type = object_types[randint(0,len(object_types))-1]
                msg.objects = [obj]
            else:
                msg.objects = []

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        object_detection()
    except rospy.ROSInterruptException:
        pass
