#!/usr/bin/env python
import rospy
from mbzirc_comm_objs.msg import ActuatorsData
from std_srvs.srv import Trigger

class ReactiveCatcher(object):
    def __init__(self):
        self.any_digital = False
        rospy.Subscriber('actuators_system/raw/actuators_data', ActuatorsData, self.actuators_callback)
        rospy.wait_for_service('actuators_system/open_gripper')
        rospy.wait_for_service('actuators_system/close_gripper')
        self.open_gripper = rospy.ServiceProxy('actuators_system/open_gripper', Trigger)
        self.close_gripper = rospy.ServiceProxy('actuators_system/close_gripper', Trigger)

    def actuators_callback(self, data):
        prev_any_digital = self.any_digital
        self.any_digital = not all(data.digital_reading)  # raw digital_readings are negated
        if not prev_any_digital and self.any_digital:
            self.close_gripper()  # Catch!
        if prev_any_digital and not self.any_digital:
            self.open_gripper()  # Release

if __name__ == '__main__':
    rospy.init_node('test_reactive_catch', anonymous = True)
    reactive_catcher = ReactiveCatcher()
    rospy.spin()
