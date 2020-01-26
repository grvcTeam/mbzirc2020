#!/usr/bin/env python
import rospy
from mbzirc_comm_objs.msg import ActuatorsData
from mbzirc_comm_objs.srv import SetPWM

class ReactiveCatcher(object):
    def __init__(self):
        self.any_digital = False
        rospy.Subscriber('actuators_system/raw/actuators_data', ActuatorsData, self.actuators_callback)
        rospy.wait_for_service('actuators_system/raw/set_pwm')
        self.set_pwm = rospy.ServiceProxy('actuators_system/raw/set_pwm', SetPWM)

    def actuators_callback(self, data):
        prev_any_digital = self.any_digital
        self.any_digital = not all(data.digital_reading)  # raw digital_readings are negated
        if not prev_any_digital and self.any_digital:
            self.set_all_pwm(1800)  # Catch!
        if prev_any_digital and not self.any_digital:
            self.set_all_pwm(1500)  # Release

    def set_all_pwm(self, pwm):
        try:
            for i in range(5):
                self.set_pwm(i, pwm)

        except:
            print("Service call failed!")

if __name__ == '__main__':
    rospy.init_node('test_reactive_catch', anonymous = True)
    reactive_catcher = ReactiveCatcher()
    rospy.spin()
