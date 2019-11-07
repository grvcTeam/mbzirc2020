#! /usr/bin/env python


from hw_api import Board
import rospy
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from ur5_driver.msg import SystemPowerSupply, SystemStatus


class Node(object):

    def __init__(self, device_name):
        self.device_name = device_name
        self.board = Board(device_name)
        self.turn_on_ur5 = self.turn_off_ur5 = False
        rospy.Service("/idmind_ur5/enable_ur5", SetBool, self.on_enable_ur5)
        self.system_power_supply_pub = rospy.Publisher("/idmind_ur5/system_power_supply", SystemPowerSupply, queue_size=10)
        self.system_status_pub = rospy.Publisher("/idmind_ur5/system_status", SystemStatus, queue_size=10)
        rospy.Service("/idmind_ur5/reset_connection", Trigger, self.on_reset_connection)

    def on_enable_ur5(self, req):
        if req.data:
            self.turn_on_ur5 = True
            return SetBoolResponse(True, 'ur5 enabled')
        else:
            self.turn_off_ur5 = True
            return SetBoolResponse(True, 'ur5 disabled')

    def on_reset_connection(self, _):
        try:
            self.board = None
            rospy.sleep(1.0)
            self.board = Board(self.device_name)
            return TriggerResponse(True, "connection to ur5 reset")
        except Exception as e:
            return TriggerResponse(False, "connection to ur5 reset FAILED. reason: %s" % e)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not self.board:
                rate.sleep()
                continue

            # reads
            if self.board.get_system_power_supply():
                # publishes
                sps_msg = SystemPowerSupply()
                sps_msg.battery1_voltage        = self.board.battery1_voltage
                sps_msg.battery2_voltage        = self.board.battery2_voltage
                sps_msg.internal48v_voltage    = self.board.internal48v_voltage
                sps_msg.external48v_voltage    = self.board.external48v_voltage
                self.system_power_supply_pub.publish(sps_msg)

            if self.board.get_system_status():
                ss_msg = SystemStatus()
                ss_msg.pc_ready                               = self.board.pc_ready                             
                ss_msg.ur5_enable_48v                         = self.board.ur5_enable_48v                       
                ss_msg.internal_dc_enable                     = self.board.internal_dc_enable                   
                ss_msg.external_dc_enable                     = self.board.external_dc_enable                   
                ss_msg.external_power_supply_enable           = self.board.external_power_supply_enable         
                ss_msg.ur5_internal_external_power_selection  = self.board.ur5_internal_external_power_selection
                ss_msg.external_power_supply_exists           = self.board.external_power_supply_exists         
                ss_msg.internal_power_supply_exists           = self.board.internal_power_supply_exists         
                self.system_status_pub.publish(ss_msg)

            # writes
            if self.turn_off_ur5    :
                self.board.set_ur5_computer_power_off()
                self.turn_off_ur5 = False
            if self.turn_on_ur5     :
                self.board.set_ur5_computer_power_on()
                self.turn_on_ur5 = False

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("idmind_ur5")
    ur5_device_name = rospy.get_param("ur5_device_name") if rospy.has_param("ur5_device_name") else "/dev/ttyUSB0"
    node = Node(ur5_device_name)
    node.run()
    rospy.spin()
