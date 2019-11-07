#! /usr/bin/env python


"""

This module serves as an interface for serial communication in the ATRV Robot


"""


from idmind_serial import IDMindSerial


def err(command_name):
    print "UR5Board: failed to send %s command" % command_name


class Board(IDMindSerial):


    battery1_voltage       = 0.0
    battery2_voltage       = 0.0
    internal48v_voltage    = 0.0
    external48v_voltage    = 0.0

    pc_ready                               = False
    ur5_enable_48v                         = False
    internal_dc_enable                     = False
    external_dc_enable                     = False
    external_power_supply_enable           = False
    ur5_internal_external_power_selection  = False
    external_power_supply_exists           = False
    internal_power_supply_exists           = False

    firmware_board = ""

    def __init__(self, address):
        try:
            IDMindSerial.__init__(self, address, baudrate=115200, timeout=.5)
        except Exception as e:
            print 'Exception raised connecting to Arm'
            raise e

    def set_ur5_computer_power_on(self):
        """
        Command     -> 0x40
        Write       -> [0x40]
        Read        -> [0x40][SN][CHH][CHL]
        """
        response = []
        if not self.command([0x40], 4, response):
            err("set_ur5_computer_power_on")
            return False
        return True

    def set_ur5_computer_power_off(self):
        """
        Command     -> 0x41
        Write       -> [0x41]
        Read        -> [0x41][SN][CHH][CHL]
        """
        response = []
        if not self.command([0x41], 4, response):
            err("set_ur5_computer_power_off")
            return False
        return True

    def get_system_power_supply(self):
        """
        Command     -> 0x30
        Write       -> [0x30]
        Read        ->  [0x51]
                        [Battery1_Voltage_H]
                        [Battery1_Voltage_L]
                        [Battery2_Voltage_H]
                        [Battery2_Voltage_L]
                        [Internal48V_Voltage_H]
                        [Internal48V_Voltage_L]
                        [External48V_Voltage_H]
                        [External48V_Voltage_L]
                        [SN]
                        [CHH]
                        [CHL]
        """

        response = []
        if not self.command([0x30], 12, response):
            err("get_system_power_supply")
            return False
        response = map(ord, response)
        self.battery1_voltage       = response[0] * 256 + response[1]
        self.battery2_voltage       = response[2] * 256 + response[3]
        self.internal48v_voltage    = response[4] * 256 + response[5]
        self.external48v_voltage    = response[6] * 256 + response[7]
        return True

    def get_system_status(self):
        """
        Command     -> 0x31
        Write       -> [0x31]
        Read        ->  [0x51]
                        [SystemStatus]
                        [SN]
                        [CHH]
                        [CHL]
        """

        response = []
        if not self.command([0x31], 5, response):
            err("get_system_status")
            return False
        response = map(ord, response)
        s = response[0]
        self.pc_ready                               = bool(s & 1)
        self.ur5_enable_48v                         = bool(s & 2)
        self.internal_dc_enable                     = bool(s & 4)
        self.external_dc_enable                     = bool(s & 8)
        self.external_power_supply_enable           = bool(s & 16)
        self.ur5_internal_external_power_selection  = bool(s & 32)
        self.external_power_supply_exists           = bool(s & 64)
        self.internal_power_supply_exists           = bool(s & 128)
        return True
    
    def get_firmware_version_number(self):
        """
        Command     -> 0x20
        Write       -> [0x20]
        Read        -> [0x20]
                    -> 25 bytes for firmware version
                        [SN]
                        [CHH]
                        [CHL]
        """
        response = []
        self.firmware_board = ''
        if not self.command([0x20], 29, response):
            err("get_firmware")
            return False
        for c in response:
            self.firmware_board += c
        return True
