#!/usr/bin/env python
# -*- coding: utf-8 -*-
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from uav_abstraction_layer.msg import State
import rospkg
import rospy
import time
from os import system
from math import atan2

def ualStateToString(state):
    if state == State.UNINITIALIZED:
        state_as_string = "UNINITIALIZED"
    elif state == State.LANDED_DISARMED:
        state_as_string = "LANDED_DISARMED"
    elif state == State.LANDED_ARMED:
        state_as_string = "LANDED_ARMED"
    elif state == State.TAKING_OFF:
        state_as_string = "TAKING_OFF"
    elif state == State.FLYING_AUTO:
        state_as_string = "FLYING_AUTO"
    elif state == State.FLYING_MANUAL:
        state_as_string = "FLYING_MANUAL"
    elif state == State.LANDING:
        state_as_string = "LANDING"
    return state_as_string

class AgentStatus:
    def __init__(self,id,ns_prefix):
        self.id = id
        self.gps_fix = 0
        self.n_satellites = 0
        self.gps_cov = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        self.orientation = 0
        self.autopilot_type = ""
        self.autopilot_mode = ""
        self.vehicle_type = ""
        self.battery_voltage = 0.0
        self.battery_per_cell = 0.0
        self.battery_remaining = 0.0
        self.battery_current = 0.0
        self.ual_state = ""

        rospy.Subscriber(ns_prefix + str(id) + "/ual/pose", PoseStamped, self.ualPoseCallback)
        rospy.Subscriber(ns_prefix + str(id) + "/ual/state", State, self.ualStateCallback)
        rospy.Subscriber(ns_prefix + str(id) + "/mavros/global_position/global", NavSatFix, self.globalPositionCallback)

    def ualPoseCallback(self,data):
        self.pose_x = data.pose.position.x
        self.pose_y = data.pose.position.y
        self.pose_z = data.pose.position.z
        
        siny_cosp = 2.0 * (data.pose.orientation.w * data.pose.orientation.z + data.pose.orientation.x * data.pose.orientation.y)
        cosy_cosp = 1.0 - 2.0 * (data.pose.orientation.y * data.pose.orientation.y + data.pose.orientation.z * data.pose.orientation.z)
        self.orientation = int(atan2(siny_cosp, cosy_cosp) * 180.0/3.141592654)

    def ualStateCallback(self,data):
        self.ual_state = ualStateToString(data.state)

    def globalPositionCallback(self,data):
        self.gps_cov = data.position_covariance[0]

class SystemStatus:
    def __init__(self,ns_prefix):
        self.agents = {}
        self.ns_prefix = ns_prefix

        rospy.Subscriber("/diagnostics", DiagnosticArray, self.diagnosticsCallback)

    def diagnosticsCallback(self,data):
        fix_type = 0
        found_heartbeat = False
        n_satellites = 0
        autopilot_mode = ""
        for element in data.status:
            if element.name.find("Heartbeat") >= 0:
                for value in element.values:
                    if "Autopilot type" in value.key:
                        autopilot_type = value.value
                        found_heartbeat = True
                    if "Vehicle type" in value.key:
                        vehicle_type = value.value
                    if "Mode" in value.key:
                        autopilot_mode = value.value
            if element.name.find("GPS") >= 0:
                for value in element.values:
                    if "Fix type" in value.key:
                        fix_type = int(value.value)
                    if "Satellites visible" in value.key:
                        n_satellites = int(value.value)
            if element.name.find("Battery") >= 0:
                for value in element.values:
                    if "Voltage" in value.key:
                        battery_voltage = float(value.value)
                    if "Remaining" in value.key:
                        battery_remaining = float(value.value)
                    if "Current" in value.key:
                        battery_current = float(value.value)

        if found_heartbeat:
            agent_id_pos = data.status[0].name.find(self.ns_prefix)
            if agent_id_pos >= 0:
                agent_id = int( data.status[0].name[agent_id_pos+len(self.ns_prefix)] )
                if not( agent_id in self.agents ):
                    self.agents[agent_id] = AgentStatus(agent_id,self.ns_prefix)
                self.agents[agent_id].gps_fix           = fix_type
                self.agents[agent_id].n_satellites      = n_satellites
                self.agents[agent_id].autopilot_type    = autopilot_type
                self.agents[agent_id].autopilot_mode    = autopilot_mode
                self.agents[agent_id].vehicle_type      = vehicle_type
                self.agents[agent_id].battery_voltage   = battery_voltage
                self.agents[agent_id].battery_per_cell  = battery_voltage/6
                self.agents[agent_id].battery_remaining = battery_remaining
                self.agents[agent_id].battery_current   = battery_current

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def print_info(system_status):
    system("clear")
    print bcolors.OKBLUE + "===================================" + bcolors.ENDC
    print bcolors.OKBLUE + "============== " + bcolors.ENDC + bcolors.BOLD + "pyGCS" + bcolors.ENDC + bcolors.OKBLUE + " ==============\n" + bcolors.ENDC
    print "           --- Agents ---       "
    for id, agent in system_status.agents.items():
        print bcolors.BOLD + "\nAgent " + str(agent.id) + bcolors.ENDC
        print "  AP: " + agent.autopilot_type + " - " + agent.vehicle_type
        print "  Battery: " + ( (bcolors.BOLD + bcolors.FAIL) if agent.battery_per_cell<3.6 else (bcolors.OKGREEN if agent.battery_per_cell>=3.9 else bcolors.WARNING) ) + "{:.2f}V - {:.2f}V - {:.1f}%".format(agent.battery_voltage, agent.battery_per_cell, agent.battery_remaining) + bcolors.ENDC
        print "  Current: {:.2f}A".format(agent.battery_current)
        print "  AP mode: " + agent.autopilot_mode
        print "  UAL state: " + (bcolors.FAIL if agent.ual_state=="FLYING_MANUAL" else (bcolors.OKGREEN if agent.ual_state=="FLYING_AUTO" else "")) + agent.ual_state + bcolors.ENDC
        print "  GPS Fix/nSat/Cov: " + bcolors.BOLD + (bcolors.FAIL if agent.gps_fix<=4 else (bcolors.OKGREEN if agent.gps_fix==6 else bcolors.WARNING) ) + str(agent.gps_fix) + bcolors.ENDC + " / " + str(agent.n_satellites) + " / {:.4f}".format(agent.gps_cov)
        print "  Pose: {:6.2f} {:6.2f} {:5.2f} {:3d}º".format(agent.pose_x,agent.pose_y,agent.pose_z,agent.orientation)

    print bcolors.OKBLUE + "===================================" + bcolors.ENDC

if __name__ == "__main__":
    rospy.init_node('pygcs', anonymous=False)

    ns_prefix = rospy.get_param('~ns_prefix',"mbzirc2020_")

    system_status = SystemStatus(ns_prefix)

    while not rospy.is_shutdown():   
        print_info(system_status)
        time.sleep(0.1)
