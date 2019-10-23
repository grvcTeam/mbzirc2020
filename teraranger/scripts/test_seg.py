#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from teraranger.cfg import EvoThermalConfig
from dynamic_reconfigure.server import Server

from geometry_msgs.msg import Twist


class TestSeg(object):
    def __init__(self):
        

        rospy.init_node("test_seg") #talvez passar isto para dentro do main?



        self.cmd_vel_pub = rospy.Publisher("/rflex/atrvjr_node/cmd_vel", Twist, queue_size=1)
        self.temp_array_sub= rospy.Subscriber("teraranger_evo_thermal/raw_temp_array", Float64MultiArray, self.temp_array_callback)
       

    def temp_array_callback (self, msg):

    	self.temp_array = msg.data
    	#print self.temp_array


    def run(self):

        while not rospy.is_shutdown():

            rospy.sleep(1)
            temps=self.temp_array

            # for temp in temps:
            #     if temp>30 and temps.index(temp)>500:
            #         print "acima dos 500"
            #         print temp
            #         print temps.index(temp)


            #     if temp>30 and temps.index(temp)<500:
            #         print "ABAIXO dos 500"
            #         print temps.index(temp)

            hots = []

            for temp in temps:
                if temp>50:

                    #hots.append(temp)

                    result=float(temps.index(temp))/float(32)

                    decimal = result - int(result)

                    column = decimal * 32

                    line = int(result) + 1

                    print "index " + str(temps.index(temp))

                    print "line " + str(line)
                    print "column " + str(column)

                    if column < 11:
                        print "need to turn right"
                        vel_msg = Twist()
                        vel_msg.linear.y = 0
                        vel_msg.linear.z = 0
                        vel_msg.angular.x = 0
                        vel_msg.angular.y = 0
                        vel_msg.angular.z = -0.1


                        self.cmd_vel_pub.publish(vel_msg)
                        

                    elif column > 22:
                        print "need to turn left"

                        vel_msg = Twist()
                        vel_msg.linear.y = 0
                        vel_msg.linear.z = 0
                        vel_msg.angular.x = 0
                        vel_msg.angular.y = 0
                        vel_msg.angular.z = 0.1


                        self.cmd_vel_pub.publish(vel_msg)

            #print len(hots)

                    

if __name__ == "__main__":
    test_seg = TestSeg()
    try:
        test_seg.run()
    except rospy.ROSInterruptException:
        exit()
