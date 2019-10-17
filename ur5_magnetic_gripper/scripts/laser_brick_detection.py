#!/usr/bin/env python

import roslib
import rospy
import laser_line_extraction.msg
import numpy
import math

from geometry_msgs.msg import Twist


class line_node():

    def __init__(self):

       # self.pub = rospy.Publisher('attached', GripperAttached, queue_size=1)
        self.sub = rospy.Subscriber('/line_segments', laser_line_extraction.msg.LineSegmentList, self.line_segments_cb)
        self.cmd_vel_pub = rospy.Publisher("/rflex/atrvjr_node/cmd_vel", Twist, queue_size=1)

    def line_segments_cb(self, msg):

		self.line_segment_list = msg

		self.line_segment = self.line_segment_list.line_segments

		for line in self.line_segment:
			start = line.start
			end = line.end

			diff_x=abs(start[0]-end[0])
			diff_y=abs(start[1]-end[1])

			#print diff_x
			#print diff_y

			length = numpy.sqrt(diff_x**2 + diff_y**2)

			if length<0.65 and length > 0.55:
				print "brick de 60 cm detected: ", length
				print "distancia: ", line.radius
				print "angulo em radians: ", line.angle

			if length <0.35 and length > 0.25:
				print "brick de 30 cm detected: ", length
				print "distancia: ", line.radius
				print "angulo em radians: ", line.angle

			if line.radius < 1.3:
				if line.angle > 0.05:
					vel_msg = Twist()
					vel_msg.linear.y = 0
					vel_msg.linear.z = 0
					vel_msg.angular.x = 0
					vel_msg.angular.y = 0
					vel_msg.angular.z = -0.1
					self.cmd_vel_pub.publish(vel_msg)

					print "turn right"

				elif line.angle < -0.05:
					vel_msg = Twist()
					vel_msg.linear.y = 0
					vel_msg.linear.z = 0
					vel_msg.angular.x = 0
					vel_msg.angular.y = 0
					vel_msg.angular.z = 0.1
					self.cmd_vel_pub.publish(vel_msg)

					print "turn left"

				#rospy.sleep(0.1)
				elif line.radius > 0.4:
					vel_msg = Twist()
					vel_msg.linear.x = 0.1
					vel_msg.linear.y = 0
					vel_msg.linear.z = 0
					vel_msg.angular.x = 0
					vel_msg.angular.y = 0
					vel_msg.angular.z = 0
					self.cmd_vel_pub.publish(vel_msg)

					print "go forward"





     #            while line.angle < 0.05:

					# print "need to turn left"
     #                    vel_msg = Twist()
     #                    vel_msg.linear.y = 0
     #                    vel_msg.linear.z = 0
     #                    vel_msg.angular.x = 0
     #                    vel_msg.angular.y = 0
     #                    vel_msg.angular.z = 0.1


                        #self.cmd_vel_pub.publish(vel_msg)



		print "acabou o ciclo forrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"
		#rospy.sleep(0.2)

def main():

    rospy.init_node('laser_brick_detection')
    n = line_node()
    rospy.spin()

if __name__ == '__main__':
    main()

