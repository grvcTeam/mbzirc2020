#!/usr/bin/env python

import roslib
import rospy
import laser_line_extraction.msg
import numpy
import math
import geometry_msgs.msg

class Allign():

    def __init__(self):

        self.sub = rospy.Subscriber('/line_segments', laser_line_extraction.msg.LineSegmentList, self.line_segments_cb)

        self.cmd_vel_pub = rospy.Publisher("/rflex/atrvjr_node/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self.flag=0

        self.approach_wall = False
        self.counter = 0
        self.brick_to_be_placed = 0.3

        print "Allign init" 

    def line_segments_cb(self, msg):

		self.line_segment_list = msg

    def execute(self):

		self.line_segs = self.line_segment_list.line_segments

		if len(self.line_segs) == 0:

			print "line segments list is EMPTY"

			self.flag = 6

			print self.angle_goal

		lowest = 126782 #random big number
		line_populated = False

		vel_msg=geometry_msgs.msg.Twist()

		for line in self.line_segs:

			if line.radius<2:

				if line.radius < lowest:

					lowest = line.radius
					chosen_line = line
					line_populated = True

		print "lowest", lowest

		if line_populated:

			start = chosen_line.start   #start e o ponto mais a esquerda
			end = chosen_line.end     #end e o ponto mais a direita
			
			diff_x=abs(start[0]-end[0])
			diff_y=abs(start[1]-end[1])

			middle_point_x=(start[0]+end[0])/2
			middle_point_y=-(start[1]+end[1])/2 #aqui esta menos pq o frame do hokuyo esta invertido

			point_value_y = -(end[1] + self.brick_to_be_placed/2)  

			print "X", middle_point_x, "Y", middle_point_y, "point_value_y", point_value_y

			length = numpy.sqrt(diff_x**2 + diff_y**2)

			print "length", length
			print "line radius", chosen_line.radius
			print "line angle", chosen_line.angle

		print self.flag
		print chosen_line


		if self.flag == 0:

			if chosen_line.angle < - 0.02:

				vel_msg.linear.x = 0
				vel_msg.angular.z = 0.15

			if chosen_line.angle > 0.02:

				vel_msg.linear.x = 0
				vel_msg.angular.z = -0.15

			if chosen_line.angle > -0.02 and chosen_line.angle<0.02:

				vel_msg.linear.x = 0
				vel_msg.angular.z = 0
				self.flag= 1

			self.cmd_vel_pub.publish(vel_msg)

		if self.flag == 1:

			diff_angle = math.atan2(middle_point_y,middle_point_x-0.2)

			# diff_angle = math.atan2(point_value_y,middle_point_x-0.2)    #0.2 para ficar 20cm

			# hipotenusa=sqrt(point_30_y**,(middle_point_x-0.2)**)

			self.angle_goal = diff_angle
			print "angle goal", self.angle_goal

			rospy.sleep(1)
			self.flag=2
			

		if self.flag == 2:

			if chosen_line.angle > self.angle_goal:

				vel_msg.linear.x = 0
				vel_msg.angular.z = -0.15

			if chosen_line.angle < self.angle_goal:

				vel_msg.linear.x = 0
				vel_msg.angular.z = 0.15

			if abs(chosen_line.angle)>abs(self.angle_goal)-0.03: #este 0.03 depende da vel.angular

				vel_msg.linear.x = 0
				vel_msg.angular.z = 0
				self.flag=3

			self.cmd_vel_pub.publish(vel_msg)

		if self.flag == 3:

			vel_msg.linear.x = 0.2

			# if abs(middle_point_y)-0.15 <= middle_point_x <= abs(middle_point_y)+0.05:   #CHANGE THIS

			if chosen_line.radius < 0.4:
				vel_msg.linear.x = 0
				self.flag = 4

			self.cmd_vel_pub.publish(vel_msg)

		if self.flag == 4:

			if chosen_line.angle < - 0.02:

				vel_msg.linear.x = 0
				vel_msg.angular.z = 0.15

			if chosen_line.angle > 0.02:

				vel_msg.linear.x = 0
				vel_msg.angular.z = -0.15

			if chosen_line.angle > -0.02 and chosen_line.angle<0.02:

				vel_msg.linear.x = 0
				vel_msg.angular.z = 0
				self.flag= 5

			self.cmd_vel_pub.publish(vel_msg)

		if self.flag == 6 and self.approach_wall:

			self.counter=self.counter+1

			vel_msg.linear.x = 0.2
			self.cmd_vel_pub.publish(vel_msg)

			print "Counter", self.counter
			print "Angle goal", self.angle_goal

			if self.counter > abs(self.angle_goal) * 18*self.brick_to_be_placed:
				
				vel_msg.linear.x = 0
				self.cmd_vel_pub.publish(vel_msg)

		print "acabou o ciclo forrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"

# def main():

#     rospy.init_node('laser_brick_detection')
#     n = Allign()
#     rospy.spin()

# if __name__ == '__main__':
#     main()

