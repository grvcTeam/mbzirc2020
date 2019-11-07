#!/usr/bin/env python

import roslib
import rospy
import laser_line_extraction.msg
import numpy
import math

class line_node():

    def __init__(self):

       # self.pub = rospy.Publisher('attached', GripperAttached, queue_size=1)
        self.sub = rospy.Subscriber('/line_segments', laser_line_extraction.msg.LineSegmentList, self.line_segments_cb)

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

	print "acabou o ciclo forrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"
	#rospy.sleep(0.2)

def main():

    rospy.init_node('laser_brick_detection')
    n = line_node()
    rospy.spin()

if __name__ == '__main__':
    main()

