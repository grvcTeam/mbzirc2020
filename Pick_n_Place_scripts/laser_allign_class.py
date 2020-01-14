#!/usr/bin/env python

import roslib
import rospy
import laser_line_extraction.msg
import numpy
import math
import geometry_msgs.msg

color_dict = {"Red" : 0.3,
              "Green" : 0.6,
              "Blue" : 1.2,
              "green" : 0.6,
              "red" : 0.3,
              "blue" : 1.2
              }
class Allign():

  def __init__(self, reference_side, approach_wall, distance_to_place, brick_to_be_placed):

      self.sub = rospy.Subscriber('/line_segments', laser_line_extraction.msg.LineSegmentList, self.line_segments_cb)
      self.cmd_vel_pub = rospy.Publisher("/mbzirc2020_0/base_controller/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
      self.goal_pose_pub = rospy.Publisher("/mbzirc2020_0/laser_brick_detection/goal_pose", geometry_msgs.msg.PoseStamped, queue_size=1)

      self.approach_wall = approach_wall
      if self.approach_wall:
        self.flag=0
      else:
      	self.flag=1

      self.brick_to_be_placed = color_dict[brick_to_be_placed] 
      self.reference_side = reference_side
      self.distance_to_place = distance_to_place
      self.counter = 0
      self.condition_stop = 25
      self.execute = False
      self.finish = False

  def execute(self):
    self.execute = True
    while self.finish == False:
      pass
    self.execute = False
    self.finish = False
    if self.flag == 5:
      return 'success', self.line_segs[0].radius + 0.37
    else 
      return 

  def line_segments_cb(self, msg):
    
    if self.execute:

      self.line_segment_list = msg
      self.line_segs = self.line_segment_list.line_segments

      vel_msg=geometry_msgs.msg.Twist()

      if len(self.line_segs) == 0 or self.line_segs[0].radius <0.25 and self.approach_wall:
        print "line segments list is EMPTY or line is super close"
        if self.flag > 2:
          self.flag = 7
        else:
          self.flag = 6
        print self.angle_goal

      else:
        lowest = 999 #random big number
        line_populated = False
          
        for line in self.line_segs:

          if line.radius<3:
            start = line.start   #start e o ponto mais a esquerda
            end = line.end     #end e o ponto mais a direita

            diff_x=abs(start[0]-end[0])
            diff_y=abs(start[1]-end[1])

            middle_point_x=(start[0]+end[0])/2
            middle_point_y=(start[1]+end[1])/2   #aqui estava menos pq o frame do hokuyo esta invertido

            distance_middle_point = numpy.sqrt(middle_point_x**2 + middle_point_y**2)
            print "ANGULO", line.angle

            try:
              if distance_middle_point < lowest and self.angle_commited-0.05 <= abs(line.angle) <= self.angle_commited+0.05:
                print "COMMITED", self.angle_commited
                lowest = distance_middle_point
                chosen_line = line
                line_populated = True

            except:
              print "foi para o except pq nao tem angle_commited"
              if distance_middle_point < lowest:
                lowest = distance_middle_point
                chosen_line = line
                line_populated = True

        print "lowest", lowest

        if line_populated:

          start = chosen_line.start   #start e o ponto mais a esquerda
          end = chosen_line.end     #end e o ponto mais a direita

          diff_x=abs(start[0]-end[0])
          diff_y=abs(start[1]-end[1])
          length = numpy.sqrt(diff_x**2 + diff_y**2)

          middle_point_x=(start[0]+end[0])/2
          middle_point_y=(start[1]+end[1])/2  #aqui esta menos pq o frame do hokuyo esta invertido

          print "X", middle_point_x, "Y", middle_point_y
          print "length", length
          print "line radius", chosen_line.radius
          print "line angle", chosen_line.angle

        else:
          print "line not populated"
          if self.flag > 2:
            self.flag = 7
          else:
            self.flag = 6

      print "FLAG", self.flag

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

        vector_y=middle_point_x - start[0]
        vector_x=middle_point_y - start[1]
        if vector_x > 0:
          vector_x = -vector_x
        print middle_point_x + vector_x/2
        print middle_point_y + vector_y/2

        if self.reference_side == "left":
          side = start[1]
        else: 
          side = end[1]

        point_value_y =-(side + self.distance_to_place + self.brick_to_be_placed/2) #manter approach anterior para a wall

        if self.approach_wall:
          diff_angle = math.atan2(point_value_y, middle_point_x - 0.25)
          self.angle_goal = diff_angle
        else:
          diff_angle = math.atan2(middle_point_y + vector_y*(0.3/(length/2)), middle_point_x + vector_x*(0.3/(length/2)))
          self.angle_goal = - diff_angle + chosen_line.angle

        print "angle goal", self.angle_goal
        self.flag=2
        rospy.sleep(1)

      if self.flag == 2:

        if chosen_line.angle > self.angle_goal:
          vel_msg.linear.x = 0
          vel_msg.angular.z = -0.15
          self.angle_commited= abs(chosen_line.angle)
        if chosen_line.angle < self.angle_goal:
          vel_msg.linear.x = 0
          vel_msg.angular.z = 0.15
          self.angle_commited= abs(chosen_line.angle)
        if chosen_line.angle>self.angle_goal-0.03 and chosen_line.angle<self.angle_goal+0.03: #este 0.03 depende da vel.angular
          vel_msg.linear.x = 0
          vel_msg.angular.z = 0
          self.flag=3
          self.angle_commited= abs(chosen_line.angle)
        self.cmd_vel_pub.publish(vel_msg)

      if self.flag == 3:

        vel_msg.linear.x = 0.2
        if self.approach_wall:
          stop = 0.3
        else:
          stop = 0.25

        if chosen_line.radius < (stop + abs(chosen_line.angle)/12):
          vel_msg.linear.x = 0
          self.flag = 4
          self.angle_commited= None
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

      if self.flag == 7: # and self.approach_wall:

        self.counter=self.counter+1
        vel_msg.linear.x = 0.2
        self.cmd_vel_pub.publish(vel_msg)
        print "Counter", self.counter
        print "Angle goal", self.angle_goal

        if self.counter > abs(self.angle_goal) * self.condition_stop * abs(self.brick_to_be_placed):
        #quanto mais pequeno o angle_goal mais cedo deve parar
          try:
            if self.line_segs[0].radius< 0.3:
              self.angle_goal = self.line_segs[0].angle
          except:
            pass

        if self.angle_goal < - 0.02:
          vel_msg.linear.x = 0
          vel_msg.angular.z = 0.15
        if self.angle_goal > 0.02:
          vel_msg.linear.x = 0
          vel_msg.angular.z = -0.15
        if self.angle_goal > -0.02 and self.angle_goal<0.02:
          vel_msg.linear.x = 0
          vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(vel_msg)
    
      if self.flag == 6:
        print "no line detected"
        self.finish = True 

      if self.flag == 5:
        self.finish = True

      print "-------------------------------------------------------------------------------------"

def main():

    rospy.init_node('laser_brick_detection')
    n = line_node("left", False, 1.9, 0.6)
    rospy.spin()

if __name__ == '__main__':
    main()