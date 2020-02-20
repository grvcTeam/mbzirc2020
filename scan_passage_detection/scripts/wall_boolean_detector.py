#!/usr/bin/python
import rospy
from threading import Thread
import numpy as np

from mbzirc_comm_objs.msg import WallList, Wall

class WallBooleanDetector(Thread):
  def __init__(self):
    Thread.__init__(self)
    self.enable = True
    self.walls_sub = rospy.Subscriber("/mbzirc2020_6/walls", WallList, self.callback_walls, queue_size=1)
    rospy.wait_for_message("/mbzirc2020_6/walls", WallList)
    self.passage = False
    
  def run(self):
    rospy.spin()

  def is_passage(self):
    return self.passage

  def enable(self, enable):
    self.enable = enable
  
  def squared_length(self, wall):
    dx = wall.end[0]-wall.start[0]
    dy = wall.end[1]-wall.start[1]
    # print str(dx)+" "+str(dy)
    # print str(dx*dx)+" "+str(dy*dy)
    return (dx*dx + dy*dy)

  def callback_walls(self, data):
    if self.enable :
      maybe_passage = 0
      if len(data.walls) == 4:
        for i in data.walls :
          if self.squared_length(i) > 12.0:
            maybe_passage = maybe_passage + 1
          # print str(self.squared_length(i))
        if maybe_passage == 4:
          self.passage = True
        else :
          self.passage = False
      else :
        self.passage = False
      print "Detection result: "+str(self.is_passage())
      print "---------------------------"

def main():
  rospy.init_node('wall_boolean_detector')
  wallBooleanDetector = WallBooleanDetector()
  wallBooleanDetector.start()
  try:
    while not rospy.is_shutdown():
      rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass

  
