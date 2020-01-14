import sys
import rospy
import numpy as np
import cv2
import tf
import math


class Wall_dispatcher:

  def __init__(self):
#     30.    30.    120.    30.    60.    60.    30.  
#     30.    60.    30.    30.    60.    120.    30.  
#     30.    30.    30.    60.    30.    60.    120.  
#     30.    30.    120.    30.    30.    60.    60.  
#     60.    30.    30.    60.    30.    120.    30.  


    self.wall = [["red", "red", "blue", "red", "green", "green", "red"], #level5 highest
                 ["red", "green", "red", "red", "green", "blue", "red"], #level4
                 ["red", "red", "red", "green", "red", "green", "blue"], #level3
                 ["red", "red", "blue", "red", "red", "green", "green"], #level2
                 ["green", "red", "red", "green", "red", "blue", "green"]] #level1 ground level
    self.colors = ["Red","Green","Blue","green", "red", "blue"]
    self.color_dict = {"Red" : 0.3,
                       "Green" : 0.6,
                       "Blue" : 1.2,
                       "green" : 0.6,
                       "red" : 0.3,
                       "blue" : 1.2
                      }
    self.wall_check()

    self.side = "right" #set this to fixed value

  def wall_check(self):
    for x in self.wall:
      for y in x:
        if y not in self.colors:
          print "wrong wall format"
          exit()

  def opposite_side(self, side):
    if side == "left":
      return "right"
    if side == "right":
      return "left"

  def dispatcher_execute(self):
    #CODIGO PARA POR O PRIMEIRO BRICK talvez criar excepcao do place_algorithm
    lenght = self.color_dict[self.wall[-1][0]]
    reference_brick = self.wall[-1][0]
    first = True
    level = 1
    over = False
    side = self.side
    for x in reversed(self.wall):
      for y in x:
        if first:
          first = False
          continue
        #CALL PICK PLACE ALGORITHM pick_place(self.side, reference_brick, level, over, y)
        print "side: ",side, "|reference: ",reference_brick,"|level: ", level,"|over: ", over,"|bricktoput: ", y,"|lenght: ", lenght
        c = sys.stdin.read(1)
        if c == "q":
          exit()
        lenght += self.color_dict[y]
        reference_brick = y #previous is reference for next
        if over:
          level += 1 
          side = self.side
          over = False
      over = True
      side = self.opposite_side(side)
      reference_brick = x[0] #first brick of the previous level is the reference for the next level
      lenght = 0

def main(args):

  classe = Wall_dispatcher()
  classe.dispatcher_execute()

  
if __name__ == '__main__':
  main(sys.argv)