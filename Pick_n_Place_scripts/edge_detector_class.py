import sys
import rospy
import numpy as np
import cv2
import tf
import math

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

CTR_TRESHOLD = 1000
DEBUG = True
#HSV values ToDo: put in a yaml file
redLower = (0, 50, 0)
redUpper = (12, 255, 255)
redLower2 = (170, 50, 0)
redUpper2 =(180, 255, 255)
blueLower = (95, 180, 0)
blueUpper = (118, 255, 255)
orangeLower = (0, 180, 140)
orangeUpper = (20, 255, 235)
greenLower = (30, 50, 50)
greenUpper = (70, 255, 255)


class edge_detector:

  def __init__(self, side, color, level):


    self.side = side
    self.color = color
    self.level = level
    self.bridge = CvBridge()
    rospy.wait_for_message('/camera/color/camera_info', CameraInfo)
    self.image_sub = rospy.Subscriber("/camera/color/image_rect_color",Image,self.callback_color, queue_size=1)
    self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",Image,self.callback_depth, queue_size=1)
    self.camera_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.callback_camera, queue_size=1)
    self.point_pub = rospy.Publisher("/edge_point", PointStamped, queue_size=1)
    self.t = tf.TransformListener() 
    self.image = Image()
    self.depth_image = Image()
    #self.colors = ["Red","Green","Blue","Orange"]
    rospy.wait_for_message("/camera/color/image_rect_color", Image)
    self.update_img = self.image

    rospy.sleep(1)

  def callback_camera(self,data):
    # set values for local variables
    self.frame = data.header.frame_id
    self.cam_height = data.height
    self.cam_width = data.width
    self.inv_fx = 1/data.K[0]
    self.inv_fy = 1/data.K[4]
    self.center_x = data.K[2]
    self.center_y = data.K[5]

  def callback_depth(self,depth_data):
    try:
      self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, "passthrough")
    except CvBridgeError as e:
      print(e)

  def callback_color(self,data):
    try:
      self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.frame = data.header.frame_id
    except CvBridgeError as e:
      print(e)
    
  def check_color(self, color):
    imhsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
    if (color == "red"):
      mask = cv2.inRange(imhsv, redLower, redUpper)+cv2.inRange(imhsv, redLower2, redUpper2)
    elif (color == "green"):
      mask = cv2.inRange(imhsv, greenLower, greenUpper)
    elif (color == "blue"):
      mask = cv2.inRange(imhsv, blueLower, blueUpper)
    elif (color == "orange"):
      mask = cv2.inRange(imhsv, orangeLower, orangeUpper)
    else:
      return []
    return mask

  def depth_filter(self):
    (trans, rot) = self.t.lookupTransform('/base_link', self.frame, rospy.Time(0))
    camera_height = trans[2]
    lower_bound = max((camera_height - (self.level*0.2) - 0.1)*1000,50) #make sure the value is above zero
    upper_bound = (camera_height - (self.level*0.2) + 0.1)*1000
    crop_img = cv2.inRange(self.depth_image, lower_bound, upper_bound)
    return crop_img


  def create_point(self, x, y, z, frame):
    point = PointStamped()
    point.point.x = x
    point.point.y = y
    point.point.z = z
    point.header.stamp = rospy.Time(0)
    point.header.frame_id = frame
    return point

  def get_side(self, x, y, z, h):
    if self.side == "left":
      crop_img = self.depth_image[y+40:y+h-40, x+15:x+30]
      cv2.rectangle(self.update_img,(x+15,y+40),(x+30,y+h-40),(0,255,255),2)
      mean = crop_img[np.nonzero(crop_img)].mean()
      #mean, std = cv2.meanStdDev(crop_img)
      lado_x, lado_y = self.converte_xy(x, y+h/2, mean/1000)
    if self.side == "right":
      crop_img = self.depth_image[y+40:y+h-40, x+z-30:x+z-15]
      cv2.rectangle(self.update_img,(x+z-30,y+40),(x+z-15,y+h-40),(255,255,255),2)
      mean = crop_img[np.nonzero(crop_img)].mean()
      #mean, std = cv2.meanStdDev(crop_img)
      lado_x, lado_y = self.converte_xy(x+z, y+h/2, mean/1000)
    return self.create_point(lado_x, lado_y, mean/1000, self.frame)

  def execute(self):
    self.update_img = self.image
    mask = self.check_color(self.color)
    if mask == []:
      return
  #testing new function
    mask = cv2.bitwise_and(mask,mask,mask = self.depth_filter())
    if mask == []:
      return
    area = cv2.erode(mask, None, iterations=2)
    area = cv2.dilate(mask, None, iterations=2)
    _, ctr, hierarchy = cv2.findContours(area, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if ctr == []:
      return

    for item in ctr:
      if cv2.contourArea(item) < CTR_TRESHOLD:
        pass
      else:
        M = cv2.moments(item)
        x_center = int(M["m10"] / M["m00"])
        y_center = int(M["m01"] / M["m00"])
        cv2.circle(self.update_img, (x_center,y_center), 5, (0,0,0), thickness=-1, lineType=8, shift=0)
        cv2.putText(self.update_img, self.color, (x_center +5,y_center+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 0)
        x,y,z,h = cv2.boundingRect(item)
        cv2.rectangle(self.update_img,(x,y),(x+z,y+h),(0,0,0),4)
        lado = self.get_side(x, y, z, h)
        self.point_pub.publish(lado)
        cv2.drawContours(self.update_img, item, -1, (0,255,0), 2)
        if DEBUG:
          cv2.imshow("image", self.update_img)
          cv2.waitKey(3)
        return lado
    return

  def converte_xy(self, pixel_x, pixel_y, z = 0):
    # pixel_x = max(0, pixel_x)
    # pixel_x = min(639, pixel_x)
    # pixel_y = max(0, pixel_y)
    # pixel_y = min(479, pixel_y)
    if z == 0:
      z = float(self.depth_image[int(pixel_y),int(pixel_x)])/1000
    rx =(pixel_x-self.center_x)*z*self.inv_fx #conversao de imagem para ponto real
    ry =(pixel_y-self.center_y)*z*self.inv_fy
    return rx, ry
