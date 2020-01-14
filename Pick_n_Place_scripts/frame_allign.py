
import sys
import rospy
import numpy as np
import cv2
import tf
import geometry_msgs.msg
import math 

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


#------------------------------WARNING-------------------
DEBUG = False
# prepare the code for circular grasping plate. not only rectangle

#SET FRAME COLORS
Lower = (90,  0 ,70)
Upper = (140 ,255, 255)

class FrameAllign:

  def __init__(self):

    #rospy.init_node('plate_detector', anonymous=True)
    self.bridge = CvBridge()
    rospy.wait_for_message('/camera/color/camera_info', CameraInfo)
    self.list_tf = tf.TransformListener() 
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback_color, queue_size=1)
    self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",Image,self.callback_depth, queue_size=1)
    self.camera_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.callback_camera, queue_size=1)
    self.cmd_vel_pub = rospy.Publisher("/mbzirc2020_0/base_controller/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
    self.visual = Image()
    self.image = Image()
    self.depth = Image()
    rospy.wait_for_message("/camera/color/image_raw", Image)
    (trans, rot) = self.list_tf.lookupTransform('/base_link', self.frame, rospy.Time(0))
    self.upper_depth  = trans[2]*1000 + 20 #parameter to tune
    self.lower_depth = trans[2]*1000 - 20
    # self.upper_depth =  2000
    rospy.sleep(1)


  def callback_color(self,data):
    try:
      self.frame = data.header.frame_id
      self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.execute()
    except CvBridgeError as e:
      print(e)
    
  def callback_depth(self,data):
    try:
      self.depth = self.bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
      print(e)
  
  def callback_camera(self,data):
    # set values for local variables
    self.frame = data.header.frame_id
    self.cam_height = data.height
    self.cam_width = data.width
    self.inv_fx = 1/data.K[0]
    self.inv_fy = 1/data.K[4]
    self.center_x = data.K[2]
    self.center_y = data.K[5]

  def execute(self):

    self.visual = self.image
    mask_depth = cv2.inRange(self.depth, self.lower_depth, self.upper_depth)
    new_image = cv2.bitwise_and(self.image,self.image,mask = mask_depth)
    imhsv = cv2.cvtColor(new_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(imhsv, Lower, Upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    _, ctr, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if ctr != []:
      ctr = max(ctr, key=cv2.contourArea)
      cv2.drawContours(self.visual, ctr, -1, (0,255,0), 3)
      #RECTANGLE FIT TO IMAGE
      self.rect = cv2.minAreaRect(ctr)
      box = cv2.boxPoints(rect)
      box = np.int0(box)
      cv2.drawContours(self.visual,[box],0,(0,0,255),2)
      if DEBUG:
        cv2.imshow("image", self.visual)
        cv2.waitKey(3)
      return "success"
    else:
      return

  def execute_loop(self):
    vel_msg=geometry_msgs.msg.Twist()
    while abs(self.rect[2]) > 0.02:
      if self.rect[2] < - 0.02:
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0.15
      if self.rect[2] > 0.02:
        vel_msg.linear.x = 0
        vel_msg.angular.z = -0.15
      self.cmd_vel_pub.publish(vel_msg)
      rospy.sleep(0.05)
    return "success"
