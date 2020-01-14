
import sys
import rospy
import numpy as np
import cv2
import tf

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


#------------------------------WARNING-------------------
DEBUG = False
# prepare the code for circular grasping plate. not only rectangle
CTR_TRESHOLD = 2000
LOWER_DEPTH = 250
SQUARE_OFFSET = 35

#HSV values 
Lower = (90,  0 ,70)
Upper = (140 ,255, 255)

class plate_detector:

  def __init__(self):

    #rospy.init_node('plate_detector', anonymous=True)
    self.bridge = CvBridge()
    rospy.wait_for_message('/camera/color/camera_info', CameraInfo)
    self.list_tf = tf.TransformListener() 
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback_color, queue_size=1)
    self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",Image,self.callback_depth, queue_size=1)
    self.camera_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.callback_camera, queue_size=1)
    self.visual = Image()
    self.image = Image()
    self.depth = Image()
    rospy.wait_for_message("/camera/color/image_raw", Image)
    (trans, rot) = self.list_tf.lookupTransform('/base_link', self.frame, rospy.Time(0))
    self.upper_depth  = trans[2]*1000 - 80 #parameter to tune
    # self.upper_depth =  2000
    rospy.sleep(1)


  def callback_color(self,data):
    try:
      self.frame = data.header.frame_id
      self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
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

    mask_depth = cv2.inRange(self.depth, LOWER_DEPTH, self.upper_depth)
    # cv2.imshow("depth", mask_depth)
    # cv2.waitKey(3)
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
      rect = cv2.minAreaRect(ctr)
      box = cv2.boxPoints(rect)
      box = np.int0(box)
      x_center = (box[0,0]+box[2,0])/2
      y_center = (box[1,1]+box[3,1])/2
      # M = cv2.moments(ctr)
      # x_center = int(M["m10"] / M["m00"])
      # y_center = int(M["m01"] / M["m00"])
      cv2.drawContours(self.visual,[box],0,(0,0,255),2)
      # cv2.circle(self.visual, (x_center,y_center), 5, (0,0,0), thickness=-1, lineType=8, shift=0)
      #CENTER BY MOMENTS- WORKS IN ANY SURFACE AREA
      result = self.depth[y_center-SQUARE_OFFSET:y_center+SQUARE_OFFSET, x_center-SQUARE_OFFSET:x_center+SQUARE_OFFSET]
      cv2.rectangle(self.visual,(x_center-SQUARE_OFFSET,y_center-SQUARE_OFFSET),(x_center+SQUARE_OFFSET,y_center+SQUARE_OFFSET),(0,255,255),2)
      #mean, std = cv2.meanStdDev(result)
      mean = result[np.nonzero(result)].mean()
      if mean == []:
        return
      #print mean
      cv2.circle(self.visual, (x_center,y_center), 5, (0,0,0), thickness=-1, lineType=8, shift=0)
      real_center = PoseStamped()
      (real_center.pose.position.x, real_center.pose.position.y, real_center.pose.position.z) = self.point_converter(x_center, y_center, mean/1000)
      real_center.header.frame_id = self.frame
      real_center.header.stamp = rospy.Time.now()
      quaternion = tf.transformations.quaternion_from_euler(-np.pi/2, -np.pi, rect[2]*0.0175+np.pi) #angle to rad conversion
      #quaternion = tf.transformations.quaternion_from_euler(0, 0, rect[2]*0.0175) #angle to rad conversion
      real_center.pose.orientation.x = quaternion[0]
      real_center.pose.orientation.y = quaternion[1]
      real_center.pose.orientation.z = quaternion[2]
      real_center.pose.orientation.w = quaternion[3]
      self.list_tf.waitForTransform(self.frame, "/base_link", real_center.header.stamp,rospy.Duration(1))
      new_pose = self.list_tf.transformPose("/base_link",real_center)
      x_rot = 0.5
      y_rot = -0.5
      z_rot = 0.5
      w_rot = -0.5
      quaternion = Quaternion()
      x = 0
      y = 0
      z = new_pose.pose.orientation.z
      w = new_pose.pose.orientation.w
      quaternion.w = w*w_rot - x*x_rot - y*y_rot - z*z_rot
      quaternion.x = w*x_rot + x*w_rot + y*z_rot - z*y_rot
      quaternion.y = w*y_rot - x*z_rot + y*w_rot + z*x_rot
      quaternion.z = w*z_rot + x*y_rot - y*x_rot + z*w_rot
      new_pose.pose.orientation = quaternion
      if DEBUG:
        cv2.imshow("image", self.visual)
        cv2.waitKey(3)
      return new_pose
    else:
      return

  def point_converter(self,pixel_x, pixel_y, z = 0):
    if z == 0:
      z = float(self.depth[int(pixel_y),int(pixel_x)])/1000 #convert to meters
    rx =(pixel_x-self.center_x)*z*self.inv_fx #conversao de imagem para ponto real
    ry =(pixel_y-self.center_y)*z*self.inv_fy
    return (rx, ry, z)