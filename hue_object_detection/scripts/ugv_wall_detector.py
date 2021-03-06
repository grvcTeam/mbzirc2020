#!/usr/bin/python
import rospy, tf, cv2
import numpy as np

from math import sqrt
from tf import transformations
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Image, CameraInfo, Range
from geometry_msgs.msg import PoseStamped
from mbzirc_comm_objs.msg import ObjectDetectionList, ObjectDetection

from std_srvs.srv import SetBool, SetBoolResponse

yellowLower = (0, 40, 150)
yellowUpper = (35, 255, 255)
purpleLower = (154, 40, 100)
purpleUpper = (174, 255, 255)

enable_node = False

THRESHOLD = 0.7   # Threshold to detect the rectangle in image
DELTA = 0.1       # Distance (m) between realsense and SF11
MAX_HIGHT = 20.0    # Known max hight to avoid failed measures from sf11 

class frame_detector:
  def __init__(self):
    global enable_node

    self.bridge = CvBridge()
    self.image = Image()
    self.laser_measure = 0

    self.uav_id = rospy.get_param("~uav_id")
    enable_node = rospy.get_param("~enable_node")
    self.debug_view = rospy.get_param("~debug_view")
    self.debug_publisher = rospy.get_param("~debug_publisher")
    sigma_pos = [rospy.get_param("~sigma_x"), rospy.get_param("~sigma_y"), rospy.get_param("~sigma_z")]
    sigma_orientation = [rospy.get_param("~sigma_pitch"), rospy.get_param("~sigma_roll"), rospy.get_param("~sigma_yaw")]
    self.covariance_matrix = np.diag(np.square(sigma_pos + sigma_orientation)).flatten()

    self.image_sub = rospy.Subscriber("camera/color/image_rect_color", Image,self.callback_color, queue_size=1)
    self.laser_sub = rospy.Subscriber("sf11", Range, self.callback_laser, queue_size=1)
    self.camera_sub = rospy.Subscriber("camera/color/camera_info", CameraInfo, self.callback_camera, queue_size=1)
    self.sensed_pub = rospy.Publisher("sensed_objects", ObjectDetectionList, queue_size=1)
    if self.debug_publisher:
      self.debug_image_pub = rospy.Publisher("ugv_wall_detector/debug_image", Image, queue_size=1)

    self.enable_detection_srv = rospy.Service('ugv_wall_detector/enable', SetBool, self.enable_detection)

    rospy.wait_for_message("camera/color/image_rect_color", Image)
    rospy.wait_for_message("sf11", Range)

  def enable_detection(self, req):
    global enable_node
    enable_node = req.data
    res = SetBoolResponse()
    res.success = True
    if enable_node:
      res.message = "Node enabled"
    else:
      res.message = "Node disabled"
    return res

  def callback_camera(self,data):
    if enable_node:
      # set values for local variables
      self.frame = data.header.frame_id
      self.cam_height = data.height
      self.cam_width = data.width
      self.inv_fx = 1/data.K[0]
      self.inv_fy = 1/data.K[4]
      self.center_x = data.K[2]
      self.center_y = data.K[5]

  def callback_laser(self, data):
    if enable_node and data.range <= MAX_HIGHT:
      self.laser_measure = data.range - DELTA

  def callback_color(self,data):
    if enable_node:
      try:
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
        print(e)

  def converte_xy(self, pixel_x, pixel_y):
    z = self.laser_measure
    rx =(pixel_x-self.center_x)*z*self.inv_fx #convertion from image plane to meters
    ry =(pixel_y-self.center_y)*z*self.inv_fy
    return rx, ry, z
  
  def closest_point(self, box, ctr):
    M = cv2.moments(ctr)
    if M["m00"] != 0.0:
      cx = int(M["m10"] / M["m00"])
      cy = int(M["m01"] / M["m00"])

    else:
      return box[0], 0

    closest = np.inf
    index = 0
    for i in range(4):
      dist = np.sqrt((cx-box[i][0])**2 + (cy-box[i][1])**2)
      if dist < closest:
        closest = dist
        index = i
    return box[index], index

  def execute(self):
    self.update_img = self.image
    imhsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
    mask = (cv2.inRange(imhsv, yellowLower, yellowUpper)+cv2.inRange(imhsv, purpleLower, purpleUpper))
    _, ctr, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if ctr != []:
      contorno = max(ctr, key=cv2.contourArea)
      rect = cv2.minAreaRect(contorno)
      box = cv2.boxPoints(rect)
      box = np.int0(box)
      cv2.drawContours(self.update_img, [box], 0, (0,0,255), 2)

      # get width and height of the detected rectangle
      width = int(rect[1][0])
      height = int(rect[1][1])
      src_pts = box.astype("float32")
      # corrdinate of the points in box points after the rectangle has been
      # straightened
      dst_pts = np.array([[0, height-1],
                          [0, 0],
                          [width-1, 0],
                          [width-1, height-1]], dtype="float32")
      # the perspective transformation matrix
      M = cv2.getPerspectiveTransform(src_pts, dst_pts)
      # directly warp the rotated rectangle to get the straightened rectangle
      self.warped = cv2.warpPerspective(self.update_img, M, (width, height))
      #cv2.drawContours(self.update_img,contorno,-1,(255,0,255),2)
      gray = cv2.cvtColor(self.warped, cv2.COLOR_BGR2GRAY)
      black_pixels = width*height - cv2.countNonZero(gray)
      self.warped = mask

      if (cv2.contourArea(contorno) < THRESHOLD*(width*height - black_pixels)):
        corner, idx = self.closest_point(box, contorno)
        cv2.circle(self.update_img, (corner[0], corner[1]), 10, (255, 0, 0), 2)
        ponto = self.converte_xy(corner[0],corner[1])
        angle = rect[2]+90*idx

        # Fill ObjectDetection msg
        object_detected = ObjectDetection()

        object_detected.header.stamp = rospy.Time.now()
        object_detected.header.frame_id = self.frame

        object_detected.pose.pose.position.x = ponto[0]
        object_detected.pose.pose.position.y = ponto[1]
        object_detected.pose.pose.position.z = ponto[2]

        quaternion = transformations.quaternion_from_euler(-np.pi, 0, angle*(np.pi/180)) # TODO: Why -pi roll angle
        object_detected.pose.pose.orientation.x = quaternion[0]
        object_detected.pose.pose.orientation.y = quaternion[1]
        object_detected.pose.pose.orientation.z = quaternion[2]
        object_detected.pose.pose.orientation.w = quaternion[3]

        object_detected.pose.covariance = self.covariance_matrix

        # Scale = size of L (m)
        object_detected.scale.x = 4
        object_detected.scale.y = 4
        object_detected.scale.z = 0.1

        object_detected.type = object_detected.TYPE_LWALL
        object_detected.color = object_detected.COLOR_UNKNOWN

        # Fill ObjectDetectionList msg 
        object_list_detected = ObjectDetectionList()

        object_list_detected.agent_id = str(self.uav_id)
        object_list_detected.stamp = rospy.Time.now()
        object_list_detected.objects = [object_detected]

        self.sensed_pub.publish(object_list_detected)

        if self.debug_view:
          print "Corner found - Angle:"+str(angle)
        if self.debug_publisher:
          self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(self.update_img, "rgb8"))

      elif self.debug_view:
        print "Corner not found"

    if self.debug_view:
      cv2.imshow("image", self.update_img)
      cv2.imshow("warped", self.warped)
      cv2.waitKey(3)

def main():
  global enable_node

  rospy.init_node('ugv_wall_detector')
  rate = rospy.Rate(rospy.get_param("~rate")) 
  ic = frame_detector()

  try:
    while not rospy.is_shutdown():
      if enable_node:
        ic.execute()
      rate.sleep() # TODO: Check strange behaviour when line is commented - warped image and false negative in detections
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()
