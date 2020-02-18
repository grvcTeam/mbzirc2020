#!/usr/bin/python
import rospy, tf, cv2
import numpy as np
from tf import transformations
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from mbzirc_comm_objs.msg import ObjectDetectionList, ObjectDetection

yellowLower = (0, 40, 150)
yellowUpper = (35, 255, 255)
purpleLower = (154, 40, 100)
purpleUpper = (174, 255, 255)

class frame_detector:
  def __init__(self):
    self.current_pose = PoseStamped()
    self.L_angle = [0.0,0.0,0.0,0.0] # TODO - To be calculated
    self.cov_x, self.cov_y, self.cov_z, self.cov_pitch, self.cov_roll, self.cov_yaw = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 # TODO - To pass as argument
    self.covariance = np.diag([self.cov_x, self.cov_y, self.cov_z, self.cov_pitch, self.cov_roll, self.cov_yaw]).flatten()

    # self.covariance[0], self.covariance[7] =self.cov_x, self.cov_y, self.cov_z, self.cov_qx, self.cov_qy, self.cov_qz, self.cov_qw]
    self.bridge = CvBridge()
    self.image = Image()

    self.image_sub = rospy.Subscriber("camera/color/image_rect_color",Image,self.callback_color, queue_size=1)
    self.depth_sub = rospy.Subscriber("ual/pose",PoseStamped,self.callback_ual, queue_size=1)
    self.camera_sub = rospy.Subscriber("camera/color/camera_info", CameraInfo, self.callback_camera, queue_size=1)
    self.point_pub = rospy.Publisher("frame_corner", PoseStamped, queue_size=1)
    self.sensed_pub = rospy.Publisher("sensed_objects", ObjectDetectionList, queue_size=1)

    self.uav_id = rospy.get_param("~uav_id")
    self.debug_view = rospy.get_param("~debug_view")
    self.debug_publisher = rospy.get_param("~debug_publisher")

    rospy.wait_for_message("camera/color/image_rect_color", Image)
    rospy.wait_for_message("ual/pose", PoseStamped) # wait to init depth var with uav pose

  def callback_camera(self,data):
    # set values for local variables
    self.frame = data.header.frame_id
    self.cam_height = data.height
    self.cam_width = data.width
    self.inv_fx = 1/data.K[0]
    self.inv_fy = 1/data.K[4]
    self.center_x = data.K[2]
    self.center_y = data.K[5]

  def callback_ual(self,ual_pose):
    self.current_pose = ual_pose

  def callback_color(self,data):
    try:
      self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

  def converte_xy(self, pixel_x, pixel_y):
    z = self.current_pose.pose.position.z
    rx =(pixel_x-self.center_x)*z*self.inv_fx #convertion from image plane to meters
    ry =(pixel_y-self.center_y)*z*self.inv_fy
    return rx, ry, z
  
  def closest_point(self, box, ctr):
    M = cv2.moments(ctr)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    closest = np.inf
    index = 0
    for i in range(4):
      dist = np.sqrt((cx-box[i][0])**2 + (cy-box[i][1])**2)
      if dist < closest:
        closest = dist
        index = i
    return box[index], index

  def create_point(self, x, y, z, frame, angle):
    point = PoseStamped()
    point.pose.position.x = x
    point.pose.position.y = y
    point.pose.position.z = z
    quaternion = transformations.quaternion_from_euler(-np.pi, 0, angle*(np.pi/180))
    point.pose.orientation.x = quaternion[0]
    point.pose.orientation.y = quaternion[1]
    point.pose.orientation.z = quaternion[2]
    point.pose.orientation.w = quaternion[3]
    point.header.stamp = rospy.Time(0)
    point.header.frame_id = frame
    return point

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

      if (cv2.contourArea(contorno) < 0.7*(width*height - black_pixels)): # TODO - Why 0.7
        if self.debug_view:
          print "its a corner"
        corner, idx = self.closest_point(box, contorno)
        cv2.circle(self.update_img, (corner[0], corner[1]), 10, (255, 0, 0), 2)
        ponto = self.converte_xy(corner[0],corner[1])

        # Fill ObjectDetection msg
        object_detected = ObjectDetection()

        object_detected.header.stamp = rospy.Time.now()
        object_detected.header.frame_id = self.frame

        point = self.create_point(ponto[0],ponto[1],ponto[2],self.frame, rect[2]+90*idx) # TODO - Remove function, do operation in line
        object_detected.pose.pose.position = point.pose.position
        object_detected.pose.pose.orientation.x = self.L_angle[0] # TODO - Calculate L angle - mult quaternions
        object_detected.pose.pose.orientation.y = self.L_angle[1] # TODO - Calculate L angle - mult quaternions
        object_detected.pose.pose.orientation.z = self.L_angle[2] # TODO - Calculate L angle - mult quaternions
        object_detected.pose.pose.orientation.w = self.L_angle[3] # TODO - Calculate L angle - mult quaternions
        object_detected.pose.covariance = self.covariance # TODO - Calculate cov in class init

        object_detected.scale.x = 4 # Size of L (m)
        object_detected.scale.y = 4 # Size of L (m)
        object_detected.scale.z = 0 # Size of L (m)

        object_detected.type = object_detected.TYPE_LWALL
        object_detected.color = object_detected.COLOR_UNKNOWN

        # Fill ObjectDetectionList msg 
        object_list_detected = ObjectDetectionList()

        object_list_detected.agent_id = str(self.uav_id)
        object_list_detected.stamp = rospy.Time.now()
        object_list_detected.objects = [object_detected]

        #self.point_pub.publish(self.create_point(ponto[0],ponto[1],ponto[2], self.frame, rect[2]+90*idx)) # TODO - Remove information related with old message
        self.sensed_pub.publish(object_list_detected) # TODO - Publish new message
      elif self.debug_view:
        print "no cornerino"

    if self.debug_view:
      cv2.imshow("image", self.update_img)
      cv2.imshow("warped", self.warped)
      cv2.waitKey(3)

def main():
  rospy.init_node('ugv_wall_detector')
  rate = rospy.Rate(rospy.get_param("~rate")) 
  ic = frame_detector()

  try:
    while not rospy.is_shutdown():
      ic.execute()
      rate.sleep() # TODO: Check strange behaviour when line is commented - warped image and false negative in detections
      # rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()
