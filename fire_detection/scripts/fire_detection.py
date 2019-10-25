#!/usr/bin/env python
import rospy
import sys
import cv2
from primer_tutorial.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Routine to read data from teraranger topics 
def read_camera_topics():    
    rospy.init_node('Thermal_cam', anonymous=True)
    rospy.Subscriber('teraranger_evo_thermal/rgb_image',Image,image_operations)
    rospy.spin()

def image_operations(data):

    # Declaration needed in order to change Image format to a readable format in OpenCV
    bridge = CvBridge()
    background = 0
    flag = 0

    # Threeshold filter, 255 = white
    white_thres=205
    try:
        cv_image = bridge.imgmsg_to_cv2(data,'8UC3')
        gray_im = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY);    
        gray_im = cv2.GaussianBlur(gray_im, (21, 21), 0)
        if background is None or flag == 100:
            flag = 0
    	    background = gray_im
        flag += 1
        dif = cv2.absdiff(gray_im,background)

        # Apply a threshold so it may be possible to vary intensity captures
        thres = cv2.threshold(dif, white_thres, 255, cv2.THRESH_BINARY)[1]
        outlineimg = thres.copy()
        _,outline, hierarchy = cv2.findContours(outlineimg, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        # Mesh to create a rectangle on the image
        for c in outline:
    	    (x, y, w, h) = cv2.boundingRect(c)
    	    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    	    cv2.rectangle(gray_im, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        #Show camera livestream in terminal
        cv2.imshow("Teraranger", cv_image)
        cv2.moveWindow("Teraranger",0,0)
        cv2.imshow("BW_Teraranger", gray_im)
        cv2.moveWindow("BW_Teraranger",565, 0)
        cv2.imshow("Outline", outlineimg)
        cv2.moveWindow("Outline", 1120, 0)
        cv2.waitKey(3)
        
        #Convert the previous operations to a format that ROS can receive as a topic
        cv_image = bridge.cv2_to_imgmsg(cv_image,'8UC3')
        gray_im = bridge.cv2_to_imgmsg(gray_im,'8UC1')

        #Publish new topics 
        pub_1 = rospy.Publisher('mbzirc2020_1/teraranger_cam_thermal',Image, queue_size=2)
        pub_2 = rospy.Publisher('mbzirc2020_1/teraranger_cam_thermal_gray',Image, queue_size=2)
        pub_1.publish(cv_image)
        pub_2.publish(gray_im)
    except CvBridgeError as e:
      print(e)

if __name__ == '__main__':
 
    read_camera_topics()