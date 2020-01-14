#!/usr/bin/env python
import rospy
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mbzirc_comm_objs.msg import ObjectDetection
from mbzirc_comm_objs.msg import ObjectDetectionList
from geometry_msgs.msg import PoseStamped

# Routine to read data from teraranger topics and ual pose
def read_camera_topics():    
    rospy.init_node('Thermal_cam', anonymous=True)
    rospy.Subscriber('teraranger_evo_thermal/raw_temp_array',Float64MultiArray,thermal_data)
    rospy.Subscriber('teraranger_evo_thermal/rgb_image',Image,image_operations)
    rospy.Subscriber('/ual/pose',PoseStamped,ual_to_fire_position)
    rospy.spin()

# Routine to obtain data pose to create fire messages
def ual_to_fire_position(data):
    # Position in x,y,z
    global complete_position, x_pose, y_pose
    complete_position=data.pose
    x_pose=data.pose.position.x
    y_pose=data.pose.position.y

# Routine to find fire in the image
def thermal_data(data):
    global maxim, minim, fila, col, temp_matrix
    maxim=0
    minim=1000
    #Obtaining temperature array
    for i in range(len(data.data)):
        if data.data[i]>maxim:
            fila=int(i/32)
            col=i-fila*32
            vec=data.data
            temp_matrix=np.reshape(vec,(32,32))            
            maxim=data.data[i]
        if data.data[i]<minim:
            minim=data.data[i]

# Routine to process the image and determine if there is fire and where
def image_operations(data):
    global detection_sampling, maxim, minim, fila, col, temp_matrix, last_detection, complete_position, o_c, fire_list_x, fire_list_y, x_pose, y_pose
    background = 0
    flag = 0    
    white_thres=205   # Threeshold filter, 255 = white
    thermal_threshold=35  #Thermal threshold to detect fire
    detection = 0
    a=32
    i=0
    j=0
    k = 0
    xsize=515
    ysize=550
    fire_list=[0,0,0,0,0,0,0,0]
    fire_list_x=[0,0,0,0,0,0,0,0]
    fire_list_y=[0,0,0,0,0,0,0,0]
    count = 0
    max_count=0
    Color_green= np.array([0,255,0])
    # Conversion 1 pixel = 0.01 cm = 0.0001 m
    pixel_to_m=0.0001


    # Declaration needed in order to change Image format to a readable format in OpenCV
    bridge = CvBridge()
    # Creation rec_list in appropiate format
    rec_list=ObjectDetectionList()
    rec_object = ObjectDetection()
    rec_object.type = 3
    try:
        cv_image = bridge.imgmsg_to_cv2(data,'8UC3')
        gray_im = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY);    
        gray_im = cv2.GaussianBlur(gray_im, (21, 21), 0)
    
        if background is None or flag == 100:
            flag = 0
	    background = gray_im
        flag += 1
        dif = cv2.absdiff(gray_im,background)

        ## Uncomment next lines to obtain a temperature txt based on the thermal
        # f = open( 'Matriz_temperaturas.txt', 'w' )
        # for i in range (a):
        #     for j in range (a):
        #         f.write( str(int(temp_matrix[i,j])) + ' ')
        #     f.write ('\n')
        # f.close()

        # Definition of the black and white matrix
        for i in range (a):
            for j in range (a):
                if temp_matrix[i,j]>thermal_threshold:
                    temp_matrix[i,j]=255
                else:
                    temp_matrix[i,j]=0
           
        # Conversion from matrix to uint8 image
        temp_matrix=np.uint8(temp_matrix)
        temp_matrix=cv2.resize(temp_matrix,(xsize,ysize))
        # Obtain the number of fires in the image
        outlineimg = temp_matrix.copy()
        _,outline, hierarchy = cv2.findContours(outlineimg, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
  
        # Draw zones and contours for the fire zone
        if maxim>thermal_threshold:
            detection = 1   
            for c in outline:
                if max_count<(count+1):
                    max_count=count+1
                count = count + 1

                M = cv2.moments(c)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                o_c = (int(cx), int(cy))

                fire_list_x[count-1]= cx
                fire_list_y[count-1]= cy
                fire_list[count-1]=cx,cy
                # print(fire_list)

                # Create rectangles and draw the fire zones in the image     
                (x, y, w, h) = cv2.boundingRect(c)
                # Draw minimum fire area in the camera feed
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(cv_image,[box],0,(0,255,0),2)
                # Draw minimum fire area in black and white filter
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(temp_matrix,[box],0,(255,255,255),2)
                # Draw centroid on the fire
                cv2.circle(temp_matrix, o_c , 5, Color_green, -1)
                cv2.circle(cv_image, o_c , 5, Color_green, -1)

            count = 0
               
        #Show camera livestream in terminal
        cv_image= cv2.flip(cv_image,0)
        cv_image=cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow("Teraranger", cv_image)
        cv2.moveWindow("Teraranger",0,0)
        temp_matrix= cv2.flip(temp_matrix,0)
        temp_matrix=cv2.rotate(temp_matrix, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow("Outline", temp_matrix)
        cv2.moveWindow("Outline",565, 0)
        cv2.waitKey(3)
        
        #Convert the previous operations to a format that ROS can receive as a topic
        cv_image = bridge.cv2_to_imgmsg(cv_image,'8UC3')
        # gray_im = bridge.cv2_to_imgmsg(gray_im,'8UC1')

        #Publish new topics 
        pub_1 = rospy.Publisher('mbzirc2020_1/teraranger_cam_thermal',Image, queue_size=2)
        # pub_2 = rospy.Publisher('mbzirc2020_1/teraranger_cam_thermal_gray',Image, queue_size=2)
        pub_1.publish(cv_image)
        # pub_2.publish(gray_im)
        
        #Counter to print fire every 10 frames
        detection_sampling=detection_sampling+1
        # print(detection_sampling)

        if detection and detection_sampling>=10:
            # print("Fire detected!")
            detection_sampling = 0
            last_detection=last_detection+1
            # rec_object.type = mbzirc_comm_objs::ObjectDetection::TYPE_FIRE
            # this->rec_list.objects.push_back(rec_object);
            # this->pub_.publish(this->rec_list);
            # TO DO: get fire pose. Not Add every pixel as fire object
            while k<max_count:
                pub = rospy.Publisher('Fire_detect',ObjectDetectionList,queue_size=2)
                rec_object = ObjectDetection()
                # Create a fire type message
                rec_object.type = 3
                # Calculate distance of the fire toward image center
                image_center_x=ysize/2
                image_center_y=xsize/2
                relative_pose_x=x_pose + ((ysize - fire_list_y[k]) - image_center_x) * pixel_to_m
                relative_pose_y=y_pose + ((xsize - fire_list_x[k]) - image_center_y) * pixel_to_m
                # Write relative positions on the message
                rec_object.relative_position.x = relative_pose_x 
                rec_object.relative_position.y = relative_pose_y 
                rec_object.pose.pose= complete_position
                rec_list.objects.append(rec_object) # It should add it only if it's not the same object
                pub.publish(rec_list)
                k=k+1
            last_detection = detection
        if detection==0 and last_detection>=1:
            last_detection=0
            # print("Waiting...")
        last_detection=detection

    except CvBridgeError as e:
      print(e)

if __name__ == '__main__':
    global last_detection, detection_sampling
    last_detection=0
    detection_sampling = 0
    read_camera_topics()
