# drone_detect_tracking

ROS Node for detecting and tracking foreign drones implementing Matouš Vrba and Martin Saska technique described in http://mrs.felk.cvut.cz/iros2019-mav-detection


1. [ Setting up. ](#setup)
2. [ Usage. ](#usage)
3. [ Nodes. ](#nodes)


# 1. Setting up

Clone this repo into your catkin_ws, make sure you satisfy all the dependencies and run catkin_make.

## Dependencies 

* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) 
* [Eigen_Conversions](http://wiki.ros.org/eigen_conversions)


      sudo apt-get install ros-kinetic-eigen-conversions




# 2. Usage

It's possible to run the detector node with the required static transform publisher and also the auxiliar dji visualization node via:

      roslaunch drone_detector_tracking drone_detection.launch
   

or you can just run the detector node.

      rosrun drone_detector_tracking drone_detector_tracking_node 

**Do not forget to broadcast required transformation**,if no transformation is found in 1 sec, identity transform will be use instead.



# 3. Nodes


## *drone_detector_node*

This node gets depth images and poses of the camera as inputs to detect and geolocalice foreign drones in the 3D space. That is achieved performing some image pre-processing to the depth image, thresholding with different bounds to obtain multiple binary masks. For each mask contours are detected and filtered by area, circularity, convexity and repetitivity. 

#### Subscribed Topics ####

* **depth_image** ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))
* **camera_info** ([sensor_msgs/CameraInfo](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html))
* **pose** ([geometry_msgs/Pose](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html))

#### Published Topics ####

* **/drone_detector/tracking_result**  ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))
 This image shows the detection results in the image plane marked with a random coloured circle.

* **/drone_detector/debug_n**  ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)) 
This topics are used to debug the thesholded binary masks associated with each frame. The numbers of topics may vary depending on the parameters. 

* **/drone_detector/detection_markers** ([visualization_msgs/MarkerArray](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/MarkerArray.html))
 Markers corresponding to each detection geolocaliced in 3D space.


#### Required transform ####

* **Static Transformation** between /base_link and /camera_link

#### Parameters ####

All the parameters can be modified in cfg/drone_detector_reconfigure.cfg or via dynamic reconfigure.

* **erosion_size** [Default: 3] Erosion kernel size of depth pre-processing (px).

* **unit_depth** [Default: 0.001] Depth unit from realsense d435 camera (m).
* **min_depth** [Default: 0.5] Minimal depth threshold value (m).
* **max_depth** [Default: 20.0] Max depth threshold value (m). **This parameter is automatically updated** to the height of the drone to avoid the floor in binary masks.
* **step_depth** [Default: 1] The step between thresholds (m).

* **min_area** [Default: 500] Minimal area of detected contours to be a good candidate (px).
* **max_area** [Default: 10000] Max area for good detections (px):

* **min_circularity** [Default: 0.3] Minimal circularity for detections.
* **max_circularity** [Default: 0.9 ]Max circularity for detections.

* **min_convexity** [Default: 0.75] Min convexity for good candidates.
* **max_convexity** [Default: 1] Max convexity for good candidates.

* **max_dist_detections** [Default: 100] Max distance between detections in different masks at same time (px)".
* **min_group_size** [Default: 4] Minimal consecutive frame detections in order to be considered".


* **sigma_xy** [Default: 0.3] Std desviation in image plane (m) .
* **sigma_z** [Default: 0.3] Std desviation coefficient in depth dimension (m).



## *dji_pose_aux.py*

This is an auxiliar node made to visualizate dji's pose. 

#### Subscribed Topic ####

* **/dji_control/pose** ([geometry_msgs/Pose](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html)) Topic of the dji_ual.

#### Published Topic ####
* **/dji_control/pose_stamped** ([geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)) Same pose but stamped with the execution time.

#### Published Transform ####

This node publishes the transformation between /map and /base_link (asssociated with dji frame).




## Help ##

* Rafael Caballero González (RCG) - rcaballero@catec.aero

* Fidel González Leiva (FGL) - fgonzalez@catec.aero