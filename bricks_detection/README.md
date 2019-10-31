# Brick Detection


## Overview
This package provides a brick detection tool for MBZIRC2020 Challenge 2

## Dependencies

- [PCL v1.9.1](https://github.com/PointCloudLibrary/pcl) 
- OpenCV >= v3 
- Boost v1.58
- ROS Kinetic

## Usage

Just run the main launch file:

```bash
roslaunch bricks_detection bricks_detection.launch
```

In case a namespace would be needed, a specific launch is provided for that:

```bash
roslaunch bricks_detection ns_bricks_detection.launch
```

**Dynamic reconfigure** is enabled for most of the parameters. This means that these can be changed dynamically at runtime. For starting the dynamic reconfigure GUI:
```bash
rosrun dynamic_reconfigure reconfigure_gui
```

## Config files

Configuration files for stablishing HSV color limits are ubicated inside the **cfg** folder. These are JSON files with the tag **colors** which is a list of colors with a **min** and **max** values for HSV and as well as its **name** as a string. Duplicated color names with differents HSV ranges are allowed since the node will regroup them after.

Possible ranges are:
* **h** -> [0, 360]  
* **s** -> [0, 1]  
* **v** -> [0, 1]  

```json
{
   "colors": [
      {
         "min": {
            "h": 0,
            "s": 0.45,
            "v": 0.45
         },
         "max": {
            "h": 15,
            "s": 1.0,
            "v": 0.95
         },
         "name": "red"
      },
      {
         "min": {
            "h": 194,
            "s": 0.25,
            "v": 0.2
         },
         "max": {
            "h": 214,
            "s": 1.0,
            "v": 1.0
         },
         "name": "blue"
      }
   ]
}
```

Files included by default are:
* **ist_colors.json**  
Limits from IST at the beginning of the integration week.

* **grvc_field_testing.json**  
Limits after testing in the GRVC fly zone during integration week.

## Launch files
* **bricks_detection.launch:**

     Arguments:  

     - **`use_pointcloud`** If true, node uses pointcloud for bricks detection. If false, just RGB images. Type: **bool**. Default: `false`.

     - **`rgb_img_topic`** Topic for RGB images. Type: **string**. Default: `/camera/color/image_raw`.

     - **`pcloud_topic`**  Topic for pointclouds. Type: **string**. Default: `/camera/depth_registered/points`.

     - **`colors_json`** JSON file path for HSV colors. Type: **string**. Default: `$(find bricks_detection)/cfg/ist_colors.json`.  

* **ns_bricks_detection.launch:**

     Same arguments as **bricks_detection.launch** plus:

     - **`ns`** Namespace. Type: **string**. Default: `false`.  

## Topics

#### Subscribed

* **`/camera/color/image_raw`** ([sensor_msgs/Image])

	Image topic from a RGB camera. This topic can be configured using the **rgb_image_topic** parameter.  

* **`/camera/depth_registered/points`** ([sensor_msgs/PointCloud2])

	Pointcloud topic from a 3D camera. This topic can be configured using the **pcloud_topic** parameter.  

#### Published

* **`bricks`** ([mbzirc_comm_objs/ObjectDetectionList])

	List of detected objects for a given image or pointcloud.  

* **`bricks/color`** ([sensor_msgs/PointCloud2])

	Four topics giving the pointcloud after being processed. The word **color** can be **red**, **blue**, **orange**, and **green**.  

* **`bricks/filtered_image`** ([sensor_msgs/Image])

	Image after being processed.

## Services

* **`use_pointcloud`** ([std_srvs/SetBool])

	Changes between RGB or Pointcloud processing
