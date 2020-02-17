//------------------------------------------------------------------------------
// GRVC MBZIRC
// Author Jesus Capitan <jcapitan@us.es>
// 
//------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2020 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//------------------------------------------------------------------------------

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>
#include <mbzirc_comm_objs/Object.h>
#include <mbzirc_comm_objs/ObjectList.h>
#include <tf2/utils.h>

#include<string>
#include<vector>
#include<sstream>

using namespace std;
using namespace mbzirc_comm_objs;


/** This class subscribes to MBZIRC data to publish visual markers for rViz.
*/
class Visualizer 
{

public:
	Visualizer();
	~Visualizer();
    void clearCache();

    void publishArena();
    void publishMarkers();
	
protected:

	/// Callbacks
	void objectDetectionsReceived(const ObjectDetectionListConstPtr& msg);
	void uavPoseReceived(const geometry_msgs::PoseStamped::ConstPtr& uav_pose, const string uav_id);
    void objectsReceived(const ObjectListConstPtr& msg);
    std_msgs::ColorRGBA getColor(int color);
    void eigendec(double c11, double c22, double c12, vector<double> &D, vector<double> &E);

	/// Subscribers
	vector<ros::Subscriber> uav_subs_;
    vector<ros::Subscriber> sensed_subs_;
    ros::Subscriber object_sub_;
    
	/// Publishers
	ros::Publisher arena_pub_;
    ros::Publisher uavs_pub_;
    ros::Publisher sensed_pub_;
    ros::Publisher objects_pub_;

	/// Last data received
	map<string, map<int, vector<ObjectDetection> > > object_detections_;
    map<string, geometry_msgs::PoseStamped> uavs_poses_;
    map<int, vector<Object> > objects_;
    
	/// Id and number of UAVs
    vector<string> uav_ids_;
	int n_uavs_;

};

/** \brief Constructor
*/
Visualizer::Visualizer()
{
    // Get parameters
    string robot_ns, object_topic;
    ros::param::param<string>("~robot_ns", robot_ns, "mbzirc2020");
    ros::param::param<string>("~object_topic", object_topic, "estimated_objects");
    ros::param::param<vector<string> >("~uav_ids", uav_ids_, vector<string>());

                
    n_uavs_ = uav_ids_.size();

    // Subscriptions/publications
    ros::NodeHandle nh;
      
    for (int i = 0; i < n_uavs_; i++) 
    {
        string sensed_topic = robot_ns + "_" + uav_ids_[i] + "/sensed_objects";
        string uav_topic = robot_ns + "_" + uav_ids_[i] + "/ual/pose";

        sensed_subs_.push_back(nh.subscribe(sensed_topic, 1, &Visualizer::objectDetectionsReceived, this));
        uav_subs_.push_back(nh.subscribe<geometry_msgs::PoseStamped>(uav_topic, 1, std::bind(&Visualizer::uavPoseReceived, this, std::placeholders::_1, uav_ids_[i]) ));
    }

    object_sub_ = nh.subscribe(object_topic, 1, &Visualizer::objectsReceived, this);

    arena_pub_ = nh.advertise<visualization_msgs::Marker>("mbzirc_markers/arena", 0);
    uavs_pub_ = nh.advertise<visualization_msgs::MarkerArray>("mbzirc_markers/uavs", 0);
    sensed_pub_ = nh.advertise<visualization_msgs::MarkerArray>("mbzirc_markers/object_detections", 1);
    objects_pub_ = nh.advertise<visualization_msgs::MarkerArray>("mbzirc_markers/objects", 1);
}

/** \brief Destructor
*/
Visualizer::~Visualizer()
{
}

/** \brief Clear cached data
*/
void Visualizer::clearCache()
{
    object_detections_.clear();
    uavs_poses_.clear();
    objects_.clear();
}

/** \brief Callback to receive observations from vision module
*/
void Visualizer::objectDetectionsReceived(const ObjectDetectionListConstPtr& msg)
{
    if(msg->objects.size())
        object_detections_[msg->agent_id][msg->objects[0].type] = msg->objects;
}

/** \brief Callback to receive UAVs poses
*/
void Visualizer::uavPoseReceived(const geometry_msgs::PoseStamped::ConstPtr& uav_pose, const string uav_id)
{
    uavs_poses_[uav_id] = *uav_pose;
}

/** \brief Callback to receive objects
*/
void Visualizer::objectsReceived(const ObjectListConstPtr& msg)
{   
    if(msg->objects.size())
        objects_[msg->objects[0].type] = msg->objects;
}

/** \brief Compute color for markers
\param color Color type
*/
std_msgs::ColorRGBA Visualizer::getColor(int color)
{
    // Set color for the target, default if UNKNOWN
    std_msgs::ColorRGBA col;

    switch(color)
    {
        case ObjectDetection::COLOR_RED:
        col.r = 1.0;
        col.g = 0.0;
        col.b = 0.0;
        break;
        case ObjectDetection::COLOR_BLUE:
        col.r = 0.0;
        col.g = 0.0;
        col.b = 1.0;
        break;
        case ObjectDetection::COLOR_GREEN:
        col.r = 0.0;
        col.g = 1.0;
        col.b = 0.0;
        break;
        case ObjectDetection::COLOR_ORANGE:
        col.r = 1.0;
        col.g = 0.65;
        col.b = 0.0;
        break;
        default:
        col.r = 0.0;
        col.g = 1.0;
        col.b = 0.0;
        break;
    }

    col.a = 1;    
                    
    return col;
}

/**
Closed form eigenvalue decomposition
    
C  Positive definite input matrix of form
        I[0] I[1]
        I[2] I[3]   where I[1]=I[2]
D  Output eigenvalues
E  Eigenvector matrix of form
        E[0] E[2]
        E[1] E[3]
*/
void Visualizer::eigendec(double c11, double c22, double c12, vector<double> &D, vector<double> &E)
{ 
    double a,b,enorm;

    // Find eigenvalues 
    a = c11+c22;                         // trace 
    b = sqrt((c11-c22)*(c11-c22)+4*c12*c12);
    D[0] = (a+b)/2;
    D[1] = (a-b)/2;

    // Find eigenvector 1 
    E[0] = c22+c12-D[0];
    E[1] = D[0]-c11-c12;
    enorm = sqrt(E[0]*E[0]+E[1]*E[1]);

    if(enorm > 0.0) {
        E[0] = E[0]/enorm;
        E[1] = E[1]/enorm;
    }

    // Find eigenvector 2 
    E[2] = c22+c12-D[1];
    E[3] = D[1]-c11-c12;
    enorm = sqrt(E[2]*E[2]+E[3]*E[3]);

    if(enorm > 0.0) {
        E[2] = E[2]/enorm;
        E[3] = E[3]/enorm;
    }
}

/** Publish arena marker
*/
void Visualizer::publishArena()
{
    // Publish the scenario
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "arena";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://mbzirc_visualization/meshes/stage.dae";
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = -0.99893;
    marker.pose.orientation.w = 0.046306;

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.mesh_use_embedded_materials = true;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration();

    arena_pub_.publish(marker);
}


/** Publish markers
*/
void Visualizer::publishMarkers()
{
    // Publish UAVs and states
    visualization_msgs::MarkerArray uav_markers;
    
    for (int id = 0; id < n_uavs_; id++)
    {
        string uav_id = uav_ids_[id];
        // If not empty (never received)
        if(uavs_poses_.find(uav_id) != uavs_poses_.end())
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = uavs_poses_[uav_id].header.frame_id;
            marker.header.stamp = ros::Time();
            marker.id = id;
            marker.ns = "uavs";
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.mesh_resource = "package://robots_description/models/mbzirc/meshes/multirotor.dae";
            marker.color.a = 1;    
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose = uavs_poses_[uav_id].pose;

            marker.scale.x = 0.001;
            marker.scale.y = 0.001;
            marker.scale.z = 0.001;
            marker.mesh_use_embedded_materials = true;

            switch(id)
            {
                case 0:
                // orange
                marker.color.r = 1.0;
                marker.color.g = 0.647;
                marker.color.b = 0.0;
                break;
                case 1:
                // indigo
                marker.color.r = 0.294;
                marker.color.g = 0.0;
                marker.color.b = 0.510; 
                break;
                case 2:
                // zinc yellow
                marker.color.r = 0.945;
                marker.color.g = 0.812;
                marker.color.b = 0.267;
                break;
                default:
                // gray
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
                break;
            }

            uav_markers.markers.push_back(marker);
        }
    }

    uavs_pub_.publish(uav_markers);
    
    // Publish object detections
    visualization_msgs::MarkerArray detection_markers;

    for (int id = 0; id < n_uavs_; id++)
    {
        // If not empty (never received)
        auto it = object_detections_.find(uav_ids_[id]);
        if(it != object_detections_.end())
        {
            // Publish markers for all detection types
            int cont = 0;

            for(auto detectors = (it->second).begin(); detectors != (it->second).end(); ++detectors)
            {
                for(int i = 0; i < (detectors->second).size(); i++)
                {
                    ObjectDetection observation = (detectors->second)[i];

                    visualization_msgs::Marker marker;
                    marker.header.frame_id = observation.header.frame_id;
                    marker.header.stamp = ros::Time();
                    marker.id = cont++;
                    marker.ns = uav_ids_[id];
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.lifetime = ros::Duration(1.0);
                    marker.color = getColor(observation.color);
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose = observation.pose.pose;
                    marker.scale = observation.scale;

                    if(marker.scale.x == 0.0)
                        marker.scale.x = 0.5;
                    if(marker.scale.y == 0.0)
                        marker.scale.y = 0.5;
                    if(marker.scale.z == 0.0)
                        marker.scale.z = 0.5;
                    
                    detection_markers.markers.push_back(marker);

                    marker.id = cont++;
                    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                    marker.pose.position.x += 1.0;
                    marker.pose.position.y += 1.0;
                    marker.pose.position.z += 1.0;
                    marker.scale.z = 1.0;

                    // white
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 1.0; 

                    if(observation.type == ObjectDetection::TYPE_BRICK)
                        marker.text = "Brick";
                    else if(observation.type == ObjectDetection::TYPE_LWALL)
                        marker.text = "Lwall";
                    else if(observation.type == ObjectDetection::TYPE_UCHANNEL)
                        marker.text = "Uchannel";
                    else if(observation.type == ObjectDetection::TYPE_FIRE)
                        marker.text = "Fire";
                    else if(observation.type == ObjectDetection::TYPE_HOLE)
                        marker.text = "Hole";
                    else if(observation.type == ObjectDetection::TYPE_PASSAGE)
                        marker.text = "Passage";
                    else
                        marker.text = "Unknown";
                    
                    detection_markers.markers.push_back(marker);
                }
            }
       }
    }

    sensed_pub_.publish(detection_markers);

    // Publish object markers
    
    for(auto it = objects_.begin(); it != objects_.end(); ++it)
    {
        visualization_msgs::MarkerArray object_markers;

        for(int ob_id = 0; ob_id < (it->second).size(); ob_id++)
        {
            double a,b,c,cov_yaw;
            vector<double> w(2);
            vector<double> v(4);

            Object obj = (it->second)[ob_id];

            visualization_msgs::Marker marker;

            if(obj.pose.covariance[0] == 0.0 && obj.pose.covariance[7] == 0.0 && obj.pose.covariance[14] == 0.0)
            {    
                // No uncertainty
                a = 0.5;
                b = 0.5;
                c = 0.5;
                cov_yaw = 0.0;
            }
            else
            {
                // Compute SVD of cholesky. The singular values are the square roots of the eigenvalues of
                // the covariance matrix 
                eigendec(4*obj.pose.covariance[0], 4*obj.pose.covariance[7], 4*obj.pose.covariance[1], w, v);

                a = sqrt(fabs(w[0]));
                b = sqrt(fabs(w[1]));
                c = obj.pose.covariance[14];
                cov_yaw = atan2(v[1],v[0]);        
            }
            
            marker.header.frame_id = obj.header.frame_id;
            marker.header.stamp = ros::Time();
            marker.id = ob_id;
            marker.ns = "cov_ellipse_" + obj.type;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.lifetime = ros::Duration(1.0);
            marker.color = getColor(obj.color);    
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = obj.pose.pose.position;

            marker.scale.x = a;
            marker.scale.y = b;    
            marker.scale.z = c;
            
            tf2::Quaternion q;
            q.setRPY(0.0,0.0, cov_yaw);
            marker.pose.orientation = tf2::toMsg(q);

            object_markers.markers.push_back(marker);

            if(obj.type == Object::TYPE_PILE || obj.type == Object::TYPE_WALL || obj.type == Object::TYPE_PASSAGE )
            {
                // Plot orientation
                marker.ns = "orientation_" + obj.type;
                marker.type = visualization_msgs::Marker::ARROW;    
                marker.scale.x = 3.0;
                marker.scale.y = 0.2;    
                marker.scale.z = 0.2;
                marker.pose = obj.pose.pose;
                    
                object_markers.markers.push_back(marker);
            }

            // Plot object ID
            marker.ns = "object_id" + obj.type;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            
            if(obj.type == Object::TYPE_PILE)
                marker.text = "P";
            else if(obj.type == Object::TYPE_WALL)
                marker.text = "W";
            else if(obj.type == Object::TYPE_FIRE)
                marker.text = "F";
            else if(obj.type == Object::TYPE_PASSAGE)
                marker.text = "PS";

            marker.text += to_string(obj.id);
     
            marker.pose.position.x += 1.0;
            marker.pose.position.y += 1.0;
            marker.pose.position.z += 1.0;    
            marker.scale.z = 1.0;

            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0; 
                
            object_markers.markers.push_back(marker);
        }

        objects_pub_.publish(object_markers);
    }                
}

/** Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualizer_node");
  
    Visualizer vis; 

    ros::Rate loop(30);
    int cont = 0;

    while(ros::ok())
    {
        ros::spinOnce();
        vis.publishMarkers();
        vis.clearCache();

        cont++;
        if(cont == 30)
        {
            vis.publishArena();
            cont = 0;
        }

        loop.sleep();
    }
}