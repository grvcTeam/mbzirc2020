#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <object_estimation/centralized_estimator.h>

#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>
#include <mbzirc_comm_objs/SetObjectStatus.h>

#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>

using namespace std;
using namespace mbzirc_comm_objs;

class Estimator {
public:
    Estimator() {

        iteration_counter_ = 0;

        double frequency;
        bool a_priori_info;
        string conf_file;
        string robot_ns;
        vector<string> object_types;
        
        double lost_time_th, min_update_count, association_th;

        // Get parameters
        ros::param::param<double>("~frequency", frequency, 1.0);
        ros::param::param<string>("~robot_ns", robot_ns, "mbzirc2020");
        ros::param::param<vector<string> >("~object_types", object_types, vector<string>());
        ros::param::param<bool>("~a_priori_info", a_priori_info, false);
        ros::param::param<string>("~conf_file", conf_file, "conf.yaml");
        ros::param::param<bool>("~visualization", visualization_, false);
        ros::param::param<vector<int> >("~uav_ids", uav_ids_, vector<int>());
        
        ros::param::param<double>("~lost_time_th", lost_time_th, 20.0);
        ros::param::param<double>("~min_update_count", min_update_count, 0.0);
        ros::param::param<double>("~association_th", association_th, 6.0);
        ros::param::param<double>("~delay_max", delay_max_, 2.0);

        n_uavs_ = uav_ids_.size();

        if(n_uavs_ == 0 && !a_priori_info)
            ROS_ERROR("Missing parameter to get objects information.");

        if(object_types.size() == 0)
            ROS_ERROR("Missing parameter object types.");

        if(a_priori_info)
        {
            string config_folder = ros::package::getPath("object_estimation") + "/config/";
            conf_file = config_folder + conf_file;  
        }

        if (frequency <= 0) {
            ROS_ERROR("Estimator: Trying to set frequency to invalid value [%lf], using 1Hz instead", frequency);
            frequency = 1.0;
        }

        // Subscriptions/publications
        ros::NodeHandle nh;
        estimation_timer_ = nh.createTimer(ros::Duration(1.0/frequency), &Estimator::estimateCallback, this);

        for (int i = 0; i < n_uavs_ + 1; i++) {
            string sensed_topic = robot_ns + "_" + std::to_string(i) + "/sensed_objects";
            sensed_sub_.push_back(nh.subscribe(sensed_topic, 1, &Estimator::updateCallback, this));
        }

        objects_pub_ = nh.advertise<mbzirc_comm_objs::ObjectDetectionList>("estimated_objects", 1);

        if(visualization_)
            markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("objects_makers", 1);
        
        set_object_status_srv_ = nh.advertiseService("set_object_status", &Estimator::setObjectStatus, this);

        for(int i = 0; i < object_types.size(); i++)
        {
            int type = obj_type_from_string(object_types[i]);

            if(estimators_.find(type) != estimators_.end())
                ROS_WARN("Object type repeated.");
            else if(type != ObjectDetection::TYPE_UNKNOWN)
            {
                estimators_[type] = new CentralizedEstimator(type, association_th, lost_time_th, min_update_count);

                if(a_priori_info)
                    estimators_[type]->initializeAPrioriInfo(conf_file);

                for(int id = 0; id < n_uavs_; id++)
                {
                    vector<ObjectDetection *> empty_cand_vector;
                    map<int, vector<ObjectDetection *> > uav_cands;
                    uav_cands[uav_ids_[id]] = empty_cand_vector;

                    candidates_[type] = uav_cands;
                }
            }
        }
    }

    ~Estimator() {

        for(auto it = estimators_.begin(); it != estimators_.end(); ++it)
	    {
            delete it->second;
	    }
	
        // Delete candidates for each object type and UAV
        for(auto it = candidates_.begin(); it != candidates_.end(); ++it)
	    {
            for(int i = 0; i < n_uavs_; i++)
            {
                for(int j = 0; j < (it->second)[uav_ids_[i]].size(); j++)
                {
                    delete (it->second)[uav_ids_[i]][j];
                }
            }
	    }
    }

protected:

    /** \brief Callback to receive object detections
    */
    void updateCallback(const mbzirc_comm_objs::ObjectDetectionListConstPtr& msg) 
    {

        double delay = (ros::Time::now() - msg->stamp).toSec();

        int uav = msg->uav_id;

        if(msg->objects.size() && delay < delay_max_ && find(uav_ids_.begin(), uav_ids_.end(), uav) != uav_ids_.end())
        {
            int obj_type = msg->objects[0].type;    // TODO. We assume the same type for all objects in the list. Should we have a type in ObjectDetectionList?

            // Remove existing candidates
            if(candidates_[obj_type][uav].size())
            {
                for(int j = 0; j < candidates_[obj_type][uav].size(); j++)
                {
                    delete candidates_[obj_type][uav][j];
                }
                candidates_[obj_type][uav].clear();
            }

            // Store received candidates
            for(int j = 0; j < msg->objects.size(); j++)
            {
                ObjectDetection* detection_p = new ObjectDetection();
                *detection_p = msg->objects[j];
                candidates_[obj_type][uav].push_back(detection_p);
            }
        }
        else
            ROS_WARN("Received detections empty, with long delay or from a UAV not considered.");
    }

    /** \brief Callback to execute main thread at node's rate
    */
    void estimateCallback(const ros::TimerEvent& event) {
        
        // For each type of object
        for(auto it = estimators_.begin(); it != estimators_.end(); ++it)
        {
            int obj_type = it->first;
            CentralizedEstimator * est_ptr = it->second;

            // Predict objects
            est_ptr->predict( (event.current_real - event.last_real).toSec() );

            // Update objects 
            for(auto cand_it = candidates_[obj_type].begin(); cand_it != candidates_[obj_type].end(); ++cand_it)
            {
                if((cand_it->second).size())
                {
                    est_ptr->update(cand_it->second);

                    // Remove candidates
                    
                    for(int j = 0; j < (cand_it->second).size(); j++)
                    {
                        delete (cand_it->second)[j];
                    }
                    (cand_it->second).clear();
                }
            }

            est_ptr->removeLostTargets();
            
            publishObjects(obj_type);

            if(visualization_)
                publishMarkers(obj_type);

            if(iteration_counter_ == 5)
                est_ptr->printTargetsInfo();
        }
        		
        if(iteration_counter_ == 5)
            iteration_counter_ = 0;
        else
            iteration_counter_++;    
    }

    /** \brief Callback for service. Set the status of an object
    */
    bool setObjectStatus(mbzirc_comm_objs::SetObjectStatus::Request &req, mbzirc_comm_objs::SetObjectStatus::Response &res)
    {
        ObjectStatus object_status;

        switch(req.object_status)
        {
            case mbzirc_comm_objs::SetObjectStatus::Request::UNASSIGNED:
            object_status = UNASSIGNED;
            break;
            case mbzirc_comm_objs::SetObjectStatus::Request::ASSIGNED:
            object_status = ASSIGNED;
            break;
            case mbzirc_comm_objs::SetObjectStatus::Request::CAUGHT:
            object_status = CAUGHT;
            break;
            case mbzirc_comm_objs::SetObjectStatus::Request::DEPLOYED:
            object_status = DEPLOYED;
            break;
            case mbzirc_comm_objs::SetObjectStatus::Request::LOST:
            object_status = LOST;
            break;
            case mbzirc_comm_objs::SetObjectStatus::Request::FAILED:
            object_status = FAILED;
            break;
            default:
            ROS_ERROR("Not valid object status for assignment.");
        }	

        return estimators_[req.object_type]->setTargetStatus(req.object_id, object_status);
    }

    // TODO: Move to some kind of utils lib, as it is repeated
    int obj_type_from_string(const string& type) 
    {

        int obj_type;
        if(type == "balloon")
            obj_type = mbzirc_comm_objs::ObjectDetection::TYPE_BALLOON;
        else if(type == "drone")
            obj_type = mbzirc_comm_objs::ObjectDetection::TYPE_DRONE;
        else if(type == "brick")
            obj_type = mbzirc_comm_objs::ObjectDetection::TYPE_BRICK;
        else if(type == "fire")
            obj_type = mbzirc_comm_objs::ObjectDetection::TYPE_FIRE;
        else if(type == "passage")
            obj_type = mbzirc_comm_objs::ObjectDetection::TYPE_PASSAGE;    
        else
            {
                ROS_ERROR("Unknown object type %s", type.c_str());
                obj_type = mbzirc_comm_objs::ObjectDetection::TYPE_UNKNOWN;
            }

        return obj_type;
    }

    /** \brief Publish object beliefs
    \param obj_type Object type
    */
    void publishObjects(int obj_type)
    {
        ObjectDetectionList list;
        vector<vector<double> > covariances;
        double x, y, z, vx, vy, vz;
        ObjectStatus target_status;
        int target_color;

        // Get ids to plot active targets
        vector<int> active_targets = estimators_[obj_type]->getActiveTargets();

        list.uav_id = -1;
        list.stamp = ros::Time::now();

        for(int i = 0; i < active_targets.size(); i++)
        {
            if(estimators_[obj_type]->getTargetInfo(active_targets[i], x, y, z, covariances, vx, vy, vz))
            {
                estimators_[obj_type]->getTargetInfo(active_targets[i], x, y, z, target_status, target_color);

                ObjectDetection object;

                object.header.stamp = ros::Time::now();
                object.header.frame_id = "/map";    // TODO: /game?
                object.type = obj_type;
                object.color = target_color;

                object.pose.pose.position.x = x;
                object.pose.pose.position.y = y;
                object.pose.pose.position.z = z;

                // TODO. Get orientation
                for(int row = 0; row < 3; row++)
                    for(int col = 0; col < 3; col++)
                        object.pose.covariance[row*6 + col] = covariances[row][col];

                list.objects.push_back(object);
            }
            else
                ROS_ERROR("Object ID not found");
        }        

        if(list.objects.size())
            objects_pub_.publish(list);
    }

    /** \brief Publish markers to represent object beliefs
    \param obj_type Object type
    */
    void publishMarkers(int obj_type)
    {
        visualization_msgs::MarkerArray marker_array;
        ros::Time curr_time = ros::Time::now();

        vector<double> w(2);
        vector<double> v(4);
        vector<vector<double> > covariances;
        double a, b, c, yaw, x, y, z, vx, vy, vz;
        ObjectStatus target_status;
        int target_color;

        // Get ids to plot active targets
        vector<int> active_targets = estimators_[obj_type]->getActiveTargets();

        for(int i = 0; i < active_targets.size(); i++)
        {
            if(estimators_[obj_type]->getTargetInfo(active_targets[i], x, y, z, covariances, vx, vy, vz))
            {
                estimators_[obj_type]->getTargetInfo(active_targets[i], x, y, z, target_status, target_color);

                if(covariances[0][0] == 0.0 && covariances[1][1] == 0.0 && covariances[2][2] == 0.0)
                {    
                    // No uncertainty
                    a = 0.5;
                    b = 0.5;
                    c = 0.5;
                    yaw = 0.0;
                }
                else
                {
                    // Compute SVD of cholesky. The singular values are the square roots of the eigenvalues of
                    // the covariance matrix 
                    eigendec(4*covariances[0][0], 4*covariances[1][1], 4*covariances[0][1], w, v);

                    a = sqrt(fabs(w[0]));
                    b = sqrt(fabs(w[1]));
                    c = 4*covariances[2][2];
                    yaw = atan2(v[1],v[0]);        
                }
                
                // Fill in marker
                visualization_msgs::Marker marker;

                // Set color for the target, default if UNKNOWN
                switch(target_color)
                {
                    case ObjectDetection::COLOR_RED:
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    marker.color.a = 1;
                    break;
                    case ObjectDetection::COLOR_BLUE:
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                    marker.color.a = 1;
                    break;
                    case ObjectDetection::COLOR_GREEN:
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    marker.color.a = 1;
                    break;
                    case ObjectDetection::COLOR_ORANGE:
                    marker.color.r = 1.0;
                    marker.color.g = 0.65;
                    marker.color.b = 0.0;
                    marker.color.a = 1;
                    break;
                    default:
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    marker.color.a = 1;
                    break;
                }
            
                // Set the frame ID and timestamp
                marker.header.frame_id = "/map";    // TODO: /game?
                marker.header.stamp = curr_time;

                // Set the namespace and id for this marker.  This serves to create a unique ID    
                // Any marker sent with the same namespace and id will overwrite the old one    
                marker.ns = "cov_ellipse";    
                marker.id = i;
            
                // Set the marker type    
                marker.type = visualization_msgs::Marker::SPHERE;

                // Set the marker action.  Options are ADD and DELETE    
                marker.action = visualization_msgs::Marker::ADD;
                
                // Set the scale of the marker -- 1x1x1 here means 1m on a side

                marker.scale.x = a;
                marker.scale.y = b;    
                marker.scale.z = c;

                marker.lifetime = ros::Duration(1.0);

                // Set the central pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header    
                marker.pose.position.x = x;
                marker.pose.position.y = y;
                marker.pose.position.z = z;
                
                tf2::Quaternion q;
                q.setRPY(0.0,0.0,yaw);
                marker.pose.orientation = tf2::toMsg(q);

                marker_array.markers.push_back(marker);

                // Plot target ID
                marker.ns = "target_id";
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.text = to_string(active_targets[i]);
                marker.pose.position.x = x + 1.0;
                marker.pose.position.y = y + 1.0;
                marker.pose.position.z = z + 1.0;    
                marker.scale.z = 1.0;
                
                marker_array.markers.push_back(marker);

                if(obj_type == ObjectDetection::TYPE_DRONE)
                {
                    // Plot velocity
                    marker.ns = "velocity";
                    marker.type = visualization_msgs::Marker::ARROW;    
                    marker.scale.x = sqrt(vx*vx+vy*vy+vz*vz);
                    marker.scale.y = 0.1;    
                    marker.scale.z = 0.1;
                    marker.pose.position.x = x;
                    marker.pose.position.y = y;
                    marker.pose.position.z = z;    
                
                    if(marker.scale.x != 0.0)
                    {
                        tf2::Quaternion q;
                        q.setRPY(0.0,atan2(vz,sqrt(vx*vx+vy*vy)),atan2(vy,vx));
                        marker.pose.orientation = tf2::toMsg(q);
                    }
                    
                    marker_array.markers.push_back(marker);
                }
            }
            else
                ROS_ERROR("Object ID not found");
        }

        // Publish the marker    
        markers_pub_.publish(marker_array);
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
    void eigendec(double c11, double c22, double c12, vector<double> &D, vector<double> &E)
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

    /// Ids of UAVs to connect
    int n_uavs_;
    vector<int> uav_ids_;
    
    int iteration_counter_;
    bool visualization_;        /// Activate to publish markers
    double delay_max_;          /// Maximum delay allowed for object detections

    /// Timer, publishers and subscribers
    ros::Timer estimation_timer_;
    ros::Publisher objects_pub_;
    ros::Publisher markers_pub_;
    ros::ServiceServer set_object_status_srv_;
    vector<ros::Subscriber> sensed_sub_;

    /// Candidates queues with object detections, one per object and UAV
	map<int, map<int, vector<ObjectDetection *> > > candidates_;

    /// Centralized filter for each type of object 
	map<int, CentralizedEstimator*> estimators_;
	
};

float squared_distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float dz = b.z - a.z;
    return dx*dx + dy*dy + dz*dz;
}

class SimpleClusteringEstimator: public Estimator {
public:

    SimpleClusteringEstimator(const std::string& _uav_ns): Estimator() {
        ros::NodeHandle nh;

        int max_uav_count = 3;  // Even if uavs does not exist, subscribing makes no harm
        for (int i = 1; i < max_uav_count + 1; i++) {
            std::string sensed_topic = _uav_ns + "_" + std::to_string(i) + "/sensed_objects";
            sensed_sub_.push_back(nh.subscribe(sensed_topic, 1, &SimpleClusteringEstimator::updateCallback, this));
        }
    }

protected:

    void updateCallback(const mbzirc_comm_objs::ObjectDetectionListConstPtr& msg) {
        for (auto sensed: msg->objects) {
            bool first_time_sensed = true;
            for (auto &target: targets_.objects) {
                if (squared_distance(sensed.pose.pose.position, target.pose.pose.position) < squared_distance_th_) {
                    // TODO: Supose all object poses are defined in the same frame_id!
                    float sensed_radius = 0.5 * std::max(fabs(sensed.scale.x), fabs(sensed.scale.y));
                    if (sensed_radius > 0.01) {  // threshold: 1 [cm]
                        float target_radius = 0.5 * std::max(fabs(target.scale.x), fabs(target.scale.y));
                        float x_max = std::max(sensed.pose.pose.position.x + sensed_radius, target.pose.pose.position.x + target_radius);
                        float y_max = std::max(sensed.pose.pose.position.y + sensed_radius, target.pose.pose.position.y + target_radius);
                        float x_min = std::min(sensed.pose.pose.position.x - sensed_radius, target.pose.pose.position.x - target_radius);
                        float y_min = std::min(sensed.pose.pose.position.y - sensed_radius, target.pose.pose.position.y - target_radius);

                        float x_scale = x_max - x_min;
                        float y_scale = y_max - y_min;
                        float z_scale = std::max(sensed.scale.z, target.scale.z);  // Supose all objects are at the same height

                        float x_new = x_min + 0.5 * x_scale;
                        float y_new = y_min + 0.5 * y_scale;
                        float z_new = 0.5 * z_scale;

                        // TODO: Something with covariance?
                        target.pose.pose.position.x = x_new;
                        target.pose.pose.position.y = y_new;
                        target.pose.pose.position.z = z_new;
                        target.scale.x = x_scale;
                        target.scale.y = y_scale;
                        target.scale.z = z_scale;
                    }
                    first_time_sensed = false;
                    break;
                }
            }
            if (first_time_sensed) {
                // TODO: id? Use index?
                mbzirc_comm_objs::ObjectDetection new_estimated;
                new_estimated.header = sensed.header;
                new_estimated.type = sensed.type;
                new_estimated.pose.pose.position = sensed.pose.pose.position;
                new_estimated.pose.pose.orientation.w = 1.0;  // Forget orientation...
                new_estimated.pose.covariance = sensed.pose.covariance;
                float xy_scale = std::max(fabs(sensed.scale.x), fabs(sensed.scale.y));
                new_estimated.scale.x = xy_scale;
                new_estimated.scale.y = xy_scale;  // ...and make scale.x = scale.y
                new_estimated.scale.z = sensed.scale.z;
                new_estimated.color = sensed.color;
                targets_.objects.push_back(new_estimated);
            }
        }
    }

    std::vector<ros::Subscriber> sensed_sub_;
    float squared_distance_th_ = 25.0;  // TODO: tune, from params?
    ObjectDetectionList targets_;
};

struct PileData {
    std::string color;
    std::string frame_id;
    float position_x;
    float position_y;
    float position_z;
    float orientation_yaw;
    float scale_x;
    float scale_y;
    float scale_z;

    void print() {
        printf("color: %s, frame_id: %s, position_x: %f, position_y: %f, position_z: %f, orientation_yaw: %f, scale_x: %f, scale_y: %f, scale_z: %f \n", \
                color.c_str(), frame_id.c_str(), position_x, position_y, position_z, orientation_yaw, scale_x, scale_y, scale_z);
    }
};

void operator>>(const YAML::Node& in, PileData& pile_data) {
    // TODO: Check that expected fields do exist
    pile_data.color = in["color"].as<std::string>();
    pile_data.frame_id = in["frame_id"].as<std::string>();
    pile_data.position_x = in["position_x"].as<float>();
    pile_data.position_y = in["position_y"].as<float>();
    pile_data.position_z = in["position_z"].as<float>();
    pile_data.orientation_yaw = in["orientation_yaw"].as<float>();
    pile_data.scale_x = in["scale_x"].as<float>();
    pile_data.scale_y = in["scale_y"].as<float>();
    pile_data.scale_z = in["scale_z"].as<float>();
}

// TODO: Move to some kind of utils lib, as it is repeated
int color_from_string(const string& color) {
    int out_color;
    switch(color[0]) {
        case 'R':
        case 'r':
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_RED;
            break;
        case 'G':
        case 'g':
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_GREEN;
            break;
        case 'B':
        case 'b':
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_BLUE;
            break;
        case 'O':
        case 'o':
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_ORANGE;
            break;
        default:
        ROS_ERROR("Unknown color %s", color.c_str());
            out_color = mbzirc_comm_objs::ObjectDetection::COLOR_UNKNOWN;
    }
    return out_color;
}

class APrioriInfoEstimator: public Estimator {
public:

    APrioriInfoEstimator(): Estimator() {
        std::string config_folder = ros::package::getPath("target_estimation") + "/config/";
        std::string config_filename = config_folder + "piles.yaml";  // TODO: from parameter?

        YAML::Node yaml_config = YAML::LoadFile(config_filename);
        for (std::size_t i = 0; i < yaml_config["piles"].size(); i++) {
            PileData pile_data;
            yaml_config["piles"][i] >> pile_data;
            // pile_data.print();

            mbzirc_comm_objs::ObjectDetection object;
            object.header.stamp = ros::Time::now();
            object.header.frame_id = pile_data.frame_id;
            object.type = mbzirc_comm_objs::ObjectDetection::TYPE_BRICK;
            // object.pose.covariance...  // TODO: needed?
            // object.relative_position  // ignored here
            // object.relative_yaw  // ignored here
            object.pose.pose.position.x = pile_data.position_x;
            object.pose.pose.position.y = pile_data.position_y;
            object.pose.pose.position.z = pile_data.position_z;
            float half_yaw = 0.5 * pile_data.orientation_yaw;
            object.pose.pose.orientation.x = 0.0;
            object.pose.pose.orientation.y = 0.0;
            object.pose.pose.orientation.z = sin(half_yaw);
            object.pose.pose.orientation.w = cos(half_yaw);
            object.scale.x = pile_data.scale_x;
            object.scale.y = pile_data.scale_y;
            object.scale.z = pile_data.scale_z;
            object.color = color_from_string(pile_data.color);
            targets_.objects.push_back(object);
        }
    }
protected:
    ObjectDetectionList targets_;
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "estimation_node");
     
    Estimator estimator;
    
    ros::spin();

    return 0;
}
