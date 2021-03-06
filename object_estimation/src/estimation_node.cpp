#include <ros/ros.h>
#include <ros/package.h>
#include <object_estimation/centralized_estimator.h>

#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>
#include <mbzirc_comm_objs/Object.h>
#include <mbzirc_comm_objs/ObjectList.h>
#include <mbzirc_comm_objs/SetObjectStatus.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/utils.h>

using namespace std;
using namespace mbzirc_comm_objs;
using namespace geometry_msgs;

class Estimator {
public:
    Estimator() {

        iteration_counter_ = 0;
        arena_limits_ = false;

        tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

        double frequency;
        bool a_priori_info;
        string conf_file;
        string robot_ns;
        vector<string> object_types;
        vector<double> association_th;
        double lost_time_th, min_update_count;
        
        // Get parameters
        ros::param::param<double>("~frequency", frequency, 1.0);
        ros::param::param<string>("~robot_ns", robot_ns, "mbzirc2020");
        ros::param::param<vector<string> >("~object_types", object_types, vector<string>());
        ros::param::param<bool>("~a_priori_info", a_priori_info, false);
        ros::param::param<string>("~conf_file", conf_file, "config/conf.yaml");
        ros::param::param<vector<string> >("~uav_ids", uav_ids_, vector<string>());
        
        ros::param::param<double>("~lost_time_th", lost_time_th, 20.0);
        ros::param::param<double>("~min_update_count", min_update_count, 0.0);
        ros::param::param<vector<double> >("~association_th", association_th, vector<double>(object_types.size(),6.0));
        ros::param::param<double>("~delay_max", delay_max_, 2.0);

        n_uavs_ = uav_ids_.size();

        if(n_uavs_ == 0 && !a_priori_info)
            ROS_ERROR("Missing parameter to get objects information.");

        if(object_types.size() == 0)
            ROS_ERROR("Missing parameter object types.");

        if(a_priori_info)
        {
            setArenaLimits(conf_file);  
        }

        if (frequency <= 0) {
            ROS_ERROR("Estimator: Trying to set frequency to invalid value [%lf], using 1Hz instead", frequency);
            frequency = 1.0;
        }

        // Subscriptions/publications
        ros::NodeHandle nh;
        estimation_timer_ = nh.createTimer(ros::Duration(1.0/frequency), &Estimator::estimateCallback, this);

        for (int i = 0; i < n_uavs_; i++) {
            string sensed_topic = robot_ns + "_" + uav_ids_[i] + "/sensed_objects";
            sensed_sub_.push_back(nh.subscribe(sensed_topic, 1, &Estimator::updateCallback, this));
        }

        objects_pub_ = nh.advertise<mbzirc_comm_objs::ObjectList>("estimated_objects", 1);
        
        set_object_status_srv_ = nh.advertiseService("set_object_status", &Estimator::setObjectStatus, this);

        for(int i = 0; i < object_types.size(); i++)
        {
            vector<int> detectors;

            int type = obj_type_from_string(object_types[i], detectors);

            if(estimators_.find(type) != estimators_.end())
                ROS_WARN("Object type repeated.");
            else
            {
                estimators_[type] = new CentralizedEstimator(type, association_th[i], lost_time_th, min_update_count);
                detectors_[type] = detectors;

                if(a_priori_info)
                    estimators_[type]->initializeAPrioriInfo(conf_file);

                for(int j = 0; j < detectors.size(); j++)
                {
                    vector<ObjectDetection *> empty_cand_vector;

                    for(int id = 0; id < n_uavs_; id++)
                    {    
                        candidates_[detectors[j]][uav_ids_[id]] = empty_cand_vector;
                    }
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

        delete tf_listener_;
    }

protected:

    /** \brief Take arena limits from config file
    \param conf_file Name of configuration file 
    */
    void setArenaLimits(string conf_file)
    {
        YAML::Node yaml_config = YAML::LoadFile(conf_file);
        
        if(yaml_config["arena"])
        {
            x_min_ = yaml_config["arena"]["x_min"].as<float>();
            x_max_ = yaml_config["arena"]["x_max"].as<float>();
            y_min_ = yaml_config["arena"]["y_min"].as<float>();
            y_max_ = yaml_config["arena"]["y_max"].as<float>();

            arena_limits_ = true;
        }
    }

    /** \brief Callback to receive object detections
    */
    void updateCallback(const mbzirc_comm_objs::ObjectDetectionListConstPtr& msg) 
    {
        double delay = (ros::Time::now() - msg->stamp).toSec();

        string uav = msg->agent_id;

        if(msg->objects.size() && delay < delay_max_ && find(uav_ids_.begin(), uav_ids_.end(), uav) != uav_ids_.end())
        {
            int detector_type = msg->objects[0].type;   

            // Remove existing candidates
            if(candidates_[detector_type][uav].size())
            {
                for(int j = 0; j < candidates_[detector_type][uav].size(); j++)
                {
                    delete candidates_[detector_type][uav][j];
                }
                candidates_[detector_type][uav].clear();
            }

            // Store received candidates
            for(int j = 0; j < msg->objects.size(); j++)
            {
                ObjectDetection* detection_p = new ObjectDetection();
                *detection_p = msg->objects[j];

                // Transform to arena
                if (detection_p->header.frame_id != "arena") 
                {
                    TransformStamped transformToArena;
                    string pose_frame = detection_p->header.frame_id;

                    try
                    {
                        transformToArena = tf_buffer_.lookupTransform("arena", pose_frame, ros::Time(0), ros::Duration(1.0));
                    }
                    catch (tf2::TransformException ex)
                    {
                        ROS_ERROR("%s",ex.what());
                        ros::Duration(1.0).sleep();
                    }

                    tf2::doTransform(detection_p->pose.pose, detection_p->pose.pose, transformToArena);
                    detection_p->header.frame_id = "arena";
                }

                if(!arena_limits_ || (x_min_ <= detection_p->pose.pose.position.x && detection_p->pose.pose.position.x <= x_max_ 
                && y_min_ <= detection_p->pose.pose.position.y && detection_p->pose.pose.position.y <= y_max_ ) )
                    
                    candidates_[detector_type][uav].push_back(detection_p);
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

            // Update objects for all associated detectors
            for(auto det_it = detectors_[obj_type].begin(); det_it != detectors_[obj_type].end(); ++det_it)
            {
                for(auto cand_it = candidates_[*det_it].begin(); cand_it != candidates_[*det_it].end(); ++cand_it)
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
            }
            
            est_ptr->computeSubtypesTargets();
            est_ptr->removeLostTargets();
        
            publishObjects(obj_type);
            if(iteration_counter_ == 5)
            {
                //est_ptr->printTargetsInfo();
            }
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
            case mbzirc_comm_objs::SetObjectStatus::Request::INACTIVE:
            object_status = INACTIVE;
            break;
            case mbzirc_comm_objs::SetObjectStatus::Request::ACTIVE:
            object_status = ACTIVE;
            break;
            case mbzirc_comm_objs::SetObjectStatus::Request::DETECTED:
            object_status = DETECTED;
            break;
            case mbzirc_comm_objs::SetObjectStatus::Request::LOST:
            object_status = LOST;
            break;
            default:
            ROS_ERROR("Not valid object status for assignment.");
        }	

        return estimators_[req.object_type]->setTargetStatus(req.object_id, object_status);
    }

    int obj_type_from_string(const string& type, vector<int> &detectors) 
    {

        int obj_type;
        if(type == "pile")
        {
            obj_type = mbzirc_comm_objs::Object::TYPE_PILE;
            detectors.push_back(mbzirc_comm_objs::ObjectDetection::TYPE_BRICK);
        }
        else if(type == "wall")
        {
            obj_type = mbzirc_comm_objs::Object::TYPE_WALL;
            detectors.push_back(mbzirc_comm_objs::ObjectDetection::TYPE_UCHANNEL);
            detectors.push_back(mbzirc_comm_objs::ObjectDetection::TYPE_LWALL);
        }
        else if(type == "fire")
        {
            obj_type = mbzirc_comm_objs::Object::TYPE_FIRE;
            detectors.push_back(mbzirc_comm_objs::ObjectDetection::TYPE_FIRE);
        }
        else if(type == "passage")
        {
            obj_type = mbzirc_comm_objs::Object::TYPE_PASSAGE;
            detectors.push_back(mbzirc_comm_objs::ObjectDetection::TYPE_PASSAGE);
        }
        else
        {
            obj_type = -1;
            ROS_ERROR("Unknown object type %s", type.c_str());
        }

        return obj_type;
    }

    /** \brief Publish object beliefs
    \param obj_type Object type
    */
    void publishObjects(int obj_type)
    {
        ObjectList list;
        vector<vector<double> > covariances;
        vector<double> position;
        double obj_yaw;
        vector<double> scale;
        ObjectStatus target_status;
        int target_color, target_subtype;

        // Get ids to plot active targets
        vector<int> active_targets = estimators_[obj_type]->getActiveTargets();

        list.stamp = ros::Time::now();

        for(int i = 0; i < active_targets.size(); i++)
        {
            if(estimators_[obj_type]->getTargetInfo(active_targets[i], position, obj_yaw, covariances))
            {
                estimators_[obj_type]->getTargetInfo(active_targets[i], position, scale, target_status, target_color, target_subtype);

                Object object;

                object.header.stamp = ros::Time::now();
                object.header.frame_id = "arena";  
                object.type = obj_type;
                object.sub_type = target_subtype;
                object.id = active_targets[i];
                object.color = target_color;
                object.scale.x = scale[0];
                object.scale.y = scale[1];
                object.scale.z = scale[2];
                object.pose.pose.position.x = position[0];
                object.pose.pose.position.y = position[1];
                object.pose.pose.position.z = position[2];
                tf2::Quaternion q;
                q.setRPY(0.0,0.0,obj_yaw);
                object.pose.pose.orientation = tf2::toMsg(q);
                
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

    
    /// Ids of UAVs to connect
    int n_uavs_;
    vector<string> uav_ids_;
    
    int iteration_counter_;
    double delay_max_;          /// Maximum delay allowed for object detections

    /// Arena limits
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;
    bool arena_limits_;

    /// Timer, publishers and subscribers
    ros::Timer estimation_timer_;
    ros::Publisher objects_pub_;
    ros::ServiceServer set_object_status_srv_;
    vector<ros::Subscriber> sensed_sub_;

    /// TF transforms
    map<string, geometry_msgs::TransformStamped> cached_transforms_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener* tf_listener_;

    /// Candidates queues with object detections, one per detector and UAV
	map<int, map<string, vector<ObjectDetection *> > > candidates_;

    /// Centralized filter for each type of object 
	map<int, CentralizedEstimator*> estimators_;

    /// Detector types for each type of object 
	map<int, vector<int> > detectors_;
	
};


int main(int argc, char** argv) {

    ros::init(argc, argv, "estimation_node");
     
    Estimator estimator;
    
    ros::spin();

    return 0;
}
