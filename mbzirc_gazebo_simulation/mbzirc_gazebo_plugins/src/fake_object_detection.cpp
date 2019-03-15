#include "mbzirc_gazebo_plugins/fake_object_detection.h"

namespace gazebo
{

GZ_REGISTER_SENSOR_PLUGIN(FakeObjectDetection)

std::string type_from_name(const std::string &link_name);

FakeObjectDetection::FakeObjectDetection()
{
}

FakeObjectDetection::FakeObjectDetection(gazebo::physics::ModelPtr _model)
{
}

FakeObjectDetection::~FakeObjectDetection()
{
  this->laser_queue_.clear();
  this->laser_queue_.disable();
  this->rosnode_->shutdown();
  this->callback_laser_queue_thread_.join();
}

void FakeObjectDetection::Init()
{
}

void FakeObjectDetection::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  gzmsg << "Loading fake object detection plugin" << std::endl;

  // load plugin
  RayPlugin::Load(_parent, _sdf);

  // Get simulation objects
  this->parent_sensor_ = _parent;
  std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);
  this->last_update_time_ = this->world_->SimTime();

  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init(worldName);

  this->parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(this->parent_sensor_);

  if (!this->parent_ray_sensor_)
    gzthrow("fake object detection plugin requires a Ray Sensor as its parent");

  this->parent_link_ =
    boost::dynamic_pointer_cast<physics::Link>(
    this->world_->EntityByName(this->parent_sensor_->ParentName()));

  if (!this->parent_link_)
      gzthrow("fake object detection: error loading plugin, sensor parent can not be found");

  this->frame_name_ = this->parent_link_->GetName();

  // Get parameters
  std::string robot_name = "";
  sdf::ElementPtr robotName = _sdf->GetElement("robot_name");
  if (!robotName)
  {
    gzmsg << "fake object detection: Using default robot_name " <<
          robot_name <<
          " because no <robot_name> element specified." <<
          std::endl;
  }
  else
  {
    robot_name = robotName->Get<std::string>()  + "/";
    gzmsg << "fake object detection: Using robot_name " <<
          robot_name << std::endl;
  }

  this->topic_name_ = "sensed_objects";
  sdf::ElementPtr topicName = _sdf->GetElement("topic_name");
  if (!topicName)
  {
    gzmsg << "fake object detection: Using default topicName " <<
          this->topic_name_ <<
          " because no <topicName> element specified." <<
          std::endl;
  }
  else
  {
    this->topic_name_ = topicName->Get<std::string>();
    gzmsg << "fake object detection: Using topicName " <<
          this->topic_name_ << std::endl;
  }

  this->laser_connect_count_ = 0;

  // Initialize ROS stuff

  if (!ros::isInitialized())
  {
    gzerr << "ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  this->rosnode_.reset(new ros::NodeHandle(robot_name));

  // resolve tf prefix
  //TODO: works?
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<mbzirc_comm_objs::ObjectDetectionList>(
      this->topic_name_,1,
      boost::bind( &FakeObjectDetection::LaserConnect,this),
      boost::bind( &FakeObjectDetection::LaserDisconnect,this), ros::VoidPtr(), &this->laser_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
  }

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
  this->callback_laser_queue_thread_ = boost::thread( boost::bind( &FakeObjectDetection::LaserQueueThread,this ) );

  gzmsg << "Loaded fake object detection plugin" << std::endl;
}

// Increment count
void FakeObjectDetection::LaserConnect()
{
  this->laser_connect_count_++;
  this->parent_ray_sensor_->SetActive(true);
}

// Decrement count
void FakeObjectDetection::LaserDisconnect()
{
  this->laser_connect_count_--;

  if (this->laser_connect_count_ == 0)
    this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void FakeObjectDetection::OnNewLaserScans()
{
  if (this->topic_name_ != "")
  {
    common::Time sensor_update_time = this->parent_sensor_->LastUpdateTime();
    if (this->last_update_time_ < sensor_update_time)
    {
      this->PutRecData(sensor_update_time);
      this->last_update_time_ = sensor_update_time;
    }
  }
  else
  {
    ROS_INFO("gazebo_ros_recognition_sensor topic name not set");
  }
}

// Publish Object Detection data
void FakeObjectDetection::PutRecData(common::Time &_updateTime)
{
  // Clear the ObjectDetectionList
  rec_list.objects.clear();

  physics::ShapePtr shape_ptr;
  //sdf::ElementPtr sdf_element;
  ignition::math::Vector3d scale;
  ignition::math::Pose3d link_pose;
  ignition::math::Pose3d sensor_frame_pose = this->parent_link_->WorldPose();

  mbzirc_comm_objs::ObjectDetection rec_object;
  rec_object.header.frame_id = this->frame_name_;
  rec_object.header.stamp.sec = _updateTime.sec;
  rec_object.header.stamp.nsec = _updateTime.nsec;


  std::string collision_name;
  double distance = 0;
  std::set<std::string> collisions;

  this->parent_ray_sensor_->SetActive(false);

  double maxRange = this->parent_ray_sensor_->RangeMax();
  double minRange = this->parent_ray_sensor_->RangeMin();

  int rayCount = this->parent_ray_sensor_->RayCount();
  int verticalRayCount = this->parent_ray_sensor_->VerticalRayCount();

  //  Point scan from laser
  boost::mutex::scoped_lock sclock(this->lock);

  // Get Intersection for every ray, get the collision and add it to the list.
  for (int i = 0; i < (verticalRayCount * rayCount - 1); i++)
  {
      if(i >= this->parent_ray_sensor_->LaserShape()->RayCount()) {
           gzthrow("Trying to access more rays than defined in the sensor!!");
      }

      this->parent_ray_sensor_->LaserShape()->Ray(i)->GetIntersection(distance, collision_name);

      // If distance is between min range and max range add it to the list
      if(collision_name != "" && distance >= minRange && distance <= maxRange)
          collisions.insert(collision_name);
  }

  // For every collision in the list create an ObjectDetection msg and add it to the ObjectDetectionList msg
  for(std::set<std::string>::iterator it = collisions.begin(); it != collisions.end(); it++)
  {
      std::string collision_name = *it;
      physics::CollisionPtr collision =
        boost::dynamic_pointer_cast<physics::Collision>(
          this->world_->EntityByName(collision_name));

      //Compose object detection message
      if(collision)
      {
          //Type
          rec_object.type = type_from_name(collision->GetLink()->GetName());

          //Pose
          link_pose = collision->GetLink()->WorldPose();
          link_pose = sensor_frame_pose.Inverse() * link_pose;

          rec_object.pose.pose.position.x = link_pose.Pos().X();
          rec_object.pose.pose.position.y = link_pose.Pos().Y();
          rec_object.pose.pose.position.z = link_pose.Pos().Z();
          rec_object.pose.pose.orientation.w = link_pose.Rot().W();
          rec_object.pose.pose.orientation.x = link_pose.Rot().X();
          rec_object.pose.pose.orientation.y = link_pose.Rot().Y();
          rec_object.pose.pose.orientation.z = link_pose.Rot().Z();

          for(u_int i = 0; i < rec_object.pose.covariance.size(); i++)
            rec_object.pose.covariance[i] = 0;

          rec_object.pose.covariance[0] = 0.01;
          rec_object.pose.covariance[7] = 0.01;
          rec_object.pose.covariance[14] = 0.01;

          // Scale
          rec_object.scale.x = 1;
          rec_object.scale.y = 1;
          rec_object.scale.z = 1;

          shape_ptr = collision->GetShape();
          if (shape_ptr)
          {
            rec_object.scale.x = shape_ptr->Scale().X();
            rec_object.scale.y = shape_ptr->Scale().Y();
            rec_object.scale.z = shape_ptr->Scale().Z();
          }

          //Other properties. Color.
          /*sdf_element = shape_ptr->GetSDF();
          if (sdf_element->HasAttribute("scale"))
          {
            scale = sdf_element->GetValueVector3("scale");
          }*/

          // Add the message to the list
          this->rec_list.objects.push_back(rec_object);
      }
      else
        gzmsg << "Collision object for collision name " << collision_name <<
          " could not be found" << std::endl;
  }

  // Publish the message

  this->parent_ray_sensor_->SetActive(true);
  this->pub_.publish(this->rec_list);
}


// Custom Callback Queue
void FakeObjectDetection::LaserQueueThread()
{
  static const double timeout = 0.01;
  while (this->rosnode_->ok())
  {
    this->laser_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// Infer an object type from a link name. Expected convention is name = id.type
std::string type_from_name(const std::string &link_name)
{
  if(std::count(link_name.begin(), link_name.end(), '.') != 1)
    return "unknown";
  else
    return link_name.substr(link_name.find_first_of(".")+1,std::string::npos);
}

}
