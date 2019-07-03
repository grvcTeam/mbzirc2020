#include "mbzirc_gazebo_plugins/fake_object_detection.h"
#if GAZEBO_MAJOR_VERSION < 8
#include "mbzirc_gazebo_plugins/dirty_hacks.h"

//error: ‘std::vector<boost::shared_ptr<gazebo::physics::RayShape> > gazebo::physics::MultiRayShape::rays’ is protected
//       protected: std::vector<RayShapePtr> rays;
  ENABLE_ACCESS(rays, ::gazebo::physics::MultiRayShape, rays, std::vector<gazebo::physics::RayShapePtr>);
#endif

namespace gazebo
{

GZ_REGISTER_SENSOR_PLUGIN(FakeObjectDetection)

std::string type_from_name(const std::string &link_name);

#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Vector3d scaleFromShape(physics::ShapePtr shape_ptr);
#else
  gazebo::math::Vector3 scaleFromShape(physics::ShapePtr shape_ptr);
#endif

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

  #if GAZEBO_MAJOR_VERSION >= 8
    this->last_update_time_ = this->world_->SimTime();
  #else
    this->last_update_time_ = this->world_->GetSimTime();
  #endif

  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init(worldName);

  this->parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(this->parent_sensor_);

  if (!this->parent_ray_sensor_)
    gzthrow("fake object detection plugin requires a Ray Sensor as its parent");

  #if GAZEBO_MAJOR_VERSION >= 8
    this->parent_link_ =
      boost::dynamic_pointer_cast<physics::Link>(
      this->world_->EntityByName(this->parent_sensor_->ParentName()));
  #else
    this->parent_link_ =
      boost::dynamic_pointer_cast<physics::Link>(
      this->world_->GetEntity(this->parent_sensor_->ParentName()));
  #endif

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

  this->service_name_ = "set_types";
  sdf::ElementPtr serviceName = _sdf->GetElement("service_name");
  if (!serviceName)
  {
    gzmsg << "fake object detection: Using default serviceName " <<
          this->service_name_ <<
          " because no <serviceName> element specified." <<
          std::endl;
  }
  else
  {
    this->service_name_ = serviceName->Get<std::string>();
    gzmsg << "fake object detection: Using serviceName " <<
          this->service_name_ << std::endl;
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
  this->rosnode_->getParam("/"+robot_name+"/tf_prefix", prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<mbzirc_comm_objs::ObjectDetectionList>(
      this->topic_name_,1,
      boost::bind( &FakeObjectDetection::LaserConnect,this),
      boost::bind( &FakeObjectDetection::LaserDisconnect,this), ros::VoidPtr(), &this->laser_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
  }

  if (this->service_name_ != "")
    this->srv_ = this->rosnode_->advertiseService(this->service_name_, &FakeObjectDetection::ChangeTypesCB, this);

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
  this->callback_laser_queue_thread_ = boost::thread( boost::bind( &FakeObjectDetection::LaserQueueThread,this ) );

  gzmsg << "Loaded fake object detection plugin" << std::endl;
}

// Change Detection Types
bool FakeObjectDetection::ChangeTypesCB(mbzirc_comm_objs::DetectTypes::Request& req,
                          mbzirc_comm_objs::DetectTypes::Response &res)
{
  this->type_list = req.types;
  res.success = true;
  return true;
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

  #if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Vector3d scale;
    ignition::math::Pose3d link_pose;
    ignition::math::Pose3d sensor_frame_pose = this->parent_link_->WorldPose();
  #else
    gazebo::math::Vector3 scale;
    gazebo::math::Pose link_pose;
    gazebo::math::Pose sensor_frame_pose = this->parent_link_->GetWorldPose();
  #endif

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

  //  Point scan from laser
  boost::mutex::scoped_lock sclock(this->lock);

  // Get Intersection for every ray, get the collision and add it to the list.
  #if GAZEBO_MAJOR_VERSION >= 8
  int rayCount = this->parent_ray_sensor_->RayCount();
  int verticalRayCount = this->parent_ray_sensor_->VerticalRayCount();
  for (int i = 0; i < (verticalRayCount * rayCount - 1); i++)
  {
    if(i >= this->parent_ray_sensor_->LaserShape()->RayCount()) {
         gzthrow("Trying to access more rays than defined in the sensor!!");
    }

    this->parent_ray_sensor_->LaserShape()->Ray(i)->GetIntersection(distance, collision_name);
  #else
  std::vector<physics::RayShapePtr> rays = ACCESS(*this->parent_ray_sensor_->LaserShape(), rays);
  for (int i = 0; i < rays.size(); i++)
  {
      rays[i]->GetIntersection(distance, collision_name);
  #endif

      // If distance is between min range and max range add it to the list
      if(collision_name != "" && distance >= minRange && distance <= maxRange)
          collisions.insert(collision_name);
  }

  // For every collision in the list create an ObjectDetection msg and add it to the ObjectDetectionList msg
  for(std::set<std::string>::iterator it = collisions.begin(); it != collisions.end(); it++)
  {
      std::string collision_name = *it;

      #if GAZEBO_MAJOR_VERSION >= 8
        physics::CollisionPtr collision =
          boost::dynamic_pointer_cast<physics::Collision>(
            this->world_->EntityByName(collision_name));
      #else
        physics::CollisionPtr collision =
          boost::dynamic_pointer_cast<physics::Collision>(
            this->world_->GetEntity(collision_name));
      #endif

      //Compose object detection message
      if(collision)
      {
          //Type
          rec_object.type = type_from_name(collision_name);
          if(this->type_list.size() && std::find(this->type_list.begin(), this->type_list.end(), rec_object.type) == this->type_list.end())
            continue;

          //Pose
          #if GAZEBO_MAJOR_VERSION >= 8
          ignition::math::Pose3d link_pose_g = collision->GetLink()->WorldPose();
          link_pose = link_pose_g - sensor_frame_pose;

          /*if(rec_object.type == "brick") {
          std::cout << this->parent_link_->GetName() << std::endl;
          std::cout << sensor_frame_pose << std::endl;
          std::cout << link_pose_g << std::endl;
        std::cout << link_pose << std::endl;}*/

            rec_object.pose.pose.position.x = link_pose.Pos().X();
            rec_object.pose.pose.position.y = link_pose.Pos().Y();
            rec_object.pose.pose.position.z = link_pose.Pos().Z();
            rec_object.pose.pose.orientation.w = link_pose.Rot().W();
            rec_object.pose.pose.orientation.x = link_pose.Rot().X();
            rec_object.pose.pose.orientation.y = link_pose.Rot().Y();
            rec_object.pose.pose.orientation.z = link_pose.Rot().Z();
          #else
            gazebo::math::Pose link_pose_g = collision->GetLink()->GetWorldPose();
            link_pose = link_pose_g - sensor_frame_pose;

            rec_object.pose.pose.position.x = link_pose.pos.x;
            rec_object.pose.pose.position.y = link_pose.pos.y;
            rec_object.pose.pose.position.z = link_pose.pos.z;
            rec_object.pose.pose.orientation.w = link_pose.rot.w;
            rec_object.pose.pose.orientation.x = link_pose.rot.x;
            rec_object.pose.pose.orientation.y = link_pose.rot.y;
            rec_object.pose.pose.orientation.z = link_pose.rot.z;
          #endif

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
            #if GAZEBO_MAJOR_VERSION >= 8
            ignition::math::Vector3d scale = scaleFromShape(shape_ptr);
            rec_object.scale.x = scale.X();
            rec_object.scale.y = scale.Y();
            rec_object.scale.z = scale.Z();
            #else
            gazebo::math::Vector3 scale = scaleFromShape(shape_ptr);
            rec_object.scale.x = scale.x;
            rec_object.scale.y = scale.y;
            rec_object.scale.z = scale.z;
            #endif
          }
          else
            gzmsg << "Collision object " << collision_name << " has no shape" << std::endl;

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

// Infer an object type from a link name
std::string type_from_name(const std::string &link_name)
{
  if (link_name.find("brick") != std::string::npos)
    return "brick";

  if (link_name.find("fire") != std::string::npos)
    return "fire";

  return "unknown";
}

//
#if GAZEBO_MAJOR_VERSION >= 8
ignition::math::Vector3d scaleFromShape(physics::ShapePtr shape_ptr) {
  //Box case
  physics::BoxShapePtr box =
    boost::dynamic_pointer_cast<physics::BoxShape>(
      shape_ptr);
  if(box)
    return box->Size();
  //Cylinder case
  physics::CylinderShapePtr cylinder =
    boost::dynamic_pointer_cast<physics::CylinderShape>(
      shape_ptr);
  if(cylinder)
    return ignition::math::Vector3d(cylinder->GetRadius(),cylinder->GetRadius(),cylinder->GetLength());
  //Sphere case
  physics::SphereShapePtr sphere =
    boost::dynamic_pointer_cast<physics::SphereShape>(
      shape_ptr);
  if(sphere)
    return ignition::math::Vector3d(sphere->GetRadius(),sphere->GetRadius(),sphere->GetRadius());
  //Mesh case
  physics::MeshShapePtr mesh =
    boost::dynamic_pointer_cast<physics::MeshShape>(
      shape_ptr);
  if(mesh)
    return mesh->Size();
  //Plane case
  physics::PlaneShapePtr plane =
    boost::dynamic_pointer_cast<physics::PlaneShape>(
      shape_ptr);
  if(plane)
    return ignition::math::Vector3d(plane->Size().X(),plane->Size().Y(),1);

  return ignition::math::Vector3d(1,1,1);
}
#else
gazebo::math::Vector3 scaleFromShape(physics::ShapePtr shape_ptr) {
  //Box case
  physics::BoxShapePtr box =
    boost::dynamic_pointer_cast<physics::BoxShape>(
      shape_ptr);
  if(box)
    return box->GetSize();
  //Cylinder case
  physics::CylinderShapePtr cylinder =
    boost::dynamic_pointer_cast<physics::CylinderShape>(
      shape_ptr);
  if(cylinder)
    return gazebo::math::Vector3(cylinder->GetRadius(),cylinder->GetRadius(),cylinder->GetLength());
  //Sphere case
  physics::SphereShapePtr sphere =
    boost::dynamic_pointer_cast<physics::SphereShape>(
      shape_ptr);
  if(sphere)
    return gazebo::math::Vector3(sphere->GetRadius(),sphere->GetRadius(),sphere->GetRadius());
  //Mesh case
  physics::MeshShapePtr mesh =
    boost::dynamic_pointer_cast<physics::MeshShape>(
      shape_ptr);
  if(mesh)
    return mesh->GetSize();
  //Plane case
  physics::PlaneShapePtr plane =
    boost::dynamic_pointer_cast<physics::PlaneShape>(
      shape_ptr);
  if(plane)
    return gazebo::math::Vector3(plane->GetSize().x,plane->GetSize().y,1);

  return gazebo::math::Vector3(1,1,1);
}
#endif
}
