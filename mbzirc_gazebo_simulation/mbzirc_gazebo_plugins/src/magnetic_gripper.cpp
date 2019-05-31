#include <mbzirc_gazebo_plugins/magnetic_gripper.h>


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MagneticGripper)


MagneticGripper::MagneticGripper()
{
  InitValues();
}

MagneticGripper::MagneticGripper(gazebo::physics::ModelPtr _model)
{
  InitValues();
}

MagneticGripper::~MagneticGripper()
{
  this->update_connection.reset();
  if (this->node) this->node->Fini();
  this->node.reset();
}

void MagneticGripper::Init()
{
  this->prevUpdateTime = gazebo::common::Time::GetWallTime();
}

void MagneticGripper::InitValues()
{
  gazebo::common::Console::SetQuiet(false);
  this->prevUpdateTime = gazebo::common::Time::GetWallTime();
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
}

void MagneticGripper::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  gzmsg << "Loading magnetic gripper plugin" << std::endl;

  this->isMagnetized = false;

  gazebo::physics::ModelPtr model = _parent;
  this->world = model->GetWorld();
  #if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::PhysicsEnginePtr physics = this->world->Physics();
  #else
    gazebo::physics::PhysicsEnginePtr physics = this->world->GetPhysicsEngine();
  #endif

  this->fixedJoint = nullptr;

  std::string gripper_link_name = "gripper";
  std::string robot_name = "";

  sdf::ElementPtr gripperLinkNameElem =
    _sdf->GetElement("gripper_link_name");
  if (!gripperLinkNameElem)
  {
    gzmsg << "MagneticGripper: Using default " <<
          gripper_link_name <<
          " because no <gripper_link_name> element specified." <<
          std::endl;
  }
  else
  {
    gripper_link_name = gripperLinkNameElem->Get<std::string>();
    gzmsg << "MagneticGripper: Using gripper_link_name " <<
          gripper_link_name << std::endl;
  }

  sdf::ElementPtr agentNameElem =
    _sdf->GetElement("robot_name");
  if (!agentNameElem)
  {
    gzmsg << "MagneticGripper: Using default " <<
          robot_name <<
          " because no <robot_name> element specified." <<
          std::endl;
  }
  else
  {
    robot_name = agentNameElem->Get<std::string>();
    gzmsg << "MagneticGripper: Using robot_name " <<
          robot_name << std::endl;
  }

  gazebo::physics::LinkPtr link = model->GetLink(gripper_link_name);
  std::vector<std::string> collisionNames;

  if(!link)
  {
    gzerr << "Link " << gripper_link_name << " can not be found in model "
          << model->GetName() << ". Plugin can not be loaded." << std::endl;
    return;
  }

  for (unsigned int j = 0; j < link->GetChildCount(); ++j)
  {
    gazebo::physics::CollisionPtr collision = link->GetCollision(j);
    collisionNames.push_back(collision->GetScopedName());
  }

  #if GAZEBO_MAJOR_VERSION >= 8
    this->node->Init(this->world->Name());
  #else
    this->node->Init(this->world->GetName());
  #endif

  gazebo::physics::ContactManager *contactManager = physics->GetContactManager();
  contactManager->PublishContacts();

  std::string topic = contactManager->CreateFilter(model->GetScopedName(),
                      collisionNames);
  if (!this->contactSub)
  {
    gzmsg << "Subscribing contact manager to topic " << topic << std::endl;
    bool latching = false;
    this->contactSub = this->node->Subscribe(topic, &MagneticGripper::OnContact,
                       this, latching);
  }

  update_connection = gazebo::event::Events::ConnectWorldUpdateEnd(boost::bind(
                        &MagneticGripper::OnUpdate, this));


  //Initialize ROS stuff
  if (!ros::isInitialized())
  {
    gzerr << "ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  this->rosNode.reset(new ros::NodeHandle(robot_name));
  this->magnet_service = this->rosNode->advertiseService("magnetize", &MagneticGripper::magnetize, this);
  this->attached_pub = this->rosNode->advertise<mbzirc_comm_objs::GripperAttached>("attached",1);

  this->rosQueueThread =
  std::thread(std::bind(&MagneticGripper::QueueThread, this));

  gzmsg << "Loaded magnetic gripper plugin" << std::endl;
}

void MagneticGripper::OnUpdate() {
  if(!this->isMagnetized && this->fixedJoint) {
    this->fixedJoint->Detach();
    this->fixedJoint->Fini();
    this->fixedJoint = nullptr;
    gzmsg << "Detached" << std::endl;
    //advertise dettached
    mbzirc_comm_objs::GripperAttached msg;
    msg.attached = false;
    this->attached_pub.publish(msg);
  }
}

void MagneticGripper::OnContact(const ConstContactsPtr &contacts) {
  if(this->isMagnetized && !this->fixedJoint && contacts->contact_size()) {
    #if GAZEBO_MAJOR_VERSION >= 8
      gazebo::physics::CollisionPtr collision1 =
        boost::dynamic_pointer_cast<gazebo::physics::Collision>(
          this->world->EntityByName(contacts->contact(0).collision1()));
      gazebo::physics::CollisionPtr collision2 =
        boost::dynamic_pointer_cast<gazebo::physics::Collision>(
          this->world->EntityByName(contacts->contact(0).collision2()));
    #else
      gazebo::physics::CollisionPtr collision1 =
        boost::dynamic_pointer_cast<gazebo::physics::Collision>(
        this->world->GetEntity(contacts->contact(0).collision1()));
      gazebo::physics::CollisionPtr collision2 =
        boost::dynamic_pointer_cast<gazebo::physics::Collision>(
        this->world->GetEntity(contacts->contact(0).collision2()));
    #endif

    gzmsg << "Atached links: [" << collision1->GetLink()->GetName()
          << "] and [" << collision2->GetLink()->GetName() << "]\n";

  #if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d diff = collision1->GetLink()->WorldPose() -
                                  collision2->GetLink()->WorldPose();
    this->fixedJoint = this->world->Physics()->CreateJoint("fixed");
  #else
    gazebo::math::Pose diff = collision1->GetLink()->GetWorldPose() -
                              collision2->GetLink()->GetWorldPose();
    this->fixedJoint = this->world->GetPhysicsEngine()->CreateJoint("fixed");
  #endif


    this->fixedJoint->Load(collision1->GetLink(), collision2->GetLink(), diff);
    this->fixedJoint->Init();
    //advertise attached
    mbzirc_comm_objs::GripperAttached msg;
    msg.attached = true;
    this->attached_pub.publish(msg);
  }
}


bool MagneticGripper::magnetize(mbzirc_comm_objs::Magnetize::Request& request, mbzirc_comm_objs::Magnetize::Response& response) {
  this->isMagnetized = request.magnetize;
  response.success = true;
  return true;
}

void MagneticGripper::QueueThread() {
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
