#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include <mbzirc_gazebo_plugins/SimDroplet.h>
#include <ros/ros.h>
#include <thread>

namespace gazebo {
class DropletFactory: public WorldPlugin {

  public: ~DropletFactory() {
    if (rosnode_) {
      rosnode_->shutdown();
      delete rosnode_;
    }
  }

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/) {
    // World pointer!
    parent_ =  _parent;

    if (!ros::isInitialized()) {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc, argv, "droplet_factory", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    rosnode_ = new ros::NodeHandle();
    spawn_sub_ = rosnode_->subscribe("/spawn_droplet", 10, &DropletFactory::SpawnDroplet, this);

    // Listen to the update event. This event is broadcast every simulation iteration
    update_connection_ = event::Events::ConnectRender(
      boost::bind(&DropletFactory::UpdateChild, this));
  }

  public: void UpdateChild() {
    ros::spinOnce();
  }

  public: void SpawnDroplet(const mbzirc_gazebo_plugins::SimDropletConstPtr &_droplet) {
    math::Pose pose = math::Pose::Zero;
    pose.pos.x = _droplet->position.x;
    pose.pos.y = _droplet->position.y;
    pose.pos.z = _droplet->position.z;
    math::Vector3 velocity;
    velocity.x = _droplet->velocity.x;
    velocity.y = _droplet->velocity.y;
    velocity.z = _droplet->velocity.z;

    sdf::SDF sphereSDF;
    sphereSDF.SetFromString(
      "<sdf version ='1.5'>\
        <model name ='sphere'>\
          <pose> 0 0 1000 0 0 0</pose>\
          <link name ='link'>\
            <collision name ='collision'>\
              <geometry>\
                <sphere><radius>0.01</radius></sphere>\
              </geometry>\
            </collision>\
            <visual name ='visual'>\
              <geometry>\
                <sphere><radius>0.01</radius></sphere>\
              </geometry>\
              <material>\
                <ambient>0 0 1 0.5</ambient>\
                <diffuse>0 0 1 0.5</diffuse>\
                <specular>0 0 0 0</specular>\
                <emissive>0 0 0 0</emissive>\
              </material>\
            </visual>\
            <inertial>\
              <mass>0.005</mass>\
              <inertia>\
                <ixx>2e-7</ixx>\
                <ixy>0</ixy>\
                <ixz>0</ixz>\
                <iyy>2e-7</iyy>\
                <iyz>0</iyz>\
                <izz>2e-7</izz>\
              </inertia>\
            </inertial>\
          </link>\
        </model>\
      </sdf>");
    sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
    std::string model_name = "droplet" + std::to_string(counter_++);
    model->GetAttribute("name")->SetFromString(model_name);
    parent_->InsertModelSDF(sphereSDF);

    int iter = 0;
    physics::ModelPtr model_ptr = parent_->GetModel(model_name);
    while (!model_ptr) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      model_ptr = parent_->GetModel(model_name);
      iter++;
    }
    printf("Got model after %d iterations!\n", iter);
    std::cout.flush();
    model_ptr->SetLinkWorldPose(pose, "link");
    model_ptr->SetLinearVel(velocity);
    model_ptr->Update();
  }

protected:
  ros::NodeHandle* rosnode_ = nullptr;
  ros::Subscriber spawn_sub_;
  event::ConnectionPtr update_connection_;
  physics::WorldPtr parent_;
  unsigned int counter_ = 0;
};

GZ_REGISTER_WORLD_PLUGIN(DropletFactory)

}