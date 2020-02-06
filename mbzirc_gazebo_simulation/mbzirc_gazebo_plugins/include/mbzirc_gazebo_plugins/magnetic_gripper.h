#ifndef GAZEBO_MagneticGripper_H
#define GAZEBO_MagneticGripper_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "mbzirc_comm_objs/Magnetize.h"
#include "mbzirc_comm_objs/GripperAttached.h"

#include <stdio.h>
#include <thread>
#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>


class MagneticGripper : public gazebo::ModelPlugin
{
  public:
    MagneticGripper();
    MagneticGripper(gazebo::physics::ModelPtr _model);
    virtual ~MagneticGripper();

  private:
    virtual void Init();
    virtual void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void OnUpdate();
    void InitValues();
    void OnContact(const ConstContactsPtr &ptr);
    bool magnetize(mbzirc_comm_objs::Magnetize::Request& request, mbzirc_comm_objs::Magnetize::Response& response);

    gazebo::physics::WorldPtr world;
    gazebo::physics::JointPtr fixedJoint;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::ServiceServer magnet_service;
    ros::Publisher attached_pub;

    void QueueThread();
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;

    gazebo::event::ConnectionPtr update_connection;
    gazebo::transport::NodePtr node;
    gazebo::transport::SubscriberPtr contactSub; //subscriber to contact updates

    gazebo::common::Time updateRate;
    gazebo::common::Time prevUpdateTime;

    bool isMagnetized;
    mbzirc_comm_objs::GripperAttached msg;
    int pub_msg_decimate_index = 0;
};

#endif  // GAZEBO_MagneticGripper_H
