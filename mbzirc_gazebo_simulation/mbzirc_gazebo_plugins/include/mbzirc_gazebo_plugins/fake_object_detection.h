#ifndef GAZEBO_ROS_RECOGNITION_SENSOR_HH
#define GAZEBO_ROS_RECOGNITION_SENSOR_HH

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <mbzirc_comm_objs/ObjectDetection.h>
#include <mbzirc_comm_objs/ObjectDetectionList.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <string>
#include <cstdlib>
#include <sstream>
#include <set>
#include <algorithm>
#include <assert.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>

namespace gazebo
{

  class FakeObjectDetection : public RayPlugin
  {

    public:
      FakeObjectDetection();
      FakeObjectDetection(physics::ModelPtr _model);
      virtual ~FakeObjectDetection();

    protected:
      virtual void OnNewLaserScans();

    private:
      virtual void Init();
      virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

      // Keep track of number of connctions
      int laser_connect_count_;
      void LaserConnect();
      void LaserDisconnect();

      // Publish Recognition data to the ROS topic
      void PutRecData(common::Time &_updateTime);

      physics::WorldPtr world_;
      physics::LinkPtr parent_link_;
      common::Time last_update_time_;

      sensors::SensorPtr parent_sensor_;
      sensors::RaySensorPtr parent_ray_sensor_;

      std::unique_ptr<ros::NodeHandle> rosnode_;
      ros::Publisher pub_;
      mbzirc_comm_objs::ObjectDetectionList rec_list;

      std::string topic_name_;
      std::string frame_name_;
      std::string robot_namespace_;

      boost::mutex lock;
      double update_rate_;

      ros::CallbackQueue laser_queue_;
      void LaserQueueThread();
      boost::thread callback_laser_queue_thread_;

      transport::NodePtr node_;
  };

}

#endif
