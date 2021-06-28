#ifndef DVL_PLUGIN_HH
#define DVL_PLUGIN_HH

#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <ignition/math/Vector3.hh>

#include <string>

namespace gazebo
{
class GazeboRosDvlSensor : public ModelPlugin
{
public:
  GazeboRosDvlSensor();
  ~GazeboRosDvlSensor();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  void OnUpdate();

private:
  bool LoadParameters();

  ros::NodeHandle* node;
  ros::Publisher dvl_data_publisher;
  geometry_msgs::Vector3Stamped dvl_msg;

  common::Time last_time;
  gazebo::event::ConnectionPtr connection;
  physics::WorldPtr world;
  physics::ModelPtr model;
  physics::LinkPtr link;
  sdf::ElementPtr sdf;

  std::default_random_engine rndGen;
  std::normal_distribution<double> velNoiseModel;

  std::string robot_namespace;
  double update_rate;
  std::string topic_name;
  std::string body_name;
  double vel_stddev;
};
}

#endif // DVL_PLUGIN_HH
