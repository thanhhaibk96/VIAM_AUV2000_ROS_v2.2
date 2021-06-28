#ifndef PRESSURE_SENSOR_PLUGIN_HH
#define PRESSURE_SENSOR_PLUGIN_HH

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/sensors/GaussianNoiseModel.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include <string>
#include <utils/keller_pa3_stamped.h>
#include <utils/board_arm2_stamped.h>
using namespace utils ;
namespace gazebo
{
namespace sensors
{
class PressureSensor;
}

class GazeboRosPressureSensor : public ModelPlugin
{
public:
  GazeboRosPressureSensor();
  virtual ~GazeboRosPressureSensor();

  virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  void OnUpdate();

private:

  physics::WorldPtr world;
  physics::ModelPtr model;

/// \brief Pointer to model link in gazebo,
/// optionally specified by the bodyName parameter.
/// The states are taken from this link and forces applied to this link.
  physics::LinkPtr link;
  ros::NodeHandle* node;
  ros::Publisher pressure_data_publisher;


  utils::keller_pa3_stamped pressure_msg;

  common::Time last_time;
  gazebo::event::ConnectionPtr updateConnection;

  sdf::ElementPtr sdf;

  std::string robot_namespace;
  std::string topic_name;
  std::string body_name;

  double saturation ;
  double standard_pressure ;
  double bar_per_meter ;
  int    update_rate ;
  double gaussianNoise;
  double GaussianKernel(double mu, double sigma);
};
}

#endif // PRESSURE_SENSOR_PLUGIN_HH
