#ifndef MAGNET_PLUGIN_HH
#define MAGNET_PLUGIN_HH

#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/sensors/GaussianNoiseModel.hh>
#include <gazebo/sensors/MagnetometerSensor.hh>
#include <ignition/math/Vector3.hh>

#include <string>

namespace gazebo
{
namespace sensors
{
class MagnetSensor;
}

class GazeboRosMagnetSensor : public SensorPlugin
{
public:
  GazeboRosMagnetSensor();
  virtual ~GazeboRosMagnetSensor();

  virtual void Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);

protected:
  void OnUpdate();

private:
  bool LoadParameters();

  ros::NodeHandle* node;
  ros::Publisher magnet_data_publisher;
  sensor_msgs::MagneticField magnet_msg;

  common::Time last_time;
  gazebo::event::ConnectionPtr connection;
  sensors::MagnetometerSensor* sensor;
  sdf::ElementPtr sdf;

  std::string robot_namespace;
  std::string topic_name;
  std::string body_name;
  double offset;
  double drift;
  double drift_frequency;
  double gaussian_noise;

  ignition::math::Vector3d currDrift;
  ignition::math::Vector3d currError;

  double NoiseVariance(const gazebo::sensors::Noise& _noise);
  double NoiseVariance(const gazebo::sensors::NoisePtr& _noise_ptr);
  double GaussianKernel(double mu, double sigma);
};
}
#endif // MAGNET_PLUGIN_HH
