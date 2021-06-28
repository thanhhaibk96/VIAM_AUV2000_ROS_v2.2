#ifndef IMU_PLUGIN_HH
#define IMU_PLUGIN_HH

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/sensors/GaussianNoiseModel.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <string>

namespace gazebo
{
namespace sensors
{
class ImuSensor;
}

class GazeboRosImuSensor : public SensorPlugin
{
public:
  GazeboRosImuSensor();
  virtual ~GazeboRosImuSensor();

  virtual void Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);

protected:
  void OnUpdate();

private:
  bool LoadParameters();

  ros::NodeHandle* node;
  ros::Publisher imu_data_publisher;
  sensor_msgs::Imu imu_msg;

  common::Time last_time;
  gazebo::event::ConnectionPtr connection;
  sensors::ImuSensor* sensor;
  sdf::ElementPtr sdf;

  std::string robot_namespace;
  std::string topic_name;
  std::string body_name;
  double accel_offset;
  double accel_drift;
  double accel_drift_frequency;
  double accel_gaussian_noise;
  double rate_offset;
  double rate_drift;
  double rate_drift_frequency;
  double rate_gaussian_noise;

  ignition::math::Vector3d currAccelDrift;
  ignition::math::Vector3d currRateDrift;
  ignition::math::Vector3d currAccelError;
  ignition::math::Vector3d currRateError;

  double NoiseVariance(const gazebo::sensors::Noise& _noise);
  double NoiseVariance(const gazebo::sensors::NoisePtr& _noise_ptr);
  double GaussianKernel(double mu, double sigma);
};
}

#endif // IMU_PLUGIN_HH
