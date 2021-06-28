#ifndef GPS_PLUGIN_HH
#define GPS_PLUGIN_HH

#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <ignition/math/Vector3.hh>

#include <string>

#include "geo.h"

namespace gazebo
{
  namespace sensors
  {
    class GpsSensor;
  }

  class GazeboRosGpsSensor : public SensorPlugin
  {
  public:
    GazeboRosGpsSensor();
    virtual ~GazeboRosGpsSensor();

    virtual void Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);

  protected:
    void OnUpdate();

  private:
    bool LoadParameters();

    ros::NodeHandle* node;
    ros::Publisher gps_data_publisher;
    ros::Publisher gps_vel_data_publisher;
    sensor_msgs::NavSatFix gps_msg;
    geometry_msgs::Vector3Stamped vel_gps_msg;

    common::Time last_time;
    gazebo::event::ConnectionPtr connection;
    sensors::GpsSensor* sensor;
    sdf::ElementPtr sdf;

    std::string robot_namespace;
    std::string topic_name;
    std::string velocity_topic_name;
    std::string body_name;
    double offset;
    double drift;
    double drift_frequency;
    double gaussian_noise;
    double velocity_offset;
    double velocity_drift;
    double velocity_drift_frequency;
    double velocity_gaussian_noise;

    ignition::math::Vector3d currDrift;
    ignition::math::Vector3d currVelDrift;
    ignition::math::Vector3d currError;
    ignition::math::Vector3d currVelError;

    double NoiseVariance(const gazebo::sensors::Noise & _noise);
    double NoiseVariance(const gazebo::sensors::NoisePtr & _noise_ptr);
    double GaussianKernel(double mu, double sigma);
  };
}

#endif // GPS_PLUGIN_HH
