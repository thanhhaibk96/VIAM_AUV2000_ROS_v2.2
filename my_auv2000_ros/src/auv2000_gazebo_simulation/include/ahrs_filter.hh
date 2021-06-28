#ifndef AHRS_FILTER_HH
#define AHRS_FILTER_HH

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <utils/Odometry.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <ignition/math/Vector3.hh>

#include "geo.h"

using namespace message_filters;
using namespace utils;
using namespace sensor_msgs;

namespace gazebo
{
class AHRSFilter : public ModelPlugin
{
  typedef sync_policies::ApproximateTime<Imu, MagneticField> SynINSPolicy;

public:
  AHRSFilter();
  ~AHRSFilter();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  void OnUpdate();

private:
  double ned_lat;
  double ned_lon;

  ros::NodeHandle* node;
  ros::Publisher pubOdom;
  ros::Subscriber subGPS;
  Subscriber<Imu>* subIMU;
  Subscriber<MagneticField>* subMagnet;
  Synchronizer<SynINSPolicy>* synINS;

  gazebo::event::ConnectionPtr connection;
  physics::WorldPtr world;
  physics::ModelPtr model;
  physics::LinkPtr link;
  sdf::ElementPtr sdf;

  std::string robot_namespace;
  std::string topic_name;
  std::string body_name;

  Odometry odom_msg;

  void onGPSCallBack(const NavSatFix::ConstPtr& msg);
  void onSyncINSCallBack(const Imu::ConstPtr& msg1, const MagneticField::ConstPtr& msg2);
  bool LoadParameters();
};
}

#endif // AHRS_FILTER_HH
