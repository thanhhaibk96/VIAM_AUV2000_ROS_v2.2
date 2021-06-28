#ifndef GPSINSDVL_FILTER_HH
#define GPSINSDVL_FILTER_HH

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <utils/Odometry.h>

#include <mavros_msgs/CommandLong.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <ignition/math/Vector3.hh>

#include "geo.h"

using namespace message_filters;
using namespace utils ;
using namespace sensor_msgs;
using namespace geometry_msgs;

namespace gazebo
{
class GPSINSDVLFilter : public ModelPlugin
{
  typedef sync_policies::ApproximateTime<Imu, MagneticField> SynINSPolicy;

public:
  GPSINSDVLFilter();
  ~GPSINSDVLFilter();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  void OnUpdate();

private:
  double ned_lat;
  double ned_lon;

  ros::NodeHandle* node;
  ros::Publisher pubOdom;
  ros::Subscriber subGPS;
  ros::Subscriber subDVL;
  Subscriber<Imu>* subIMU;
  Subscriber<MagneticField>* subMagnet;
  Synchronizer<SynINSPolicy>* synIns;

  gazebo::event::ConnectionPtr connection;
  physics::WorldPtr world;
  physics::ModelPtr model;
  physics::LinkPtr link;
  sdf::ElementPtr sdf;

  std::string robot_namespace;
  std::string topic_name;
  std::string body_name;

  Odometry odom_msg;
  double tmp_lat;
  double tmp_lon;
  double tmp_alt;
  Vector3 tmp_linear_vel;
  Vector3 last_angular_vel;
  common::Time last_ins_time;

  void onGPSCallBack(const NavSatFix::ConstPtr& msg);
  void onDVLCallBack(const Vector3Stamped::ConstPtr& msg);
  void onSyncInsCallBack(const Imu::ConstPtr& msg1, const MagneticField::ConstPtr& msg2);
  bool LoadParameters();
};
} // namespace gazebo

#endif // GPSINSDVL_FILTER_HH
