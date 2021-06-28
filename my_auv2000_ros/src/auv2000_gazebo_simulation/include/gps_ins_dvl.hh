#ifndef GPSINSDVL_HH
#define GPSINSDVL_HH

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <utils/Odometry.h>
#include <auv_msgs_stamped/gps_dvl_ins_stamped.h>
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
using namespace auv_msgs_stamped;

namespace gazebo
{
class GPSINSDVL : public ModelPlugin
{
  typedef sync_policies::ApproximateTime<Imu, MagneticField> SynINSPolicy;

public:
  GPSINSDVL();
  ~GPSINSDVL();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  void OnUpdate();
public: int cnt =0;

private:
  double ned_lat;
  double ned_lon;

  ros::NodeHandle* node;
  ros::Publisher pubGpsDvlIns;
  ros::Subscriber subGPS;
  ros::Subscriber subDVL;
  ros::Subscriber subIMU;
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

  gps_dvl_ins_stamped gps_dvl_ins_msg;
  double tmp_lat;
  double tmp_lon;
  double tmp_alt;
  Vector3 tmp_linear_vel;
  Vector3 last_angular_vel;
  ignition::math::Vector3d orientation;
  common::Time last_ins_time;

  void onGPSCallBack(const NavSatFix::ConstPtr& msg);
  void onDVLCallBack(const Vector3Stamped::ConstPtr& msg);
  void onIMUCallBack(const sensor_msgs::Imu::ConstPtr& msg);
  void onSyncInsCallBack(const Imu::ConstPtr& msg1, const MagneticField::ConstPtr& msg2);
  ignition::math::Vector3d convertRotToRPY(double x, double y, double z, double w);
  bool LoadParameters();
};
} // namespace gazebo

#endif // GPSINSDVL_HH
