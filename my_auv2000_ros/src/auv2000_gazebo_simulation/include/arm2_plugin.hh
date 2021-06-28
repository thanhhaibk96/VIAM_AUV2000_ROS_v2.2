#ifndef ARM2_HH
#define ARM2_HH

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandLong.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <ignition/math/Vector3.hh>
#include <utils/board_arm2_stamped.h>
#include "geo.h"

using namespace message_filters;
using namespace utils ;
using namespace sensor_msgs;
using namespace geometry_msgs;

namespace gazebo
{
class ARM2 : public ModelPlugin
{
  typedef sync_policies::ApproximateTime<Imu, MagneticField> SynINSPolicy;

public:
  ARM2();
  ~ARM2();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  void OnUpdate();
public: int cnt =0;

private:
  double ned_lat;
  double ned_lon;

  ros::NodeHandle* node;
  ros::Publisher pubArm2;
  ros::Subscriber subPressureSensor;
  ros::Subscriber subRudder;
  ros::Subscriber subAntiRolling;
  ros::Subscriber subBMS;

  gazebo::event::ConnectionPtr connection;
  physics::WorldPtr world;
  physics::ModelPtr model;
  physics::LinkPtr link;
  sdf::ElementPtr sdf;

  std::string robot_namespace;
  std::string topic_name;
  std::string body_name;

  board_arm2_stamped arm2_msg;

  common::Time last_ins_time;

 
  void onPressureSensorCallBack(const utils::keller_pa3_stamped::ConstPtr& msg);
  void onRudderCallBack(const utils::mx28_stamped::ConstPtr& msg);
  void onAntiRollingCallBack(const utils::anti_rolling_stamped::ConstPtr& msg);
  void onBMSCallBack(const utils::bms_stamped::ConstPtr& msg);

  bool LoadParameters();
};
} // namespace gazebo

#endif // GPSINSDVL_HH
