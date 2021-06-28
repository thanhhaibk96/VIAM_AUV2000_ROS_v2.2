#include "gps_ins_dvl.hh"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GPSINSDVL);

GPSINSDVL::GPSINSDVL() {}

GPSINSDVL::~GPSINSDVL()
{
  if (connection.get())
  {
    connection.reset();
    connection = gazebo::event::ConnectionPtr();
  }

  node->shutdown();
}

void GPSINSDVL::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  model = _parent;
  world = _parent->GetWorld();
  sdf = _sdf;

  if (!LoadParameters())
  {
    ROS_FATAL("Error Loading Parameters!");
    return;
  }

  node = new ros::NodeHandle(this->robot_namespace);
  pubGpsDvlIns = node->advertise<gps_dvl_ins_stamped>(topic_name, 10);
  subGPS = node->subscribe("gps/fix", 10, &GPSINSDVL::onGPSCallBack, this);
  subDVL = node->subscribe("dvl", 10, &GPSINSDVL::onDVLCallBack, this);
  subIMU = node->subscribe("ins/imu", 10, &GPSINSDVL::onIMUCallBack, this);
 
  last_ins_time = this->world->SimTime();

  connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GPSINSDVL::OnUpdate, this));
}

void GPSINSDVL::OnUpdate() 
{
 
cnt = cnt +1 ;
if (cnt == 1)
{
  pubGpsDvlIns.publish(gps_dvl_ins_msg);
  cnt = 0;
}
  
}

void GPSINSDVL::onGPSCallBack(const NavSatFix::ConstPtr& msg)
{
  gps_dvl_ins_msg.ekf_lat = msg->latitude;
  gps_dvl_ins_msg.ekf_lon = msg->longitude;
  gps_dvl_ins_msg.ekf_alt = msg->altitude;
}

void GPSINSDVL::onDVLCallBack(const Vector3Stamped::ConstPtr& msg) 
{ 
  gps_dvl_ins_msg.ekf_vX = msg->vector.x;
  gps_dvl_ins_msg.ekf_vY = msg->vector.y;
  gps_dvl_ins_msg.ekf_vZ = msg->vector.z;
}

void GPSINSDVL::onIMUCallBack(const sensor_msgs::Imu::ConstPtr& msg)
{ 
  common::Time curr_ins_time = this->world->SimTime();
  double dt = curr_ins_time.Double() - last_ins_time.Double();
  
	ignition::math::Vector3d RPY = convertRotToRPY(msg->orientation.x ,msg->orientation.y , msg->orientation.z, msg->orientation.w );

  gps_dvl_ins_msg.ekf_roll      =   RPY.X()*57.3;
  gps_dvl_ins_msg.ekf_pitch     = - RPY.Y()*57.3;
  gps_dvl_ins_msg.ekf_yaw       = - RPY.Z()*57.3;

  gps_dvl_ins_msg.rad_gyro_X    =   msg->angular_velocity.x;
  gps_dvl_ins_msg.rad_gyro_Y    = - msg->angular_velocity.y;
  gps_dvl_ins_msg.rad_gyro_Z    = - msg->angular_velocity.z;

  gps_dvl_ins_msg.imu_acc_X     =   msg->linear_acceleration.x;
  gps_dvl_ins_msg.imu_acc_Y     = - msg->linear_acceleration.y;
  gps_dvl_ins_msg.imu_acc_Z     = - msg->linear_acceleration.z;

  gps_dvl_ins_msg.angular_acc_X =   (msg->angular_velocity.x - last_angular_vel.x) / dt;
  gps_dvl_ins_msg.angular_acc_Y = - (msg->angular_velocity.y - last_angular_vel.y) / dt;
  gps_dvl_ins_msg.angular_acc_Z = - (msg->angular_velocity.z - last_angular_vel.z) / dt;

  last_angular_vel              = msg->angular_velocity ; 
  last_ins_time                 = curr_ins_time;
}

void GPSINSDVL::onSyncInsCallBack(const Imu::ConstPtr& msg1, const MagneticField::ConstPtr& msg2)
{
}

bool GPSINSDVL::LoadParameters()
{
  // loading parameters from the sdf file

  // NAMESPACE
  if (sdf->HasElement("robotNamespace"))
  {
    robot_namespace = sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("<robotNamespace> set to: " << robot_namespace);
  }

  // TOPIC
  if (sdf->HasElement("topicName"))
  {
    topic_name = sdf->Get<std::string>("topicName");
    ROS_INFO_STREAM("<topicName> set to: " << topic_name);
  }
  else
  {
    topic_name = "/ahrs";
    ROS_WARN_STREAM("missing <topicName>, set to /namespace/default: " << topic_name);
  }

  // BODY NAME
  if (sdf->HasElement("bodyName"))
  {
    body_name = sdf->GetElement("bodyName")->Get<std::string>();
    link = model->GetLink(body_name);
    ROS_INFO_STREAM("<bodyName> set to: " << body_name);
  }
  else
  {
    ROS_FATAL("missing <bodyName>, cannot proceed");
    return false;
  }

  // NED_LAT_LON
  if (sdf->HasElement("nedLat"))
  {
    ned_lat = sdf->GetElement("nedLat")->Get<double>();
  }
  else
  {
    ROS_FATAL("missing <nedLat>, cannot proceed");
    return false;
  }
  if (sdf->HasElement("nedLon"))
  {
    ned_lon = sdf->GetElement("nedLon")->Get<double>();
  }
  else
  {
    ROS_FATAL("missing <nedLon>, cannot proceed");
    return false;
  }

  return true;
}
ignition::math::Vector3d GPSINSDVL::convertRotToRPY(double x, double y, double z, double w)
{
	 ignition::math::Vector3d RPY;
	 double t0 = +2.0 * (w * x + y * z);
     double t1 = +1.0 - 2.0 * (x * x + y * y);
     RPY.X() = atan2(t0, t1);
   
     double t2 = +2.0 * (w * y - z * x);
     if (t2 > 1)
     {
     	t2 = 1; 
     }
     if (t2 < -1 )
     {
     	t2 = -1;
     }

     RPY.Y() = asin(t2);
     
     double t3 = +2.0 * (w * z + x * y);
     double t4 = +1.0 - 2.0 * (y * y + z * z);

     RPY.Z() = atan2(t3, t4);
     return RPY;
}
} // namespace gazebo
