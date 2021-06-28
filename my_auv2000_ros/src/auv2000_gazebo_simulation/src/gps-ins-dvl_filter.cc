#include "gps-ins-dvl_filter.hh"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GPSINSDVLFilter);

GPSINSDVLFilter::GPSINSDVLFilter() {}

GPSINSDVLFilter::~GPSINSDVLFilter()
{
  if (connection.get())
  {
    connection.reset();
    connection = gazebo::event::ConnectionPtr();
  }

  node->shutdown();
}

void GPSINSDVLFilter::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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
  pubOdom = node->advertise<Odometry>(topic_name, 10);
  subGPS = node->subscribe("gps/fix", 10, &GPSINSDVLFilter::onGPSCallBack, this);
  subDVL = node->subscribe("dvl", 10, &GPSINSDVLFilter::onDVLCallBack, this);
  subIMU = new Subscriber<Imu>(*node, "ins/imu", 10);
  subMagnet = new Subscriber<MagneticField>(*node, "ins/magnet", 10);
  synIns = new Synchronizer<SynINSPolicy>(SynINSPolicy(10), *subIMU, *subMagnet);
  synIns->registerCallback(boost::bind(&GPSINSDVLFilter::onSyncInsCallBack, this, _1, _2));

  odom_msg.header.frame_id = "ned_link";
  odom_msg.child_frame_id = body_name;
  last_ins_time = this->world->SimTime();

  connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GPSINSDVLFilter::OnUpdate, this));
}

void GPSINSDVLFilter::OnUpdate() {}

void GPSINSDVLFilter::onGPSCallBack(const NavSatFix::ConstPtr& msg)
{
  tmp_lat = msg->latitude;
  tmp_lon = msg->longitude;
  tmp_alt = msg->altitude;
}

void GPSINSDVLFilter::onDVLCallBack(const Vector3Stamped::ConstPtr& msg) { tmp_linear_vel = msg->vector; }

void GPSINSDVLFilter::onSyncInsCallBack(const Imu::ConstPtr& msg1, const MagneticField::ConstPtr& msg2)
{
  common::Time curr_ins_time = this->world->SimTime();
  double dt = curr_ins_time.Double() - last_ins_time.Double();
  last_ins_time = curr_ins_time;

  auto orientation = link->WorldPose().Rot();
  odom_msg.orientation.x = orientation.Roll();
  odom_msg.orientation.y = -orientation.Pitch();
  odom_msg.orientation.z = -orientation.Yaw();

  //  auto curr_angular_vel = msg1->angular_velocity;
  //  odom_msg.angular_acceleration.x = (curr_angular_vel.x - last_angular_vel.x)/dt;
  //  odom_msg.angular_acceleration.y = -(curr_angular_vel.y - last_angular_vel.y)/dt;
  //  odom_msg.angular_acceleration.z = -(curr_angular_vel.z - last_angular_vel.z)/dt;
  //  last_angular_vel = curr_angular_vel;
  auto angularAccel = link->RelativeAngularAccel();
  odom_msg.angular_acceleration.x = angularAccel.X();
  odom_msg.angular_acceleration.y = -angularAccel.Y();
  odom_msg.angular_acceleration.z = -angularAccel.Z();

  //  odom_msg.linear_velocity.x = tmp_linear_vel.x;
  //  odom_msg.linear_velocity.y = tmp_linear_vel.y;
  //  odom_msg.linear_velocity.z = tmp_linear_vel.z;
  auto linearVel = link->RelativeLinearVel();
  odom_msg.linear_velocity.x = linearVel.X();
  odom_msg.linear_velocity.y = -linearVel.Y();
  odom_msg.linear_velocity.z = -linearVel.Z();

  //  odom_msg.angular_velocity.x = msg1->angular_velocity.x;
  //  odom_msg.angular_velocity.y = msg1->angular_velocity.y;
  //  odom_msg.angular_velocity.z = msg1->angular_velocity.z;
  //  odom_msg.linear_acceleration.x = msg1->linear_acceleration.x;
  //  odom_msg.linear_acceleration.y = msg1->linear_acceleration.y;
  //  odom_msg.linear_acceleration.z = msg1->linear_acceleration.z;
  auto angularVel = link->RelativeAngularVel();
  odom_msg.angular_velocity.x = angularVel.X();
  odom_msg.angular_velocity.y = -angularVel.Y();
  odom_msg.angular_velocity.z = -angularVel.Z();
  auto linearAccel = link->RelativeLinearAccel();
  odom_msg.linear_acceleration.x = linearAccel.X();
  odom_msg.linear_acceleration.y = -linearAccel.Y();
  odom_msg.linear_acceleration.z = -linearAccel.Z();

  odom_msg.latitude = tmp_lat;
  odom_msg.longitude = tmp_lon;
  odom_msg.altitude = tmp_alt;
  convert_global_to_local_coords(tmp_lat, tmp_lon, ned_lat, ned_lon, odom_msg.position.x, odom_msg.position.y);
  odom_msg.position.z = -tmp_alt;

  odom_msg.header.stamp = ros::Time::now();
  pubOdom.publish(odom_msg);
}

bool GPSINSDVLFilter::LoadParameters()
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

} // namespace gazebo
