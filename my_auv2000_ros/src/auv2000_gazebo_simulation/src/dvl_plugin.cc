#include "dvl_plugin.hh"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosDvlSensor);

GazeboRosDvlSensor::GazeboRosDvlSensor() {}

GazeboRosDvlSensor::~GazeboRosDvlSensor()
{
  if (connection.get())
  {
    connection.reset();
    connection = gazebo::event::ConnectionPtr();
  }

  node->shutdown();
}

void GazeboRosDvlSensor::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  model = _parent;
  world = _parent->GetWorld();
  sdf = _sdf;

  if (!LoadParameters())
  {
    ROS_FATAL("Error Loading Parameters!");
    return;
  }

  auto seed = std::chrono::system_clock::now().time_since_epoch().count();
  rndGen = std::default_random_engine(seed);
  velNoiseModel = std::normal_distribution<double>(0.0, vel_stddev);

  node = new ros::NodeHandle(this->robot_namespace);
  dvl_data_publisher = node->advertise<geometry_msgs::Vector3Stamped>(topic_name, 10);
  dvl_msg.header.frame_id = body_name;

  connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosDvlSensor::OnUpdate, this));  
}

void GazeboRosDvlSensor::OnUpdate()
{
  common::Time cur_time = world->SimTime();
  if (update_rate > 0 && (cur_time - last_time).Double() < (1.0 / update_rate))
    return;

  ignition::math::Vector3d bodyVel = link->RelativeLinearVel();
  bodyVel.X() += velNoiseModel(rndGen);
  bodyVel.Y() += velNoiseModel(rndGen);
  bodyVel.Z() += velNoiseModel(rndGen);

  dvl_msg.header.stamp.sec = cur_time.sec;
  dvl_msg.header.stamp.nsec = cur_time.nsec;
  dvl_msg.vector.x = bodyVel.X();
  dvl_msg.vector.y = -bodyVel.Y();
  dvl_msg.vector.z = -bodyVel.Z();
  dvl_data_publisher.publish(dvl_msg);

  last_time = cur_time;
}

bool GazeboRosDvlSensor::LoadParameters()
{
  // loading parameters from the sdf file

  // NAMESPACE
  if (sdf->HasElement("robotNamespace"))
  {
    robot_namespace = sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("<robotNamespace> set to: " << robot_namespace);
  }

  // UPDATE RATE
  if (!sdf->HasElement("updateRate"))
  {
    ROS_DEBUG("missing <updateRate>, defaults to 0.0 (as fast as possible)");
    this->update_rate = 0;
  }
  else
    this->update_rate = sdf->GetElement("updateRate")->Get<double>();

  // TOPIC
  if (sdf->HasElement("topicName"))
  {
    topic_name = sdf->Get<std::string>("topicName");
    ROS_INFO_STREAM("<topicName> set to: " << topic_name);
  }
  else
  {
    topic_name = "/dvl_data";
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

  // Added noise for velocity
  if (sdf->HasElement("velStdDev"))
  {
    vel_stddev = sdf->Get<double>("velStdDev");
    ROS_INFO_STREAM("<velStdDev> set to: " << vel_stddev);
  }
  else
  {
    ROS_FATAL("missing <velVar>, cannot proceed");
    return false;
  }

  return true;
}
}
