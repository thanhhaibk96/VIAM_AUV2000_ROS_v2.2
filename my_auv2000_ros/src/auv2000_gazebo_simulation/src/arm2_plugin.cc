#include "arm2_plugin.hh"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(ARM2);

ARM2::ARM2() {}
ARM2::~ARM2()
{
  if (connection.get())
  {
    connection.reset();
    connection = gazebo::event::ConnectionPtr();
  }

  node->shutdown();
}

void ARM2::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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
  pubArm2 = node->advertise<board_arm2_stamped>(topic_name, 10);

  subPressureSensor = node->subscribe("keller_pa3/pressure", 10, &ARM2::onPressureSensorCallBack, this);
  subRudder = node->subscribe("rudder/position", 10, &ARM2::onRudderCallBack, this);
  subAntiRolling = node->subscribe("irm1/position", 10, &ARM2::onAntiRollingCallBack, this);
  //subBMS = node->subscribe("arm2/bms", 10, &ARM2::onBMSCallBackCallBackCallBack, this);
 
  last_ins_time = this->world->SimTime();

  connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ARM2::OnUpdate, this));
}

void ARM2::OnUpdate() 
{
 
cnt = cnt +1 ;
if (cnt == 10)
{
   pubArm2.publish(arm2_msg);
  cnt = 0;
}
  
}

void ARM2::onPressureSensorCallBack(const utils::keller_pa3_stamped::ConstPtr& msg)
{
  arm2_msg.pressure_status.pressure = msg->pressure; 
  arm2_msg.pressure_status.temperature = msg->temperature; 
}

void ARM2::onRudderCallBack(const utils::mx28_stamped::ConstPtr& msg)
{
  arm2_msg.mx28_status.position = msg->position * 57.3;
}

void ARM2::onAntiRollingCallBack(const utils::anti_rolling_stamped::ConstPtr& msg)
{
  arm2_msg.roll_motor_status.angle = msg->angle ;
}

bool ARM2::LoadParameters()
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


  return true;
}

} // namespace gazebo
