#include "arm1_plugin.hh"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(ARM1);

ARM1::ARM1() {}
ARM1::~ARM1()
{
  if (connection.get())
  {
    connection.reset();
    connection = gazebo::event::ConnectionPtr();
  }

  node->shutdown();
}

void ARM1::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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
  pubArm1 = node->advertise<board_arm1_stamped>(topic_name, 10);

  subMassShifter = node->subscribe("mass_shifter/position", 10, &ARM1::onMassShifterCallBack, this);
  subPistol = node->subscribe("piston/position", 10, &ARM1::onPistolCallBack, this);
  subAntiRolling = node->subscribe("irm1/position", 10, &ARM1::onAntiRollingCallBack, this);
  //subBMS = node->subscribe("arm2/bms", 10, &ARM2::onBMSCallBackCallBackCallBack, this);
 
  connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ARM1::OnUpdate, this));
}

void ARM1::OnUpdate() 
{
cnt = cnt +1 ;
if (cnt == 10)
{
   pubArm1.publish(arm1_msg);
  cnt = 0;
}
  
}

void ARM1::onMassShifterCallBack(const utils::motor_stamped::ConstPtr& msg)
{
  arm1_msg.mass_shifter_status.position = msg->position;
}

void ARM1::onPistolCallBack(const utils::motor_stamped::ConstPtr& msg)
{
  arm1_msg.pistol_status.position = msg->position;
}

void ARM1::onAntiRollingCallBack(const utils::anti_rolling_stamped::ConstPtr& msg)
{
  arm1_msg.roll_motor_status.angle = msg->angle;
}

bool ARM1::LoadParameters()
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
