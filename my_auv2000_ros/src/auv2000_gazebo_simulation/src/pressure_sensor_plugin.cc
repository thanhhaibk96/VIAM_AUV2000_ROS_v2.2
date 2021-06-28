#include "pressure_sensor_plugin.hh"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosPressureSensor);

GazeboRosPressureSensor::GazeboRosPressureSensor()
{
}

GazeboRosPressureSensor::~GazeboRosPressureSensor()
{
  if (updateConnection.get())
  {
    updateConnection.reset();
    updateConnection = gazebo::event::ConnectionPtr();
  }

  node->shutdown();
}
void GazeboRosPressureSensor::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
   model = _parent;
   world = _parent->GetWorld();
   sdf = _sdf;
  ROS_INFO_STREAM("Load pressure sensor Plugin");
   std::string linkName;
  if (!_sdf->HasElement("bodyName") ||
      !_sdf->GetElement("bodyName")->GetValue())
  {
    this->link = model->GetLink();
    linkName = this->link->GetName();
    ROS_INFO_STREAM("Did not find SDF parameter bodyName");
  }
  else
  {
    linkName = _sdf->GetElement("bodyName")->Get<std::string>();
    this->link = model->GetLink(linkName);
    ROS_INFO_STREAM("Found SDF parameter bodyName as <" << linkName<< ">");
  }
  
  if (sdf->HasElement("robotNamespace"))
  {
    robot_namespace = sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("<robotNamespace> set to: " << robot_namespace);
  }
  else
  {
  }
  // TOPIC
  if (sdf->HasElement("topicName"))
  {
    topic_name = sdf->Get<std::string>("topicName");
    ROS_INFO_STREAM("<topicName> set to: " << topic_name);
  }
  else
  {
    topic_name = robot_namespace + "/pressure_data";
    ROS_WARN_STREAM("missing <topicName>, set to /namespace/default: " << topic_name);
  }

  // BODY NAME
  if (sdf->HasElement("frameName"))
  {
    body_name = sdf->Get<std::string>("frameName");
    ROS_INFO_STREAM("<frameName> set to: " << body_name);
  }
  else
  {
    ROS_FATAL("missing <frameName>, cannot proceed");
  }

  // NODE

  node = new ros::NodeHandle(this->robot_namespace);
  pressure_data_publisher = node->advertise<keller_pa3_stamped>(topic_name, 10);
  
  // arm2_data_publisher = node->advertise<board_arm2_stamped>(topic_name, 10);
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboRosPressureSensor::OnUpdate, this));
  // PARAMETERS
  this->saturation = sdf->Get<double>("saturation");
  this->standard_pressure = sdf->Get<double>("standard_pressure");
  this->bar_per_meter = sdf->Get<double>("bar_per_meter");
  this->update_rate = sdf->Get<int>("update_rate");
  this->gaussianNoise = sdf->Get<double>("gaussianNoise");
}

void GazeboRosPressureSensor::OnUpdate()
{
 

  const ignition::math::Pose3d kPose = this->link->WorldPose();
 double depth = std::fabs(kPose.Pos().Z());
 double pressure = this->standard_pressure;
  if (depth >= 0)
  {
    // Convert depth to pressure
    pressure += depth * this->bar_per_meter;
  }
  pressure += GaussianKernel(0, this->gaussianNoise);
 
  double temperature = 20; 
  temperature = temperature +  GaussianKernel(0, this->gaussianNoise);
  if (pressure_data_publisher.getNumSubscribers() > 0)
  {
    //
  }
  //arm2_msg.pressure_status.pressure = pressure ;
  //arm2_msg.pressure_status.temperature = temperature;
  pressure_msg.pressure =  pressure;
  pressure_msg.temperature = temperature;
    // Publish data
  pressure_data_publisher.publish(pressure_msg);
  //arm2_data_publisher.publish(arm2_msg);

}



double GazeboRosPressureSensor::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard normally distributed normal variables
  double U = double(rand()) / double(RAND_MAX);
  double V = double(rand()) / double(RAND_MAX);
  double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
  X = sigma * X + mu;
  return X;
}
}