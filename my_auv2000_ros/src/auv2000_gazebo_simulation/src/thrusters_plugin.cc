#include <boost/algorithm/clamp.hpp>
#include <ros/time.h>
#include <cmath>
#include <functional>
#include "thrusters_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
Thruster::Thruster(AUVThrust *_parent)
{
  // Initialize fields
  this->plugin = _parent;
  this->currCmd = 0.0;
}

//////////////////////////////////////////////////
double AUVThrust::SdfParamDouble(sdf::ElementPtr _sdfPtr,
  const std::string &_paramName, const double _defaultVal) const
{
  if (!_sdfPtr->HasElement(_paramName))
  {
    ROS_INFO_STREAM("Parameter <" << _paramName << "> not found: "
                    "Using default value of <" << _defaultVal << ">.");
    return _defaultVal;
  }

  double val = _sdfPtr->Get<double>(_paramName);
  ROS_DEBUG_STREAM("Parameter found - setting <" << _paramName <<
                   "> to <" << val << ">.");
  return val;
}

//////////////////////////////////////////////////
void AUVThrust::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  ROS_INFO_STREAM("Loading AUV_gazebo_thrust_plugin");
  this->model = _parent;
  this->world = this->model->GetWorld();


  std::string linkName;
  if (!_sdf->HasElement("bodyName") ||
      !_sdf->GetElement("bodyName")->GetValue())
  {
    this->link = _parent->GetLink();
    linkName = this->link->GetName();
  }
  else
  {
    linkName = _sdf->GetElement("bodyName")->Get<std::string>();
    this->link = _parent->GetLink(linkName);
  }
  // Get parameters from SDF
  std::string nodeNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    nodeNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("Thruster namespace <" << nodeNamespace << ">");
  }

  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", 1.0);
  ROS_DEBUG_STREAM("Loading thrusters from SDF");
  // For each thruster
  int thrusterCounter = 0;
  if (_sdf->HasElement("thruster"))
  {
    sdf::ElementPtr thrusterSDF = _sdf->GetElement("thruster");
    while (thrusterSDF)
    {
      // Instatiate
      Thruster thruster(this);
      ROS_DEBUG_STREAM("Thruster #" << thrusterCounter);
      // REQUIRED PARAMETERS
      // Find link by name in SDF
      if (thrusterSDF->HasElement("linkName"))
      {
        std::string linkName = thrusterSDF->Get<std::string>("linkName");
        thruster.link = this->model->GetLink(linkName);
        if (!thruster.link)
        {
          ROS_ERROR_STREAM("Could not find a link by the name <" << linkName << "> in the model!");
        }
        else
        {
          ROS_DEBUG_STREAM("Thruster added to link <" << linkName << ">");
        }
      }
      else
      {
        ROS_ERROR_STREAM("Please specify a link name for each thruster!");
      }
      // Parse out propellor joint name
      if (thrusterSDF->HasElement("propJointName"))
      {
        std::string propName =
          thrusterSDF->GetElement("propJointName")->Get<std::string>();
        thruster.propJoint = this->model->GetJoint(propName);
        if (!thruster.propJoint)
        {
          ROS_ERROR_STREAM("Could not find a propellor joint by the name of <"<< propName << "> in the model!");
        }
        else
        {
          ROS_DEBUG_STREAM("Propellor joint <" << propName << "> added to thruster");
        }
      }
      else
      {
        ROS_ERROR_STREAM("No propJointName SDF parameter for thruster #"
          << thrusterCounter);
      }
      // OPTIONAL PARAMETERS
      // Parse individual thruster SDF parameters
      if (thrusterSDF->HasElement("mappingType"))
      {
        thruster.mappingType = thrusterSDF->Get<int>("mappingType");
        ROS_DEBUG_STREAM("Parameter found - setting <mappingType> to <" <<
          thruster.mappingType << ">.");
      }
      else
      {
        thruster.mappingType = 0;
        ROS_INFO_STREAM("Parameter <mappingType> not found: "
          "Using default value of <" << thruster.mappingType << ">.");
      }
      thruster.maxForceFwd =
        this->SdfParamDouble(thrusterSDF, "maxForceFwd", 250.0);
      thruster.maxForceRev =
        this->SdfParamDouble(thrusterSDF, "maxForceRev", -100.0);
      // Push to vector and increment
      this->thrusters.push_back(thruster);
      thrusterSDF = thrusterSDF->GetNextElement("thruster");
      thrusterCounter++;
    }
  }
  else
  {
    ROS_WARN_STREAM("No 'thruster' tags in description - how will you move?");
  }
  ROS_DEBUG_STREAM("Found " << thrusterCounter << " thrusters");
  // Initialize the ROS node and subscribe to cmd_drive
  this->rosnode.reset(new ros::NodeHandle(nodeNamespace));
  // Parse for cmd subscription topic
  if (_sdf->HasElement("cmdTopic"))
  {
    cmdTopic = _sdf->Get<std::string>("cmdTopic");
    cmdSub = this->rosnode->subscribe(cmdTopic, 10, &AUVThrust::OnThrustCmdCallBack, this);
  }
  else
  {
    ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subscription) "
      "for thrusters!");
  }

  if (_sdf->HasElement("pubTopic"))
  {
     ROS_INFO_STREAM("PUB TOPIC --------------------------------");
    pubTopic = _sdf->Get<std::string>("pubTopic");
    pubSpeed = this->rosnode->advertise<motor_stamped>(pubTopic, 10);
  }
  else
  {
    ROS_INFO_STREAM("Please specify a cmdTopic (for ROS subscription) "
      "for thrusters!");
  }


  // Parse for cmd from keyboard subscription topic
  if (_sdf->HasElement("cmdKeyboardTopic"))
  {
    cmdKeyboardTopic = _sdf->Get<std::string>("cmdKeyboardTopic");
    cmdKeyboardSub = this->rosnode->subscribe(cmdKeyboardTopic, 10, &AUVThrust::OnThrustCmdKeyboardCallBack, this);
  }
  else
  {
    ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subscription) "
      "for thrusters!");
  }
  if(_sdf->HasElement("setArmingService"))
  {
    setArmingService = _sdf->Get<std::string>("setArmingService");
    resSetArming = this->rosnode->advertiseService(setArmingService,
                                                 &AUVThrust::onSetArmingCallBack, this);
  }
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AUVThrust::Update, this));
#if GAZEBO_MAJOR_VERSION >= 8
  lastCmdTime = world->SimTime();
#else
  lastCmdTime = world->GetSimTime();
#endif
}
//////////////////////////////////////////////////
double AUVThrust::convertSpeedToForce(float speed)
{
  double force;
  double X2 =   6.234239409765260/100000;
  double X1 =   0.002070652916595 ;
  double X0 = - 0.442834294940528 ;
  force = X2 * speed * speed + X1 * speed + X0 ;
  return force;
}

//////////////////////////////////////////////////
void AUVThrust::Update()
{
  #if GAZEBO_MAJOR_VERSION >= 8
    common::Time now = this->world->SimTime();
  #else
    common::Time now = this->world->GetSimTime();
  #endif
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    // Enforce command timeout
    double dtc = (now - this->lastCmdTime).Double();
    if (dtc > this->cmdTimeout && this->cmdTimeout > 0.0)
    {
      for (auto it = thrusters.begin(); it != thrusters.end(); it++)
        it->currCmd = 0.0;
      ROS_DEBUG_STREAM_THROTTLE(1.0, "Cmd Timeout");
      // ROS_INFO_STREAM("Thurster TIME OUT");
    }
    for (size_t i = 0; i < this->thrusters.size(); ++i)
    {
      // Apply the thrust mapping
      ignition::math::Vector3d tforcev(0, 0, 0);
      tforcev.X() = convertSpeedToForce(thrusters[i].currCmd);
      if ( tforcev.X() < 0)
      {
         tforcev.X() = 0;
      }
      //ROS_INFO_STREAM(tforcev.X());

      motor_stamped thrust_msg;
      thrust_msg.motor_rspeed = thrusters[i].currCmd ; 
      pubSpeed.publish(thrust_msg);
     
      // Apply force for each thruster
      this->thrusters[i].link->AddLinkForce(ignition::math::Vector3d(tforcev.X(), 0 , 0 ));
      this->link->AddRelativeTorque(ignition::math::Vector3d(-tforcev.X()*0.0625/2, 0 , 0 ));
      if (tforcev.X() > 1 )
      {
         this->SpinPropeller(i);
      }
      cnt = 0;
      
    }
  }
}
//////////////////////////////////////////////////
void AUVThrust::SpinPropeller(size_t _i)
{
  physics::JointPtr propeller = this->thrusters[_i].propJoint;
  propeller->SetForce(0, 0);
}

//////////////////////////////////////////////////
bool AUVThrust::onSetArmingCallBack(utils::CommandLongRequest &_req,
                                    utils::CommandLongResponse &_res)
{
  if (_req.param1 == 1.0f)
    propLocked = false;
  else if (_req.param1 == 0.0f)
    propLocked = true;
  _res.result = 0; // MAV_RESULT_ACCEPTED
  return true;
}

//////////////////////////////////////////////////
void AUVThrust::OnThrustCmdCallBack(const utils::motor_stamped::ConstPtr &_msg)
{
   std::lock_guard<std::mutex> lock(mutex);
   if (propLocked)
     return;

   #if GAZEBO_MAJOR_VERSION >= 8
      lastCmdTime = world->SimTime();
   #else
      lastCmdTime = world->GetSimTime();
   #endif
   thrusters[0].currCmd = _msg->motor_rspeed;
}

void AUVThrust::OnThrustCmdKeyboardCallBack(const utils::KeyboardCommand::ConstPtr &_msg)
{
}

GZ_REGISTER_MODEL_PLUGIN(AUVThrust);
