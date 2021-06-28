#include <boost/algorithm/clamp.hpp>
#include <ros/time.h>

#include <cmath>
#include <functional>
#include "math.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <std_msgs/Float64.h>
#include "mass_shifter_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
MassShifter::MassShifter(AUVMassShifter *_parent)
{
  // Initialize fields
  this->plugin = _parent;
  this->currCmd = 0.0;
}

//////////////////////////////////////////////////
double AUVMassShifter::SdfParamDouble(sdf::ElementPtr _sdfPtr,
  const std::string &_paramName, const double _defaultVal) const
{
  if (!_sdfPtr->HasElement(_paramName))
  {
    ROS_INFO_STREAM("Parameter <" << _paramName << "> not found: "
                    "Using default value of <" << _defaultVal << ">.");
    return _defaultVal;
  }

  double val = _sdfPtr->Get<double>(_paramName);
  ROS_INFO_STREAM("Parameter found - setting <" << _paramName <<
                   "> to <" << val << ">.");
  return val;
}

//////////////////////////////////////////////////
void AUVMassShifter::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  std::string linkName;
  if (!_sdf->HasElement("bodyName") ||
      !_sdf->GetElement("bodyName")->GetValue())
  {
    this->link = _parent->GetLink();
    linkName = this->link->GetName();
    ROS_INFO_STREAM("Did not find SDF parameter bodyName");
  }
  else
  {
    linkName = _sdf->GetElement("bodyName")->Get<std::string>();
    this->link = _parent->GetLink(linkName);
    ROS_INFO_STREAM("Found SDF parameter bodyName as <" << linkName<< ">");
  }

  ROS_DEBUG("Loading AUV_gazebo_mass_shifter_plugin");
  this->model = _parent;
  this->world = this->model->GetWorld();

  // Get parameters from SDF
  std::string nodeNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    nodeNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("MassShifter namespace <" << nodeNamespace << ">");
  }

  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", 1.0);

  ROS_INFO_STREAM("Loading MassShifter from SDF");

  // For each Mass Shifter
 
 int massShifterCounter = 0;
  if (_sdf->HasElement("massShifter"))
  {
    //ROS_INFO_STREAM("CONFIG RUDDER TERMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM");

    sdf::ElementPtr massShifterSDF = _sdf->GetElement("massShifter");
    while (massShifterSDF)
    {
      // Instatiate
      MassShifter massShifter(this);
      // Find link by name in SDF
      if (massShifterSDF->HasElement("linkName"))
      {
        std::string linkName = massShifterSDF->Get<std::string>("linkName");
        massShifter.link = this->model->GetLink(linkName);
        if (!massShifter.link)
        {
          ROS_INFO_STREAM("Could not find a link by the name <" << linkName
            << "> in the model!");
        }
        else
        {
          ROS_INFO_STREAM("massShifter added to link <" << linkName << ">");
        }
      }
      else
      {
        ROS_ERROR_STREAM("Please specify a link name for each thruster!");
      }


      // Parse out propellor joint name
      if (massShifterSDF->HasElement("massShifterJointName"))
      {
        std::string massShifterName =
          massShifterSDF->GetElement("massShifterJointName")->Get<std::string>();
        massShifter.massShifterJoint = this->model->GetJoint(massShifterName);
        if (!massShifter.massShifterJoint)
        {
          ROS_INFO_STREAM("Could not find a rudder joint by the name of "
            << massShifterName << "> in the model!");
        }
        else
        {
          ROS_INFO_STREAM("massShifter joint <" << massShifterName <<
            "> added to massShifter");
        }
      }
      else
      {
        ROS_ERROR_STREAM("No massShifterJointName SDF parameter for massShifter #"
          << massShifterCounter);
      }

      // OPTIONAL PARAMETERS
      // Parse individual rudder SDF parameters
      massShifter.upperLimit =
        this->SdfParamDouble(massShifterSDF, "upperLimit", 0.2);
      massShifter.lowerLimit =
        this->SdfParamDouble(massShifterSDF, "lowerLimit", -0.2);
     this->massShifter.push_back(massShifter);
      massShifterSDF = massShifterSDF->GetNextElement("massShifter");
      massShifterCounter++;
  }
}
  else
  {
    ROS_WARN_STREAM("No 'massShifter' tags in description - how will you move?");
  }
  ROS_DEBUG_STREAM("Found " << massShifterCounter << " massShifter");
  // Initialize the ROS node and subscribe to cmd_drive
  this->rosnode.reset(new ros::NodeHandle(nodeNamespace));

  // Parse for cmd subscription topic
  if (_sdf->HasElement("cmdTopic"))
  {
    cmdTopic = _sdf->Get<std::string>("cmdTopic");
    cmdSub = this->rosnode->subscribe(cmdTopic, 10, &AUVMassShifter::OnMassShifterCmdCallBack, this);
  }
  else
  {
    ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subscription) "
      "for thrusters!");
  }

  if (_sdf->HasElement("pubTopic"))
  {
    pubTopic = _sdf->Get<std::string>("pubTopic");
    pubPosition = this->rosnode->advertise<motor_stamped>(pubTopic, 10);
  }
  else
  {
    ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subscription) "
      "for thrusters!");
  }

  // Parse for cmd from keyboard subscription topic
  if (_sdf->HasElement("cmdKeyboardTopic"))
  {
    cmdKeyboardTopic = _sdf->Get<std::string>("cmdKeyboardTopic");
    cmdKeyboardSub = this->rosnode->subscribe(cmdKeyboardTopic, 10, &AUVMassShifter::OnMassShifterCmdKeyboardCallBack, this);
  }
  else
  {
    ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subscription) "
      "for MassShifter!");
  }

  if(_sdf->HasElement("setMassShifterService"))
  {
    setMassShifterService = _sdf->Get<std::string>("setMassShifterService");
    resSetMassShifter = this->rosnode->advertiseService(setMassShifterService,
                                                 &AUVMassShifter::onSetMassShifterCallBack, this);
    pubMassShifter = this->rosnode->advertise<std_msgs::Float64>("/auv/joint1_position_controller/command", 1);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AUVMassShifter::Update, this));

#if GAZEBO_MAJOR_VERSION >= 8
  lastCmdTime = world->SimTime();
#else
  lastCmdTime = world->GetSimTime();
#endif
  this->pre_error = this->error;
  this->sum_error = this->sum_error + this->error; 
}


////////////////////////////////////////////
void AUVMassShifter::Update()
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
      for (auto it = massShifter.begin(); it != massShifter.end(); it++)
        it->currCmd = 0.0;
      ROS_DEBUG_STREAM_THROTTLE(1.0, "Cmd Timeout");
     //  ROS_INFO_STREAM("MASS SHIFTER TIME OUT");
    }
cnt = cnt +1 ;
if (cnt == 10)
{
  pos_set = massShifter[0].currCmd;
  double alpha = 0.05;
  pos_set_filtered = (1 - alpha)*pos_set_filtered + alpha * pos_set; 
  std_msgs::Float64 MassShifterCmd;
  MassShifterCmd.data = pos_set_filtered;
  pubMassShifter.publish(MassShifterCmd);

  mass_shifter_msg.position = massShifter[0].massShifterJoint->Position();
  pubPosition.publish(mass_shifter_msg);
  cnt = 0;
}
  

  }
}
//////////////////////////////////////////////////
bool AUVMassShifter::onSetMassShifterCallBack(utils::CommandLongRequest &_req,
                                    utils::CommandLongResponse &_res)
{
  if (_req.param1 == 1.0f)
    massShifterLocked = false;
  else if (_req.param1 == 0.0f)
    massShifterLocked = true;
  _res.result = 0; // MAV_RESULT_ACCEPTED
  return true;
}

//////////////////////////////////////////////////
void AUVMassShifter::OnMassShifterCmdCallBack(const utils::motor_stamped::ConstPtr &_msg)
{
   #if GAZEBO_MAJOR_VERSION >= 8
      lastCmdTime = world->SimTime();
   #else
      lastCmdTime = world->GetSimTime();
   #endif
  massShifter[0].currCmd = _msg->position;
  
}

void AUVMassShifter::OnMassShifterCmdKeyboardCallBack(const utils::KeyboardCommand::ConstPtr &_msg)
{
}

GZ_REGISTER_MODEL_PLUGIN(AUVMassShifter);
