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
#include "irm2_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
IRM2::IRM2(AUVIRM2 *_parent)
{
  // Initialize fields
  this->plugin = _parent;
  this->currCmd = 0.0;
}

//////////////////////////////////////////////////
double AUVIRM2::SdfParamDouble(sdf::ElementPtr _sdfPtr,
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
void AUVIRM2::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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

  ROS_DEBUG("Loading AUV_gazebo_IRM2_plugin");
  this->model = _parent;
  this->world = this->model->GetWorld();

  // Get parameters from SDF
  std::string nodeNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    nodeNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("IRM2 namespace <" << nodeNamespace << ">");
  }

  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", 1.0);

  ROS_INFO_STREAM("Loading MassShifter from SDF");

  // For each IRM2
 
 int IRM2Counter = 0;
  if (_sdf->HasElement("irm2"))
  {
    //ROS_INFO_STREAM("CONFIG RUDDER TERMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM");

    sdf::ElementPtr irm2SDF = _sdf->GetElement("irm2");
    while (irm2SDF)
    {
      // Instatiate
      IRM2 IRM2(this);
      // Find link by name in SDF
      if (irm2SDF->HasElement("linkName"))
      {
        std::string linkName = irm2SDF->Get<std::string>("linkName");
        IRM2.link = this->model->GetLink(linkName);
        if (!IRM2.link)
        {
          ROS_INFO_STREAM("Could not find a link by the name <" << linkName
            << "> in the model!");
        }
        else
        {
          ROS_INFO_STREAM("IRM2 added to link <" << linkName << ">");
        }
      }
      else
      {
        ROS_ERROR_STREAM("Please specify a link name for each thruster!");
      }


      // Parse out IRM joint name
      if (irm2SDF->HasElement("irm2JointName"))
      {
        std::string irm2Name =
          irm2SDF->GetElement("irm2JointName")->Get<std::string>();
        IRM2.irm2Joint = this->model->GetJoint(irm2Name);
        if (!IRM2.irm2Joint)
        {
          ROS_INFO_STREAM("Could not find a rudder joint by the name of "
            << irm2Name << "> in the model!");
        }
        else
        {
          ROS_INFO_STREAM("irm2 joint <" << irm2Name <<
            "> added to irm");
        }
      }
      else
      {
        ROS_ERROR_STREAM("No irm2JointName SDF parameter for irm #"
          << IRM2Counter);
      }

      // OPTIONAL PARAMETERS
      // Parse individual rudder SDF parameters
      IRM2.upperLimit =
        this->SdfParamDouble(irm2SDF, "upperLimit", 0.53);
      IRM2.lowerLimit =
        this->SdfParamDouble(irm2SDF, "lowerLimit", -0.53);
     this->irm2.push_back(IRM2);
      irm2SDF = irm2SDF->GetNextElement("irm2");
      IRM2Counter++;
  }
}
  else
  {
    ROS_WARN_STREAM("No 'irm2' tags in description - how will you move?");
  }
  ROS_DEBUG_STREAM("Found " << IRM2Counter << " irm2");
  // Initialize the ROS node and subscribe to cmd_drive
  this->rosnode.reset(new ros::NodeHandle(nodeNamespace));

  // Parse for cmd subscription topic
  if (_sdf->HasElement("cmdTopic"))
  {
    cmdTopic = _sdf->Get<std::string>("cmdTopic");
    cmdSub = this->rosnode->subscribe(cmdTopic, 10, &AUVIRM2::OnIRM2CmdCallBack, this);
  }
  else
  {
    ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subscription) "
      "for thrusters!");
  }

  if (_sdf->HasElement("pubTopic"))
  {
    pubTopic = _sdf->Get<std::string>("pubTopic");
    pubPosition = this->rosnode->advertise<anti_rolling_stamped>(pubTopic, 10);
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
    cmdKeyboardSub = this->rosnode->subscribe(cmdKeyboardTopic, 10, &AUVIRM2::OnIRM2CmdKeyboardCallBack, this);
  }
  else
  {
    ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subscription) "
      "for IRM2!");
  }

  if(_sdf->HasElement("setIRM2Service"))
  {
    setIRM2Service = _sdf->Get<std::string>("setIRM2Service");
    resSetIRM2 = this->rosnode->advertiseService(setIRM2Service,
                                                 &AUVIRM2::onSetIRM2CallBack, this);
    pubIRM2 = this->rosnode->advertise<std_msgs::Float64>("/auv/joint3_position_controller/command", 1);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AUVIRM2::Update, this));

#if GAZEBO_MAJOR_VERSION >= 8
  lastCmdTime = world->SimTime();
#else
  lastCmdTime = world->GetSimTime();
#endif
}


////////////////////////////////////////////
void AUVIRM2::Update()
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
      for (auto it = irm2.begin(); it != irm2.end(); it++)
        it->currCmd = 0.0;
      ROS_DEBUG_STREAM_THROTTLE(1.0, "Cmd Timeout");
       //ROS_INFO_STREAM("IRM TIME OUT");
    }
    double IRM2_pos_set;
    IRM2_pos_set = irm2[0].currCmd;
    irm2_msg.angle = irm2[0].irm2Joint->Position();
    pubPosition.publish(irm2_msg);
    std_msgs::Float64 IRM2Cmd;
    IRM2Cmd.data = IRM2_pos_set;
    pubIRM2.publish(IRM2Cmd);

  }
}
//////////////////////////////////////////////////
bool AUVIRM2::onSetIRM2CallBack(utils::CommandLongRequest &_req,
                                    utils::CommandLongResponse &_res)
{
  if (_req.param1 == 1.0f)
    IRM2Locked = false;
  else if (_req.param1 == 0.0f)
    IRM2Locked = true;
  _res.result = 0; // MAV_RESULT_ACCEPTED
  return true;
}

//////////////////////////////////////////////////
void AUVIRM2::OnIRM2CmdCallBack(const utils::anti_rolling_stamped::ConstPtr &_msg)
{
   #if GAZEBO_MAJOR_VERSION >= 8
      lastCmdTime = world->SimTime();
   #else
      lastCmdTime = world->GetSimTime();
   #endif
  irm2[0].currCmd = _msg->angle;
 
}

void AUVIRM2::OnIRM2CmdKeyboardCallBack(const utils::KeyboardCommand::ConstPtr &_msg)
{
}

GZ_REGISTER_MODEL_PLUGIN(AUVIRM2);
