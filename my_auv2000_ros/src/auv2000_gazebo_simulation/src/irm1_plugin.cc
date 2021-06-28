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
#include "irm1_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
IRM1::IRM1(AUVIRM1 *_parent)
{
  // Initialize fields
  this->plugin = _parent;
  this->currCmd = 0.0;
}

//////////////////////////////////////////////////
double AUVIRM1::SdfParamDouble(sdf::ElementPtr _sdfPtr,
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
void AUVIRM1::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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

  ROS_DEBUG("Loading AUV_gazebo_IRM1_plugin");
  this->model = _parent;
  this->world = this->model->GetWorld();

  // Get parameters from SDF
  std::string nodeNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    nodeNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("IRM1 namespace <" << nodeNamespace << ">");
  }

  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", 1.0);

  ROS_INFO_STREAM("Loading MassShifter from SDF");

  // For each IRM1
 
 int IRM1Counter = 0;
  if (_sdf->HasElement("irm1"))
  {
    //ROS_INFO_STREAM("CONFIG RUDDER TERMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM");

    sdf::ElementPtr irm1SDF = _sdf->GetElement("irm1");
    while (irm1SDF)
    {
      // Instatiate
      IRM1 IRM1(this);
      // Find link by name in SDF
      if (irm1SDF->HasElement("linkName"))
      {
        std::string linkName = irm1SDF->Get<std::string>("linkName");
        IRM1.link = this->model->GetLink(linkName);
        if (!IRM1.link)
        {
          ROS_INFO_STREAM("Could not find a link by the name <" << linkName
            << "> in the model!");
        }
        else
        {
          ROS_INFO_STREAM("IRM1 added to link <" << linkName << ">");
        }
      }
      else
      {
        ROS_ERROR_STREAM("Please specify a link name for each thruster!");
      }


      // Parse out IRM joint name
      if (irm1SDF->HasElement("irm1JointName"))
      {
        std::string irm1Name =
          irm1SDF->GetElement("irm1JointName")->Get<std::string>();
        IRM1.irm1Joint = this->model->GetJoint(irm1Name);
        if (!IRM1.irm1Joint)
        {
          ROS_INFO_STREAM("Could not find a rudder joint by the name of "
            << irm1Name << "> in the model!");
        }
        else
        {
          ROS_INFO_STREAM("irm1 joint <" << irm1Name <<
            "> added to irm");
        }
      }
      else
      {
        ROS_ERROR_STREAM("No irm1JointName SDF parameter for irm #"
          << IRM1Counter);
      }

      // OPTIONAL PARAMETERS
      // Parse individual rudder SDF parameters
      IRM1.upperLimit =
        this->SdfParamDouble(irm1SDF, "upperLimit", 0.53);
      IRM1.lowerLimit =
        this->SdfParamDouble(irm1SDF, "lowerLimit", -0.53);
     this->irm1.push_back(IRM1);
      irm1SDF = irm1SDF->GetNextElement("irm1");
      IRM1Counter++;
  }
}
  else
  {
    ROS_WARN_STREAM("No 'irm1' tags in description - how will you move?");
  }
  ROS_DEBUG_STREAM("Found " << IRM1Counter << " irm1");
  // Initialize the ROS node and subscribe to cmd_drive
  this->rosnode.reset(new ros::NodeHandle(nodeNamespace));

  // Parse for cmd subscription topic
  if (_sdf->HasElement("cmdTopic"))
  {
    cmdTopic = _sdf->Get<std::string>("cmdTopic");
    cmdSub = this->rosnode->subscribe(cmdTopic, 10, &AUVIRM1::OnIRM1CmdCallBack, this);
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
    cmdKeyboardSub = this->rosnode->subscribe(cmdKeyboardTopic, 10, &AUVIRM1::OnIRM1CmdKeyboardCallBack, this);
  }
  else
  {
    ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subscription) "
      "for IRM1!");
  }

  if(_sdf->HasElement("setIRM1Service"))
  {
    setIRM1Service = _sdf->Get<std::string>("setIRM1Service");
    resSetIRM1 = this->rosnode->advertiseService(setIRM1Service,
                                                 &AUVIRM1::onSetIRM1CallBack, this);
    pubIRM1 = this->rosnode->advertise<std_msgs::Float64>("/auv/joint2_position_controller/command", 1);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AUVIRM1::Update, this));

#if GAZEBO_MAJOR_VERSION >= 8
  lastCmdTime = world->SimTime();
#else
  lastCmdTime = world->GetSimTime();
#endif
}


////////////////////////////////////////////
void AUVIRM1::Update()
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
      for (auto it = irm1.begin(); it != irm1.end(); it++)
        it->currCmd = 0.0;

      //  ROS_INFO_STREAM("IRM TIME OUT");
    }

    double IRM1_pos_set;
    IRM1_pos_set = irm1[0].currCmd;
    irm1_msg.angle = irm1[0].irm1Joint->Position();
    pubPosition.publish(irm1_msg);
    std_msgs::Float64 IRM1Cmd;
    IRM1Cmd.data = IRM1_pos_set;
    pubIRM1.publish(IRM1Cmd);
  }
}
//////////////////////////////////////////////////
bool AUVIRM1::onSetIRM1CallBack(utils::CommandLongRequest &_req,
                                    utils::CommandLongResponse &_res)
{
  if (_req.param1 == 1.0f)
    IRM1Locked = false;
  else if (_req.param1 == 0.0f)
    IRM1Locked = true;
  _res.result = 0; // MAV_RESULT_ACCEPTED
  return true;
}

//////////////////////////////////////////////////
void AUVIRM1::OnIRM1CmdCallBack(const utils::anti_rolling_stamped::ConstPtr &_msg)
{
  #if GAZEBO_MAJOR_VERSION >= 8
      lastCmdTime = world->SimTime();
   #else
      lastCmdTime = world->GetSimTime();
   #endif
  irm1[0].currCmd = _msg->angle;
  //ROS_INFO_STREAM("test bla____________________________________________________");
}

void AUVIRM1::OnIRM1CmdKeyboardCallBack(const utils::KeyboardCommand::ConstPtr &_msg)
{
}

GZ_REGISTER_MODEL_PLUGIN(AUVIRM1);
