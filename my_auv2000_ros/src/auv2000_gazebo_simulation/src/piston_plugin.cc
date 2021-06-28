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
#include "piston_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
Piston::Piston(AUVPiston *_parent)
{
  // Initialize fields
  this->plugin = _parent;
  this->currCmd = 0.0;
}

//////////////////////////////////////////////////
double AUVPiston::SdfParamDouble(sdf::ElementPtr _sdfPtr,
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
void AUVPiston::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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

  ROS_DEBUG("Loading AUV_gazebo_Piston_plugin");
  this->model = _parent;
  this->world = this->model->GetWorld();

  // Get parameters from SDF
  std::string nodeNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    nodeNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("Piston namespace <" << nodeNamespace << ">");
  }

  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", 1.0);

  ROS_INFO_STREAM("Loading MassShifter from SDF");

  // For each Piston
 
 int PistonCounter = 0;
  if (_sdf->HasElement("piston"))
  {
    //ROS_INFO_STREAM("CONFIG RUDDER TERMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM");

    sdf::ElementPtr pistonSDF = _sdf->GetElement("piston");
    while (pistonSDF)
    {
      // Instatiate
      Piston Piston(this);
      // Find link by name in SDF
      if (pistonSDF->HasElement("linkName"))
      {
        std::string linkName = pistonSDF->Get<std::string>("linkName");
        Piston.link = this->model->GetLink(linkName);
        if (!Piston.link)
        {
          ROS_INFO_STREAM("Could not find a link by the name <" << linkName
            << "> in the model!");
        }
        else
        {
          ROS_INFO_STREAM("Piston added to link <" << linkName << ">");
        }
      }
      else
      {
        ROS_ERROR_STREAM("Please specify a link name for each thruster!");
      }


      // Parse out IRM joint name
      if (pistonSDF->HasElement("pistonJointName"))
      {
        std::string pistonName =
          pistonSDF->GetElement("pistonJointName")->Get<std::string>();
        Piston.pistonJoint = this->model->GetJoint(pistonName);
        if (!Piston.pistonJoint)
        {
          ROS_INFO_STREAM("Could not find a rudder joint by the name of "
            << pistonName << "> in the model!");
        }
        else
        {
          ROS_INFO_STREAM("piston joint <" << pistonName <<
            "> added to piston");
        }
      }
      else
      {
        ROS_ERROR_STREAM("No pistonJointName SDF parameter for irm #"
          << PistonCounter);
      }

      // OPTIONAL PARAMETERS
      // Parse individual rudder SDF parameters
      Piston.upperLimit =
        this->SdfParamDouble(pistonSDF, "upperLimit", 0.53);
      Piston.lowerLimit =
        this->SdfParamDouble(pistonSDF, "lowerLimit", -0.53);
     this->piston.push_back(Piston);
      pistonSDF = pistonSDF->GetNextElement("piston");
      PistonCounter++;
  }
}
  else
  {
    ROS_WARN_STREAM("No 'piston' tags in description - how will you move?");
  }
  ROS_DEBUG_STREAM("Found " << PistonCounter << " piston");
  // Initialize the ROS node and subscribe to cmd_drive
  this->rosnode.reset(new ros::NodeHandle(nodeNamespace));

  // Parse for cmd subscription topic
  if (_sdf->HasElement("cmdTopic"))
  {
    cmdTopic = _sdf->Get<std::string>("cmdTopic");
    cmdSub = this->rosnode->subscribe(cmdTopic, 10, &AUVPiston::OnPistonCmdCallBack, this);
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
    cmdKeyboardSub = this->rosnode->subscribe(cmdKeyboardTopic, 10, &AUVPiston::OnPistonCmdKeyboardCallBack, this);
  }
  else
  {
    ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subscription) "
      "for IRM1!");
  }

  if(_sdf->HasElement("setPistonService"))
  {
    setPistonService = _sdf->Get<std::string>("setPistonService");
    resSetPiston = this->rosnode->advertiseService(setPistonService,
                                                 &AUVPiston::onSetPistonCallBack, this);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AUVPiston::Update, this));
piston_msg.position = piston[0].pistonJoint->Position();
    pubPosition.publish(piston_msg);
    piston[0].pistonJoint->SetPosition(0,Piston_pos_set);
#if GAZEBO_MAJOR_VERSION >= 8
  lastCmdTime = world->SimTime();
#else
  lastCmdTime = world->GetSimTime();
#endif
}


////////////////////////////////////////////
void AUVPiston::Update()
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
      for (auto it = piston.begin(); it != piston.end(); it++)
        it->currCmd = 0.0;
      ROS_DEBUG_STREAM_THROTTLE(1.0, "Cmd Timeout");
      // ROS_INFO_STREAM("PISTON TIME OUT");
    }
    Piston_pos_set = piston[0].currCmd;
    piston_msg.position = piston[0].pistonJoint->Position();
    cnt = cnt +1 ;
if (cnt == 10)
{
    pubPosition.publish(piston_msg);
    cnt = 0;
}
    piston[0].pistonJoint->SetPosition(0,Piston_pos_set);

    //Handle piston position
    double V_fix = 0.096711321306931;
    double S_cd  = 12271.85e-6;
    double l_c = 0.08;
    double V_var = S_cd*(l_c - Piston_pos_set);
    double B = 1010 * 9.81 * (V_fix + V_var);

    const ignition::math::Pose3d kPose = this->link->WorldPose();

   double Buoy_sum;
   double z = kPose.Pos().Z();
     if (z + 0.25 / 2 > 0 && z < 0)
    {
      Buoy_sum = B * (std::fabs(z) + 0.25 / 2) / 0.25;
    }
    else if (z + 0.25 / 2 < 0)

    {
      Buoy_sum = B ; 
    }
    //ROS_INFO_STREAM("buoyancyForce  " << Buoy_sum);
    ignition::math::Vector3d  buoyancyForce = ignition::math::Vector3d(0, 0, Buoy_sum);
    this->link->AddForceAtRelativePosition(buoyancyForce, ignition::math::Vector3d(0,0,0.01));
  }
}
//////////////////////////////////////////////////
bool AUVPiston::onSetPistonCallBack(utils::CommandLongRequest &_req,
                                    utils::CommandLongResponse &_res)
{
  if (_req.param1 == 1.0f)
    pistonLocked = false;
  else if (_req.param1 == 0.0f)
    pistonLocked = true;
  _res.result = 0; // MAV_RESULT_ACCEPTED
  return true;
}

//////////////////////////////////////////////////
void AUVPiston::OnPistonCmdCallBack(const utils::motor_stamped::ConstPtr &_msg)
{
   #if GAZEBO_MAJOR_VERSION >= 8
      lastCmdTime = world->SimTime();
   #else
      lastCmdTime = world->GetSimTime();
   #endif
  piston[0].currCmd = _msg->position;
 
}

void AUVPiston::OnPistonCmdKeyboardCallBack(const utils::KeyboardCommand::ConstPtr &_msg)
{
}

GZ_REGISTER_MODEL_PLUGIN(AUVPiston);
