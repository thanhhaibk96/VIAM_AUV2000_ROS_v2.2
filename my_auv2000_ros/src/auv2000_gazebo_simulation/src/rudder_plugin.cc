#include <boost/algorithm/clamp.hpp>
#include <ros/time.h>

#include <cmath>
#include <functional>
#include "math.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include "rudder_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
Rudder::Rudder(AUVRudder *_parent)
{
  // Initialize fields
  this->plugin = _parent;
  this->currCmd = 0.0;
}

//////////////////////////////////////////////////
double AUVRudder::SdfParamDouble(sdf::ElementPtr _sdfPtr,
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
void AUVRudder::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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


  ROS_DEBUG("Loading AUV_gazebo_rudder_plugin");
  this->model = _parent;
  this->world = this->model->GetWorld();

  // Get parameters from SDF
  std::string nodeNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    nodeNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("Rudder namespace <" << nodeNamespace << ">");
  }

  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", 1.0);

  ROS_INFO_STREAM("Loading rudder from SDF");

  // For each thruster
 

 int rudderCounter = 0;
  if (_sdf->HasElement("rudder"))
  {
    sdf::ElementPtr rudderSDF = _sdf->GetElement("rudder");
    while (rudderSDF)
    {
      // Instatiate
      Rudder rudder(this);
      // Find link by name in SDF
      if (rudderSDF->HasElement("linkName"))
      {
        std::string linkName = rudderSDF->Get<std::string>("linkName");
        rudder.link = this->model->GetLink(linkName);
        if (!rudder.link)
        {
          ROS_INFO_STREAM("Could not find a link by the name <" << linkName
            << "> in the model!");
        }
        else
        {
          ROS_INFO_STREAM("rudder added to link <" << linkName << ">");
        }
      }
      else
      {
        ROS_ERROR_STREAM("Please specify a link name for each thruster!");
      }


      // Parse out propellor joint name
      if (rudderSDF->HasElement("rudderJointName"))
      {
        std::string rudderName =
          rudderSDF->GetElement("rudderJointName")->Get<std::string>();
        rudder.rudderJoint = this->model->GetJoint(rudderName);
        if (!rudder.rudderJoint)
        {
          ROS_INFO_STREAM("Could not find a rudder joint by the name of <-------------------------------------------------------------------"
            << rudderName << "> in the model!");
        }
        else
        {
          ROS_INFO_STREAM("Rudder joint <" << rudderName <<
            "> added to rudder");
        }
      }
      else
      {
        ROS_ERROR_STREAM("No propJointName SDF parameter for thruster #"
          << rudderCounter);
      }

      // OPTIONAL PARAMETERS
      // Parse individual rudder SDF parameters
      rudder.upperLimit =
        this->SdfParamDouble(rudderSDF, "upperLimit", 0.7);
      rudder.lowerLimit =
        this->SdfParamDouble(rudderSDF, "lowerLimit", -0.7);
     this->paramXuudd       = this->SdfParamDouble(rudderSDF, "xuudd",-10.060862499999997);
     this->paramXuvd        = this->SdfParamDouble(rudderSDF, "xuvd",-3.822121663749999); 
     this->paramXurd        = this->SdfParamDouble(rudderSDF, "xurd",5.236941915958013);
     this->paramYuudd       = this->SdfParamDouble(rudderSDF, "yuudd",-11.590855414746541);
     this->paramYuvd        = this->SdfParamDouble(rudderSDF, "yuvd",-4.403365972062211); 
     this->paramYurd        = this->SdfParamDouble(rudderSDF, "yurd",6.033343221149786);
     this->rudder.push_back(rudder);
      rudderSDF = rudderSDF->GetNextElement("thruster");
      rudderCounter++;
  }
}
  else
  {
    ROS_WARN_STREAM("No 'rudder' tags in description - how will you move?");
  }
  ROS_DEBUG_STREAM("Found " << rudderCounter << " thrusters");
  // Initialize the ROS node and subscribe to cmd_drive
  this->rosnode.reset(new ros::NodeHandle(nodeNamespace));

  // Parse for cmd subscription topic
  if (_sdf->HasElement("cmdTopic"))
  {
    ROS_INFO_STREAM("cmd Topic ---------------------------------------------");
    cmdTopic = _sdf->Get<std::string>("cmdTopic");
    cmdSub = this->rosnode->subscribe(cmdTopic, 10, &AUVRudder::OnRudderCmdCallBack, this);
  }
  else
  {
    ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subscription) "
      "for thrusters!");
  }

  if (_sdf->HasElement("pubTopic"))
  {
    pubTopic = _sdf->Get<std::string>("pubTopic");
    pubPosition = this->rosnode->advertise<mx28_stamped>(pubTopic, 10);
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
    cmdKeyboardSub = this->rosnode->subscribe(cmdKeyboardTopic, 10, &AUVRudder::OnRudderCmdKeyboardCallBack, this);
  }
  else
  {
    ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subscription) "
      "for rudder!");
  }

  if(_sdf->HasElement("setRudderService"))
  {
    setRudderService = _sdf->Get<std::string>("setRudderService");
    resSetRudder = this->rosnode->advertiseService(setRudderService,
                                                 &AUVRudder::onSetRudderCallBack, this);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AUVRudder::Update, this));

#if GAZEBO_MAJOR_VERSION >= 8
  lastCmdTime = world->SimTime();
#else
  lastCmdTime = world->GetSimTime();
#endif
}


////////////////////////////////////////////
void AUVRudder::Update()
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
      for (auto it = rudder.begin(); it != rudder.end(); it++)
        it->currCmd = 0.0;
       ROS_DEBUG_STREAM_THROTTLE(1.0, "Cmd Timeout");
       //ROS_INFO_STREAM("RUDDER TIME OUT");
    }
 
      // Apply the thrust mapping
      ignition::math::Vector3d rudderForce(0, 0, 0);
      ignition::math::Vector3d rudderMoment(0, 0, 0);

     double delta_R  =rudder[0].currCmd;
      //double delta_R  = 0.25;
    
      
      rudder_msg.position =rudder[0].currCmd;
      pubPosition.publish(rudder_msg);

      double lr = 1.370166200000000; 
  	 // Calculate Force and moment: 
  // Get body-centered linear and angular rates
  #if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Vector3d kVelLinearBody =
      this->link->RelativeLinearVel();
  #else
    const ignition::math::Vector3d kVelLinearBody =
      this->link->GetRelativeLinearVel().Ign();
  #endif
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Vel linear: " << kVelLinearBody);

  #if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Vector3d kVelAngularBody =
      this->link->RelativeAngularVel();
  #else
    const ignition::math::Vector3d kVelAngularBody =
      this->link->GetRelativeAngularVel().Ign();
  #endif
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Vel angular: " << kVelAngularBody);

     double u =   kVelLinearBody.X();
     double v = - kVelLinearBody.Y();
     double w = - kVelLinearBody.Z();
     double p =   kVelAngularBody.X();
     double q = - kVelAngularBody.Y();
     double r = - kVelAngularBody.Z();


    rudderForce.X() = this->paramXuudd * (u * delta_R) * (u * delta_R) + this->paramXuvd * u * v * delta_R + this->paramXurd * u * r * delta_R;
    rudderForce.Y() = this->paramYuudd * u * u * delta_R + this->paramYuvd * u * v + this->paramYurd * u * r;
    rudderMoment.Z() = -rudderForce.Y()*lr;

  
  	rudderForce.Y() = rudderForce.Y() * -1;
  	rudderMoment.Z() = rudderMoment.Z()* -1;
  	//ROS_INFO_STREAM("RUDDER FORCE: " << rudderForce << "    RUDDER FORCE:  "<< rudderMoment);
  	//ROS_INFO_STREAM("RUDDER TERM: " << kVelLinearBody << kVelAngularBody);
    this->link->AddRelativeForce(rudderForce);
    this->link->AddRelativeTorque(rudderMoment);
    this->rudder[0].rudderJoint->SetPosition(0, delta_R);
  }
}

//////////////////////////////////////////////////
void AUVRudder::aniRudder(double delta_R)
{
}

//////////////////////////////////////////////////
bool AUVRudder::onSetRudderCallBack(utils::CommandLongRequest &_req,
                                    utils::CommandLongResponse &_res)
{
  if (_req.param1 == 1.0f)
    rudderLocked = false;
  else if (_req.param1 == 0.0f)
    rudderLocked = true;
  _res.result = 0; // MAV_RESULT_ACCEPTED

  return true;
}

//////////////////////////////////////////////////
void AUVRudder::OnRudderCmdCallBack(const utils::mx28_stamped::ConstPtr &_msg)
{
 #if GAZEBO_MAJOR_VERSION >= 8
      lastCmdTime = world->SimTime();
   #else
      lastCmdTime = world->GetSimTime();
   #endif
  rudder[0].currCmd = _msg->position;
}

void AUVRudder::OnRudderCmdKeyboardCallBack(const utils::KeyboardCommand::ConstPtr &_msg)
{
}

GZ_REGISTER_MODEL_PLUGIN(AUVRudder);
