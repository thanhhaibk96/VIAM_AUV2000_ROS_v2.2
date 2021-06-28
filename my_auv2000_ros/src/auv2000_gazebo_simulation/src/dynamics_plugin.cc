/*

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the _gazebo_dynamics_plugin package,
known as this Package.

This Package free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This Package s distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this package.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>

#include <cmath>
#include <functional>
#include <sstream>
#include <algorithm>

#include <ignition/math/Pose3.hh>

#include "dynamics_plugin.hh"
#include "def.hh"



//using namespace auv;
using namespace gazebo;

//////////////////////////////////////////////////
AUVDynamicsPlugin::AUVDynamicsPlugin()
{
}

//////////////////////////////////////////////////
double AUVDynamicsPlugin::SdfParamDouble(sdf::ElementPtr _sdfPtr,
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
void AUVDynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_INFO_STREAM("Loading _gazebo_dynamics_plugin");
  this->world = _model->GetWorld();
  // Get parameters from SDF
  std::string linkName;
  if (!_sdf->HasElement("bodyName") ||
      !_sdf->GetElement("bodyName")->GetValue())
  {
    this->link = _model->GetLink();
    linkName = this->link->GetName();
    ROS_INFO_STREAM("Did not find SDF parameter bodyName");
  }
  else
  {
    linkName = _sdf->GetElement("bodyName")->Get<std::string>();
    this->link = _model->GetLink(linkName);
    ROS_INFO_STREAM("Found SDF parameter bodyName as <" << linkName<< ">");
  }
  if (!this->link)
  {
    ROS_FATAL("_gazebo_dynamics_plugin error: bodyName: %s does not exist\n",
      linkName.c_str());
    return;
  }
  else
  {
    ROS_DEBUG_STREAM(" Dynamics Model Link Name = " << linkName);
  }

  this->waterLevel       = this->SdfParamDouble(_sdf, "waterLevel"  , 0.5);
  this->waterDensity     = this->SdfParamDouble(_sdf, "waterDensity", 1010);

  this->paramXudot       = this->SdfParamDouble(_sdf, "Xudot", -1.772882186755249);
  this->paramKpdot       = this->SdfParamDouble(_sdf, "Kpdot", -29.084143553958992/2);
  this->paramYvdot       = this->SdfParamDouble(_sdf, "Yvdot", -115.3657671280223);
  this->paramZwdot       = this->SdfParamDouble(_sdf, "Zwdot", -465.9961573833940);
  this->paramNvdot       = this->SdfParamDouble(_sdf, "Nvdot", 5.771490405635055);
  this->paramMwdot       = this->SdfParamDouble(_sdf, "Mwdot", -30.046332342049066);
  this->paramNrdot       = this->SdfParamDouble(_sdf, "Nrdot", -58.670168921771712);
  this->paramMqdot       = this->SdfParamDouble(_sdf, "Mqdot", -62.475950666453606);
  this->paramZqdot       = this->SdfParamDouble(_sdf, "Zqdot", -30.046332342049066);
  this->paramYrdot       = this->SdfParamDouble(_sdf, "Yrdot",  5.771490405635055);

  this->paramXuu         = this->SdfParamDouble(_sdf, "Xuu", -10.500794572286573);
  this->paramYvv         = this->SdfParamDouble(_sdf, "Yvv", -326.1010024119076);
  this->paramNvv         = this->SdfParamDouble(_sdf, "Zww",-543.0611224119077);
  this->paramNvv         = this->SdfParamDouble(_sdf, "Nvv",10.934977314383778);
  this->paramMww         = this->SdfParamDouble(_sdf, "Mww",52.314582020760234);
  this->paramYrr         = this->SdfParamDouble(_sdf, "Yrr",21.419397320203966);
  this->paramZqq         = this->SdfParamDouble(_sdf, "Zqq",-39.858301004853033);
  this->paramNrr         = this->SdfParamDouble(_sdf, "Nrr",-164.4394943644915);
  this->paramMqq         = this->SdfParamDouble(_sdf, "Mqq",-169.8149178878433);
  this->paramKpp         = this->SdfParamDouble(_sdf, "Kpp",-100.2116918191101);


  this->paramYuvl        = this->SdfParamDouble(_sdf, "Yuvl",-72.580791084977221);
  this->paramZuwl        = this->SdfParamDouble(_sdf, "Zuwl",-72.580791084977221);
  this->paramNuvl        = this->SdfParamDouble(_sdf, "Nuvl",26.576632464579994);
  this->paramMuwl        = this->SdfParamDouble(_sdf, "Muwl",-26.576632464579994);

  this->paramZuwf        = this->SdfParamDouble(_sdf, "Zuwf",-468.1261333183318);
  this->paramZuqf        = this->SdfParamDouble(_sdf, "Zuqf",-136.4710327669867);
  this->paramMuwf        = this->SdfParamDouble(_sdf, "Muwf",-136.4710327669867);
  this->paramMuqf        = this->SdfParamDouble(_sdf, "Muqf",-39.784881592635109);


  if (!_sdf->HasElement("length_n"))
  {
    int defaultVal = 2;
    ROS_INFO_STREAM("Parameter <length_n> not found: "
                    "Using default value of <" << defaultVal << ">.");
    this->paramLengthN = defaultVal;
  }
  else
  {
    this->paramLengthN = _sdf->GetElement("length_n")->Get<int>();
  }


  //  Wave model

 // if (_sdf->HasElement("wave_model"))
 // {
  //  this->waveModelName = _sdf->Get<std::string>("wave_model");
 // }
 // this->waveParams = nullptr;

  // Get inertia and mass of AUV
  #if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Vector3d kInertia =
      this->link->GetInertial()->PrincipalMoments();
    const double kMass = this->link->GetInertial()->Mass();
  #else
    const ignition::math::Vector3d kInertia =
      this->link->GetInertial()->GetPrincipalMoments().Ign();
    const double kMass = this->link->GetInertial()->GetMass();
  #endif

  // Report some of the pertinent parameters for verification
 // ROS_DEBUG(" Dynamics Parameters: From URDF XACRO model definition");
//  ROS_INFO_STREAM("AUV Mass (rigid-body):                                        " << kMass);
//  ROS_DEBUG_STREAM("Vessel Inertia Vector (rigid-body): X:" << kInertia[0] <<
 //                 " Y:" << kInertia[1] << " Z:" << kInertia[2]);

  // Initialize time and odometry position
  #if GAZEBO_MAJOR_VERSION >= 8
    this->prevUpdateTime = this->world->SimTime();
  #else
    this->prevUpdateTime = this->world->GetSimTime();
  #endif

  // Listen to the update event broadcastes every physics iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AUVDynamicsPlugin::Update, this));

  // Initialize Added Mass Matrix
  this->Ma = Eigen::MatrixXd(6, 6);
  this->Ma <<
    this->paramXudot, 0,                0,                0,                0,                0,
    0,                this->paramYvdot, 0,                0,                0,                this->paramYrdot,
    0,                0,                this->paramZwdot, 0,                this->paramZqdot, 0,
    0,                0,                0,                this->paramKpdot, 0,                0,
    0,                0,                this->paramMwdot, 0,                this->paramMqdot, 0,
    0,                this->paramNvdot, 0,                0,                0,                this->paramNrdot;

}
////////////////////////////////

double AUVDynamicsPlugin::sat(double x, double min, double max )
{
	if (x > max)
		x = max ;
	else if (x < min )
		x = min;
	return x;
}

double AUVDynamicsPlugin::CircleSegment(double R, double h)
{
  return R*R*acos( (R-h)/R ) - (R-h)*sqrt(2*R*h-h*h);
}

//////////////////////////////////////////////////
void AUVDynamicsPlugin::Update()
{
	

 
  const double kMass = this->link->GetInertial()->Mass();

 //const ignition::math::Pose3d  COG_WORLD = this->link-> WorldCoGPose();
  // Report some of the pertinent parameters for verification
  //ROS_INFO_STREAM(" Dynamics Parameters: From URDF XACRO model definition");
  // ROS_INFO_STREAM("AUV Mass (rigid-body):                                        " << kMass);
  // ROS_INFO_STREAM("AUV COG (rigid-body):                                        " << COG_WORLD);

  // If we haven't yet, retrieve the wave parameters from ocean model plugin.


  #if GAZEBO_MAJOR_VERSION >= 8
    const common::Time kTimeNow = this->world->SimTime();
  #else
    const common::Time kTimeNow = this->world->GetSimTime();
  #endif
  double dt = (kTimeNow - this->prevUpdateTime).Double();
  //ROS_INFO_STREAM(dt);
  this->prevUpdateTime = kTimeNow;

  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  #if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Pose3d kPose = this->link->WorldPose();
    //ROS_INFO_STREAM("POSE:");
 // ROS_INFO_STREAM("POSE: " <<kPose);
  #else
    const ignition::math::Pose3d kPose = this->link->GetWorldPose().Ign();

  #endif
  const ignition::math::Vector3d kEuler = kPose.Rot().Euler();
// ROS_INFO_STREAM("RPY: " <<kEuler*57.3);

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

  // Estimate the linear and angular accelerations.
  // Note the the GetRelativeLinearAccel() and AngularAccel() functions
  // appear to be unreliable
  
  ignition::math::Vector3d kAccelLinearBody =  
 // ROS_INFO_STREAM(kAccelLinearBody);
  (kVelLinearBody - this->prevLinVel) / dt;
  this->prevLinVel = kVelLinearBody;
  //ROS_DEBUG_STREAM_THROTTLE(0.5, "Accel linear: " << kAccelLinearBody);
  ignition::math::Vector3d kAccelAngularBody = 
  //ROS_INFO_STREAM(kAccelAngularBody);
  (kVelAngularBody - this->prevAngVel) / dt;
  this->prevAngVel = kVelAngularBody;
  //ROS_DEBUG_STREAM_THROTLE(0.5, "Accel angular: " << kAccelAngularBody);
////
  double alpha  = 0.1;
 this->linearAccFiltered = (1.0 - alpha) * this->linearAccFiltered + alpha * kAccelLinearBody;
 this->angularAccFiltered = (1.0 - alpha) * this->angularAccFiltered + alpha * kAccelAngularBody;
  /////
  // Create state and derivative of state (accelerations)
  Eigen::VectorXd stateDot = Eigen::VectorXd(6);
  Eigen::VectorXd state    = Eigen::VectorXd(6);
  // Hydrostatic Force and Moment( HS):
  Eigen::VectorXd HS       = Eigen::VectorXd(6);
  // Get COG:

  stateDot << this->linearAccFiltered.X(), this->linearAccFiltered.Y(), this->linearAccFiltered.Z(),
    this->angularAccFiltered.X(), this->angularAccFiltered.Y(), this->angularAccFiltered.Z();

  state << kVelLinearBody.X(), kVelLinearBody.Y(), kVelLinearBody.Z(),
    kVelAngularBody.X(), kVelAngularBody.Y(), kVelAngularBody.Z();


  // Sum all forces - in body frame
    Eigen::Vector6d kForceSum;

  //Redefine some variable: 
  double u =   kVelLinearBody.X();
  double v = - kVelLinearBody.Y();
  double w = - kVelLinearBody.Z();
  double p =   kVelAngularBody.X();
  double q = - kVelAngularBody.Y();
  double r = - kVelAngularBody.Z();

  double u_dot =   stateDot(0);
  double v_dot = - stateDot(1);
  double w_dot = - stateDot(2);
  double p_dot =   stateDot(3);
  double q_dot = - stateDot(4);
  double r_dot = - stateDot(5);


  //ROS_INFO_STREAM("Linear Vel  : " << kVelLinearBody);
  //ROS_INFO_STREAM("Angualr Vel : " << kVelAngularBody);

  kForceSum(0) = (this->paramXudot*u_dot) + 1*(+ this->paramXuu * u * std::fabs(u) - this->paramYvdot * v * r + this->paramZwdot * w * q  + this->paramZqdot  * q * q   - this->paramYrdot* r * r );
  kForceSum(1) = (this->paramYvdot*v_dot + this->paramYrdot*r_dot)  + 1*(+ this->paramYvv * v * std::fabs(v) + this->paramYrr * r * std::fabs(r) + this->paramXudot * u * r - this->paramZwdot * w * p
           - this->paramZqdot * p * q  + this->paramYuvl* 1 * u * v );
  kForceSum(2) = (this->paramZwdot*w_dot + this->paramZqdot*q_dot)  + 1*(+ this->paramZww * w * std::fabs(w) + this->paramZqq*q*std::fabs(q)+ (-this->paramXudot + this->paramZuqf * 1 ) * u * q + this->paramYvdot * v * p 
           + this->paramYrdot  * r * p + (this->paramZuwl* 1 + this->paramZuwf* 1) * u * w);
  kForceSum(3) = (this->paramKpdot*p_dot)  + 1*(+ this->paramKpp * p * std::fabs(p) + (this->paramZwdot - this->paramYvdot) * w * v + (this->paramZqdot + this->paramYrdot) * v * q - (this->paramZqdot + this->paramYrdot) * w * r 
           + (this->paramNrdot  - this->paramMqdot ) * q *r);
  kForceSum(4) = (this->paramMwdot*w_dot+this->paramMqdot*q_dot)  + 1*( + this->paramMww * w * std::fabs(w) + this->paramMqq * q * std::fabs(q) + (this->paramXudot - this->paramZwdot + this->paramMuwl* 1 + this->paramMuwf* 1) * u * w  - this->paramYrdot * v * p 
           + (this->paramKpdot - this->paramNrdot ) * r * p + (this->paramMuqf * 1 - this->paramMwdot ) * u * q );
  kForceSum(5) = (this->paramNvdot*v_dot + this->paramNrdot*r_dot)  + 1*(+ this->paramNvv * v * std::fabs(v) + this->paramNrr * r * std::fabs(r) + (this->paramNuvl * 1 - this->paramXudot + this->paramYvdot) * u * v / 10  + this->paramZqdot  * w * p 
           + (this->paramYrdot ) * u * r + (this->paramMqdot - this->paramKpdot ) * p * q );          
  // Forces in fixed frame

  ROS_DEBUG_STREAM_THROTTLE(1.0, "forceSum :\n" << kForceSum);
  if (!std::isnan(kForceSum.norm()))
  {
  this->link->AddRelativeForce(
   ignition::math::Vector3d(kForceSum(0), - kForceSum(1), - kForceSum(2)));
  this->link->AddRelativeTorque(
    ignition::math::Vector3d(kForceSum(3), - kForceSum(4), - kForceSum(5)));
  }  
}

ignition::math::Vector3d AUVDynamicsPlugin::ToNED(ignition::math::Vector3d _vec)
{
  ignition::math::Vector3d output = _vec;
  output.Y() = -1 * output.Y();
  output.Z() = -1 * output.Z();
  return output;
}

/////////////////////////////////////////////////
ignition::math::Vector3d AUVDynamicsPlugin::FromNED(ignition::math::Vector3d _vec)
{
  return this->ToNED(_vec);
}

GZ_REGISTER_MODEL_PLUGIN(AUVDynamicsPlugin);