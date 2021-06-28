#include "auv_controller.h"

AUV_Controller::AUV_Controller()
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("roll_Kp", rollPID.Kp);
  private_nh.getParam("roll_Ki", rollPID.Ki);
  private_nh.getParam("roll_Kd", rollPID.Kd);
  private_nh.getParam("ned_lat", ned_lat);
  private_nh.getParam("ned_lon", ned_lon);
  ros::NodeHandle nh;
  
  pubThrustCmd      = nh.advertise<motor_stamped>("thrust/cmd", 1);  // Send Thruster Speed
  pubPistonCmd      = nh.advertise<motor_stamped>("piston/cmd", 1);  // Send Piston piston
  pubIRMCmd         = nh.advertise<anti_rolling_stamped>("irm1/cmd",1); // Send anti rolling angle
  pubMassShifterCmd = nh.advertise<motor_stamped>("mass_shifter/cmd", 1);  // Send Mass Shifter piston
  pubRudderCmd      = nh.advertise<mx28_stamped>("rudder/cmd", 1);  // Send Rudder position

  loopControl = nh.createTimer(ros::Duration(0.1), &AUV_Controller::onControlLoop, this);
  subSensor = nh.subscribe("my_auv2000_sensors/gps_dvl_ins_data", 10, &AUV_Controller::onSensorCallBack, this);
  subItemList = nh.subscribe("mission/item_list", 1, &AUV_Controller::onItemListCallBack, this);
  subSonarScan = nh.subscribe("sonar", 10, &AUV_Controller::onSonarScanCallBack, this);
  subJoystick = nh.subscribe("joystick/data",1,&AUV_Controller::onGetJoystickCallBack,this);
  resStartMission = nh.advertiseService("command/start_Mission", &AUV_Controller::onStartMissionCallBack, this);
  resSetMode = nh.advertiseService("command/set_Mode", &AUV_Controller::onSetModeCallBack, this);
  resSetAUVController = nh.advertiseService("command/set_AUV_Controller", &AUV_Controller::onSetAUVControllerCallBack, this);


  reqSetSpeed_PID = nh.advertiseService("parameter/set_Speed_PID", &AUV_Controller::onSetSpeed_PIDCallBack, this);
  reqGetSpeed_PID = nh.advertiseService("parameter/get_Speed_PID", &AUV_Controller::onGetSpeed_PIDCallBack, this);
  reqSetHeading_PID = nh.advertiseService("parameter/set_Heading_PID", &AUV_Controller::onSetHeading_PIDCallBack, this);
  reqGetHeading_PID = nh.advertiseService("parameter/get_Heading_PID", &AUV_Controller::onGetHeading_PIDCallBack, this);
  reqSetPitch_PID = nh.advertiseService("parameter/set_Pitch_PID", &AUV_Controller::onSetPitch_PIDCallBack, this);
  reqGetPitch_PID = nh.advertiseService("parameter/get_Pitch_PID", &AUV_Controller::onGetPitch_PIDCallBack, this);
  reqSetDepth_PID = nh.advertiseService("parameter/set_Depth_PID", &AUV_Controller::onSetDepth_PIDCallBack, this);
  reqGetDepth_PID = nh.advertiseService("parameter/get_Depth_PID", &AUV_Controller::onGetDepth_PIDCallBack, this);
  reqSetLOS_PID = nh.advertiseService("parameter/set_LOS_PID", &AUV_Controller::onSetLOS_PIDCallBack, this);
  reqGetLOS_PID = nh.advertiseService("parameter/get_LOS_PID", &AUV_Controller::onGetLOS_PIDCallBack, this);


  reqSetSpeed_SM = nh.advertiseService("parameter/set_Speed_SM", &AUV_Controller::onSetSpeed_SMCallBack, this);
  reqGetSpeed_SM = nh.advertiseService("parameter/get_Speed_SM", &AUV_Controller::onGetSpeed_SMCallBack, this);
  reqSetHeading_SM = nh.advertiseService("parameter/set_Heading_SM", &AUV_Controller::onSetHeading_SMCallBack, this);
  reqGetHeading_SM = nh.advertiseService("parameter/get_Heading_SM", &AUV_Controller::onGetHeading_SMCallBack, this);
  reqSetPitch_SM = nh.advertiseService("parameter/set_Pitch_SM", &AUV_Controller::onSetPitch_SMCallBack, this);
  reqGetPitch_SM = nh.advertiseService("parameter/get_Pitch_SM", &AUV_Controller::onGetPitch_SMCallBack, this);
  reqSetDepth_SM = nh.advertiseService("parameter/set_Depth_SM", &AUV_Controller::onSetDepth_SMCallBack, this);
  reqGetDepth_SM = nh.advertiseService("parameter/get_Depth_SM", &AUV_Controller::onGetDepth_SMCallBack, this);
  reqSetLOS_SM = nh.advertiseService("parameter/set_LOS_SM", &AUV_Controller::onSetLOS_SMCallBack, this);
  reqGetLOS_SM = nh.advertiseService("parameter/get_LOS_SM", &AUV_Controller::onGetLOS_SMCallBack, this);

  reqSetLOS_Config = nh.advertiseService("parameter/set_LOS_Config", &AUV_Controller::onSetLOS_ConfigCallBack, this);
  reqGetLOS_Config = nh.advertiseService("parameter/get_LOS_Config", &AUV_Controller::onGetLOS_ConfigCallBack, this);
  resGetComPassParam=nh.advertiseService("parameter/get_ComPass_param",&AUV_Controller::OnGetComPassCallBack,this);
  resGetBatteryParam=nh.advertiseService("parameter/get_battery_param",&AUV_Controller::OnGetBatteryCallBack,this);
  resGetFailSafeParam=nh.advertiseService("parameter/get_failsafe_param",&AUV_Controller::OnGetFailSafeCallBack,this);
   resGetARMINGParam=nh.advertiseService("parameter/get_arming_param",&AUV_Controller::OnGetARMINGCallBack,this);

  resGetJoystickConfigParamReal32=nh.advertiseService("parameter/get_joystick_config_param_real32",&AUV_Controller::OnGetJoystickConfigParamReal32CallBack,this);
  resGetJoystickConfigParamInt16=nh.advertiseService("parameter/get_joystick_config_param_int16",&AUV_Controller::OnGetJoystickConfigParamInt16CallBack,this);
  resGetJoystickConfigParamInt8=nh.advertiseService("parameter/get_joystick_config_param_int8",&AUV_Controller::OnGetJoystickConfigParamInt8CallBack,this);
  resGetJoystickFunctionParam=nh.advertiseService("parameter/get_joystick_function_param",&AUV_Controller::OnGetJoystickFunctionParamCallBack,this);
  
  lastControlUpdateTime = lastSetpointTime = ros::Time::now();



  speedPID.upper_bound       = 100;
  speedPID.lower_bound       = 0;

  rollPID.upper_bound        =  0.53;
  rollPID.lower_bound        = -0.53;

  headingPID.upper_bound     =   0.78;
  headingPID.lower_bound     = - 0.78;

  pitchPID.upper_bound       =   0.2;
  pitchPID.lower_bound       = - 0.2;

  LOS_DepthPID.upper_bound   =   0.2;
  LOS_DepthPID.lower_bound   = - 0.2;

  LOS_PID_Heading.upper_bound       =    0.78;
  LOS_PID_Heading.lower_bound        = - 0.78;

}

AUV_Controller::~AUV_Controller()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void AUV_Controller::onSonarScanCallBack(const LaserScan::ConstPtr& msg)
{
   cnt += 1;
   if (cnt == 10){
  if (std::fabs(currPitch) < 0.08 )
   {
      psi_oa_tot = straightLOSGuider.Calc_psi_oa(currHeading, msg->ranges.data(), msg->ranges.size(), msg->angle_min, msg->angle_increment);
       if((psi_oa_tot != 0) && (OA_FLAG == 0))
       {
          OA_FLAG = 1 ;
          psi_current_0 = currHeading ;
       }
       else if(psi_oa_tot == 0) 
       {
          OA_FLAG = 0;
       }

      ROS_INFO_STREAM("psi_oa_tot  = " << psi_oa_tot*57.3);
      ROS_INFO_STREAM("left_bar   = " <<  straightLOSGuider.left_bar);
      ROS_INFO_STREAM("center_bar   = " <<  straightLOSGuider.center_bar);
      ROS_INFO_STREAM("right_bar   = " <<  straightLOSGuider.right_bar);
      ROS_INFO_STREAM("MODE OA  = " << OA_FLAG);
  }
  cnt = 0;
   }
}

void AUV_Controller::onSensorCallBack(const gps_dvl_ins_stamped::ConstPtr& msg)
{
  currSpeed   = msg->ekf_vX ;
  currHeading = msg->ekf_yaw/57.2957795131;
  currRoll    = msg->ekf_roll/57.2957795131 ;
  currPitch   = msg->ekf_pitch/57.2957795131;
  currDepth   = msg->ekf_alt;
  convert_global_to_local_coords(msg->ekf_lat, msg->ekf_lon, ned_lat, ned_lon, currX, currY);
  //ROS_INFO_STREAM("currX = " << currX << " currY = "<< currY << "currZ = "<< currDepth);
}

void AUV_Controller::onControlLoop(const ros::TimerEvent& event)
{
  double dtc = (event.current_real - lastSetpointTime).toSec();
  double dt = event.current_real.toSec() - lastControlUpdateTime.toSec();


/////////////////anti rolling tern //////////////////////////////////
if (currSpeed > 0.1)
{
  rollPID.error = currRoll - 0;
  rollPID.Ts = dt;
  rollPID.runPID();
  irm_msg.angle = rollPID.output;
  pubIRMCmd.publish(irm_msg);
}
//////////////////Thruster tern .////////////////////////////////////////
  if (manualEnable == 0)
  {
     if(speedPIDEnable||isMissionStarted)
     {
         speedPID.error = desiredSpeed - currSpeed;
         speedPID.Ts = dt;
         speedPID.runPID();
         thrust_msg.motor_rspeed = convertForcetoSpeed(speedPID.output);
         pubThrustCmd.publish(thrust_msg);
     }
////////////PISTON TERN ////////////////////////////////////////////////
      piston_msg.position = 0.04;
      pubPistonCmd.publish(piston_msg);


///////////////////////Rudder Tern /////////////////////////////////////
     if (headingPIDEnable)
      {     
         headingPID.error = atan2(sin(desiredHeading - currHeading), cos(desiredHeading - currHeading));
         headingPID.Ts = dt;
         headingPID.runPID();
         rudder_msg.position = headingPID.output ; 
         pubRudderCmd.publish(rudder_msg);
      }
      else if(LOS_PID_Heading_Enable)
      {

         if (straightLOSGuider.runLOS(currX, currY, currDepth))
         {
            
            //ROS_INFO_STREAM("desired heading = " <<straightLOSGuider.desiredHeading);
           // ROS_INFO_STREAM("desired pitch = " <<straightLOSGuider.desiredPitch);
if (OA_FLAG == 1)
{
   LOS_PID_Heading.error = atan2(sin(psi_current_0 - currHeading + psi_oa_tot), cos(psi_current_0 - currHeading + psi_oa_tot));
}
else {
            LOS_PID_Heading.error = atan2(sin(straightLOSGuider.desiredHeading - currHeading + psi_oa_tot), cos(straightLOSGuider.desiredHeading - currHeading + psi_oa_tot));
}
            LOS_PID_Heading.Ts = dt;
            LOS_PID_Heading.runPID();
            rudder_msg.position = LOS_PID_Heading.output ; 
            pubRudderCmd.publish(rudder_msg);

            pitchPID.error = atan2(sin(currPitch - straightLOSGuider.desiredPitch), cos(currPitch - straightLOSGuider.desiredPitch)); 
            pitchPID.Ts = dt;
            pitchPID.runPID();
            mass_shifter_msg.position = pitchPID.output; 
            //ROS_INFO_STREAM("mass shifter output " <<  pitchPID.output);
            pubMassShifterCmd.publish(mass_shifter_msg);
         }
         else
         isMissionStarted = false;  
      }
      

///////////////// mass shifter tern ///////////////////////////////////////
      mass_shifter_msg.position = 0;
      if(pitchPIDEnable)
      {
         pitchPID.error = atan2(sin(currPitch - desiredPitch), cos(currPitch - desiredPitch)); 
         pitchPID.Ts = dt;
         pitchPID.runPID();
         mass_shifter_msg.position = pitchPID.output; 
         pubMassShifterCmd.publish(mass_shifter_msg);
      }
      else if(LOS_DepthPIDEnable)
      {
         double pitch_des = atan2(currDepth - desiredDepth ,10);
         if (pitch_des > 0.69)
            pitch_des = 0.69 ;
         else if (pitch_des < -0.69)
         {
         pitch_des  = -0.69 ;
         }
      LOS_DepthPID.error = atan2(sin(currPitch - pitch_des), cos(currPitch - pitch_des));  
      LOS_DepthPID.Ts = dt;
      LOS_DepthPID.runPID();
      mass_shifter_msg.position = LOS_DepthPID.output;
      pubMassShifterCmd.publish(mass_shifter_msg);
      }
   
///////////////////////////////////////////////////////////////////////////////////
   
  }
 else
  {
    rudder_msg.position     = joystick_msg.rudder/57.2957795131;
    thrust_msg.motor_rspeed = joystick_msg.thruster;
    if (joystick_msg.mass_shifter_up)
    {
       massShifterPositionFromJoyStick += 0.01;
       if (massShifterPositionFromJoyStick > 0.2)
       {
          massShifterPositionFromJoyStick = 0.2;
       }
    }

    if (joystick_msg.mass_shifter_down)
    {
       massShifterPositionFromJoyStick -= 0.01;
       if (massShifterPositionFromJoyStick < -0.2)
       {
          massShifterPositionFromJoyStick = -0.2;
       }
    }

   if (joystick_msg.pistol_up)
    {
       pistonPositionFromJoyStick += 0.01;
       if (pistonPositionFromJoyStick > 0.08)
       {
          pistonPositionFromJoyStick = 0.08;
       }
    }


   if (joystick_msg.pistol_down)
    {
       pistonPositionFromJoyStick -= 0.01;
       if (pistonPositionFromJoyStick < 0)
       {
          pistonPositionFromJoyStick = 0;
       }
    }

    irm_msg.angle = 0;
    mass_shifter_msg.position = massShifterPositionFromJoyStick ; 
    piston_msg.position = pistonPositionFromJoyStick;


    pubThrustCmd.publish(thrust_msg);
    pubMassShifterCmd.publish(mass_shifter_msg);
    pubPistonCmd.publish(piston_msg);
    pubRudderCmd.publish(rudder_msg);
  }



  lastControlUpdateTime = event.current_real;
}

void AUV_Controller::onItemListCallBack(const WaypointList::ConstPtr& msg)
{
   speedPID.resetPID();
   LOS_PID_Heading.resetPID();
   pitchPID.resetPID();
   straightLOSGuider.resetLOS();
  for (auto it = msg->waypoints.begin(); it != msg->waypoints.end(); it++)
  {
    switch (it->command)
    {
    case 16: // MAV_CMD_NAV_WAYPOINT
     // ROS_INFO_STREAM("Waypoint: "
   //                   << "lat = " << it->x_lat * 1e-7 << ", lon = " << it->y_long * 1e-7 << ", alt = " << it->z_alt
    //                  << ".");
      double x, y, z;
      convert_global_to_local_coords(it->x_lat * 1e-7, it->y_long * 1e-7, ned_lat, ned_lon, x, y);
      z = it -> z_alt ;
      ROS_INFO_STREAM("Waypoint: x = " << x << ", y = " << y << ", z = " <<z);
      BaseLOS::waypoints.push_back(BaseLOS::Point(x, y, z));
      break;
    case 20: // MAV_CMD_NAV_RETURN_TO_LAUNCH
      ROS_INFO_STREAM("Waypoint (return to launch): "
                      << "lat = (1st waypoint lat), lon = (1st waypoint lon)");
      break;
    }
  }
  straightLOSGuider.setupLOS();
}

bool AUV_Controller::onStartMissionCallBack(CommandLongRequest& /*req*/, CommandLongResponse& res)
{
  ROS_INFO("Mission started.");
  res.result = 0; // MAV_RESULT_ACCEPTED
   isMissionStarted = true ;
  return true;
}

bool AUV_Controller::onSetModeCallBack(SetModeRequest& req, SetModeResponse& res)
{
  ROS_INFO_STREAM("Base mode set to " << int(req.base_mode) << ", custom mode set to " << req.custom_mode << ".");
  res.mode_sent = 0; // MAV_RESULT_ACCEPTED
if ( req.custom_mode == "19")
{
   manualEnable = 1;
   ROS_INFO_STREAM("MANUAL ENABLE");
}
else
{
   manualEnable = 0 ;
}
  return true;
}

bool AUV_Controller::onSetAUVControllerCallBack(CommandLongRequest& req, CommandLongResponse& res)
{
  if (static_cast<int>(req.param1) == HEADING_PID){
      if(static_cast<int>(req.param2) == 1){
        ROS_INFO("AUV CONTROLLER: HEADING_PID --> ENABLE");
        headingPIDEnable = 1;
        headingPID.resetPID();
      }
      else{
        ROS_INFO("AUV CONTROLLER: HEADING_PID --> DISABLE");
        headingPIDEnable = 0;
      }
  }
  else if (static_cast<int>(req.param1) == HEADING_SM){
      if(static_cast<int>(req.param2) == 1){
        ROS_INFO("AUV CONTROLLER: HEADING_SM --> ENABLE");
      }
      else{
        ROS_INFO("AUV CONTROLLER: HEADING_SM --> DISABLE");
      }
  }
  else if (static_cast<int>(req.param1) == SPEED_PID){
      if(static_cast<int>(req.param2) == 1){
        ROS_INFO("AUV CONTROLLER: SPEED_PID --> ENABLE");
         speedPIDEnable = 1;
         speedPID.resetPID();
      }
      else{
        ROS_INFO("AUV CONTROLLER: SPEED_PID --> DISABLE");
        speedPIDEnable = 0;
      }
  }
  else if (static_cast<int>(req.param1) == SPEED_SM){
      if(static_cast<int>(req.param2) == 1){
        ROS_INFO("AUV CONTROLLER: SPEED_SM --> ENABLE");
      }
      else{
        ROS_INFO("AUV CONTROLLER: SPEED_SM --> DISABLE");
      }
  }
  else if (static_cast<int>(req.param1) == DEPTH_PID){
      if(static_cast<int>(req.param2) == 1){
        ROS_INFO("AUV CONTROLLER: DEPTH_PID --> ENABLE");
        LOS_DepthPIDEnable = 1;
        LOS_DepthPID.resetPID();
      }
      else{
        ROS_INFO("AUV CONTROLLER: DEPTH_PID --> DISABLE");
        LOS_DepthPIDEnable = 0;
      }
  }
  else if (static_cast<int>(req.param1) == DEPTH_SM){
      if(static_cast<int>(req.param2) == 1){
        ROS_INFO("AUV CONTROLLER: DEPTH_SM --> ENABLE");
      }
      else{
        ROS_INFO("AUV CONTROLLER: DEPTH_SM --> DISABLE");
      }
  }
  else if (static_cast<int>(req.param1) == PITCH_PID){
      if(static_cast<int>(req.param2) == 1){
        ROS_INFO("AUV CONTROLLER: PITCH_PID --> ENABLE");
        pitchPIDEnable = 1; 
        pitchPID.resetPID();
      }
      else{
        ROS_INFO("AUV CONTROLLER: PITCH_PID --> DISABLE");
        pitchPIDEnable = 0;
      }
  }
  else if (static_cast<int>(req.param1) == PITCH_SM){
      if(static_cast<int>(req.param2) == 1){
        ROS_INFO("AUV CONTROLLER: PITCH_SM --> ENABLE");
      }
      else{
        ROS_INFO("AUV CONTROLLER: PITCH_SM --> DISABLE");
      }
  }
  else if (static_cast<int>(req.param1) == LOS_PID){
      if(static_cast<int>(req.param2) == 1){
        ROS_INFO("AUV CONTROLLER: LOS_PID --> ENABLE");
        LOS_PID_Heading_Enable = 1;
        
      }
      else{
        ROS_INFO("AUV CONTROLLER: LOS_PID --> DISABLE");
        LOS_PID_Heading_Enable = 0;
      }
  }
  else if (static_cast<int>(req.param1) == LOS_SM){
      if(static_cast<int>(req.param2) == 1){
        ROS_INFO("AUV CONTROLLER: LOS_SM --> ENABLE");
      }
      else{
        ROS_INFO("AUV CONTROLLER: LOS_SM --> DISABLE");
      }
  }

  res.result = 0; // MAV_RESULT_ACCEPTED
  return true;
}

bool AUV_Controller::OnGetJoystickConfigParamReal32CallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;

  if(compareString(req.param_id.data(),"JS_GAIN_DEFAULT"))
  {
     res.value.real=0.5;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"JS_GAIN_MAX"))
  {
     res.value.real=1.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"JS_GAIN_MIN"))
  {
     res.value.real=0.25;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"JS_THR_GAIN"))
  {
     res.value.real=1.0;
     res.success=true;
  }

  return res.success;
}

bool AUV_Controller::OnGetJoystickConfigParamInt16CallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;

  if(compareString(req.param_id.data(),"JS_CAM_TILT_STEP"))
  {
     res.value.integer=50;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"JS_LIGHTS_STEP"))
  {
     res.value.integer=100;
     res.success=true;
  }

  return res.success;
}

bool AUV_Controller::OnGetJoystickConfigParamInt8CallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;

  if(compareString(req.param_id.data(),"JS_GAIN_STEPS"))
  {
     res.value.integer=4;
     res.success=true;
  }

  return res.success;
}

bool AUV_Controller::OnGetJoystickFunctionParamCallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;

  if(compareString(req.param_id.data(),"BTN0_FUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN0_SFUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN10_FUNCTION"))
  {
     res.value.integer=22;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN10_SFUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN11_FUNCTION"))
  {
     res.value.integer=42;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN11_SFUNCTION"))
  {
     res.value.integer=47;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN12_FUNCTION"))
  {
     res.value.integer=43;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN12_SFUNCTION"))
  {
     res.value.integer=46;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN13_FUNCTION"))
  {
     res.value.integer=33;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN13_SFUNCTION"))
  {
     res.value.integer=45;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN14_FUNCTION"))
  {
     res.value.integer=32;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN14_SFUNCTION"))
  {
     res.value.integer=44;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN15_FUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN15_SFUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN1_FUNCTION"))
  {
     res.value.integer=12;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN1_SFUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN2_FUNCTION"))
  {
     res.value.integer=7;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN2_SFUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN3_FUNCTION"))
  {
     res.value.integer=6;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN3_SFUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN4_FUNCTION"))
  {
     res.value.integer=4;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN4_SFUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN5_FUNCTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN5_SFUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN6_FUNCTION"))
  {
     res.value.integer=3;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN6_SFUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN7_FUNCTION"))
  {
     res.value.integer=21;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN7_SFUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN8_FUNCTION"))
  {
     res.value.integer=48;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN8_SFUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN9_FUNCTION"))
  {
     res.value.integer=23;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BTN9_SFUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
   return res.success;
}

bool AUV_Controller::OnGetARMINGCallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;
  if(compareString(req.param_id.data(),"ARMING_CHECK"))
  {
     res.value.integer=1478;
     res.success=true;
  }
   if(compareString(req.param_id.data(),"ARMING_MIN_VOLT"))
  {
     res.value.real=5.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"ARMING_MIN_VOLT2"))
  {
     res.value.real=5.0;
     res.success=true;
  }
  return res.success;
}


bool AUV_Controller::onSetSpeed_PIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "SPEED_PID_DES"))
  {
    ROS_INFO_STREAM("SPEED_PID_DESIRED set to " << req.value.real << ".");
    desiredSpeed = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "SPEED_PID_KP"))
  {
    ROS_INFO_STREAM("SPEED_PID_KP set to " << req.value.real << ".");
    speedPID.Kp = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "SPEED_PID_KI"))
  {
    ROS_INFO_STREAM("SPEED_PID_KI set to " << req.value.real << ".");
    speedPID.Ki = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "SPEED_PID_KD"))
  {
    ROS_INFO_STREAM("SPEED_PID_KD set to " << req.value.real << ".");
    speedPID.Kd = req.value.real;
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool AUV_Controller::onGetSpeed_PIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;
  ROS_INFO_STREAM("adhfhdsjfdksfhkjdhfjkhdskfjhdsjfjksdhkfjshkdjfsd");

  if (compareString(req.param_id.data(), "SPEED_PID_DES"))
  {
    res.value.real = 1;
    desiredSpeed   = 1;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "SPEED_PID_KP"))
  {
    res.value.real = 8;
    speedPID.Kp = 8;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "SPEED_PID_KI"))
  {
    res.value.real = 3;
    speedPID.Ki = 3;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "SPEED_PID_KD"))
  {
    res.value.real = 0.5;
    speedPID.Kd = 0.5;
    res.success = true;
  }

  return res.success;
}

//888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888
bool AUV_Controller::onSetHeading_PIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "HEADING_PID_DES"))
  {
    ROS_INFO_STREAM("HEADING_PID_DESIRED set to " << req.value.real << ".");
    desiredHeading = req.value.real/57.2957795131;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_PID_KP"))
  {
    ROS_INFO_STREAM("HEADING_PID_KP set to " << req.value.real << ".");
    headingPID.Kp = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_PID_KI"))
  {
    ROS_INFO_STREAM("HEADING_PID_KI set to " << req.value.real << ".");
    headingPID.Ki = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_PID_KD"))
  {
    ROS_INFO_STREAM("HEADING_PID_KD set to " << req.value.real << ".");
    headingPID.Kd = req.value.real;
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool AUV_Controller::onGetHeading_PIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "HEADING_PID_DES"))
  {
    res.value.real = 0;
    desiredHeading = 0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_PID_KP"))
  {
    res.value.real = 5;
    headingPID.Kp = 5 ;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_PID_KI"))
  {
    res.value.real = 0;
    headingPID.Ki = 0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_PID_KD"))
  {
    res.value.real = 7;
    headingPID.Kd = 7;
    res.success = true;
  }

  return res.success;
}

bool AUV_Controller::onSetPitch_PIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "PITCH_PID_DES"))
  {
    ROS_INFO_STREAM("PITCH_PID_DESIRED set to " << req.value.real << ".");
   // desiredPitch = req.value.real/57.2957795131;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_PID_KP"))
  {
    ROS_INFO_STREAM("PITCH_PID_KP set to " << req.value.real << ".");
    pitchPID.Kp = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_PID_KI"))
  {
    ROS_INFO_STREAM("PITCH_PID_KI set to " << req.value.real << ".");
    pitchPID.Ki = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_PID_KD"))
  {
    ROS_INFO_STREAM("PITCH_PID_KD set to " << req.value.real << ".");
    pitchPID.Kd = req.value.real;
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool AUV_Controller::onGetPitch_PIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "PITCH_PID_DES"))
  {
    res.value.real = 0;
   // desiredPitch = 0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_PID_KP"))
  {
    res.value.real = 1.25;
    pitchPID.Kp = 1.25 ; 
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_PID_KI"))
  {
    res.value.real = 0.1;
    pitchPID.Ki = 0.1; 
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_PID_KD"))
  {
    res.value.real = 5;
    pitchPID.Kd = 5 ;
    res.success = true;
  }

  return res.success;
}

bool AUV_Controller::onSetDepth_PIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "DEPTH_PID_DES"))
  {
    ROS_INFO_STREAM("DEPTH_PID_DESIRED set to " << req.value.real << ".");
    desiredDepth = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_PID_KP"))
  {
    ROS_INFO_STREAM("DEPTH_PID_KP set to " << req.value.real << ".");
    LOS_DepthPID.Kp = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_PID_KI"))
  {
    ROS_INFO_STREAM("DEPTH_PID_KI set to " << req.value.real << ".");
     LOS_DepthPID.Ki = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_PID_KD"))
  {
    ROS_INFO_STREAM("DEPTH_PID_KD set to " << req.value.real << ".");
     LOS_DepthPID.Kd = req.value.real;
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool AUV_Controller::onGetDepth_PIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "DEPTH_PID_DES"))
  {
    res.value.real = 10;
    desiredDepth = 10 ; 
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_PID_KP"))
  {
    res.value.real = 1.25;
    LOS_DepthPID.Kp = 1.25;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_PID_KI"))
  {
    res.value.real = 0.1;
    LOS_DepthPID.Ki = 0.1 ;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_PID_KD"))
  {
    res.value.real = 5;
    LOS_DepthPID.Kd = 5;
    res.success = true;
  }

  return res.success;
}

bool AUV_Controller::onSetLOS_PIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "LOS_PID_DES"))
  {
    ROS_INFO_STREAM("LOS_PID_DESIRED set to " << req.value.real << ".");

    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_PID_KP"))
  {
    ROS_INFO_STREAM("LOS_PID_KP set to " << req.value.real << ".");
        LOS_PID_Heading.Kp = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_PID_KI"))
  {
    ROS_INFO_STREAM("LOS_PID_KI set to " << req.value.real << ".");
        LOS_PID_Heading.Ki = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_PID_KD"))
  {
    ROS_INFO_STREAM("LOS_PID_KD set to " << req.value.real << ".");
        LOS_PID_Heading.Kd = req.value.real;
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool AUV_Controller::onGetLOS_PIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "LOS_PID_DES"))
  {
    res.value.real = 0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_PID_KP"))
  {
    res.value.real = 5;
    LOS_PID_Heading.Kp = 5;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_PID_KI"))
  {
    res.value.real = 0;
    LOS_PID_Heading.Ki = 0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_PID_KD"))
  {
    res.value.real = 7;
    LOS_PID_Heading.Kd = 7;
    res.success = true;
  }

  return res.success;
}


bool AUV_Controller::onSetSpeed_SMCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "SPEED_SM_DES"))
  {
    ROS_INFO_STREAM("SPEED_SM_DESIRED set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "SPEED_SM_E"))
  {
    ROS_INFO_STREAM("SPEED_SM_EPSILON set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "SPEED_SM_K"))
  {
    ROS_INFO_STREAM("SPEED_SM_K set to " << req.value.real << ".");
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool AUV_Controller::onGetSpeed_SMCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "SPEED_SM_DES"))
  {
    res.value.real = 16.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "SPEED_SM_E"))
  {
    res.value.real = 17.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "SPEED_SM_K"))
  {
    res.value.real = 18.0;
    res.success = true;
  }

  return res.success;
}
//----------------------------------------------------------------
bool AUV_Controller::onSetHeading_SMCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "HEADING_SM_DES"))
  {
    ROS_INFO_STREAM("HEADING_SM_DESIRED set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_SM_E"))
  {
    ROS_INFO_STREAM("HEADING_SM_EPSILON set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_SM_K"))
  {
    ROS_INFO_STREAM("HEADING_SM_K set to " << req.value.real << ".");
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool AUV_Controller::onGetHeading_SMCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "HEADING_SM_DES"))
  {
    res.value.real = 16.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_SM_E"))
  {
    res.value.real = 17.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_SM_K"))
  {
    res.value.real = 18.0;
    res.success = true;
  }

  return res.success;
}

bool AUV_Controller::onSetPitch_SMCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "PITCH_SM_DES"))
  {
    ROS_INFO_STREAM("PITCH_SM_DESIRED set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_SM_E"))
  {
    ROS_INFO_STREAM("PITCH_SM_EPSILON set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_SM_K"))
  {
    ROS_INFO_STREAM("PITCH_SM_K set to " << req.value.real << ".");
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool AUV_Controller::onGetPitch_SMCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "PITCH_SM_DES"))
  {
    res.value.real = 19.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_SM_E"))
  {
    res.value.real = 20.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_SM_K"))
  {
    res.value.real = 21.0;
    res.success = true;
  }

  return res.success;
}

bool AUV_Controller::onSetDepth_SMCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "DEPTH_SM_DES"))
  {
    ROS_INFO_STREAM("DEPTH_SM_DESIRED set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_SM_E"))
  {
    ROS_INFO_STREAM("DEPTH_SM_EPSILON set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_SM_K"))
  {
    ROS_INFO_STREAM("DEPTH_SM_K set to " << req.value.real << ".");
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool AUV_Controller::onGetDepth_SMCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "DEPTH_SM_DES"))
  {
    res.value.real = 22.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_SM_E"))
  {
    res.value.real = 23.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_SM_K"))
  {
    res.value.real = 24.0;
    res.success = true;
  }

  return res.success;
}

bool AUV_Controller::onSetLOS_SMCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "LOS_SM_DES"))
  {
    ROS_INFO_STREAM("LOS_SM_DESIRED set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_SM_E"))
  {
    ROS_INFO_STREAM("LOS_SM_EPSILON set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_SM_K"))
  {
    ROS_INFO_STREAM("LOS_SM_K set to " << req.value.real << ".");
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool AUV_Controller::onGetLOS_SMCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "LOS_SM_DES"))
  {
    res.value.real = 25.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_SM_E"))
  {
    res.value.real = 26.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_SM_K"))
  {
    res.value.real = 27.0;
    res.success = true;
  }

  return res.success;
}

bool AUV_Controller::onSetLOS_ConfigCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "LOS_RADIUS"))
  {
    ROS_INFO_STREAM("LOS_RADIUS set to " << req.value.real << ".");
    BaseLOS::radius = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_DELTA_MIN"))
  {
    ROS_INFO_STREAM("LOS_DELTA_MIN set to " << req.value.real << ".");
    BaseLOS::minDelta =  req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_DELTA_MAX"))
  {
    ROS_INFO_STREAM("LOS_DELTA_MAX set to " << req.value.real << ".");
    BaseLOS::maxDelta =  req.value.real ;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_BETA"))
  {
    ROS_INFO_STREAM("LOS_BETA set to " << req.value.real << ".");
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool AUV_Controller::onGetLOS_ConfigCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "LOS_RADIUS"))
  {
    res.value.real = 5;
   BaseLOS::radius =  5;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_DELTA_MIN"))
  {
    res.value.real = 5;
 BaseLOS::minDelta =  5;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_DELTA_MAX"))
  {
    res.value.real = 10;
   BaseLOS::maxDelta =  10;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_BETA"))
  {
    res.value.real = 31.0;
    res.success = true;
  }

  return res.success;
}

bool AUV_Controller::OnGetComPassCallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;
  if(compareString(req.param_id.data(),"COMPASS_DEC"))
  {
     res.value.integer=10;
     res.success=true;
  }
  return res.success;
}

bool AUV_Controller::OnGetBatteryCallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;
  if(compareString(req.param_id.data(),"BATT_MONITOR"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT2_MONITOR"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT_CAPACITY"))
  {
     res.value.integer=13000;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT2_CAPACITY"))
  {
     res.value.integer=13000;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT_CURR_PIN"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT2_CURR_PIN"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT_VOLT_PIN"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"BATT2_VOLT_PIN"))
  {
     res.value.integer=1;
     res.success=true;
  }

  return res.success;
}

bool AUV_Controller::OnGetFailSafeCallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;
  if(compareString(req.param_id.data(),"FS_PRESS_ENABLE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_PRESS_MAX"))
  {
     res.value.integer=10000;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_TEMP_ENABLE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_TEMP_MAX"))
  {
     res.value.integer=60;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_EKF_ACTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_EKF_THRESH"))
  {
     res.value.real=0.8;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_BATT_ENABLE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_BATT_MAH"))
  {
     res.value.integer=10000;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_BATT_VOLTAGE"))
  {
     res.value.real=12.0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_TERRAIN_ENAB"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_LEAK_ENABLE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_GCS_ENABLE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_PILOT_INPUT"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"FS_PILOT_TIMEOUT"))
  {
     res.value.real=1.00;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"SERVO10_FUNCTION"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"RC_FEEL_RP"))
  {
     res.value.integer=50;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"WPNAV_RFND_USE"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_AUTO_CONFIG"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_AUTO_SWITCH"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_BLEND_MASK"))
  {
     res.value.integer=5;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_GNSS_MODE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_GNSS_MODE2"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_INJECT_TO"))
  {
     res.value.integer=127;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_MIN_DGPS"))
  {
     res.value.integer=100;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_MIN_ELEV"))
  {
     res.value.integer=-100;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_NAVFILTER"))
  {
     res.value.integer=8;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_DELAY_MS"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"GPS_DELAY_MS2"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_ANGMAX_PAN"))
  {
     res.value.integer=4500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_ANGMAX_ROL"))
  {
     res.value.integer=4500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_ANGMAX_TIL"))
  {
     res.value.integer=4500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_ANGMIN_PAN"))
  {
     res.value.integer=-4500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_ANGMIN_ROL"))
  {
     res.value.integer=-4500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_ANGMIN_TIL"))
  {
     res.value.integer=-4500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_DEFLT_MODE"))
  {
     res.value.integer=3;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_JSTICK_SPD"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_RC_IN_PAN"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_RC_IN_ROLL"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_RC_IN_TILT"))
  {
     res.value.integer=8;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_STAB_PAN"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_STAB_ROLL"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_STAB_TILT"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MNT_TYPE"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_1_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_2_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_3_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_4_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_5_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_6_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_7_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"MOT_8_DIRECTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACCEL_FILTER"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC_BODYFIX"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_FAST_SAMPLE"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYRO_FILTER"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR_CAL"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_TRIM_OPTION"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_USE"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_USE2"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_USE3"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC2_ID"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC3_ID"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_ACC_ID"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR2_ID"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR3_ID"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR3_ID"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"INS_GYR_ID"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_ALT_SOURCE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_BCN_DELAY"))
  {
     res.value.integer=50;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_ENABLE"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_FLOW_DELAY"))
  {
     res.value.integer=10;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_GLITCH_RAD"))
  {
     res.value.integer=25;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_GPS_CHECK"))
  {
     res.value.integer=31;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_GPS_TYPE"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_IMU_MASK"))
  {
     res.value.integer=3;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_LOG_MASK"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_MAG_CAL"))
  {
     res.value.integer=1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_MAG_MASK"))
  {
     res.value.integer=0;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_RNG_USE_HGT"))
  {
     res.value.integer=-1;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_TAU_OUTPUT"))
  {
     res.value.integer=25;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK3_ENABLE"))
  {
     res.value.integer=0;
     res.success=true;
  }

  else if(compareString(req.param_id.data(),"EK2_BCN_I_GTE"))
  {
     res.value.integer=500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_CHECK_SCALE"))
  {
     res.value.integer=100;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_EAS_I_GATE"))
  {
     res.value.integer=400;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_FLOW_I_GATE"))
  {
     res.value.integer=300;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_GPS_DELAY"))
  {
     res.value.integer=220;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_HGT_DELAY"))
  {
     res.value.integer=60;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_HGT_I_GATE"))
  {
     res.value.integer=500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_MAG_I_GATE"))
  {
     res.value.integer=300;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_POS_I_GATE"))
  {
     res.value.integer=500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_RNG_I_GATE"))
  {
     res.value.integer=500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_VEL_I_GATE"))
  {
     res.value.integer=500;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_MAG_I_GATE"))
  {
     res.value.integer=300;
     res.success=true;
  }
  else if(compareString(req.param_id.data(),"EK2_YAW_I_GATE"))
  {
     res.value.integer=300;
     res.success=true;
  }
  return res.success;

}


void AUV_Controller::onGetJoystickCallBack(const joystick_stamped::ConstPtr& msg)
{
  joystick_msg.mass_shifter_up   = msg ->mass_shifter_up;
  joystick_msg.mass_shifter_down = msg ->mass_shifter_down;
  joystick_msg.pistol_up         = msg ->pistol_up;
  joystick_msg.pistol_down       = msg ->pistol_down;
  joystick_msg.rudder            = msg ->rudder;
  joystick_msg.thruster          = msg ->thruster;


}

float AUV_Controller::convertForcetoSpeed(float Force)
{
  float speed;
  double X2 =   6.234239409765260/100000;
  double X1 =   0.002070652916595 ;
  double X0 = - 0.442834294940528 ;
  
  float delta = X1 * X1 - 4 * X2 *(X0 - Force);
  if (delta >= 0 )
  {
    speed = (-X1 + sqrt(delta))/(2*X2) ;
  }
  return speed;

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "auv2000_control");
  AUV_Controller control;
  ros::spin();
}
