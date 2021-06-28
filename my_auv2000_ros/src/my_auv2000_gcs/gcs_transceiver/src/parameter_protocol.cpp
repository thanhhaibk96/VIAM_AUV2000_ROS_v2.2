#include "parameter_protocol.h"
#include "gcs_transceiver.h"

ParameterProtocol::ParameterProtocol(GCSTransceiver* trans, const MAVConnInterface::Ptr link) : link(link), trans(trans)
{
  Roll_PID    = {"ROLL_PID_DES", "ROLL_PID_KP", "ROLL_PID_KI", "ROLL_PID_KD"};
  Speed_PID    = {"SPEED_PID_DES", "SPEED_PID_KP", "SPEED_PID_KI", "SPEED_PID_KD"};
  Heading_PID  = {"HEADING_PID_DES", "HEADING_PID_KP", "HEADING_PID_KI", "HEADING_PID_KD"};
  Pitch_PID    = {"PITCH_PID_DES", "PITCH_PID_KP", "PITCH_PID_KI", "PITCH_PID_KD"};
  Depth_PID    = {"DEPTH_PID_DES", "DEPTH_PID_KP", "DEPTH_PID_KI", "DEPTH_PID_KD"};
  LOS_PID      = {"LOS_PID_KP", "LOS_PID_KI", "LOS_PID_KD"};

  Roll_SM    = {"ROLL_SM_DES", "ROLL_SM_E", "ROLL_SM_K"};
  Speed_SM    = {"SPEED_SM_DES", "SPEED_SM_E", "SPEED_SM_K"};
  Heading_SM  = {"HEADING_SM_DES", "HEADING_SM_E", "HEADING_SM_K"};
  Pitch_SM    = {"PITCH_SM_DES", "PITCH_SM_E", "PITCH_SM_K"};
  Depth_SM    = {"DEPTH_SM_DES", "DEPTH_SM_E", "DEPTH_SM_K"};
  LOS_SM      = {"LOS_SM_E", "LOS_SM_K"};

  LOS_Config  = {"LOS_RADIUS", "LOS_DELTA_MIN", "LOS_DELTA_MAX", "LOS_BETA"};

  Serial       = {"SERIAL0_BAUD"};
  ComPass      = {"COMPASS_DEC"};
  Battery      = {"BATT_MONITOR","BATT2_MONITOR","BATT_CAPACITY","BATT2_CAPACITY"
                  ,"BATT_CURR_PIN","BATT_VOLT_PIN","BATT2_CURR_PIN","BATT2_VOLT_PIN"};
  FailSafe     = {"FS_PRESS_ENABLE","FS_PRESS_MAX","FS_TEMP_ENABLE","FS_TEMP_MAX","FS_EKF_ACTION","FS_BATT_ENABLE","FS_BATT_MAH"
                 ,"FS_TERRAIN_ENAB","FS_LEAK_ENABLE","FS_GCS_ENABLE"
                 ,"INS_ACC3_ID","INS_ACC_ID","INS_GYR2_ID","INS_GYR3_ID","INS_GYR_ID"};
  FailSafeF    = {"FS_EKF_THRESH","FS_BATT_VOLTAGE","FS_PILOT_TIMEOUT"};

  FailSafe8    ={"FS_PILOT_INPUT","SERVO10_FUNCTION","RC_FEEL_RP"
                ,"WPNAV_RFND_USE"
                ,"GPS_AUTO_CONFIG","GPS_AUTO_SWITCH","GPS_BLEND_MASK"
                ,"GPS_GNSS_MODE"
                ,"GPS_GNSS_MODE2","GPS_INJECT_TO","GPS_MIN_DGPS","GPS_MIN_ELEV"
                ,"GPS_NAVFILTER","MNT_DEFLT_MODE","MNT_JSTICK_SPD","MNT_RC_IN_PAN"
                ,"MNT_RC_IN_ROLL","MNT_RC_IN_TILT","MNT_STAB_PAN","MNT_STAB_ROLL"
                ,"MNT_STAB_TILT","MNT_TYPE","MOT_1_DIRECTION","MOT_2_DIRECTION","MOT_3_DIRECTION"
                ,"MOT_4_DIRECTION","MOT_5_DIRECTION","MOT_6_DIRECTION","MOT_7_DIRECTION","MOT_8_DIRECTION"
                ,"INS_ACCEL_FILTER","INS_ACC_BODYFIX","INS_FAST_SAMPLE","INS_GYRO_FILTER","INS_GYR_CAL"
                ,"INS_TRIM_OPTION","INS_USE","INS_USE2","INS_USE3"
                ,"EK2_ALT_SOURCE","EK2_BCN_DELAY","EK2_ENABLE"
                ,"EK2_FLOW_DELAY","EK2_GLITCH_RAD","EK2_GPS_CHECK"
                ,"EK2_GPS_TYPE","EK2_IMU_MASK","EK2_LOG_MASK","EK2_MAG_CAL"
                ,"EK2_MAG_MASK","EK2_RNG_USE_HGT","EK2_TAU_OUTPUT"
                ,"EK3_ENABLE"};

  ARMING      ={"ARMING_MIN_VOLT","ARMING_MIN_VOLT2"};
  ARMING_CHECK={"ARMING_CHECK"};

  JoystickFunctionParam ={"BTN0_FUNCTION", "BTN0_SFUNCTION" , "BTN10_FUNCTION" , "BTN10_SFUNCTION", "BTN11_FUNCTION", "BTN11_SFUNCTION"
                  , "BTN12_FUNCTION", "BTN12_SFUNCTION", "BTN13_FUNCTION", "BTN13_SFUNCTION", "BTN14_FUNCTION", "BTN14_SFUNCTION"
                  , "BTN15_FUNCTION", "BTN15_SFUNCTION", "BTN1_FUNCTION", "BTN1_SFUNCTION", "BTN2_FUNCTION", "BTN2_SFUNCTION"
                  , "BTN3_FUNCTION", "BTN3_SFUNCTION", "BTN4_FUNCTION", "BTN4_SFUNCTION", "BTN5_FUNCTION", "BTN5_SFUNCTION"
                  , "BTN6_FUNCTION", "BTN6_SFUNCTION", "BTN7_FUNCTION", "BTN7_SFUNCTION", "BTN8_FUNCTION", "BTN8_SFUNCTION"
                  , "BTN9_FUNCTION", "BTN9_SFUNCTION"};

  JoystickConfigParamReal32 ={"JS_GAIN_DEFAULT", "JS_GAIN_MAX", "JS_GAIN_MIN", "JS_THR_GAIN"};
  JoystickConfigParamInt8 ={"JS_GAIN_STEPS"};
  JoystickConfigParamInt16 ={"JS_CAM_TILT_STEP", "JS_LIGHTS_STEP"};

  ros::NodeHandle nh;

  reqSetRoll_PID = nh.serviceClient<ParamSet>("parameter/set_Roll_PID");
  reqGetRoll_PID = nh.serviceClient<ParamGet>("parameter/get_Roll_PID");

  reqSetSpeed_PID = nh.serviceClient<ParamSet>("parameter/set_Speed_PID");
  reqGetSpeed_PID = nh.serviceClient<ParamGet>("parameter/get_Speed_PID");

  reqSetHeading_PID = nh.serviceClient<ParamSet>("parameter/set_Heading_PID");
  reqGetHeading_PID = nh.serviceClient<ParamGet>("parameter/get_Heading_PID");

  reqSetPitch_PID = nh.serviceClient<ParamSet>("parameter/set_Pitch_PID");
  reqGetPitch_PID = nh.serviceClient<ParamGet>("parameter/get_Pitch_PID");

  reqSetDepth_PID = nh.serviceClient<ParamSet>("parameter/set_Depth_PID");
  reqGetDepth_PID = nh.serviceClient<ParamGet>("parameter/get_Depth_PID");

  reqSetLOS_PID = nh.serviceClient<ParamSet>("parameter/set_LOS_PID");
  reqGetLOS_PID = nh.serviceClient<ParamGet>("parameter/get_LOS_PID");

  reqSetRoll_SM = nh.serviceClient<ParamSet>("parameter/set_Roll_SM");
  reqGetRoll_SM = nh.serviceClient<ParamGet>("parameter/get_Roll_SM");

  reqSetSpeed_SM = nh.serviceClient<ParamSet>("parameter/set_Speed_SM");
  reqGetSpeed_SM = nh.serviceClient<ParamGet>("parameter/get_Speed_SM");

  reqSetHeading_SM = nh.serviceClient<ParamSet>("parameter/set_Heading_SM");
  reqGetHeading_SM = nh.serviceClient<ParamGet>("parameter/get_Heading_SM");

  reqSetPitch_SM = nh.serviceClient<ParamSet>("parameter/set_Pitch_SM");
  reqGetPitch_SM = nh.serviceClient<ParamGet>("parameter/get_Pitch_SM");

  reqSetDepth_SM = nh.serviceClient<ParamSet>("parameter/set_Depth_SM");
  reqGetDepth_SM = nh.serviceClient<ParamGet>("parameter/get_Depth_SM");

  reqSetLOS_SM = nh.serviceClient<ParamSet>("parameter/set_LOS_SM");
  reqGetLOS_SM = nh.serviceClient<ParamGet>("parameter/get_LOS_SM");

  reqSetLOS_Config = nh.serviceClient<ParamSet>("parameter/set_LOS_Config");
  reqGetLOS_Config = nh.serviceClient<ParamGet>("parameter/get_LOS_Config");

  reqGetComPassParam=nh.serviceClient<ParamGet>("parameter/get_ComPass_param");

  reqGetBatteryParam=nh.serviceClient<ParamGet>("parameter/get_battery_param");

  reqGetFailSafeParam=nh.serviceClient<ParamGet>("parameter/get_failsafe_param");

  reqGetArmingParam=nh.serviceClient<ParamGet>("parameter/get_arming_param");

  reqGetJoystickFunctionParam=nh.serviceClient<ParamGet>("parameter/get_joystick_function_param");

  reqGetJoystickConfigParamReal32=nh.serviceClient<ParamGet>("parameter/get_joystick_config_param_real32");
  reqGetJoystickConfigParamInt16=nh.serviceClient<ParamGet>("parameter/get_joystick_config_param_int16");
  reqGetJoystickConfigParamInt8=nh.serviceClient<ParamGet>("parameter/get_joystick_config_param_int8");
}

void ParameterProtocol::handleParameter(const mavlink_message_t* msg)
{
  switch (msg->msgid)
  {
  case 21: // PARAM_REQUEST_LIST
    readAllParameters(msg);
    break;
  case 20: // PARAM_REQUEST_READ
    readParameter(msg);
    break;
  case 23: // PARAM_SET
    writeParameter(msg);
    break;
  }
}

void ParameterProtocol::readAllParameters(const mavlink_message_t* msg)
{
  PARAM_REQUEST_LIST reqPack;
  unpack_mavlink_message_t(msg, reqPack);

  /// Only procceed if the received message is aimed at this autopilot.
  if (check_for_target_id_matching(reqPack, trans->autopilot_sysid, trans->autopilot_compid))
  {
    ROS_INFO_STREAM(reqPack.to_yaml());

    uint16_t numParams = static_cast<uint16_t>(Roll_PID.size() + Speed_PID.size() + Heading_PID.size() + Pitch_PID.size() + FailSafe.size() + FailSafeF.size()
                                               + ComPass.size() + Battery.size() + FailSafe8.size()
                                               + ARMING.size() + ARMING_CHECK.size()
                                               + JoystickFunctionParam.size() + JoystickConfigParamReal32.size()
                                               + JoystickConfigParamInt16.size() + JoystickConfigParamInt8.size()
                                               + Depth_PID.size() + LOS_PID.size() + Roll_SM.size() + Speed_SM.size() + Heading_SM.size() + Pitch_SM.size()
                                               + Depth_SM.size() + LOS_SM.size() + LOS_Config.size());
    uint16_t paramId = 0;

    readParamList(Roll_PID, numParams, paramId);
    readParamList(Speed_PID, numParams, paramId);
    readParamList(Heading_PID, numParams, paramId);
    readParamList(Pitch_PID, numParams, paramId);
    readParamList(Depth_PID, numParams, paramId);
    readParamList(LOS_PID, numParams, paramId);
    readParamList(Roll_SM, numParams, paramId);
    readParamList(Speed_SM, numParams, paramId);
    readParamList(Heading_SM, numParams, paramId);
    readParamList(Pitch_SM, numParams, paramId);
    readParamList(Depth_SM, numParams, paramId);
    readParamList(LOS_SM, numParams, paramId);
    readParamList(LOS_Config, numParams, paramId);

    readParamList(ComPass,numParams, paramId);
    readParamList(Battery,numParams,paramId);
    readParamList(FailSafe,numParams,paramId);
    readParamList(FailSafeF,numParams,paramId);
    readParamList(FailSafe8,numParams,paramId);
    readParamList(ARMING,numParams,paramId);
    readParamList(ARMING_CHECK,numParams,paramId);

    readParamList(JoystickFunctionParam, numParams,paramId);
    readParamList(JoystickConfigParamReal32, numParams,paramId);
    readParamList(JoystickConfigParamInt16, numParams,paramId);
    readParamList(JoystickConfigParamInt8, numParams,paramId);
  }
}

void ParameterProtocol::readParameter(const mavlink_message_t* /*msg*/) {}

void ParameterProtocol::writeParameter(const mavlink_message_t* msg)
{
  PARAM_SET reqPack;
  unpack_mavlink_message_t(msg, reqPack);

  /// Only procceed if the received message is aimed at this autopilot.
  if (check_for_target_id_matching(reqPack, trans->autopilot_sysid, trans->autopilot_compid))
  {
    ROS_INFO_STREAM(reqPack.to_yaml());

    ParamSet srv;
    srv.request.param_id = reqPack.param_id.data();
    if (reqPack.param_type == static_cast<uint8_t>(MAV_PARAM_TYPE::INT32))
     /* memcpy(&srv.request.value.integer, &reqPack.param_value, sizeof(float));*/
      srv.request.value.integer=static_cast<int>(reqPack.param_value);
    else if (reqPack.param_type == static_cast<uint8_t>(MAV_PARAM_TYPE::REAL32))
      srv.request.value.real = static_cast<double>(reqPack.param_value);

    if (requestSetParameter(srv))
    {
      PARAM_VALUE resPack;
      resPack.param_id = reqPack.param_id;
      if (reqPack.param_type == static_cast<uint8_t>(MAV_PARAM_TYPE::INT32))
        /*memcpy(&resPack.param_value, &srv.response.value.integer, sizeof(float));*/
        resPack.param_value=static_cast<int>(srv.response.value.integer);
      else if (reqPack.param_type == static_cast<uint8_t>(MAV_PARAM_TYPE::REAL32))
        resPack.param_value = static_cast<float>(srv.response.value.real);
      resPack.param_type = reqPack.param_type;
      pack_and_send_mavlink_message_t(resPack, link);
    }
  }
}

void ParameterProtocol::readParamList(const StringList& paramList, const uint16_t& numParams, uint16_t& paramId)
{
  ParamGet srv;
  PARAM_VALUE resPack;

  for (auto it = paramList.begin(); it != paramList.end(); it++)
  {
    srv.request.param_id = it->data();

    if (requestGetParameter(srv, paramList))
    {
      strcpy(resPack.param_id.data(), it->data());

      /// Assuming that all parameters in headingPID and LOSParams is real.
      /// This piece of code needs more refinement in the future.
      if (paramList == Heading_PID || paramList == Pitch_PID|| paramList == FailSafeF
          ||paramList == ARMING || paramList == Depth_PID || paramList == LOS_PID || paramList == Heading_SM
          || paramList == Pitch_SM || paramList == Depth_SM || paramList == LOS_SM || paramList == LOS_Config 
          || paramList == Speed_PID || paramList == Speed_SM || paramList == Roll_PID || paramList == Roll_SM )
      {
        resPack.param_value = static_cast<float>(srv.response.value.real);
        resPack.param_type = static_cast<uint8_t>(MAV_PARAM_TYPE::REAL32);
      }
      else if (paramList==ComPass ||  paramList==Battery || paramList==FailSafe
               || paramList == ARMING_CHECK)
      {
        resPack.param_value=static_cast<int>(srv.response.value.integer);
        resPack.param_type=static_cast<uint8_t>(MAV_PARAM_TYPE::INT32);
      }

      else if ( paramList==FailSafe8)
      {
        resPack.param_value=static_cast<int>(srv.response.value.integer);
        resPack.param_type=static_cast<uint8_t>(MAV_PARAM_TYPE::INT8);
      }
      else if (paramList == JoystickFunctionParam)
      {
        resPack.param_value = static_cast<uint8_t>(srv.response.value.integer);
        resPack.param_type = static_cast<int8_t>(MAV_PARAM_TYPE::INT8);
      }
      else if (paramList == JoystickConfigParamReal32)
      {
        resPack.param_value = static_cast<float>(srv.response.value.real);
        resPack.param_type = static_cast<uint8_t>(MAV_PARAM_TYPE::REAL32);
      }
      else if (paramList == JoystickConfigParamInt8)
      {
        resPack.param_value = static_cast<int8_t>(srv.response.value.integer);
        resPack.param_type = static_cast<uint8_t>(MAV_PARAM_TYPE::INT8);
      }
      else if (paramList == JoystickConfigParamInt16)
      {
        resPack.param_value = static_cast<int16_t>(srv.response.value.integer);
        resPack.param_type = static_cast<uint8_t>(MAV_PARAM_TYPE::INT16);
      }

      /// The total number of parameters in the system must be assigned to every
      /// responding message back to GCS.
      resPack.param_count = numParams;
      resPack.param_index = paramId++;

      pack_and_send_mavlink_message_t(resPack, link);
//      ROS_INFO_STREAM("Chau Thanh Hai");
//      ROS_INFO_STREAM(resPack.to_yaml());
    }
  }
}

bool ParameterProtocol::requestSetParameter(ParamSet& srv)
{
  if (paramListMatched(Roll_PID, srv.request.param_id.data()))
    return reqSetRoll_PID.call(srv);
  if (paramListMatched(Speed_PID, srv.request.param_id.data()))
    return reqSetSpeed_PID.call(srv);
  if (paramListMatched(Heading_PID, srv.request.param_id.data()))
    return reqSetHeading_PID.call(srv);
  if (paramListMatched(Pitch_PID, srv.request.param_id.data()))
    return reqSetPitch_PID.call(srv);
  if (paramListMatched(Depth_PID, srv.request.param_id.data()))
    return reqSetDepth_PID.call(srv);
  if (paramListMatched(LOS_PID, srv.request.param_id.data()))
    return reqSetLOS_PID.call(srv);
  if (paramListMatched(Roll_SM, srv.request.param_id.data()))
    return reqSetRoll_SM.call(srv);
  if (paramListMatched(Speed_SM, srv.request.param_id.data()))
    return reqSetSpeed_SM.call(srv);
  if (paramListMatched(Heading_SM, srv.request.param_id.data()))
    return reqSetHeading_SM.call(srv);
  if (paramListMatched(Pitch_SM, srv.request.param_id.data()))
    return reqSetPitch_SM.call(srv);
  if (paramListMatched(Depth_SM, srv.request.param_id.data()))
    return reqSetDepth_SM.call(srv);
  if (paramListMatched(LOS_SM, srv.request.param_id.data()))
    return reqSetLOS_SM.call(srv);
  if (paramListMatched(LOS_Config, srv.request.param_id.data()))
    return reqSetLOS_Config.call(srv);
  return false;
}

bool ParameterProtocol::requestGetParameter(ParamGet& srv, const StringList& paramList)
{
  if (paramList == Roll_PID)
    return reqGetRoll_PID.call(srv);
  if (paramList == Speed_PID)
    return reqGetSpeed_PID.call(srv);
  if (paramList == Heading_PID)
    return reqGetHeading_PID.call(srv);
  if (paramList == Pitch_PID)
    return reqGetPitch_PID.call(srv);
  if (paramList == Depth_PID)
    return reqGetDepth_PID.call(srv);
  if (paramList == LOS_PID)
    return reqGetLOS_PID.call(srv);

  if (paramList == Roll_SM)
    return reqGetRoll_SM.call(srv);
  if (paramList == Speed_SM)
    return reqGetSpeed_SM.call(srv);
  if (paramList == Heading_SM)
    return reqGetHeading_SM.call(srv);
  if (paramList == Pitch_SM)
    return reqGetPitch_SM.call(srv);
  if (paramList == Depth_SM)
    return reqGetDepth_SM.call(srv);
  if (paramList == LOS_SM)
    return reqGetLOS_SM.call(srv);
  if (paramList == LOS_Config)
    return reqGetLOS_Config.call(srv);
  if(paramList == ComPass)
    return reqGetComPassParam.call(srv);
  if (paramList == Battery)
    return reqGetBatteryParam.call(srv);
  if (paramList == FailSafe || paramList == FailSafeF||paramList == FailSafe8)
    return reqGetFailSafeParam.call(srv);
  if(paramList ==ARMING || paramList ==ARMING_CHECK )
    return reqGetArmingParam.call(srv);
  if (paramList == JoystickConfigParamReal32)
    return reqGetJoystickConfigParamReal32.call(srv);
  if (paramList == JoystickConfigParamInt16)
    return reqGetJoystickConfigParamInt16.call(srv);
  if (paramList == JoystickConfigParamInt8)
    return reqGetJoystickConfigParamInt8.call(srv);
  if (paramList == JoystickFunctionParam)
    return reqGetJoystickFunctionParam.call(srv);
  return false;
}
