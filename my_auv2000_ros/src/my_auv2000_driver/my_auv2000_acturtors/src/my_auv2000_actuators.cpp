#include "my_auv2000_actuators.h"

#define MAXSOCK 16
static ros::WallTimer requestCANData;
void onRequestCANDataCallBack(const ros::WallTimerEvent& event);

static uint8_t ROS_CANBUS_Data[8];
static uint8_t UCAN_Checksum(uint8_t *_data, int start, int count);
static std::string CAN_Device = "";
static double Sample_Time = 0.0;
static std::mutex mutexPulishData;

static float Desired_Thruster_Speed = 0.0;
static float preDesired_Thruster_Speed = 0.0;

static int16_t Desired_Rudder_Position = 0.0;
static int16_t preDesired_Rudder_Position = 0.0;

static int MAX_Pistol_DutyCycle = 0;
static int STEP_Pistol_DutyCycle = 0;
static float Desired_Pistol_DutyCycle = 0.0;
static float preDesired_Pistol_DutyCycle = 0.0;
static bool isPistol_Run_CW = false;
static bool isPistol_Run_CCW = false;
static uint8_t isPistol_atHead = 0x00;
static uint8_t isPistol_atTail = 0x00;
static Motor_Direction_TypeDef Pistol_Direction = M_NONE;

static int MAX_Mass_DutyCycle = 0;
static int STEP_Mass_DutyCycle = 0;
static float Desired_Mass_DutyCycle = 0.0;
static float preDesired_Mass_DutyCycle = 0.0;
static bool isMass_Run_CW = false;
static bool isMass_Run_CCW = false;
static uint8_t isMass_atHead = 0x00;
static uint8_t isMass_atTail = 0x00;
static Motor_Direction_TypeDef Mass_Direction = M_NONE;

//CANBUS
can::ThreadedSocketCANInterface CAN_Driver;
can::CommInterface::FrameListenerConstSharedPtr CAN_Frames;
static std::mutex mutexSocketCAN;
static std::mutex mutexWriteData;
static std::mutex mutexRequestData;
static bool CANBUS_ACK = false;
static bool CANBUS_OnWriting = false;
static bool CANBUS_OnRequest = false;
uint8_t UCAN_Send(const uint32_t& _id, const uint8_t& _buffer_size, const uint8_t* _data);
/// CAN Functions
void UCAN_RequestData(AUV_BOARD_TypeDef _type_board);
void UCAN_InitSocket(void);

// Motor Status
static Motor_Data_TypeDef _mass_shifter_data;
static Motor_Data_TypeDef _pistol_data;
static Motor_Data_TypeDef _thruster_data;
static Board_ARM1_Data_TypeDef _board_arm1_data;
static Board_ARM2_Data_TypeDef _board_arm2_data;

static joystick_stamped pre_Joystick_Control;

// Pistol Cmd:
void UCAN_PistolCmd(Function_Cmd_TypeDef _state);
void UCAN_Pistol_Send_DutyCycle(Motor_Direction_TypeDef _direction, float _duty_cycle);
// Mass Cmd:
void UCAN_Mass_ShifterCmd(Function_Cmd_TypeDef _state);
void UCAN_Mass_Send_DutyCycle(Motor_Direction_TypeDef _direction, float _duty_cycle);
// Thruster Cmd:
void UCAN_ThrusterCmd(Function_Cmd_TypeDef _state);
void UCAN_Thruster_Send_DutyCycle(Motor_Direction_TypeDef _direction, float _duty_cycle);
void UCAN_Thruster_Send_PID_Kp(float _kp);
void UCAN_Thruster_Send_PID_Ki(float _ki);
void UCAN_Thruster_Send_PID_Kd(float _kd);
void UCAN_Thruster_Send_PID_DesiredSpeed(Motor_Direction_TypeDef _direction, float _dspeed);
void UCAN_Thruster_Send_PID_Ge(float _ge);
void UCAN_Thruster_Send_PID_Gde(float _gde);
void UCAN_Thruster_Send_PID_Gdu(float _gdu);
void UCAN_Thruster_Send_Fuzzy_DesiredSpeed(Motor_Direction_TypeDef _direction, float _dspeed);
// Rudder Cmd:
void UCAN_Rudder_Send_GoalPostion(int16_t _angle);
void UCAN_Rudder_Send_MovingSpeed(uint16_t _speed);
void UCAN_Rudder_Send_Kp(Rudder_KP_TypeDef _kp);
void UCAN_Rudder_Send_Ki(float _ki);
void UCAN_Rudder_Send_Kd(float _kd);

void Convert_Bytes2Float(uint8_t* _data_in, float* _data_out);
void UMOTOR_Convert_Float_to_Bytes(float _data_in, uint8_t* _data_out);

void onCANBUSHandleFrames(const can::Frame &f)
{  
//  ROS_INFO("onCANBUSHandleFrames");
  uint8_t CANBUS_Data_NoChecksum[8];
  for(uint8_t i = 0; i < 8; i++) CANBUS_Data_NoChecksum[i] = f.data[i];

  // REQ ACK
  if(((f.id == AUV_MASS_SHIFTER) || (f.id == AUV_ARMBOARD_1) || (f.id == AUV_ARMBOARD_2) || (f.id == AUV_PISTOL) || (f.id == AUV_THRUSTER))
     && (f.data[7] == UCAN_Checksum(CANBUS_Data_NoChecksum, 0, 7))
     && (f.data[0] == 'R') && (f.data[1] == 'E') && (f.data[2] == 'Q') && (f.data[3] == 'A') && (f.data[4] == 'C') && (f.data[5] == 'K'))
  {
     ROS_INFO_STREAM("CAN ID " << std::to_string(f.id) << ": Received ACK Successfully");
      CANBUS_ACK = true;
  }

  if((f.id == AUV_MASS_SHIFTER) && (f.data[7] == UCAN_Checksum(CANBUS_Data_NoChecksum, 0, 7)))
  {
    if((CANBUS_Data_NoChecksum[0] == 'R') && (CANBUS_Data_NoChecksum[1] == 'P') && (CANBUS_Data_NoChecksum[2] == 'V'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_mass_shifter_data.motor_rspeed);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'A') && (CANBUS_Data_NoChecksum[1] == 'O') && (CANBUS_Data_NoChecksum[2] == 'C'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_mass_shifter_data.motor_temp_on_chip);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'A') && (CANBUS_Data_NoChecksum[1] == 'T') && (CANBUS_Data_NoChecksum[2] == 'K'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_mass_shifter_data.motor_temp_ambient);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'A') && (CANBUS_Data_NoChecksum[1] == 'I') && (CANBUS_Data_NoChecksum[2] == 'M'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_mass_shifter_data.motor_current);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'T') && (CANBUS_Data_NoChecksum[1] == 'D') && (CANBUS_Data_NoChecksum[2] == 'C'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_mass_shifter_data.motor_duty);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'R') && (CANBUS_Data_NoChecksum[1] == 'S') && (CANBUS_Data_NoChecksum[2] == 'P'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_mass_shifter_data.motor_dspeed);
    }
  }

  if((f.id == AUV_PISTOL) && (f.data[7] == UCAN_Checksum(CANBUS_Data_NoChecksum, 0, 7)))
  {
    if((CANBUS_Data_NoChecksum[0] == 'R') && (CANBUS_Data_NoChecksum[1] == 'P') && (CANBUS_Data_NoChecksum[2] == 'V'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_pistol_data.motor_rspeed);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'A') && (CANBUS_Data_NoChecksum[1] == 'O') && (CANBUS_Data_NoChecksum[2] == 'C'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_pistol_data.motor_temp_on_chip);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'A') && (CANBUS_Data_NoChecksum[1] == 'T') && (CANBUS_Data_NoChecksum[2] == 'K'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_pistol_data.motor_temp_ambient);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'A') && (CANBUS_Data_NoChecksum[1] == 'I') && (CANBUS_Data_NoChecksum[2] == 'M'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_pistol_data.motor_current);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'T') && (CANBUS_Data_NoChecksum[1] == 'D') && (CANBUS_Data_NoChecksum[2] == 'C'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_pistol_data.motor_duty);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'R') && (CANBUS_Data_NoChecksum[1] == 'S') && (CANBUS_Data_NoChecksum[2] == 'P'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_pistol_data.motor_dspeed);
    }
  }

  if((f.id == AUV_THRUSTER) && (f.data[7] == UCAN_Checksum(CANBUS_Data_NoChecksum, 0, 7)))
  {
    if((CANBUS_Data_NoChecksum[0] == 'R') && (CANBUS_Data_NoChecksum[1] == 'P') && (CANBUS_Data_NoChecksum[2] == 'V'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_thruster_data.motor_rspeed);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'A') && (CANBUS_Data_NoChecksum[1] == 'O') && (CANBUS_Data_NoChecksum[2] == 'C'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_thruster_data.motor_temp_on_chip);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'A') && (CANBUS_Data_NoChecksum[1] == 'T') && (CANBUS_Data_NoChecksum[2] == 'K'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_thruster_data.motor_temp_ambient);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'A') && (CANBUS_Data_NoChecksum[1] == 'I') && (CANBUS_Data_NoChecksum[2] == 'M'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_thruster_data.motor_current);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'T') && (CANBUS_Data_NoChecksum[1] == 'D') && (CANBUS_Data_NoChecksum[2] == 'C'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_thruster_data.motor_duty);
    }
    else if((CANBUS_Data_NoChecksum[0] == 'R') && (CANBUS_Data_NoChecksum[1] == 'S') && (CANBUS_Data_NoChecksum[2] == 'P'))
    {
      Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_thruster_data.motor_dspeed);
    }
  }

  if((f.id == AUV_ARMBOARD_1) &&(f.data[7] == UCAN_Checksum(CANBUS_Data_NoChecksum, 0, 7)))
  {
    switch(f.data[1])
    {
      case STATUS_DATA:
      {
        switch (f.data[0]) {
          case ARM1_PISTOL:
          {
            if(f.data[2] == ARM1_REQ_ENCODER)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_pistol_data.position);
            }
            else if (f.data[2] == ARM1_REQ_PISTOL_LIMIT_SWITCH)
            {
              _board_arm1_data.pistol_ls.ls_AtHead = f.data[3];
              _board_arm1_data.pistol_ls.ls_AtTail = f.data[4];
            }
            break;
          }
          case ARM1_MASS_SHIFTER:
          {
            if(f.data[2] == ARM1_REQ_ENCODER)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_mass_shifter_data.position);
            }
            else if (f.data[2] == ARM1_REQ_MASS_LIMIT_SWITCH)
            {
              _board_arm1_data.mass_shifter_ls.ls_AtHead = f.data[3];
              _board_arm1_data.mass_shifter_ls.ls_AtTail = f.data[4];
            }
            break;
          }
          case ARM1_BMS24V40AH:
          {
            if(f.data[2] == ARM1_BATTERY_TOTAL)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm1_data.bms_status.batteryTotal);
            }
            else if (f.data[2] == ARM1_BATTERY_CAPACITY)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm1_data.bms_status.batteryCapacity);
            }
            else if (f.data[2] == ARM1_BATTERY_USED)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm1_data.bms_status.batteryUsed);
            }
            else if (f.data[2] == ARM1_BATTERY_PERCENTAGE)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm1_data.bms_status.batteryPercentage);
            }
            else if (f.data[2] == ARM1_BATTERY_CURRENT)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm1_data.bms_status.batteryCurrent);
            }
            else if (f.data[2] == ARM1_BATTERY_VOLTAGE)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm1_data.bms_status.baterryVoltage);
            }
            break;
          }
          case ARM1_ALTIMETER:
          {
            if(f.data[2] == ARM1_ALTIMETER_IN_FEET)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm1_data.altimeter_status.alt_in_feet);
            }
            else if (f.data[2] == ARM1_ALTIMETER_IN_METRES)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm1_data.altimeter_status.alt_in_metres);
            }
            else if (f.data[2] == ARM1_ALTIMETER_IN_FATHOMS)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm1_data.altimeter_status.alt_in_fathoms);
            }
            break;
          }
        }
        break;
      }
    }
  }

  if((f.id == AUV_ARMBOARD_2) &&(f.data[7] == UCAN_Checksum(CANBUS_Data_NoChecksum, 0, 7)))
  {
    switch(f.data[1])
    {
      case STATUS_DATA:
      {
        switch (f.data[0]) {
          case ARM2_RUDDER:
          {
            if(f.data[2] == ARM2_MX28_PRESENT_POSITION)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm2_data.mx28_status.position);
            }
            else if (f.data[2] == ARM2_MX28_PRESENT_SPEED)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm2_data.mx28_status.speed);
            }
            else if (f.data[2] == ARM2_MX28_PRESENT_LOAD)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm2_data.mx28_status.load);
            }
            else if (f.data[2] == ARM2_MX28_PRESENT_VOL)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm2_data.mx28_status.voltage);
            }
            else if (f.data[2] == ARM2_MX28_PRESENT_TEMP)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm2_data.mx28_status.temperature);
            }
            break;
          }
          case ARM2_BMS24V40AH:
          {
            if(f.data[2] == ARM2_BATTERY_TOTAL)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm2_data.bms_status.batteryTotal);
            }
            else if (f.data[2] == ARM2_BATTERY_CAPACITY)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm2_data.bms_status.batteryCapacity);
            }
            else if (f.data[2] == ARM2_BATTERY_USED)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm2_data.bms_status.batteryUsed);
            }
            else if (f.data[2] == ARM2_BATTERY_PERCENTAGE)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm2_data.bms_status.batteryPercentage);
            }
            else if (f.data[2] == ARM2_BATTERY_CURRENT)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm2_data.bms_status.batteryCurrent);
            }
            else if (f.data[2] == ARM2_BATTERY_VOLTAGE)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm2_data.bms_status.baterryVoltage);
            }
            break;
          }
          case ARM2_PRESSURE:
          {
            if(f.data[2] == ARM2_DEPTH_DATA)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm2_data.pressure_status.pressure);
            }
            else if (f.data[2] == ARM2_TEMP_DATA)
            {
              Convert_Bytes2Float(CANBUS_Data_NoChecksum + 3, &_board_arm2_data.pressure_status.temperature);
            }
            break;
          }
        }
        break;
      }
    }
  }
}

My_AUV2000_Actuator::My_AUV2000_Actuator()
{
  /// Get local parameters of this node through roslaunch file.
//  ros::NodeHandle private_nh("~");
//  private_nh.param<std::string>("can_device", CAN_Device, "can0");
//  private_nh.param<double>("sample_time", Sample_Time, 0.5);

  ros::NodeHandle nh;
  nh.param<int>("max_pistol_dutycycle", MAX_Pistol_DutyCycle, 60);
  nh.param<int>("step_pistol_dutycycle", STEP_Pistol_DutyCycle, 5);
  nh.param<int>("max_mass_dutycycle", MAX_Mass_DutyCycle, 80);
  nh.param<int>("step_mass_dutycycle", STEP_Mass_DutyCycle, 5);

//  subJoystick = nh.subscribe("joystick/data", 10, &My_AUV2000_Actuator::onGetJoystickCallBack, this);

  // Motor's Topic
  pubMassShifter = nh.advertise<motor_stamped>("my_auv2000_actuators/mass_shifter", 1);
  pubThruster = nh.advertise<motor_stamped>("my_auv2000_actuators/thruster", 1);
  pubPistol = nh.advertise<motor_stamped>("my_auv2000_actuators/pistol", 1);
  // Board's Topic
  pubBoard_ARM1 = nh.advertise<board_arm1_stamped>("my_auv2000_actuators/arm1", 1);
  pubBoard_ARM2 = nh.advertise<board_arm2_stamped>("my_auv2000_actuators/arm2", 1);

  subItemList = nh.subscribe("mission/item_list", 1, &My_AUV2000_Actuator::onItemListCallBack, this);

  resSetArming = nh.advertiseService("command/set_Arming", &My_AUV2000_Actuator::onSetArmingCallBack, this);
  resStartMission = nh.advertiseService("command/start_Mission", &My_AUV2000_Actuator::onStartMissionCallBack, this);
  resSetMode = nh.advertiseService("command/set_Mode", &My_AUV2000_Actuator::onSetModeCallBack, this);
  resSetAUVController = nh.advertiseService("command/set_AUV_Controller", &My_AUV2000_Actuator::onSetAUVControllerCallBack, this);

  reqSetRoll_PID = nh.advertiseService("parameter/set_Roll_PID", &My_AUV2000_Actuator::onSetRoll_PIDCallBack, this);
  reqGetRoll_PID = nh.advertiseService("parameter/get_Roll_PID", &My_AUV2000_Actuator::onGetRoll_PIDCallBack, this);
  reqSetSpeed_PID = nh.advertiseService("parameter/set_Speed_PID", &My_AUV2000_Actuator::onSetSpeed_PIDCallBack, this);
  reqGetSpeed_PID = nh.advertiseService("parameter/get_Speed_PID", &My_AUV2000_Actuator::onGetSpeed_PIDCallBack, this);
  reqSetHeading_PID = nh.advertiseService("parameter/set_Heading_PID", &My_AUV2000_Actuator::onSetHeading_PIDCallBack, this);
  reqGetHeading_PID = nh.advertiseService("parameter/get_Heading_PID", &My_AUV2000_Actuator::onGetHeading_PIDCallBack, this);
  reqSetPitch_PID = nh.advertiseService("parameter/set_Pitch_PID", &My_AUV2000_Actuator::onSetPitch_PIDCallBack, this);
  reqGetPitch_PID = nh.advertiseService("parameter/get_Pitch_PID", &My_AUV2000_Actuator::onGetPitch_PIDCallBack, this);
  reqSetDepth_PID = nh.advertiseService("parameter/set_Depth_PID", &My_AUV2000_Actuator::onSetDepth_PIDCallBack, this);
  reqGetDepth_PID = nh.advertiseService("parameter/get_Depth_PID", &My_AUV2000_Actuator::onGetDepth_PIDCallBack, this);
  reqSetLOS_PID = nh.advertiseService("parameter/set_LOS_PID", &My_AUV2000_Actuator::onSetLOS_PIDCallBack, this);
  reqGetLOS_PID = nh.advertiseService("parameter/get_LOS_PID", &My_AUV2000_Actuator::onGetLOS_PIDCallBack, this);

  reqSetRoll_SM = nh.advertiseService("parameter/set_Roll_SM", &My_AUV2000_Actuator::onSetRoll_SMCallBack, this);
  reqGetRoll_SM = nh.advertiseService("parameter/get_Roll_SM", &My_AUV2000_Actuator::onGetRoll_SMCallBack, this);
  reqSetSpeed_SM = nh.advertiseService("parameter/set_Speed_SM", &My_AUV2000_Actuator::onSetSpeed_SMCallBack, this);
  reqGetSpeed_SM = nh.advertiseService("parameter/get_Speed_SM", &My_AUV2000_Actuator::onGetSpeed_SMCallBack, this);
  reqSetHeading_SM = nh.advertiseService("parameter/set_Heading_SM", &My_AUV2000_Actuator::onSetHeading_SMCallBack, this);
  reqGetHeading_SM = nh.advertiseService("parameter/get_Heading_SM", &My_AUV2000_Actuator::onGetHeading_SMCallBack, this);
  reqSetPitch_SM = nh.advertiseService("parameter/set_Pitch_SM", &My_AUV2000_Actuator::onSetPitch_SMCallBack, this);
  reqGetPitch_SM = nh.advertiseService("parameter/get_Pitch_SM", &My_AUV2000_Actuator::onGetPitch_SMCallBack, this);
  reqSetDepth_SM = nh.advertiseService("parameter/set_Depth_SM", &My_AUV2000_Actuator::onSetDepth_SMCallBack, this);
  reqGetDepth_SM = nh.advertiseService("parameter/get_Depth_SM", &My_AUV2000_Actuator::onGetDepth_SMCallBack, this);
  reqSetLOS_SM = nh.advertiseService("parameter/set_LOS_SM", &My_AUV2000_Actuator::onSetLOS_SMCallBack, this);
  reqGetLOS_SM = nh.advertiseService("parameter/get_LOS_SM", &My_AUV2000_Actuator::onGetLOS_SMCallBack, this);

  reqSetLOS_Config = nh.advertiseService("parameter/set_LOS_Config", &My_AUV2000_Actuator::onSetLOS_ConfigCallBack, this);
  reqGetLOS_Config = nh.advertiseService("parameter/get_LOS_Config", &My_AUV2000_Actuator::onGetLOS_ConfigCallBack, this);

  resGetComPassParam=nh.advertiseService("parameter/get_ComPass_param",&My_AUV2000_Actuator::OnGetComPassCallBack,this);

  resGetBatteryParam=nh.advertiseService("parameter/get_battery_param",&My_AUV2000_Actuator::OnGetBatteryCallBack,this);

  resGetFailSafeParam=nh.advertiseService("parameter/get_failsafe_param",&My_AUV2000_Actuator::OnGetFailSafeCallBack,this);

  resGetARMINGParam=nh.advertiseService("parameter/get_arming_param",&My_AUV2000_Actuator::OnGetARMINGCallBack,this);

  resGetJoystickConfigParamReal32=nh.advertiseService("parameter/get_joystick_config_param_real32",&My_AUV2000_Actuator::OnGetJoystickConfigParamReal32CallBack,this);
  resGetJoystickConfigParamInt16=nh.advertiseService("parameter/get_joystick_config_param_int16",&My_AUV2000_Actuator::OnGetJoystickConfigParamInt16CallBack,this);
  resGetJoystickConfigParamInt8=nh.advertiseService("parameter/get_joystick_config_param_int8",&My_AUV2000_Actuator::OnGetJoystickConfigParamInt8CallBack,this);

  resGetJoystickFunctionParam=nh.advertiseService("parameter/get_joystick_function_param",&My_AUV2000_Actuator::OnGetJoystickFunctionParamCallBack,this);

// ros::Duration(2).sleep();

// UCAN_ThrusterCmd(ENABLE);
// ros::Duration(0.1).sleep();
// UCAN_PistolCmd(ENABLE);
// ros::Duration(0.1).sleep();
// UCAN_Mass_ShifterCmd(ENABLE);
// ros::Duration(0.1).sleep();

//  requestCANData = nh.createWallTimer(ros::WallDuration(0.1), onRequestCANDataCallBack);
//  loopPublishData = nh.createTimer(ros::Duration(Sample_Time), &My_AUV2000_Actuator::onLoopPublishDataCallBack, this);
//  requestCANData.start();
  start_time = ros::WallTime::now();
}

My_AUV2000_Actuator::~My_AUV2000_Actuator()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void Convert_Bytes2Float(uint8_t* _data_in, float* _data_out)
{
  union
  {
    float _value;
    uint8_t _byte[4];
  }_number;

  _number._byte[0]=_data_in[0];
  _number._byte[1]=_data_in[1];
  _number._byte[2]=_data_in[2];
  _number._byte[3]=_data_in[3];

  *_data_out = _number._value;
}

void UMOTOR_Convert_Float_to_Bytes(float _data_in, uint8_t* _data_out)
{
  union
  {
    float _value;
    uint8_t _byte[4];
  }_part;

  _part._value = _data_in;

  _data_out[0] = _part._byte[3];
  _data_out[1] = _part._byte[2];
  _data_out[2] = _part._byte[1];
  _data_out[3] = _part._byte[0];
}

float _test_count = 0;
void My_AUV2000_Actuator::onLoopPublishDataCallBack(const ros::TimerEvent& event)
{
  mutexPulishData.lock();
  last_time = ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();

  ///Mass Shifter data
  motor_stamped _massShifter_Status;
// _massShifter_Status.header.stamp = static_cast<ros::Time>(last_time);
 _massShifter_Status.position = _mass_shifter_data.position; // position
 _massShifter_Status.motor_duty = _mass_shifter_data.motor_duty; // duty cycle
 _massShifter_Status.motor_temp_on_chip = _mass_shifter_data.motor_temp_on_chip; // temperature on chip
 _massShifter_Status.motor_temp_ambient = _mass_shifter_data.motor_temp_ambient; // temperature ambient
 _massShifter_Status.motor_current = _mass_shifter_data.motor_current; // current
 _massShifter_Status.motor_rspeed = _mass_shifter_data.motor_rspeed; // current speed
 _massShifter_Status.motor_dspeed = _mass_shifter_data.motor_dspeed; // desired speed
//   _massShifter_Status.header.stamp = static_cast<ros::Time>(last_time);
//   _massShifter_Status.motor_duty = 50; // duty cycle
//   _massShifter_Status.motor_temp_on_chip = 30; // temperature on chip
//   _massShifter_Status.motor_temp_ambient = 40; // temperature ambient
//   _massShifter_Status.motor_current = 100; // current
//   _massShifter_Status.motor_rspeed = 1000; // current speed
//   _massShifter_Status.motor_dspeed = 400; // desired speed
//   _massShifter_Status.position = 18;
  pubMassShifter.publish(_massShifter_Status);

  ///Pistol data
  motor_stamped _pistol_Status;
// _pistol_Status.header.stamp = static_cast<ros::Time>(last_time);
 _pistol_Status.position = _pistol_data.position; // position
 _pistol_Status.motor_duty = _pistol_data.motor_duty; // duty cycle
 _pistol_Status.motor_temp_on_chip = _pistol_data.motor_temp_on_chip; // temperature on chip
 _pistol_Status.motor_temp_ambient = _pistol_data.motor_temp_ambient; // temperature ambient
 _pistol_Status.motor_current = _pistol_data.motor_current; // current
 _pistol_Status.motor_rspeed = _pistol_data.motor_rspeed; // current speed
 _pistol_Status.motor_dspeed = _pistol_data.motor_dspeed; // desired speed
//     _test_count += 0.2f;
//   _pistol_Status.header.stamp = static_cast<ros::Time>(last_time);
//   _pistol_Status.motor_duty = 70; // duty cycle
//   _pistol_Status.motor_temp_on_chip = 35; // temperature on chip
//   _pistol_Status.motor_temp_ambient = 40; // temperature ambient
//   _pistol_Status.motor_current = 200; // current
//   _pistol_Status.motor_rspeed = 150; // current speed
//   _pistol_Status.motor_dspeed = 15; // desired speed
//   _pistol_Status.position = _test_count;
  pubPistol.publish(_pistol_Status);

  ///Thruster data
  motor_stamped _thruster_Status;
// _thruster_Status.header.stamp = static_cast<ros::Time>(last_time);
 _thruster_Status.motor_duty = _thruster_data.motor_duty; // duty cycle
 _thruster_Status.motor_temp_on_chip = _thruster_data.motor_temp_on_chip; // temperature on chip
 _thruster_Status.motor_temp_ambient = _thruster_data.motor_temp_ambient; // temperature ambient
 _thruster_Status.motor_current = _thruster_data.motor_current; // current
 _thruster_Status.motor_rspeed = _thruster_data.motor_rspeed; // current speed
 _thruster_Status.motor_dspeed = _thruster_data.motor_dspeed; // desired speed

//   _thruster_Status.header.stamp = static_cast<ros::Time>(last_time);
//   _thruster_Status.motor_duty = 20; // duty cycle
//   _thruster_Status.motor_temp_on_chip = 30; // temperature on chip
//   _thruster_Status.motor_temp_ambient = 40; // temperature ambient
//   _thruster_Status.motor_current = 100; // current
//   _thruster_Status.motor_rspeed = 1000; // current speed
//   _thruster_Status.motor_dspeed = 0; // desired speed
  pubThruster.publish(_thruster_Status);

//  board_arm1_stamped _board_arm1_Status;
// _board_arm1_Status.header.stamp = static_cast<ros::Time>(last_time);
// _board_arm1_Status.pistol_ls.ls_AtHead = _board_arm1_data.pistol_ls.ls_AtHead;
// _board_arm1_Status.pistol_ls.ls_AtTail = _board_arm1_data.pistol_ls.ls_AtTail;
// isPistol_atHead = _board_arm1_Status.pistol_ls.ls_AtHead;
// isPistol_atTail = _board_arm1_Status.pistol_ls.ls_AtTail;
// _board_arm1_Status.mass_shifter_ls.ls_AtHead = _board_arm1_data.mass_shifter_ls.ls_AtHead;
// _board_arm1_Status.mass_shifter_ls.ls_AtTail = _board_arm1_data.mass_shifter_ls.ls_AtTail;
// isMass_atHead = _board_arm1_Status.mass_shifter_ls.ls_AtHead;
// isMass_atTail = _board_arm1_Status.mass_shifter_ls.ls_AtTail;
// _board_arm1_Status.bms_status.batteryUsed = _board_arm1_data.bms_status.batteryUsed;
// _board_arm1_Status.bms_status.batteryTotal = _board_arm1_data.bms_status.batteryTotal;
// _board_arm1_Status.bms_status.batteryCurrent = _board_arm1_data.bms_status.batteryCurrent;
// _board_arm1_Status.bms_status.batteryCapacity = _board_arm1_data.bms_status.batteryCapacity;
// _board_arm1_Status.bms_status.batteryPercentage = _board_arm1_data.bms_status.batteryPercentage;
// _board_arm1_Status.altimeter_status.alt_in_feet = _board_arm1_data.altimeter_status.alt_in_feet;
// _board_arm1_Status.altimeter_status.alt_in_metres = _board_arm1_data.altimeter_status.alt_in_metres;
// _board_arm1_Status.altimeter_status.alt_in_fathoms = _board_arm1_data.altimeter_status.alt_in_fathoms;

//   _board_arm1_Status.pistol_ls.ls_AtHead = 0x00;
//   _board_arm1_Status.pistol_ls.ls_AtTail = 0x01;
//   _board_arm1_Status.mass_shifter_ls.ls_AtHead = 0x01;
//   _board_arm1_Status.mass_shifter_ls.ls_AtTail = 0x00;
//   _board_arm1_Status.bms_status.batteryUsed = 200;
//   _board_arm1_Status.bms_status.batteryTotal = 4000;
//   _board_arm1_Status.bms_status.batteryCurrent = 1000;
//   _board_arm1_Status.bms_status.batteryCapacity = 500;
//   _board_arm1_Status.bms_status.batteryPercentage = 50;
//   _board_arm1_Status.altimeter_status.alt_in_feet = 1;
//   _board_arm1_Status.altimeter_status.alt_in_metres = 2;
//   _board_arm1_Status.altimeter_status.alt_in_fathoms = 3;
//  pubBoard_ARM1.publish(_board_arm1_Status);

  board_arm2_stamped _board_arm2_Status;
 _board_arm2_Status.header.stamp = static_cast<ros::Time>(last_time);
 _board_arm2_Status.mx28_status.load = _board_arm2_data.mx28_status.load;
 _board_arm2_Status.mx28_status.speed = _board_arm2_data.mx28_status.speed;
 _board_arm2_Status.mx28_status.voltage = _board_arm2_data.mx28_status.voltage;
 _board_arm2_Status.mx28_status.position = (_board_arm2_data.mx28_status.position * 360.0f / 4095.0f) - 180.0f - 45.0f;
 _board_arm2_Status.mx28_status.temperature = _board_arm2_data.mx28_status.temperature;
 _board_arm2_Status.bms_status.batteryUsed = _board_arm1_data.bms_status.batteryUsed;
 _board_arm2_Status.bms_status.batteryTotal = _board_arm1_data.bms_status.batteryTotal;
 _board_arm2_Status.bms_status.batteryCurrent = _board_arm1_data.bms_status.batteryCurrent;
 _board_arm2_Status.bms_status.batteryCapacity = _board_arm1_data.bms_status.batteryCapacity;
 _board_arm2_Status.bms_status.batteryPercentage = _board_arm1_data.bms_status.batteryPercentage;
//  _board_arm2_Status.bms_status.batteryUsed = _board_arm2_data.bms_status.batteryUsed;
//  _board_arm2_Status.bms_status.batteryTotal = _board_arm2_data.bms_status.batteryTotal;
//  _board_arm2_Status.bms_status.batteryCurrent = _board_arm2_data.bms_status.batteryCurrent;
//  _board_arm2_Status.bms_status.batteryCapacity = _board_arm2_data.bms_status.batteryCapacity;
//  _board_arm2_Status.bms_status.batteryPercentage = _board_arm2_data.bms_status.batteryPercentage;
 _board_arm2_Status.pressure_status.pressure = _board_arm2_data.pressure_status.pressure;
 _board_arm2_Status.pressure_status.temperature = _board_arm2_data.pressure_status.temperature;

//   _board_arm2_Status.header.stamp = static_cast<ros::Time>(last_time);
//   _board_arm2_Status.mx28_status.load = 1.0;
//   _board_arm2_Status.mx28_status.speed = 2.0;
//   _board_arm2_Status.mx28_status.voltage = 3.0;
//   _board_arm2_Status.mx28_status.position = 4.0;
//   _board_arm2_Status.mx28_status.temperature = 5.0;
//   _board_arm2_Status.bms_status.batteryUsed = 100.0;
//   _board_arm2_Status.bms_status.batteryTotal = 200.0;
//   _board_arm2_Status.bms_status.batteryCurrent = 300.0;
//   _board_arm2_Status.bms_status.batteryCapacity = 400.0;
//   _board_arm2_Status.bms_status.batteryPercentage = 30.0;
//   _board_arm2_Status.bms_status.baterryVoltage = 24.0;
//   _board_arm2_Status.pressure_status.pressure = 15.0;
//   _board_arm2_Status.pressure_status.temperature = 35.0;
  pubBoard_ARM2.publish(_board_arm2_Status);

//  ROS_INFO("Thread onLoopPublishDataCallBack");
//  ros::Duration(0.5).sleep();
  mutexPulishData.unlock();
}

void My_AUV2000_Actuator::onGetJoystickCallBack(const joystick_stamped::ConstPtr& msg)
{
  mutexWriteData.lock();

  while(CANBUS_OnRequest || CANBUS_OnWriting);
  CANBUS_OnWriting = true;

  /// Send Duty Mass-Shifter
  if((msg->mass_shifter_up == true) && (isMass_Run_CW == false) && (isMass_atHead == 0x00))
  {
    isMass_Run_CCW = true;
    Mass_Direction = M_CCW;
    Desired_Mass_DutyCycle += static_cast<float>(STEP_Mass_DutyCycle);
    if(Desired_Mass_DutyCycle >= static_cast<float>(MAX_Mass_DutyCycle)) Desired_Mass_DutyCycle = static_cast<float>(MAX_Mass_DutyCycle);
  }
  else if ((msg->mass_shifter_down == true) && (isMass_Run_CCW == false) && (isMass_atTail == 0x00))
  {
    isMass_Run_CW = true;
    Mass_Direction = M_CW;
    Desired_Mass_DutyCycle += static_cast<float>(STEP_Mass_DutyCycle);
    if(Desired_Mass_DutyCycle >= static_cast<float>(MAX_Mass_DutyCycle)) Desired_Mass_DutyCycle = static_cast<float>(MAX_Mass_DutyCycle);
  }
  else
  {
    Desired_Mass_DutyCycle -= static_cast<float>(STEP_Mass_DutyCycle/2);
    if(Desired_Mass_DutyCycle <= 0.0f)
    {
      isMass_Run_CW = false;
      isMass_Run_CCW = false;
      Desired_Mass_DutyCycle = 0.0f;
    }
  }

  if(preDesired_Mass_DutyCycle != Desired_Mass_DutyCycle)
  {
      UCAN_Mass_Send_DutyCycle(Mass_Direction, Desired_Mass_DutyCycle);
   //  ROS_INFO_STREAM("Direction: " << Mass_Direction);
   //  ROS_INFO_STREAM("Duty Mass: " << Desired_Mass_DutyCycle);
     preDesired_Mass_DutyCycle = Desired_Mass_DutyCycle;
  }
//   usleep(10);

  /// Send Duty Pistol
  if((msg->pistol_up == true) && (isPistol_Run_CCW == false) && (isPistol_atHead == 0x00))
  {
    isPistol_Run_CW = true;
    Pistol_Direction = M_CW;
    Desired_Pistol_DutyCycle += static_cast<float>(STEP_Pistol_DutyCycle);
    if(Desired_Pistol_DutyCycle >= static_cast<float>(MAX_Pistol_DutyCycle)) Desired_Pistol_DutyCycle = static_cast<float>(MAX_Pistol_DutyCycle);
  }
  else if ((msg->pistol_down == true) && (isPistol_Run_CW == false) && (isPistol_atTail == 0x00))
  {
    isPistol_Run_CCW = true;
    Pistol_Direction = M_CCW;
    Desired_Pistol_DutyCycle += static_cast<float>(STEP_Pistol_DutyCycle);
    if(Desired_Pistol_DutyCycle >= static_cast<float>(MAX_Pistol_DutyCycle)) Desired_Pistol_DutyCycle = static_cast<float>(MAX_Pistol_DutyCycle);
  }
  else
  {
    Desired_Pistol_DutyCycle -= static_cast<float>(STEP_Pistol_DutyCycle);
    if(Desired_Pistol_DutyCycle <= 0.0f)
    {
      isPistol_Run_CW = false;
      isPistol_Run_CCW = false;
      Desired_Pistol_DutyCycle = 0.0f;
    }
  }

  if(preDesired_Pistol_DutyCycle != Desired_Pistol_DutyCycle)
  {
    UCAN_Pistol_Send_DutyCycle(Pistol_Direction, Desired_Pistol_DutyCycle);
   //  ROS_INFO_STREAM("Direction: " << Pistol_Direction);
   //  ROS_INFO_STREAM("Duty Pistol: " << Desired_Pistol_DutyCycle);
    preDesired_Pistol_DutyCycle = Desired_Pistol_DutyCycle;
  }
//   usleep(10);

  Desired_Thruster_Speed = msg->thruster;
  if(preDesired_Thruster_Speed != Desired_Thruster_Speed)
  {
    UCAN_Thruster_Send_PID_DesiredSpeed(M_CCW, Desired_Thruster_Speed);
   //  ROS_INFO_STREAM("Duty Thruster: " << Desired_Thruster_Speed);
    preDesired_Thruster_Speed = Desired_Thruster_Speed;
  }
//   usleep(10);

  Desired_Rudder_Position = msg->rudder;
  if(preDesired_Rudder_Position != Desired_Rudder_Position)
  {
    UCAN_Rudder_Send_GoalPostion(Desired_Rudder_Position);
   //  ROS_INFO_STREAM("Duty Rudder: " << Desired_Rudder_Position);
    preDesired_Rudder_Position = Desired_Rudder_Position;
  }
//   usleep(10);
  ros::Duration(0.03).sleep();
  CANBUS_OnWriting = false;
  mutexWriteData.unlock();
}

void onRequestCANDataCallBack(const ros::WallTimerEvent& event)
{
  std::lock_guard<std::mutex> lock(mutexRequestData);
  requestCANData.stop();
  while(CANBUS_OnRequest || CANBUS_OnWriting);
  CANBUS_OnRequest = true;

 CANBUS_ACK = false;
 UCAN_RequestData(AUV_ARMBOARD_1);
 uint32_t _timeout = 20;
  do
  {
      _timeout--;
      ros::Duration(0.001).sleep();
  }
  while((!CANBUS_ACK) && _timeout);
//  ros::Duration(0.001).sleep();

 CANBUS_ACK = false;
 UCAN_RequestData(AUV_ARMBOARD_2);
 do
  {
      _timeout--;
      ros::Duration(0.001).sleep();
  }
  while((!CANBUS_ACK) && _timeout);
//  ros::Duration(0.001).sleep();

 CANBUS_ACK = false;
 UCAN_RequestData(AUV_THRUSTER);
 do
  {
      _timeout--;
      ros::Duration(0.001).sleep();
  }
  while((!CANBUS_ACK) && _timeout);
//  ros::Duration(0.001).sleep();

//   ROS_INFO_STREAM("OnRequestData");  
  CANBUS_ACK = false;
  UCAN_RequestData(AUV_MASS_SHIFTER);
  do
  {
      _timeout--;
      ros::Duration(0.001).sleep();
  }
  while((!CANBUS_ACK) && _timeout);  
//  ros::Duration(0.001).sleep();

  CANBUS_ACK = false;
  UCAN_RequestData(AUV_PISTOL);
  do
  {
      _timeout--;
      ros::Duration(0.001).sleep();
  }
  while((!CANBUS_ACK) && _timeout);
//   ros::Duration(0.001).sleep();

  CANBUS_OnRequest = false;
  requestCANData.start();
}

void UCAN_RequestData(AUV_BOARD_TypeDef _type_board)
{
  switch (_type_board) {
    case AUV_ARMBOARD_1:
    {
      ROS_CANBUS_Data[0] = ARM1_ALL_DATA;
      ROS_CANBUS_Data[1] = READ_DATA;
      ROS_CANBUS_Data[2] = 0x00;
      ROS_CANBUS_Data[3] = 0x00;
      ROS_CANBUS_Data[4] = 0x00;
      ROS_CANBUS_Data[5] = 0x00;
      ROS_CANBUS_Data[6] = 0x0A;
      ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
      UCAN_Send(AUV_ARMBOARD_1, 8, ROS_CANBUS_Data);
      break;
    }
    case AUV_ARMBOARD_2:
    {
      ROS_CANBUS_Data[0] = ARM2_ALL_DATA;
      ROS_CANBUS_Data[1] = READ_DATA;
      ROS_CANBUS_Data[2] = 0x00;
      ROS_CANBUS_Data[3] = 0x00;
      ROS_CANBUS_Data[4] = 0x00;
      ROS_CANBUS_Data[5] = 0x00;
      ROS_CANBUS_Data[6] = 0x0A;
      ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
      UCAN_Send(AUV_ARMBOARD_2, 8, ROS_CANBUS_Data);
      break;
    }
    case AUV_THRUSTER:
    {
      ROS_CANBUS_Data[0] = 'R';
      ROS_CANBUS_Data[1] = 'E';
      ROS_CANBUS_Data[2] = 'Q';
      ROS_CANBUS_Data[3] = 'A';
      ROS_CANBUS_Data[4] = 'L';
      ROS_CANBUS_Data[5] = 'L';
      ROS_CANBUS_Data[6] = 0x0A;
      ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
      UCAN_Send(AUV_THRUSTER, 8, ROS_CANBUS_Data);
      break;
    }
    case AUV_MASS_SHIFTER:
    {
      ROS_CANBUS_Data[0] = 'R';
      ROS_CANBUS_Data[1] = 'E';
      ROS_CANBUS_Data[2] = 'Q';
      ROS_CANBUS_Data[3] = 'A';
      ROS_CANBUS_Data[4] = 'L';
      ROS_CANBUS_Data[5] = 'L';
      ROS_CANBUS_Data[6] = 0x0A;
      ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
      UCAN_Send(AUV_MASS_SHIFTER, 8, ROS_CANBUS_Data);
      break;
    }
    case AUV_PISTOL:
    {
      ROS_CANBUS_Data[0] = 'R';
      ROS_CANBUS_Data[1] = 'E';
      ROS_CANBUS_Data[2] = 'Q';
      ROS_CANBUS_Data[3] = 'A';
      ROS_CANBUS_Data[4] = 'L';
      ROS_CANBUS_Data[5] = 'L';
      ROS_CANBUS_Data[6] = 0x0A;
      ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
      UCAN_Send(AUV_PISTOL, 8, ROS_CANBUS_Data);
      break;
    }
  }
}

void UCAN_PistolCmd(Function_Cmd_TypeDef _state)
{
  if(_state ==  ENABLE)
  {
    ROS_CANBUS_Data[0] = 'C';
    ROS_CANBUS_Data[1] = 'A';
    ROS_CANBUS_Data[2] = 'N';
    ROS_CANBUS_Data[3] = 'O';
    ROS_CANBUS_Data[4] = 0x00;
    ROS_CANBUS_Data[5] = 0x00;
    ROS_CANBUS_Data[6] = 0x00;
    ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
    UCAN_Send(AUV_PISTOL, 8, ROS_CANBUS_Data);
  }
  else
  {
    ROS_CANBUS_Data[0] = 'C';
    ROS_CANBUS_Data[1] = 'A';
    ROS_CANBUS_Data[2] = 'N';
    ROS_CANBUS_Data[3] = 'L';
    ROS_CANBUS_Data[4] = 0x00;
    ROS_CANBUS_Data[5] = 0x00;
    ROS_CANBUS_Data[6] = 0x00;
    ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
    UCAN_Send(AUV_PISTOL, 8, ROS_CANBUS_Data);
  }
}

void UCAN_Pistol_Send_DutyCycle(Motor_Direction_TypeDef _direction, float _duty_cycle)
{
  ROS_CANBUS_Data[0] = 'O';
  ROS_CANBUS_Data[1] = 'L';
  ROS_CANBUS_Data[2] = _direction;
  UMOTOR_Convert_Float_to_Bytes(_duty_cycle, &ROS_CANBUS_Data[3]);
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_PISTOL, 8, ROS_CANBUS_Data);
  ROS_INFO_STREAM("Duty Cycle: " << ROS_CANBUS_Data[0] << " " << ROS_CANBUS_Data[1] << " " << ROS_CANBUS_Data[2] << " " << ROS_CANBUS_Data[3]
                  << " " << ROS_CANBUS_Data[4] << " " << ROS_CANBUS_Data[5] << " " << ROS_CANBUS_Data[6] << " " << ROS_CANBUS_Data[7]);
//   ROS_INFO_STREAM("Duty Cycle: " << _duty_cycle);
}

void UCAN_Mass_ShifterCmd(Function_Cmd_TypeDef _state)
{
  if(_state ==  ENABLE)
  {
    ROS_CANBUS_Data[0] = 'C';
    ROS_CANBUS_Data[1] = 'A';
    ROS_CANBUS_Data[2] = 'N';
    ROS_CANBUS_Data[3] = 'O';
    ROS_CANBUS_Data[4] = 0x00;
    ROS_CANBUS_Data[5] = 0x00;
    ROS_CANBUS_Data[6] = 0x00;
    ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
    UCAN_Send(AUV_MASS_SHIFTER, 8, ROS_CANBUS_Data);
  }
  else
  {
    ROS_CANBUS_Data[0] = 'C';
    ROS_CANBUS_Data[1] = 'A';
    ROS_CANBUS_Data[2] = 'N';
    ROS_CANBUS_Data[3] = 'L';
    ROS_CANBUS_Data[4] = 0x00;
    ROS_CANBUS_Data[5] = 0x00;
    ROS_CANBUS_Data[6] = 0x00;
    ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
    UCAN_Send(AUV_MASS_SHIFTER, 8, ROS_CANBUS_Data);
  }
}

void UCAN_Mass_Send_DutyCycle(Motor_Direction_TypeDef _direction, float _duty_cycle)
{
  ROS_CANBUS_Data[0] = 'O';
  ROS_CANBUS_Data[1] = 'L';
  ROS_CANBUS_Data[2] = _direction;
  UMOTOR_Convert_Float_to_Bytes(_duty_cycle, &ROS_CANBUS_Data[3]);
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_MASS_SHIFTER, 8, ROS_CANBUS_Data);
}

void UCAN_ThrusterCmd(Function_Cmd_TypeDef _state)
{
  if(_state ==  ENABLE)
  {
    ROS_CANBUS_Data[0] = 'C';
    ROS_CANBUS_Data[1] = 'A';
    ROS_CANBUS_Data[2] = 'N';
    ROS_CANBUS_Data[3] = 'O';
    ROS_CANBUS_Data[4] = 0x00;
    ROS_CANBUS_Data[5] = 0x00;
    ROS_CANBUS_Data[6] = 0x00;
    ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
    UCAN_Send(AUV_THRUSTER, 8, ROS_CANBUS_Data);
  }
  else
  {
    ROS_CANBUS_Data[0] = 'C';
    ROS_CANBUS_Data[1] = 'A';
    ROS_CANBUS_Data[2] = 'N';
    ROS_CANBUS_Data[3] = 'L';
    ROS_CANBUS_Data[4] = 0x00;
    ROS_CANBUS_Data[5] = 0x00;
    ROS_CANBUS_Data[6] = 0x00;
    ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
    UCAN_Send(AUV_THRUSTER, 8, ROS_CANBUS_Data);
  }
}

void UCAN_Thruster_Send_DutyCycle(Motor_Direction_TypeDef _direction, float _duty_cycle)
{
  ROS_CANBUS_Data[0] = 'O';
  ROS_CANBUS_Data[1] = 'L';
  ROS_CANBUS_Data[2] = _direction;
  UMOTOR_Convert_Float_to_Bytes(_duty_cycle, &ROS_CANBUS_Data[3]);
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_THRUSTER, 8, ROS_CANBUS_Data);
}

void UCAN_Thruster_Send_PID_Kp(float _kp)
{
  ROS_CANBUS_Data[0] = 'G';
  ROS_CANBUS_Data[1] = 'K';
  ROS_CANBUS_Data[2] = 'P';
  UMOTOR_Convert_Float_to_Bytes(_kp, &ROS_CANBUS_Data[3]);
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_THRUSTER, 8, ROS_CANBUS_Data);
}

void UCAN_Thruster_Send_PID_Ki(float _ki)
{
  ROS_CANBUS_Data[0] = 'G';
  ROS_CANBUS_Data[1] = 'K';
  ROS_CANBUS_Data[2] = 'I';
  UMOTOR_Convert_Float_to_Bytes(_ki, &ROS_CANBUS_Data[3]);
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_THRUSTER, 8, ROS_CANBUS_Data);
}

void UCAN_Thruster_Send_PID_Kd(float _kd)
{
  ROS_CANBUS_Data[0] = 'G';
  ROS_CANBUS_Data[1] = 'K';
  ROS_CANBUS_Data[2] = 'D';
  UMOTOR_Convert_Float_to_Bytes(_kd, &ROS_CANBUS_Data[3]);
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_THRUSTER, 8, ROS_CANBUS_Data);
}

void UCAN_Thruster_Send_PID_DesiredSpeed(Motor_Direction_TypeDef _direction, float _dspeed)
{
  ROS_CANBUS_Data[0] = 'C';
  ROS_CANBUS_Data[1] = 'P';
  ROS_CANBUS_Data[2] = _direction;
  UMOTOR_Convert_Float_to_Bytes(_dspeed, &ROS_CANBUS_Data[3]);
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_THRUSTER, 8, ROS_CANBUS_Data);
}

void UCAN_Thruster_Send_PID_Ge(float _ge)
{
  ROS_CANBUS_Data[0] = 'G';
  ROS_CANBUS_Data[1] = 'G';
  ROS_CANBUS_Data[2] = 'E';
  UMOTOR_Convert_Float_to_Bytes(_ge, &ROS_CANBUS_Data[3]);
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_THRUSTER, 8, ROS_CANBUS_Data);
}

void UCAN_Thruster_Send_PID_Gde(float _gde)
{
  ROS_CANBUS_Data[0] = 'G';
  ROS_CANBUS_Data[1] = 'D';
  ROS_CANBUS_Data[2] = 'E';
  UMOTOR_Convert_Float_to_Bytes(_gde, &ROS_CANBUS_Data[3]);
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_THRUSTER, 8, ROS_CANBUS_Data);
}

void UCAN_Thruster_Send_PID_Gdu(float _gdu)
{
  ROS_CANBUS_Data[0] = 'G';
  ROS_CANBUS_Data[1] = 'D';
  ROS_CANBUS_Data[2] = 'U';
  UMOTOR_Convert_Float_to_Bytes(_gdu, &ROS_CANBUS_Data[3]);
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_THRUSTER, 8, ROS_CANBUS_Data);
}

void UCAN_Thruster_Send_Fuzzy_DesiredSpeed(Motor_Direction_TypeDef _direction, float _dspeed)
{
  ROS_CANBUS_Data[0] = 'C';
  ROS_CANBUS_Data[1] = 'F';
  ROS_CANBUS_Data[2] = _direction;
  UMOTOR_Convert_Float_to_Bytes(_dspeed, &ROS_CANBUS_Data[3]);
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_THRUSTER, 8, ROS_CANBUS_Data);
}

void UCAN_Rudder_Send_GoalPostion(int16_t _angle)
{
  uint16_t _desired_GoalPostion = 0;
  _desired_GoalPostion = static_cast<uint16_t>((static_cast<float>(_angle) + 180.0f + 45.0f) * 4095.0f / 360.0f);
  ROS_CANBUS_Data[0] = ARM2_RUDDER;
  ROS_CANBUS_Data[1] = WRITE_DATA;
  ROS_CANBUS_Data[2] = ARM2_MX28_GOAL_POSITION;
  ROS_CANBUS_Data[3] = static_cast<uint8_t>(_desired_GoalPostion & 0x00FF);
  ROS_CANBUS_Data[4] = static_cast<uint8_t>((_desired_GoalPostion & 0xFF00) >> 8);
  ROS_CANBUS_Data[5] = 0x00;
  ROS_CANBUS_Data[6] = 0x00;
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_ARMBOARD_2, 8, ROS_CANBUS_Data);
}

void UCAN_Rudder_Send_MovingSpeed(uint16_t _speed)
{
  ROS_CANBUS_Data[0] = ARM2_RUDDER;
  ROS_CANBUS_Data[1] = WRITE_DATA;
  ROS_CANBUS_Data[2] = ARM2_MX28_MOVING_SPEED;
  ROS_CANBUS_Data[3] = static_cast<uint8_t>(_speed & 0x00FF);
  ROS_CANBUS_Data[4] = static_cast<uint8_t>((_speed & 0xFF00) >> 8);
  ROS_CANBUS_Data[5] = 0x00;
  ROS_CANBUS_Data[6] = 0x00;
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_ARMBOARD_2, 8, ROS_CANBUS_Data);
}

void UCAN_Rudder_Send_Kp(Rudder_KP_TypeDef _kp)
{
  ROS_CANBUS_Data[0] = ARM2_RUDDER;
  ROS_CANBUS_Data[1] = WRITE_DATA;
  ROS_CANBUS_Data[2] = ARM2_MX28_KP;
  ROS_CANBUS_Data[3] = static_cast<uint8_t>(_kp & 0x00FF);
  ROS_CANBUS_Data[4] = 0x00;
  ROS_CANBUS_Data[5] = 0x00;
  ROS_CANBUS_Data[6] = 0x00;
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_ARMBOARD_2, 8, ROS_CANBUS_Data);
}

void UCAN_Rudder_Send_Ki(float _ki)
{
  ROS_CANBUS_Data[0] = ARM2_RUDDER;
  ROS_CANBUS_Data[1] = WRITE_DATA;
  ROS_CANBUS_Data[2] = ARM2_MX28_KI;
  UMOTOR_Convert_Float_to_Bytes(_ki, &ROS_CANBUS_Data[3]);
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_THRUSTER, 8, ROS_CANBUS_Data);
}

void UCAN_Rudder_Send_Kd(float _kd)
{
  ROS_CANBUS_Data[0] = ARM2_RUDDER;
  ROS_CANBUS_Data[1] = WRITE_DATA;
  ROS_CANBUS_Data[2] = ARM2_MX28_KD;
  UMOTOR_Convert_Float_to_Bytes(_kd, &ROS_CANBUS_Data[3]);
  ROS_CANBUS_Data[7] = UCAN_Checksum(ROS_CANBUS_Data, 0, 7);
  UCAN_Send(AUV_THRUSTER, 8, ROS_CANBUS_Data);
}

uint8_t UCAN_Checksum(uint8_t *_data, int start, int count)
{
  int value = 0;

  //Calculate CheckSum (Byte)
  for (int i = start; i < (count + start); i++)
  {
    value += _data[i];
  }
  value = ~value;
  value++;
  return static_cast<uint8_t>(value);
}

uint8_t UCAN_Send(const uint32_t& _id, const uint8_t& _buffer_size, const uint8_t* _data)
{
  mutexSocketCAN.lock();

  can::Header header = can::Header(_id & can::Header::ID_MASK, _id & can::Header::EXTENDED_MASK, _id & can::Header::RTR_MASK, _id & can::Header::ERROR_MASK);
  can::Frame frame(header);
  for(int i = 0; i <  _buffer_size; i++)
  {
    frame.data[i] = _data[i];
  }
  frame.dlc =  _buffer_size;

//   if (sendto(_socket, &_can_frame, sizeof(struct can_frame), MSG_DONTWAIT, (struct sockaddr*)&SocketCAN_addr, sizeof(SocketCAN_addr)))
//   {
//      perror("Write");
//      mutexSocketCAN.unlock();
//      ros::Duration(0.01).sleep();
//      return 1;
//   }
//   else
//   {
//     mutexSocketCAN.unlock();
//     ros::Duration(0.01).sleep();
//     return 0;
//   }
  if (CAN_Driver.send(frame))
  {
     perror("Write");
     mutexSocketCAN.unlock();
     ros::Duration(0.01).sleep();
     return 1;
  }
  else
  {
    mutexSocketCAN.unlock();
    ros::Duration(0.01).sleep();
    return 0;
  }
//   if (write(_socket, &_can_frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
//   {
//      perror("Write");
//      mutexSocketCAN.unlock();
//      ros::Duration(0.01).sleep();
//      return 1;
//   }
//   else
//   { 
//     mutexSocketCAN.unlock();
//     ros::Duration(0.01).sleep();
//     return 0;
//   }
}

void My_AUV2000_Actuator::onItemListCallBack(const WaypointList::ConstPtr& msg)
{
  for (auto it = msg->waypoints.begin(); it != msg->waypoints.end(); it++)
  {
    switch (it->command)
    {
    case 16: // MAV_CMD_NAV_WAYPOINT
      ROS_INFO_STREAM("Waypoint: "
                      << "lat = " << it->x_lat * 1e-7 << ", lon = " << it->y_long * 1e-7 << ", alt = " << it->z_alt
                      << ".");
      break;
    case 20: // MAV_CMD_NAV_RETURN_TO_LAUNCH
      ROS_INFO_STREAM("Waypoint (return to launch): "
                      << "lat = (1st waypoint lat), lon = (1st waypoint lon)");
      break;
    }
  }
}

bool My_AUV2000_Actuator::onSetArmingCallBack(CommandLongRequest& req, CommandLongResponse& res)
{
  if (req.param1 == 1.0f)
    ROS_INFO("Thruster unlocked.");
  else if (req.param1 == 0.0f)
    ROS_INFO("Thruster locked.");
  res.result = 0; // MAV_RESULT_ACCEPTED

  return true;
}

bool My_AUV2000_Actuator::onStartMissionCallBack(CommandLongRequest& /*req*/, CommandLongResponse& res)
{
  ROS_INFO("Mission started.");
  res.result = 0; // MAV_RESULT_ACCEPTED

  return true;
}

bool My_AUV2000_Actuator::onSetModeCallBack(SetModeRequest& req, SetModeResponse& res)
{
  ROS_INFO_STREAM("Base mode set to " << int(req.base_mode) << ", custom mode set to " << req.custom_mode << ".");
  res.mode_sent = 0; // MAV_RESULT_ACCEPTED

  return true;
}

bool My_AUV2000_Actuator::onSetAUVControllerCallBack(CommandLongRequest& req, CommandLongResponse& res)
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
  else if (static_cast<int>(req.param1) == ROLL_PID){
      if(static_cast<int>(req.param2) == 1){
        ROS_INFO("AUV CONTROLLER: ROLL_PID --> ENABLE");
         rollPIDEnable = 1;
         rollPID.resetPID();
      }
      else{
        ROS_INFO("AUV CONTROLLER: ROLL_PID --> DISABLE");
        rollPIDEnable = 0;
      }
  }
  else if (static_cast<int>(req.param1) == ROLL_SM){
      if(static_cast<int>(req.param2) == 1){
        ROS_INFO("AUV CONTROLLER: ROLL_SM --> ENABLE");
      }
      else{
        ROS_INFO("AUV CONTROLLER: ROLL_SM --> DISABLE");
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

bool My_AUV2000_Actuator::OnGetJoystickConfigParamReal32CallBack(ParamGetRequest &req, ParamGetResponse &res)
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

bool My_AUV2000_Actuator::OnGetJoystickConfigParamInt16CallBack(ParamGetRequest &req, ParamGetResponse &res)
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

bool My_AUV2000_Actuator::OnGetJoystickConfigParamInt8CallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;

  if(compareString(req.param_id.data(),"JS_GAIN_STEPS"))
  {
     res.value.integer=4;
     res.success=true;
  }

  return res.success;
}

bool My_AUV2000_Actuator::OnGetJoystickFunctionParamCallBack(ParamGetRequest &req, ParamGetResponse &res)
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

bool My_AUV2000_Actuator::OnGetARMINGCallBack(ParamGetRequest &req, ParamGetResponse &res)
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

bool My_AUV2000_Actuator::onSetRoll_PIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "ROLL_PID_DES"))
  {
    ROS_INFO_STREAM("ROLL_PID_DESIRED set to " << req.value.real << ".");
    desiredRoll = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "ROLL_PID_KP"))
  {
    ROS_INFO_STREAM("ROLL_PID_KP set to " << req.value.real << ".");
    rollPID.Kp = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "ROLL_PID_KI"))
  {
    ROS_INFO_STREAM("ROLL_PID_KI set to " << req.value.real << ".");
    rollPID.Ki = req.value.real;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "ROLL_PID_KD"))
  {
    ROS_INFO_STREAM("ROLL_PID_KD set to " << req.value.real << ".");
    rollPID.Kd = req.value.real;
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool My_AUV2000_Actuator::onGetRoll_PIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "ROLL_PID_DES"))
  {
    res.value.real = 1;
    desiredRoll   = 1;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "ROLL_PID_KP"))
  {
    res.value.real = 8;
    rollPID.Kp = 8;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "ROLL_PID_KI"))
  {
    res.value.real = 3;
    rollPID.Ki = 3;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "ROLL_PID_KD"))
  {
    res.value.real = 0.5;
    rollPID.Kd = 0.5;
    res.success = true;
  }

  return res.success;
}

bool My_AUV2000_Actuator::onSetSpeed_PIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
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

bool My_AUV2000_Actuator::onGetSpeed_PIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

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

bool My_AUV2000_Actuator::onSetHeading_PIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "HEADING_PID_DES"))
  {
    ROS_INFO_STREAM("HEADING_PID_DESIRED set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_PID_KP"))
  {
    ROS_INFO_STREAM("HEADING_PID_KP set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_PID_KI"))
  {
    ROS_INFO_STREAM("HEADING_PID_KI set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_PID_KD"))
  {
    ROS_INFO_STREAM("HEADING_PID_KD set to " << req.value.real << ".");
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool My_AUV2000_Actuator::onGetHeading_PIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "HEADING_PID_DES"))
  {
    res.value.real = 4.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_PID_KP"))
  {
    res.value.real = 2.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_PID_KI"))
  {
    res.value.real = 3.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "HEADING_PID_KD"))
  {
    res.value.real = 4.0;
    res.success = true;
  }

  return res.success;
}

bool My_AUV2000_Actuator::onSetPitch_PIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "PITCH_PID_DES"))
  {
    ROS_INFO_STREAM("PITCH_PID_DESIRED set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_PID_KP"))
  {
    ROS_INFO_STREAM("PITCH_PID_KP set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_PID_KI"))
  {
    ROS_INFO_STREAM("PITCH_PID_KI set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_PID_KD"))
  {
    ROS_INFO_STREAM("PITCH_PID_KD set to " << req.value.real << ".");
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool My_AUV2000_Actuator::onGetPitch_PIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "PITCH_PID_DES"))
  {
    res.value.real = 4.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_PID_KP"))
  {
    res.value.real = 5.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_PID_KI"))
  {
    res.value.real = 6.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "PITCH_PID_KD"))
  {
    res.value.real = 7.0;
    res.success = true;
  }

  return res.success;
}

bool My_AUV2000_Actuator::onSetDepth_PIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "DEPTH_PID_DES"))
  {
    ROS_INFO_STREAM("DEPTH_PID_DESIRED set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_PID_KP"))
  {
    ROS_INFO_STREAM("DEPTH_PID_KP set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_PID_KI"))
  {
    ROS_INFO_STREAM("DEPTH_PID_KI set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_PID_KD"))
  {
    ROS_INFO_STREAM("DEPTH_PID_KD set to " << req.value.real << ".");
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool My_AUV2000_Actuator::onGetDepth_PIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "DEPTH_PID_DES"))
  {
    res.value.real = 8.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_PID_KP"))
  {
    res.value.real = 9.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_PID_KI"))
  {
    res.value.real = 10.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "DEPTH_PID_KD"))
  {
    res.value.real = 11.0;
    res.success = true;
  }

  return res.success;
}

bool My_AUV2000_Actuator::onSetLOS_PIDCallBack(ParamSetRequest& req, ParamSetResponse& res)
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
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_PID_KI"))
  {
    ROS_INFO_STREAM("LOS_PID_KI set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_PID_KD"))
  {
    ROS_INFO_STREAM("LOS_PID_KD set to " << req.value.real << ".");
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool My_AUV2000_Actuator::onGetLOS_PIDCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "LOS_PID_DES"))
  {
    res.value.real = 12.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_PID_KP"))
  {
    res.value.real = 13.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_PID_KI"))
  {
    res.value.real = 14.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_PID_KD"))
  {
    res.value.real = 15.0;
    res.success = true;
  }

  return res.success;
}

bool My_AUV2000_Actuator::onSetRoll_SMCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "ROLL_SM_DES"))
  {
    ROS_INFO_STREAM("ROLL_SM_DESIRED set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "ROLL_SM_E"))
  {
    ROS_INFO_STREAM("ROLL_SM_EPSILON set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "ROLL_SM_K"))
  {
    ROS_INFO_STREAM("ROLL_SM_K set to " << req.value.real << ".");
    res.success = true;
  }

  res.value.real = req.value.real;
  return res.success;
}

bool My_AUV2000_Actuator::onGetRoll_SMCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "ROLL_SM_DES"))
  {
    res.value.real = 16.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "ROLL_SM_E"))
  {
    res.value.real = 17.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "ROLL_SM_K"))
  {
    res.value.real = 18.0;
    res.success = true;
  }

  return res.success;
}

bool My_AUV2000_Actuator::onSetSpeed_SMCallBack(ParamSetRequest& req, ParamSetResponse& res)
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

bool My_AUV2000_Actuator::onGetSpeed_SMCallBack(ParamGetRequest& req, ParamGetResponse& res)
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

bool My_AUV2000_Actuator::onSetHeading_SMCallBack(ParamSetRequest& req, ParamSetResponse& res)
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

bool My_AUV2000_Actuator::onGetHeading_SMCallBack(ParamGetRequest& req, ParamGetResponse& res)
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

bool My_AUV2000_Actuator::onSetPitch_SMCallBack(ParamSetRequest& req, ParamSetResponse& res)
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

bool My_AUV2000_Actuator::onGetPitch_SMCallBack(ParamGetRequest& req, ParamGetResponse& res)
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

bool My_AUV2000_Actuator::onSetDepth_SMCallBack(ParamSetRequest& req, ParamSetResponse& res)
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

bool My_AUV2000_Actuator::onGetDepth_SMCallBack(ParamGetRequest& req, ParamGetResponse& res)
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

bool My_AUV2000_Actuator::onSetLOS_SMCallBack(ParamSetRequest& req, ParamSetResponse& res)
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

bool My_AUV2000_Actuator::onGetLOS_SMCallBack(ParamGetRequest& req, ParamGetResponse& res)
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

bool My_AUV2000_Actuator::onSetLOS_ConfigCallBack(ParamSetRequest& req, ParamSetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "LOS_RADIUS"))
  {
    ROS_INFO_STREAM("LOS_RADIUS set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_DELTA_MIN"))
  {
    ROS_INFO_STREAM("LOS_DELTA_MIN set to " << req.value.real << ".");
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_DELTA_MAX"))
  {
    ROS_INFO_STREAM("LOS_DELTA_MAX set to " << req.value.real << ".");
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

bool My_AUV2000_Actuator::onGetLOS_ConfigCallBack(ParamGetRequest& req, ParamGetResponse& res)
{
  res.success = false;

  if (compareString(req.param_id.data(), "LOS_RADIUS"))
  {
    res.value.real = 28.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_DELTA_MIN"))
  {
    res.value.real = 29.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_DELTA_MAX"))
  {
    res.value.real = 30.0;
    res.success = true;
  }
  if (compareString(req.param_id.data(), "LOS_BETA"))
  {
    res.value.real = 31.0;
    res.success = true;
  }

  return res.success;
}

bool My_AUV2000_Actuator::OnGetComPassCallBack(ParamGetRequest &req, ParamGetResponse &res)
{
  res.success=false;
  if(compareString(req.param_id.data(),"COMPASS_DEC"))
  {
     res.value.integer=10;
     res.success=true;
  }
  return res.success;
}

bool My_AUV2000_Actuator::OnGetBatteryCallBack(ParamGetRequest &req, ParamGetResponse &res)
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

bool My_AUV2000_Actuator::OnGetFailSafeCallBack(ParamGetRequest &req, ParamGetResponse &res)
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

void UCAN_InitSocket(void)
{
  if(!CAN_Driver.init("can0", false)) ROS_INFO("ERROR CANBUS");
  CAN_Frames = CAN_Driver.createMsgListener(onCANBUSHandleFrames);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "My_AUV2000_Actuator");

  // Init SocketCAN Interface
 UCAN_InitSocket();
  
//  int rcvBufferSize;
//  uint sockOptSize = sizeof(rcvBufferSize);
//  getsockopt(socketCAN, SOL_SOCKET, SO_RCVBUF, &rcvBufferSize, &sockOptSize);
//  printf("initial socket receive buf %d\n", rcvBufferSize);

//  ros::AsyncSpinner spinner(0);
//  spinner.start();
  My_AUV2000_Actuator my_actuators;

//  ros::waitForShutdown();
  ros::spin();
//   if (close(socketCAN) < 0) {
//      perror("Close SocketCAN");
//      return 1;
//   }
}
