#ifndef MY_AUV2000_ACTUATORS_H
#define MY_AUV2000_ACTUATORS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <ctype.h>
#include <errno.h>
#include <libgen.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "socketcan_interface/socketcan.h"
#include "socketcan_interface/threading.h"
#include "socketcan_interface/string.h"
#include "socketcan_interface/filter.h"

#include <ros/ros.h>
#include <cstdlib>
#include <thread>
#include <mutex>
#include "std_msgs/String.h"
#include <stdbool.h>
#include <sstream>

#include <mavros_msgs/WaypointList.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs_stamped/Float64MultiArrayStamped.h>
#include "auv_msgs_stamped/gps_dvl_ins_stamped.h"
#include "auv_msgs_stamped/motor_stamped.h"
#include "auv_msgs_stamped/board_arm1_stamped.h"
#include "auv_msgs_stamped/board_arm2_stamped.h"
#include "auv_msgs_stamped/joystick_stamped.h"

#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/SetMode.h>

#include <tf2/LinearMath/Quaternion.h>

#include "math.h"
#include "heading_smc.h"
#include "pid.h"
#include "geo.h"
#include "straight_los.h"

using namespace nav_msgs;
using namespace sensor_msgs;
using namespace std_msgs_stamped;
using namespace mavros_msgs;
using namespace auv_msgs_stamped;

PID speedPID;
PID rollPID;
PID pitchPID; 
PID headingPID;
PID LOS_DepthPID;
PID LOS_PID_Heading;

double desiredSpeed;
double desiredHeading;
double desiredRoll;
double desiredPitch;
double desiredDepth;

bool manualEnable; 
bool speedPIDEnable;
bool headingPIDEnable;
bool pitchPIDEnable;
bool LOS_DepthPIDEnable;
bool LOS_PID_Heading_Enable;
bool rollPIDEnable; 

typedef struct
{
  float position;
  float motor_duty;
  float motor_temp_on_chip;
  float motor_temp_ambient;
  float motor_current;
  float motor_rspeed;
  float motor_dspeed;
}Motor_Data_TypeDef;

typedef struct
{
  uint8_t ls_AtHead;
  uint8_t ls_AtTail;
}Limit_Switch;

typedef struct
{
  float batteryTotal;
  float batteryCapacity;
  float batteryUsed;
  float batteryPercentage;
  float batteryCurrent;
  float baterryVoltage;
}BMS_Data_TypeDef;

typedef struct
{
  float alt_in_metres;
  float alt_in_fathoms;
  float alt_in_feet;
}Altimeter_Data_TypeDef;

typedef struct
{
  Limit_Switch pistol_ls;
  Limit_Switch mass_shifter_ls;
  BMS_Data_TypeDef bms_status;
  Altimeter_Data_TypeDef altimeter_status;
}Board_ARM1_Data_TypeDef;

typedef struct
{
  float position;
  float speed;
  float load;
  float voltage;
  float temperature;
}MX28_Data_Typedef;

typedef struct
{
  float pressure;
  float temperature;
}KellerPA3_Status_Typedef;

typedef struct
{
  MX28_Data_Typedef mx28_status;
  BMS_Data_TypeDef bms_status;
  KellerPA3_Status_Typedef pressure_status;
}Board_ARM2_Data_TypeDef;

typedef enum
{
  KP_8 = 8,
  KP_32 = 32,
  KP_128 = 128,
}Rudder_KP_TypeDef;

typedef enum
{
  M_CW = 'R',
  M_CCW = 'L',
  M_NONE = 0x00,
}Motor_Direction_TypeDef;

typedef enum
{
  DISABLE = 0x00,
  ENABLE = 0x01,
}Function_Cmd_TypeDef;

typedef enum
{
  AUV_ARMBOARD_1 = 0x121,
  AUV_ARMBOARD_2 = 0x122,
  AUV_MASS_SHIFTER = 0x123,
  AUV_PISTOL = 0x124,
  AUV_THRUSTER = 0x125,
  AUV_EPC = 0x126,
  AUV_BLACKBOX = 0x127,
}AUV_BOARD_TypeDef;

typedef enum
{
    //--------FUNCTION--------//
    WRITE_DATA = 0x00,
    READ_DATA = 0x01,
    STATUS_DATA = 0x02,
}Function_CANBUS_TypeDef;

typedef enum
{
    //--------ID BOARD ARM 1--------//
    ARM1_MASS_SHIFTER = 0x00,
    ARM1_PISTOL = 0x01,
    ARM1_LEAK_SENSOR = 0x02,
    ARM1_STRAIN_GAUGE = 0x03,
    ARM1_BMS24V10AH = 0x04,
    ARM1_BMS24V40AH = 0x05,
    ARM1_ALTIMETER = 0x06,
    ARM1_POWER_INT = 0x07,
    ARM1_LIGHT = 0x08,
    ARM1_ALL_DATA = 0xFF,
}ARM1_ID_CANBUS_TypeDef;

typedef enum
{
    //--------REGISTER LIGHT--------//
    ARM1_LIGHT_ENABLE = 0xFF,
    //--------REGISTER MOTOR--------//
    ARM1_REQ_ENCODER = 0x10,
    ARM1_REQ_PISTOL_LIMIT_SWITCH = 0x11,
    ARM1_REQ_MASS_LIMIT_SWITCH = 0x12,
    //--------REGISTER LEAK-SENSOR--------//
    ARM1_LEAK_POSITION = 0xFF,
    //--------REGISTER STRAIN-GAUGE--------//
    ARM1_STATUS_HULL = 0xFF,
    //--------BMS STATUS--------//
    ARM1_HOURS = 0x01,
    ARM1_MINUTES = 0x02,
    ARM1_SECONDS = 0x03,
    ARM1_BATTERY_TOTAL = 0x04,
    ARM1_BATTERY_CAPACITY = 0x05,
    ARM1_BATTERY_USED = 0x06,
    ARM1_BATTERY_PERCENTAGE = 0x07,
    ARM1_BATTERY_CURRENT = 0x08,
    ARM1_BATTERY_VOLTAGE = 0x09,
    //--------ALTIMETER STATUS--------//
    ARM1_ALTIMETER_IN_FEET = 0x01,
    ARM1_ALTIMETER_IN_METRES = 0x02,
    ARM1_ALTIMETER_IN_FATHOMS = 0x03,
    //--------POWER INT--------//
    ARM1_INT_24V40AH = 0xFE,
    ARM1_INT_24V10AH = 0xFF,
}ARM1_REGISTER_CANBUS_TypeDef;

typedef enum
{
    //--------ID BOARD ARM 2--------//
    ARM2_LEAK_SENSOR = 0x01,
    ARM2_STRAIN_GAUGE = 0x02,
    ARM2_BMS24V40AH = 0x03,
    ARM2_RUDDER = 0x04,
    ARM2_POWER_INT = 0x05,
    ARM2_PRESSURE = 0x06,
    ARM2_ALL_DATA = 0xFF,
}ARM2_CANBUS_TypeDef;

typedef enum
{
    //--------REGISTER LEAK-SENSOR--------//
    ARM2_LEAK_POSITION = 0xFF,
        //--------REGISTER STRAIN-GAUGE--------//
    ARM2_STATUS_HULL = 0xFF,
    //--------BMS STATUS--------//
    ARM2_HOURS = 0x01,
    ARM2_MINUTES = 0x02,
    ARM2_SECONDS = 0x03,
    ARM2_BATTERY_TOTAL = 0x04,
    ARM2_BATTERY_CAPACITY = 0x05,
    ARM2_BATTERY_USED = 0x06,
    ARM2_BATTERY_PERCENTAGE = 0x07,
    ARM2_BATTERY_CURRENT = 0x08,
    ARM2_BATTERY_VOLTAGE = 0x09,
    //--------REGISTER RUDDER--------//
    ARM2_MX28_GOAL_POSITION = 0x00,
    ARM2_MX28_MOVING_SPEED = 0x01,
    ARM2_MX28_KP = 0x02,
    ARM2_MX28_KI = 0x03,
    ARM2_MX28_KD = 0x04,
    ARM2_MX28_BAUDRATE = 0x05,
    ARM2_MX28_PRESENT_POSITION = 0x06,
    ARM2_MX28_PRESENT_SPEED = 0x07,
    ARM2_MX28_PRESENT_LOAD = 0x08,
    ARM2_MX28_PRESENT_VOL = 0x09,
    ARM2_MX28_PRESENT_TEMP = 0x0A,
    //--------REGISTER PRESSURE--------//
    ARM2_DEPTH_DATA = 0xFE,
    ARM2_TEMP_DATA = 0xFF,
    //--------POWER INT--------//
    ARM2_INT_24V40AH = 0xFE,
    ARM2_INT_24V10AH = 0xFF,
}ARM2_REGISTER_CANBUS_TypeDef;

typedef enum
{
    HEADING_PID = 0x01,
    HEADING_SM = 0x02,
    DEPTH_PID = 0x03,
    DEPTH_SM = 0x04,
    PITCH_PID = 0x05,
    PITCH_SM = 0x06,
    LOS_PID = 0x07,
    LOS_SM = 0x08,
    SPEED_PID = 0x09,
    SPEED_SM = 0x0A,
    ROLL_PID = 0x0B,
    ROLL_SM = 0x0C,
}AUV_CONTROLLER_TypeDef;

/**
 * @brief Test class for checking GCSTransciever's functionalities.
 */
class My_AUV2000_Actuator
{
public:
  My_AUV2000_Actuator();
  ~My_AUV2000_Actuator();

private:

  ros::Publisher pubMassShifter;
  ros::Publisher pubThruster;
  ros::Publisher pubPistol;
  ros::Publisher pubBoard_ARM1;
  ros::Publisher pubBoard_ARM2;

  ros::Timer loopPublishData;
  
  ros::WallTime start_time;
  double last_time;

  void onLoopPublishDataCallBack(const ros::TimerEvent& event);

  ros::Subscriber subItemList;
  void onItemListCallBack(const WaypointList::ConstPtr& msg);

  // MAV_CMD
  ros::ServiceServer resSetArming;
  ros::ServiceServer resStartMission;
  ros::ServiceServer resSetMode;
  ros::ServiceServer resSetAUVController;

  bool onSetArmingCallBack(CommandLongRequest& req, CommandLongResponse& res);
  bool onStartMissionCallBack(CommandLongRequest& req, CommandLongResponse& res);
  bool onSetModeCallBack(SetModeRequest& req, SetModeResponse& res);
  bool onSetAUVControllerCallBack(CommandLongRequest& req, CommandLongResponse& res);

  // MAV_PARAMETER
  ros::ServiceServer resGetJoystickFunctionParam;
  ros::ServiceServer resGetJoystickConfigParamReal32;
  ros::ServiceServer resGetJoystickConfigParamInt16;
  ros::ServiceServer resGetJoystickConfigParamInt8;

  ros::ServiceServer reqSetRoll_PID;
  ros::ServiceServer reqGetRoll_PID;
  ros::ServiceServer reqSetSpeed_PID;
  ros::ServiceServer reqGetSpeed_PID;
  ros::ServiceServer reqSetHeading_PID;
  ros::ServiceServer reqGetHeading_PID;
  ros::ServiceServer reqSetPitch_PID;
  ros::ServiceServer reqGetPitch_PID;
  ros::ServiceServer reqSetDepth_PID;
  ros::ServiceServer reqGetDepth_PID;
  ros::ServiceServer reqSetLOS_PID;
  ros::ServiceServer reqGetLOS_PID;

  ros::ServiceServer reqSetRoll_SM;
  ros::ServiceServer reqGetRoll_SM;
  ros::ServiceServer reqSetSpeed_SM;
  ros::ServiceServer reqGetSpeed_SM;
  ros::ServiceServer reqSetHeading_SM;
  ros::ServiceServer reqGetHeading_SM;
  ros::ServiceServer reqSetPitch_SM;
  ros::ServiceServer reqGetPitch_SM;
  ros::ServiceServer reqSetDepth_SM;
  ros::ServiceServer reqGetDepth_SM;
  ros::ServiceServer reqSetLOS_SM;
  ros::ServiceServer reqGetLOS_SM;
  ros::ServiceServer reqSetLOS_Config;
  ros::ServiceServer reqGetLOS_Config;

  ros::ServiceServer resGetComPassParam;
  ros::ServiceServer resGetBatteryParam;
  ros::ServiceServer resGetFailSafeParam;
  ros::ServiceServer resGetARMINGParam;

  bool onSetRoll_PIDCallBack(ParamSetRequest&  req, ParamSetResponse& res);
  bool onGetRoll_PIDCallBack(ParamGetRequest&  req, ParamGetResponse& res);
  bool onSetSpeed_PIDCallBack(ParamSetRequest&  req, ParamSetResponse& res);
  bool onGetSpeed_PIDCallBack(ParamGetRequest&  req, ParamGetResponse& res);
  bool onSetHeading_PIDCallBack(ParamSetRequest&  req, ParamSetResponse& res);
  bool onGetHeading_PIDCallBack(ParamGetRequest&  req, ParamGetResponse& res);
  bool onSetPitch_PIDCallBack(ParamSetRequest&  req, ParamSetResponse& res);
  bool onGetPitch_PIDCallBack(ParamGetRequest&  req, ParamGetResponse& res);
  bool onSetDepth_PIDCallBack(ParamSetRequest&  req, ParamSetResponse& res);
  bool onGetDepth_PIDCallBack(ParamGetRequest&  req, ParamGetResponse& res);
  bool onSetLOS_PIDCallBack(ParamSetRequest&  req, ParamSetResponse& res);
  bool onGetLOS_PIDCallBack(ParamGetRequest&  req, ParamGetResponse& res);

  bool onSetRoll_SMCallBack(ParamSetRequest&  req, ParamSetResponse& res);
  bool onGetRoll_SMCallBack(ParamGetRequest&  req, ParamGetResponse& res);
  bool onSetSpeed_SMCallBack(ParamSetRequest&  req, ParamSetResponse& res);
  bool onGetSpeed_SMCallBack(ParamGetRequest&  req, ParamGetResponse& res);
  bool onSetHeading_SMCallBack(ParamSetRequest&  req, ParamSetResponse& res);
  bool onGetHeading_SMCallBack(ParamGetRequest&  req, ParamGetResponse& res);
  bool onSetPitch_SMCallBack(ParamSetRequest&  req, ParamSetResponse& res);
  bool onGetPitch_SMCallBack(ParamGetRequest&  req, ParamGetResponse& res);
  bool onSetDepth_SMCallBack(ParamSetRequest&  req, ParamSetResponse& res);
  bool onGetDepth_SMCallBack(ParamGetRequest&  req, ParamGetResponse& res);
  bool onSetLOS_SMCallBack(ParamSetRequest&  req, ParamSetResponse& res);
  bool onGetLOS_SMCallBack(ParamGetRequest&  req, ParamGetResponse& res);

  bool onSetLOS_ConfigCallBack(ParamSetRequest&  req, ParamSetResponse& res);
  bool onGetLOS_ConfigCallBack(ParamGetRequest&  req, ParamGetResponse& res);

  bool OnGetComPassCallBack(ParamGetRequest& req, ParamGetResponse& res);
  bool OnGetBatteryCallBack(ParamGetRequest& req,ParamGetResponse& res);
  bool OnGetFailSafeCallBack(ParamGetRequest& req,ParamGetResponse& res);

  bool OnGetJoystickFunctionParamCallBack(ParamGetRequest& req, ParamGetResponse& res);
  bool OnGetJoystickConfigParamReal32CallBack(ParamGetRequest& req, ParamGetResponse& res);
  bool OnGetJoystickConfigParamInt16CallBack(ParamGetRequest& req, ParamGetResponse& res);
  bool OnGetJoystickConfigParamInt8CallBack(ParamGetRequest& req, ParamGetResponse& res);

  bool OnGetARMINGCallBack(ParamGetRequest& req,ParamGetResponse& res);

  /**
   * @brief ROS Subscriber to Joystick's Data.
   */
  ros::Subscriber subJoystick;
  /**
   * @brief Callback for My_AUV2000's Data subscription.
   * @param[in] msg My_AUV2000's message.
   */
  void onGetJoystickCallBack(const joystick_stamped::ConstPtr& msg);

  inline bool compareString(const char* str1, const char* str2) { return !strncmp(str1, str2, strlen(str2)); }

//  ros::WallTimer timerControlPistol, timerControlMass;
//  void onTimerControlPistolCallBack(const ros::WallTimerEvent& event);
//  void onTimerControlMassCallBack(const ros::WallTimerEvent& event);

};

#endif // MY_AUV2000_ACTUATORS_H