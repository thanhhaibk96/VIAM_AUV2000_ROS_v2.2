#ifndef AUV_CONTROLLER_H
#define AUV_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
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

#include <utils/Setpoint.h>
#include <utils/ThrustCommand.h>
#include <utils/gps_dvl_ins_stamped.h>
#include <utils/motor_stamped.h>
#include <utils/anti_rolling_stamped.h>

#include "utils/mx28_stamped.h"

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
#include "math.h"
#include "heading_smc.h"
#include "pid.h"
#include "geo.h"
#include "straight_los.h"


using namespace std;
using namespace nav_msgs;
using namespace sensor_msgs;
using namespace std_msgs_stamped;
using namespace mavros_msgs;
using namespace auv_msgs_stamped;

class AUV_Controller
{
public:
  AUV_Controller();
  ~AUV_Controller();

  double controlling_period;
  bool SMC_enabled;

  ros::Publisher pubThrustCmd;
  ros::Publisher pubRudderCmd;
  ros::Publisher pubPistonCmd;
  ros::Publisher pubIRMCmd;
  ros::Publisher pubMassShifterCmd;

  ros::Subscriber subSensor;
  ros::Subscriber subSetpoint;
  ros::Subscriber subItemList;
    ros::Subscriber subSonarScan;

  ros::Timer loopControl;

  // MAV_CMD
  ros::ServiceServer resSetArming;
  ros::ServiceServer resStartMission;
  ros::ServiceServer resSetMode;
  ros::ServiceServer resSetAUVController;


  ros::ServiceServer resSetSpeedPID;
  ros::ServiceServer resGetSpeedPID;
  ros::ServiceServer resSetPhiPID;
  ros::ServiceServer resGetPhiPID;
  ros::ServiceServer resSetPitchPID;
  ros::ServiceServer resGetPitchPID;
  ros::ServiceServer resSetHeadingPID;
  ros::ServiceServer resGetHeadingPID;

  PID speedPID;
  PID rollPID;
  PID pitchPID; 
  PID headingPID;
  PID LOS_DepthPID;
  PID LOS_PID_Heading;

  StraightLOS straightLOSGuider;

  double LOS_delta_min;
  double LOS_delta_max;
  double LOS_radius ;


  int OA_FLAG = 0; 
  float psi_current_0 = 0;
  int cnt =0 ;
  motor_stamped thrust_msg;
  anti_rolling_stamped irm_msg;
  motor_stamped piston_msg;
  motor_stamped mass_shifter_msg;
  mx28_stamped rudder_msg;
  joystick_stamped joystick_msg;

  bool isMissionStarted; 
  double ned_lat ;
  double ned_lon ; 

  double currSpeed;
  double currHeading;
  double currRoll;
  double currPitch;
  double currDepth;
  double currX;
  double currY;

  double psi_oa_tot = 0 ;

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

  float massShifterPositionFromJoyStick;
  float pistonPositionFromJoyStick = 0.04;
  int thrustSpeedFromJoyStick;
  int rudderPositionFromJoyStick;


  ros::Time lastSetpointTime;
  ros::Time lastControlUpdateTime;


typedef enum
{
    HEADING_PID = 0x01,
    HEADING_SM = 0x02,
    DEPTH_PID = 0x05,
    DEPTH_SM = 0x06,
    PITCH_PID = 0x03,
    PITCH_SM = 0x04,
    LOS_PID = 0x07,
    LOS_SM = 0x08,
    SPEED_PID = 0x09,
    SPEED_SM = 0x0A

}AUV_CONTROLLER_TypeDef;

//////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////

  void onItemListCallBack(const WaypointList::ConstPtr& msg);


  bool onSetArmingCallBack(CommandLongRequest& req, CommandLongResponse& res);
  bool onStartMissionCallBack(CommandLongRequest& req, CommandLongResponse& res);
  bool onSetModeCallBack(SetModeRequest& req, SetModeResponse& res);
  bool onSetAUVControllerCallBack(CommandLongRequest& req, CommandLongResponse& res);

  void onSensorCallBack(const gps_dvl_ins_stamped::ConstPtr& msg);
  void onSonarScanCallBack(const LaserScan::ConstPtr& msg);

  void onControlLoop(const ros::TimerEvent& event);


  float convertForcetoSpeed(float Force);
  // MAV_PARAMETER
  ros::ServiceServer resGetJoystickFunctionParam;
  ros::ServiceServer resGetJoystickConfigParamReal32;
  ros::ServiceServer resGetJoystickConfigParamInt16;
  ros::ServiceServer resGetJoystickConfigParamInt8;


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
  bool OnGetARMINGCallBack(ParamGetRequest &req, ParamGetResponse &res);

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
};

#endif // AUV_CONTROLLER_H
