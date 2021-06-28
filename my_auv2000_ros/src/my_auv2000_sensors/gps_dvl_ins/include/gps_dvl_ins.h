#ifndef GPS_DVL_INS_H
#define GPS_DVL_INS_H
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Empty.h>
#include <stdbool.h>
#include <iostream>
#include <fstream>
#include <string>
#include "auv_msgs_stamped/gps_dvl_ins_stamped.h"

using namespace auv_msgs_stamped;

class GPS_DVL_INS
{
public:
  GPS_DVL_INS();
  std::string gps_dvl_ins_device;
  int gps_dvl_ins_baurate;
  bool calibration_mode;

private:

};

#endif // GPS_DVL_INS_H
