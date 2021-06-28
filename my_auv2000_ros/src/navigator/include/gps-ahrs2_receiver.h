#ifndef GPSAHRS2_RECEIVER_H
#define GPSAHRS2_RECEIVER_H

#include <ros/ros.h>
#include <utils/Odometry.h>

#include <QObject>
#include <QThread>
#include <QSerialPort>
#include <QCoreApplication>
#include <QDebug>

#include <boost/algorithm/string.hpp>

#include "geo.h"

#define BUFFER_SIZE 96

using namespace utils;
using namespace std;

class GPSAHRS2ReceiverNode : public QThread
{
  Q_OBJECT

public:
  GPSAHRS2ReceiverNode();
  ~GPSAHRS2ReceiverNode();

  double ned_lat;
  double ned_lon;
  bool gps_enabled;
  bool ahrs_enabled;
  string gps_port;
  string ahrs_port;
  int gps_baudrate;
  int ahrs_baudrate;
  

  ros::Publisher pubOdom;

  Odometry odomMsg;

  void run();
};

class GPSAHRS2Receiver : public QObject
{
  Q_OBJECT

public:
  GPSAHRS2Receiver();
  ~GPSAHRS2Receiver();

  double CalLat2Deg(double Lat);
	double CalLong2Deg(double Long);
  
  GPSAHRS2ReceiverNode node;
  QSerialPort gpsDevice;
  QSerialPort ahrsDevice;

  char ahrsBuffer[BUFFER_SIZE];
  int ahrsBytesReceived = 0;
  bool isAhrsFirst = true;

public slots:
  void processGPSFrame();
  void processAHRSFrame();
};
#endif // GPSAHRS2_RECEIVER_H
