#ifndef GPSINSDVL_RECEIVER_H
#define GPSINSDVL_RECEIVER_H

#include <ros/ros.h>
#include <utils/Odometry.h>

#include <QObject>
#include <QThread>
#include <QSerialPort>
#include <QCoreApplication>
#include <QtDebug>

#include "geo.h"

#define BUFFER_SIZE 96

using namespace utils;
using namespace std;

class GPSINSDVLReceiverNode : public QThread
{
  Q_OBJECT

public:
  GPSINSDVLReceiverNode();
  ~GPSINSDVLReceiverNode();

  double ned_lat;
  double ned_lon;
  bool enabled;
  string port;
  int baudrate;

  ros::Publisher pubOdom;

  void run();
};

class GPSINSDVLReceiver : public QObject
{
  Q_OBJECT

public:
  GPSINSDVLReceiver();
  ~GPSINSDVLReceiver();

  GPSINSDVLReceiverNode node;
  QSerialPort device;

  char buffer[BUFFER_SIZE];
  int bytesReceived = 0;
  bool isFirst = true;

public slots:
  void processFrame();
};

#endif // GPSINSDVL_RECEIVER_H
