#include "gps-ins-dvl_receiver.h"

GPSINSDVLReceiverNode::GPSINSDVLReceiverNode()
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("enabled", enabled);
  private_nh.getParam("port", port);
  private_nh.getParam("baudrate", baudrate);

  ros::NodeHandle nh;
  nh.getParam("ned_lat", ned_lat);
  nh.getParam("ned_lon", ned_lon);

  pubOdom = nh.advertise<Odometry>("odom", 10);
}

GPSINSDVLReceiverNode::~GPSINSDVLReceiverNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void GPSINSDVLReceiverNode::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

GPSINSDVLReceiver::GPSINSDVLReceiver()
{ 
  if (node.enabled)
  {
    device.setPortName(QString::fromStdString(node.port));
    device.setBaudRate(node.baudrate);
    device.open(QIODevice::ReadOnly);
    connect(&device, &QSerialPort::readyRead, this, &GPSINSDVLReceiver::processFrame);
    if (device.isOpen())
      ROS_INFO("GPS-INS-DVL Serial port opened succesfull!");	
    else
      ROS_ERROR("Failed to open GPS-INS-DVL serial port!!!"); 
  }

  node.start();
}

GPSINSDVLReceiver::~GPSINSDVLReceiver()
{
  if (device.isOpen())
    device.close();
}

void GPSINSDVLReceiver::processFrame()
{
  auto tmpBuffer = device.readAll();

  if (isFirst)
  {
    ROS_INFO("Waiting for correct header.");
    if (tmpBuffer[0] == '$')
      isFirst = false;
    else
      return;
  }

  auto tmpBufferSize = tmpBuffer.length();
  for (int i = bytesReceived; i < bytesReceived + tmpBufferSize; i++)
    buffer[i] = tmpBuffer.data()[i - bytesReceived];
  bytesReceived += tmpBufferSize;
  if (bytesReceived < BUFFER_SIZE)
    return;
  bytesReceived = 0;

  char header[4] = {buffer[0], buffer[1], buffer[2], buffer[3]};
  if (strcmp(header, "$Nav"))
  {
    isFirst = true;
    return;
  }

  Odometry odomMsg;
  odomMsg.latitude = *reinterpret_cast<const double*>(buffer + 20);
  odomMsg.longitude = *reinterpret_cast<const double*>(buffer + 28);
  odomMsg.altitude = *reinterpret_cast<const float*>(buffer + 36);
  convert_global_to_local_coords(odomMsg.latitude, odomMsg.longitude, node.ned_lat, node.ned_lon, odomMsg.position.x,
                                 odomMsg.position.y);
  odomMsg.position.z = -odomMsg.altitude;

  odomMsg.orientation.x = *reinterpret_cast<const float*>(buffer + 12) * M_PI / 180.0;
  odomMsg.orientation.y = *reinterpret_cast<const float*>(buffer + 8) * M_PI / 180.0;
  float temp = *reinterpret_cast<const float*>(buffer +16)* M_PI / 180.0;
   temp += M_PI/2;
  while (temp < -M_PI) temp += 2*M_PI;
   while (temp >  M_PI) temp -= 2*M_PI;
  odomMsg.orientation.z = temp;
  //odomMsg.orientation.z = atan2f(sinf(temp),cosf(temp));

  odomMsg.angular_velocity.x = *reinterpret_cast<const float*>(buffer + 64);
  odomMsg.angular_velocity.y = *reinterpret_cast<const float*>(buffer + 68);
  odomMsg.angular_velocity.z = *reinterpret_cast<const float*>(buffer + 72);

  odomMsg.angular_acceleration.x = *reinterpret_cast<const float*>(buffer + 76);
  odomMsg.angular_acceleration.y = *reinterpret_cast<const float*>(buffer + 80);
  odomMsg.angular_acceleration.z = *reinterpret_cast<const float*>(buffer + 84);

  odomMsg.linear_velocity.x = *reinterpret_cast<const float*>(buffer + 52);
  odomMsg.linear_velocity.y = *reinterpret_cast<const float*>(buffer + 56);
  odomMsg.linear_velocity.z = *reinterpret_cast<const float*>(buffer + 60);

  odomMsg.header.stamp = ros::Time::now();
  node.pubOdom.publish(odomMsg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps-ins-dvl_receiver");
  QCoreApplication a(argc, argv);
  GPSINSDVLReceiver receiver;
  return a.exec();
}
