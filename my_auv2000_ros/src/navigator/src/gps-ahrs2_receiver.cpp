#include "gps-ahrs2_receiver.h"

GPSAHRS2ReceiverNode::GPSAHRS2ReceiverNode()
{
  ros::NodeHandle private_nh("~");
  private_nh.getParam("gps_enabled", gps_enabled);
  private_nh.getParam("ahrs_enabled", ahrs_enabled);
  private_nh.getParam("gps_port", gps_port);
  private_nh.getParam("ahrs_port", ahrs_port);
  private_nh.getParam("gps_baudrate", gps_baudrate);
  private_nh.getParam("ahrs_baudrate", ahrs_baudrate);

  ros::NodeHandle nh;
  nh.getParam("ned_lat", ned_lat);
  nh.getParam("ned_lon", ned_lon);

  pubOdom = nh.advertise<Odometry>("odom", 10);
}

GPSAHRS2ReceiverNode::~GPSAHRS2ReceiverNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}
double GPSAHRS2Receiver::CalLat2Deg(double Lat){
	static uint8_t Deg=0;
	static double Min=0,  Result=0;
	Deg= Lat/100;
	Min=Lat-Deg*100;
	Result=Deg+Min/60;
  return  Result; 
}
double GPSAHRS2Receiver::CalLong2Deg(double Long){
	static uint16_t Deg=0;
	static double Min=0,  Result=0;
	Deg= Long/100;
	Min=Long-Deg*100;
	Result=Deg+Min/60;
	return  Result; 
}
void GPSAHRS2ReceiverNode::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

GPSAHRS2Receiver::GPSAHRS2Receiver()
{
  if (node.gps_enabled)
  {
    gpsDevice.setPortName(QString::fromStdString(node.gps_port));
    gpsDevice.setBaudRate(node.gps_baudrate);
    gpsDevice.setParity(QSerialPort::NoParity);
    gpsDevice.open(QIODevice::ReadOnly);
    connect(&gpsDevice, &QSerialPort::readyRead, this, &GPSAHRS2Receiver::processGPSFrame);
    if (gpsDevice.isOpen())
      ROS_INFO("GPS Serial port opened succesfull!");	
    else
      ROS_ERROR("Failed to open GPS serial port!!!"); 
  }
  if (node.ahrs_enabled)
  {
    ahrsDevice.setPortName(QString::fromStdString(node.ahrs_port));
    ahrsDevice.setBaudRate(node.ahrs_baudrate);
    ahrsDevice.setParity(QSerialPort::NoParity);
    ahrsDevice.open(QIODevice::ReadOnly);
    connect(&ahrsDevice, &QSerialPort::readyRead, this, &GPSAHRS2Receiver::processAHRSFrame);
    if (ahrsDevice.isOpen())
      ROS_INFO("AHRS Serial port opened succesfull!");	
    else
      ROS_ERROR("Failed to open AHRS serial port!!!"); 
  }
  node.start();
}

GPSAHRS2Receiver::~GPSAHRS2Receiver()
{
  if (gpsDevice.isOpen())
    gpsDevice.close();
  if (ahrsDevice.isOpen())
    ahrsDevice.close();

}

void GPSAHRS2Receiver::processGPSFrame()
{
  auto tmpBuffer = gpsDevice.readAll();
  vector<string> data_lines;
  boost::split(data_lines, tmpBuffer, boost::is_any_of("\r\n"));
  for (auto it : data_lines)
  {
    vector<string> data;
    boost::split(data, it, boost::is_any_of(","));
    if (data[0] == "$GNGGA")
    {
      node.odomMsg.latitude = CalLat2Deg(QString::fromStdString(data[2]).toDouble());
      node.odomMsg.longitude = CalLong2Deg(QString::fromStdString(data[4]).toDouble());
      node.odomMsg.altitude = QString::fromStdString(data[9]).toDouble();
      convert_global_to_local_coords(node.odomMsg.latitude, node.odomMsg.longitude, node.ned_lat, node.ned_lon,
                                     node.odomMsg.position.x, node.odomMsg.position.y);
      node.odomMsg.position.z = -node.odomMsg.altitude;
    }
  }

  if (!ahrsDevice.isOpen())
  {
    node.odomMsg.header.stamp = ros::Time::now();
    node.pubOdom.publish(node.odomMsg);
  }
}

void GPSAHRS2Receiver::processAHRSFrame()
{
  auto tmpBuffer = ahrsDevice.readAll();

  if (isAhrsFirst)
  {
    ROS_INFO("Waiting for correct header.");
    if (tmpBuffer[0] == '$')
      isAhrsFirst = false;
    else
      return;
  }

  auto tmpBufferSize = tmpBuffer.length();
  for (int i = ahrsBytesReceived; i < ahrsBytesReceived + tmpBufferSize; i++)
    ahrsBuffer[i] = tmpBuffer.data()[i - ahrsBytesReceived];
  ahrsBytesReceived += tmpBufferSize;
  if (ahrsBytesReceived < BUFFER_SIZE)
    return;
  ahrsBytesReceived = 0;

  char header[4] = {ahrsBuffer[0], ahrsBuffer[1], ahrsBuffer[2], ahrsBuffer[3]};
  if (strcmp(header, "$Nav"))
  {
    isAhrsFirst = true;
    return;
  }

  node.odomMsg.orientation.x = *reinterpret_cast<const float*>(ahrsBuffer + 8) * M_PI / 180.0;
  node.odomMsg.orientation.y = *reinterpret_cast<const float*>(ahrsBuffer + 12) * M_PI / 180.0;
  node.odomMsg.orientation.z = *reinterpret_cast<const float*>(ahrsBuffer + 16) * M_PI / 180.0;
  node.odomMsg.angular_velocity.x = *reinterpret_cast<const float*>(ahrsBuffer + 64);
  node.odomMsg.angular_velocity.y = *reinterpret_cast<const float*>(ahrsBuffer + 68);
  node.odomMsg.angular_velocity.z = *reinterpret_cast<const float*>(ahrsBuffer + 72);
  node.odomMsg.angular_acceleration.x = *reinterpret_cast<const float*>(ahrsBuffer + 76);
  node.odomMsg.angular_acceleration.y = *reinterpret_cast<const float*>(ahrsBuffer + 80);
  node.odomMsg.angular_acceleration.z = *reinterpret_cast<const float*>(ahrsBuffer + 84);
  node.odomMsg.linear_velocity.x = *reinterpret_cast<const float*>(ahrsBuffer + 52);
  node.odomMsg.linear_velocity.y = *reinterpret_cast<const float*>(ahrsBuffer + 56);
  node.odomMsg.linear_velocity.z = *reinterpret_cast<const float*>(ahrsBuffer + 60);

  node.odomMsg.header.stamp = ros::Time::now();
  node.pubOdom.publish(node.odomMsg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps-ahrs2_receiver");
  QCoreApplication a(argc, argv);
  GPSAHRS2Receiver receiver;
  return a.exec();
}
