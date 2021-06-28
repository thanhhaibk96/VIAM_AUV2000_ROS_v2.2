#include "gps-ahrs_receiver.h"
GPSAHRSReceiverNode::GPSAHRSReceiverNode()
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

GPSAHRSReceiverNode::~GPSAHRSReceiverNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

double GPSAHRSReceiver::CalLat2Deg(double Lat){
	static uint8_t Deg=0;
	static double Min=0,  Result=0;
	Deg= Lat/100;
	Min=Lat-Deg*100;
	Result=Deg+Min/60;
  return  Result; 
}
double GPSAHRSReceiver::CalLong2Deg(double Long){
	static uint16_t Deg=0;
	static double Min=0,  Result=0;
	Deg= Long/100;
	Min=Long-Deg*100;
	Result=Deg+Min/60;
	return  Result; 
}

void GPSAHRSReceiverNode::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

GPSAHRSReceiver::GPSAHRSReceiver()
{
  if (node.gps_enabled)
  {
    gpsDevice.setPortName(QString::fromStdString(node.gps_port));
    gpsDevice.setBaudRate(node.gps_baudrate);
    gpsDevice.setParity(QSerialPort::NoParity);
    gpsDevice.open(QIODevice::ReadOnly);
    connect(&gpsDevice, &QSerialPort::readyRead, this, &GPSAHRSReceiver::processGPSFrame);
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
    connect(&ahrsDevice, &QSerialPort::readyRead, this, &GPSAHRSReceiver::processAHRSFrame);
    if(ahrsDevice.isOpen())
			ROS_INFO("AHRS Serial port opened succesfull!");			
		else
			ROS_ERROR("Failed to open AHRS serial port!!!");
  }
  node.start();
}

GPSAHRSReceiver::~GPSAHRSReceiver()
{
  if (gpsDevice.isOpen())
    gpsDevice.close();
  if (ahrsDevice.isOpen())
    ahrsDevice.close();

}

void GPSAHRSReceiver::processGPSFrame()
{	
  auto tmpBuffer = gpsDevice.readAll();
  //auto tmpBuffer = "$GNGGA,080659.50,1046.46434,N,10639.55359,E,1,06,1.65,-16.4,M,-2.6,M,,*46";
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

void GPSAHRSReceiver::processAHRSFrame()
{
  auto tmpBuffer = ahrsDevice.readAll();
  //auto tmpBuffer = $RQ,4403,4297,5173*;
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

  char header[4] = {ahrsBuffer[0], ahrsBuffer[1], ahrsBuffer[2]};
  if (strcmp(header, "$RQ"))
  {
    isAhrsFirst = true;
    return;
  }

  vector<string> data;
  boost::split(data, ahrsBuffer, boost::is_any_of(","));
  node.odomMsg.orientation.x = (stod(data[1].data()) - 4300) * 0.1 * M_PI / 180;
  node.odomMsg.orientation.y = (stod(data[2].data()) - 4300) * 0.1 * M_PI / 180;
  node.odomMsg.orientation.z = (-stod(data[3].data()) + 4300) * 0.1 * M_PI / 180;
  
  node.odomMsg.header.stamp = ros::Time::now();
  node.pubOdom.publish(node.odomMsg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps-ahrs_receiver");
  QCoreApplication a(argc, argv);
  GPSAHRSReceiver receiver;

  return a.exec();
}
