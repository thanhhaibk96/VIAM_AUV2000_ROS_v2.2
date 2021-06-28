#include "gcs_transceiver.h"
#include "mission_protocol.h"

GCSTransceiver::GCSTransceiver()
{
  /// Get local parameters of this node through roslaunch file.
  ros::NodeHandle private_nh("~");
  private_nh.getParam("main_period", main_period);
  private_nh.getParam("main_url", main_url);

  /// Temporary holders for autopilot_sysid and gcs_sysid as this method cannot parse uint8_t type.
  int autopilot_sysid_, gcs_sysid_;
  private_nh.getParam("autopilot_sysid", autopilot_sysid_);
  private_nh.getParam("gcs_sysid", gcs_sysid_);
  autopilot_sysid = static_cast<uint8_t>(autopilot_sysid_);
  gcs_sysid = static_cast<uint8_t>(gcs_sysid_);

  /// Autopilot's compid must be 1 (MAV_COMP_ID_AUTOPILOT1) in order for QGroundControl to recognize.
  autopilot_compid = static_cast<uint8_t>(MAV_COMPONENT::COMP_ID_AUTOPILOT1);

  /// GCS's compid is always 0 (MAV_COMP_ID_ALL) in QGroundControl, which means that its message must be processed by
  /// all autopilots in the network.
  gcs_compid = static_cast<uint8_t>(MAV_COMPONENT::COMP_ID_ALL);

  /// Construct a new MAVLink link management from main_url.
  MainLink = MAVConnInterface::open_url(main_url);

  /// Set the sender's sysid and compid of the link.
  MainLink->set_system_id(autopilot_sysid);
  MainLink->set_component_id(autopilot_compid);

  /// Assign a callback function to trigger when a new MAVLink message arrives.
  MainLink->message_received_cb = boost::bind(&GCSTransceiver::onMainLinkCallBack, this, _1, _2);

  /// Use main link to handle all microservices.
  missionProtocol = new MissionProtocol(this, MainLink);
  commandProtocol = new CommandProtocol(this, MainLink);
  parameterProtocol = new ParameterProtocol(this, MainLink);
  joystickProtocol = new Joystick_Protocol(this, MainLink);

  /// Initialize values for Hearbeat and SysStatus.
  setupHeartbeat();
  setupSysStatus();

  ros::NodeHandle nh;

  /// ROS subscription and publication from and to ROS messages from other nodes.
  subMass_Shifter = nh.subscribe("mass_shifter/position", 10, &GCSTransceiver::onGetMass_ShifterCallBack, this);
  subPistol = nh.subscribe("piston/position", 10, &GCSTransceiver::onGetPistolCallBack, this);
  subThruster = nh.subscribe("thrust/speed", 10, &GCSTransceiver::onGetThrusterCallBack, this);
  subBoardARM1 = nh.subscribe("my_auv2000_actuators/arm1", 10, &GCSTransceiver::onGetBoardARM1CallBack, this);
  subBoardARM2 = nh.subscribe("my_auv2000_actuators/arm2", 10, &GCSTransceiver::onGetBoardARM2CallBack, this);

  /// ROS subscription and publication from and to ROS messages from other nodes.
  subGPS_DVL_INS = nh.subscribe("my_auv2000_sensors/gps_dvl_ins_data", 10, &GCSTransceiver::onGPS_DVL_INSCallBack, this);
//  subGPS = nh.subscribe("gps/fix", 10, &GCSTransceiver::onGPSCallBack, this);
//  subIMU = nh.subscribe("imu/data", 10, &GCSTransceiver::onIMUCallBack, this);
//  subOdom = nh.subscribe("odom", 10, &GCSTransceiver::onOdomCallBack, this);

  subSetpoint = nh.subscribe("setpoint", 10, &GCSTransceiver::onSetpointCallBack, this);
  subThrusterCmd = nh.subscribe("thruster/cmd", 10, &GCSTransceiver::onThrusterCmdCallBack, this);

  /// ROS timer for periodic publication of Hearbeat and SysStatus.
  loopMainLink = nh.createTimer(ros::Duration(main_period), &GCSTransceiver::onMainLinkLoop, this);

}

GCSTransceiver::~GCSTransceiver()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void GCSTransceiver::setupHeartbeat()
{
  /// Specify which system (or vehicle) the autopilot resides in.
  Heartbeat.type = static_cast<uint8_t>(MAV_TYPE::SUBMARINE);

  /// Specify the type of autopilot (must always be ARDUPILOTMEGA for now).
  Heartbeat.autopilot = static_cast<uint8_t>(MAV_AUTOPILOT::ARDUPILOTMEGA);

  /// Base mode is only useful if we use GENERIC autopilot. For ARDUPILOTMEGA, always set this to CUSTOM_MODE_ENABLED.
  Heartbeat.base_mode = static_cast<uint8_t>(MAV_MODE_FLAG::CUSTOM_MODE_ENABLED);

  /// This is the one that we can choose in GCS if ARDUPILOTMEGA is used.
  /// See /opt/ros/melodic/include/mavlink/v2.0/ardupilotmega/ardupilotmega.hpp for list of supported custom modes.
  Heartbeat.custom_mode = static_cast<uint8_t>(mavlink::ardupilotmega::ROVER_MODE::INITIALIZING);

  /// Initial system status.
  Heartbeat.system_status = static_cast<uint8_t>(MAV_STATE::STANDBY);
}

void GCSTransceiver::setupSysStatus()
{
  SysStatus.onboard_control_sensors_present = static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::GPS) |
                                              static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_GYRO) |
                                              static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_ACCEL);
  SysStatus.onboard_control_sensors_enabled = static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::GPS) |
                                              static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_GYRO) |
                                              static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_ACCEL);
  SysStatus.onboard_control_sensors_health = static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::GPS) |
                                             static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_GYRO) |
                                             static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_ACCEL);
}

void GCSTransceiver::onMainLinkLoop(const ros::TimerEvent& /*event*/)
{
  pack_and_send_mavlink_message_t(Heartbeat, MainLink);
  pack_and_send_mavlink_message_t(SysStatus, MainLink);
}

void GCSTransceiver::onMainLinkCallBack(const mavlink_message_t* msg, const Framing framing)
{
  /// Only procceed if message is error-free.
//  if(int(msg->msgid) == 69)
//  {
//    joystickProtocol->handleJoystick(msg);
//    ROS_INFO_STREAM(msg->checksum);
//    ROS_INFO_STREAM(int(msg->len));
//    ROS_INFO_STREAM(msg->payload64[0]);
//    ROS_INFO_STREAM(int(framing));
//    ROS_INFO_STREAM("-------------------------------");
//  }

  // Excepting for MANUAL_CONTROl
  if((framing == mavconn::Framing::bad_crc) && (int(msg->msgid) == 69))
  {
    if (msg->sysid == gcs_sysid && msg->compid == gcs_compid)
    {
      /// Direct message to appropriate microservice manager based on its ID.
      switch (msg->msgid)
      {
        case 69:
          joystickProtocol->handleJoystick(msg);
          break;
        default:
          break;
      }
    }
  }

  if (framing == mavconn::Framing::ok)
  {
    ROS_INFO_STREAM("msgid = " << int(msg->msgid));

    /// In multivehicle scenario, multiple vehicles with their own autopilots can connect any exchange MAVLink messages
    /// with one another. Handling of such vehicle-related data is not the goal of this class, so non-GCS messages are
    /// filtered out.
    if (msg->sysid == gcs_sysid && msg->compid == gcs_compid)
    {
      /// Direct message to appropriate microservice manager based on its ID.
      switch (msg->msgid)
      {
      case 44:
      case 73:
      case 43:
      case 51:
      case 41:
      case 45:
        missionProtocol->handleMission(msg);
        break;
      case 76:
      case 75:
      case 11:
        commandProtocol->handleCommand(msg);
        break;
      case 21:
      case 20:
      case 23:
        parameterProtocol->handleParameter(msg);
        break;
      case 69:
        joystickProtocol->handleJoystick(msg);
        break;
      default:
        break;
      }
    }
  }
}

void GCSTransceiver::onGPS_DVL_INSCallBack(const gps_dvl_ins_stamped::ConstPtr &msg)
{
//  GPS_RAW_INT pack_GPS_RAW;
//  pack_GPS_RAW.time_usec = static_cast<uint64_t>(msg->header.stamp.toNSec() * 1e3);
//  pack_GPS_RAW.lat = static_cast<int32_t>(msg->ekf_lat * 1e7);
//  pack_GPS_RAW.lon = static_cast<int32_t>(msg->ekf_lon * 1e7);
//  pack_GPS_RAW.alt = static_cast<int32_t>(msg->ekf_alt * 1e3);

//  pack_and_send_mavlink_message_t(pack_GPS_RAW, MainLink);

  GLOBAL_POSITION_INT pack_GLOBAL_POSITION_INT;
  pack_GLOBAL_POSITION_INT.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e6);
  pack_GLOBAL_POSITION_INT.lat = static_cast<int32_t>(msg->ekf_lat * 1e7);
  pack_GLOBAL_POSITION_INT.lon = static_cast<int32_t>(msg->ekf_lon * 1e7);
  pack_GLOBAL_POSITION_INT.alt = static_cast<int32_t>(msg->ekf_alt * 1e3);
  pack_GLOBAL_POSITION_INT.vx = static_cast<int16_t>(msg->ekf_vX * 1e2);
  pack_GLOBAL_POSITION_INT.vy = static_cast<int16_t>(msg->ekf_vY * 1e2);
  pack_GLOBAL_POSITION_INT.vz = static_cast<int16_t>(msg->ekf_vZ * 1e2);

 pack_and_send_mavlink_message_t(pack_GLOBAL_POSITION_INT, MainLink);

  RAW_IMU pack_RAW_IMU;
  pack_RAW_IMU.time_usec = static_cast<uint64_t>(msg->header.stamp.toNSec() * 1e3);
  pack_RAW_IMU.xacc = static_cast<int16_t>(msg->imu_acc_X);
  pack_RAW_IMU.yacc = static_cast<int16_t>(msg->imu_acc_Y);
  pack_RAW_IMU.zacc = static_cast<int16_t>(msg->imu_acc_Z);
  pack_RAW_IMU.xgyro = static_cast<int16_t>(msg->imu_deg_gyro_X);
  pack_RAW_IMU.ygyro = static_cast<int16_t>(msg->imu_deg_gyro_Y);
  pack_RAW_IMU.zgyro = static_cast<int16_t>(msg->imu_deg_gyro_Z);
  pack_RAW_IMU.xmag = static_cast<int16_t>(msg->imu_mag_X);
  pack_RAW_IMU.ymag = static_cast<int16_t>(msg->imu_mag_Y);
  pack_RAW_IMU.zmag = static_cast<int16_t>(msg->imu_mag_Z);

  pack_and_send_mavlink_message_t(pack_RAW_IMU, MainLink);

  ATTITUDE pack_ATTITUDE;
  pack_ATTITUDE.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e6);
  pack_ATTITUDE.roll = static_cast<float>(msg->ekf_roll*static_cast<float>(M_PI)/180.0f);
  pack_ATTITUDE.pitch = static_cast<float>(msg->ekf_pitch*static_cast<float>(M_PI)/180.0f);
  pack_ATTITUDE.yaw = static_cast<float>(msg->ekf_yaw*static_cast<float>(M_PI)/180.0f);
  pack_ATTITUDE.rollspeed = static_cast<float>(msg->rad_gyro_X);
  pack_ATTITUDE.pitchspeed = static_cast<float>(msg->rad_gyro_Y);
  pack_ATTITUDE.yawspeed = static_cast<float>(msg->rad_gyro_Z);

  pack_and_send_mavlink_message_t(pack_ATTITUDE, MainLink);

  VFR_HUD pack_VFR_HUB;
  pack_VFR_HUB.groundspeed = static_cast<float>(msg->ekf_vX);
  pack_and_send_mavlink_message_t(pack_VFR_HUB, MainLink);
  
}

//void GCSTransceiver::onGPSCallBack(const NavSatFix::ConstPtr& msg)
//{
//  GPS_RAW_INT pack;
//  pack.time_usec = static_cast<uint64_t>(msg->header.stamp.toNSec() * 1e3);
//  pack.lat = static_cast<int32_t>(msg->latitude * 1e7);
//  pack.lon = static_cast<int32_t>(msg->longitude * 1e7);
//  pack.alt = static_cast<int32_t>(msg->altitude * 1e3);

//  pack_and_send_mavlink_message_t(pack, MainLink);
//}

//void GCSTransceiver::onIMUCallBack(const Imu::ConstPtr& msg)
//{
//  RAW_IMU pack;
//  pack.time_usec = static_cast<uint64_t>(msg->header.stamp.toNSec() * 1e3);
//  pack.xacc = static_cast<int16_t>(msg->linear_acceleration.x);
//  pack.yacc = static_cast<int16_t>(msg->linear_acceleration.y);
//  pack.zacc = static_cast<int16_t>(msg->linear_acceleration.z);
//  pack.xgyro = static_cast<int16_t>(msg->angular_velocity.x);
//  pack.ygyro = static_cast<int16_t>(msg->angular_velocity.y);
//  pack.zgyro = static_cast<int16_t>(msg->angular_velocity.z);

//  pack_and_send_mavlink_message_t(pack, MainLink);
//}

//void GCSTransceiver::onOdomCallBack(const Odometry::ConstPtr& msg)
//{
//  GLOBAL_POSITION_INT pack1;
//  pack1.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e6);
//  double lat, lon;
//  convert_local_to_global_coords(msg->pose.pose.position.x, msg->pose.pose.position.y, ned_lat, ned_lon, lat, lon);
//  pack1.lat = static_cast<int32_t>(lat * 1e7);
//  pack1.lon = static_cast<int32_t>(lon * 1e7);
//  pack1.alt = static_cast<int32_t>(-msg->pose.pose.position.z * 1e3);
//  pack1.vx = static_cast<int16_t>(msg->twist.twist.linear.x * 1e2);
//  pack1.vy = static_cast<int16_t>(msg->twist.twist.linear.y * 1e2);
//  pack1.vz = static_cast<int16_t>(msg->twist.twist.linear.z * 1e2);

//  float quarternion[4];
//  quarternion[0] = static_cast<float>(msg->pose.pose.orientation.w);
//  quarternion[1] = static_cast<float>(msg->pose.pose.orientation.x);
//  quarternion[2] = static_cast<float>(msg->pose.pose.orientation.y);
//  quarternion[3] = static_cast<float>(msg->pose.pose.orientation.z);
//  float euler[3];
//  mavlink_quaternion_to_euler(quarternion, euler, euler + 1, euler + 2);
//  ATTITUDE pack2;
//  pack2.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e6);
//  pack2.roll = euler[0];
//  pack2.pitch = euler[1];
//  pack2.yaw = euler[2];
//  pack2.rollspeed = static_cast<float>(msg->twist.twist.angular.x);
//  pack2.pitchspeed = static_cast<float>(msg->twist.twist.angular.y);
//  pack2.yawspeed = static_cast<float>(msg->twist.twist.angular.z);

//  pack_and_send_mavlink_message_t(pack1, MainLink);
//  pack_and_send_mavlink_message_t(pack2, MainLink);
//}

void GCSTransceiver::onGetMass_ShifterCallBack(const motor_stamped::ConstPtr &msg)
{
  CUSTOM_MSG_MASS_SHIFTER pack;
//  pack.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e-6);

  pack.position = msg->position;
  pack.motor_duty = msg->motor_duty;
  pack.motor_dspeed = msg->motor_dspeed;
  pack.motor_rspeed = msg->motor_rspeed;
  pack.motor_current = msg->motor_current;
  pack.motor_temp_ambient = msg->motor_temp_ambient;
  pack.motor_temp_on_chip = msg->motor_temp_on_chip;
  pack.position = msg ->position;

  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onGetPistolCallBack(const motor_stamped::ConstPtr &msg)
{
  CUSTOM_MSG_PISTOL pack;
//  pack.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e-6);

  pack.position = msg->position;
  pack.motor_duty = msg->motor_duty;
  pack.motor_dspeed = msg->motor_dspeed;
  pack.motor_rspeed = msg->motor_rspeed;
  pack.motor_current = msg->motor_current;
  pack.motor_temp_ambient = msg->motor_temp_ambient;
  pack.motor_temp_on_chip = msg->motor_temp_on_chip;
  pack.position = msg ->position;

  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onGetThrusterCallBack(const motor_stamped::ConstPtr &msg)
{
  CUSTOM_MSG_THRUSTER pack;
//  pack.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e-6);

  pack.motor_duty = msg->motor_duty;
  pack.motor_dspeed = msg->motor_dspeed;
  pack.motor_rspeed = msg->motor_rspeed;
  pack.motor_current = msg->motor_current;
  pack.motor_temp_ambient = msg->motor_temp_ambient;
  pack.motor_temp_on_chip = msg->motor_temp_on_chip;

  pack_and_send_mavlink_message_t(pack, MainLink);



}

void GCSTransceiver::onGetBoardARM1CallBack(const board_arm1_stamped::ConstPtr &msg)
{
  CUSTOM_MSG_BOARD_ARM1 pack;
  pack.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e-6);

//  pack.ls_pistol_athead = msg->pistol_ls.ls_AtHead;
//  pack.ls_pistol_attail = msg->pistol_ls.ls_AtTail;
//  pack.ls_mass_shifter_athead = msg->mass_shifter_ls.ls_AtHead;
//  pack.ls_mass_shifter_attail = msg->mass_shifter_ls.ls_AtTail;
  pack.altimeter_in_feet = msg->altimeter_status.alt_in_feet;
  pack.altimeter_in_metres = msg->altimeter_status.alt_in_metres;
  pack.altimeter_in_fathoms = msg->altimeter_status.alt_in_fathoms;

  SysStatus.voltage_battery = static_cast<uint16_t>(msg->bms_status.baterryVoltage);
  SysStatus.current_battery = static_cast<int16_t>(msg->bms_status.batteryCurrent);
  SysStatus.battery_remaining = static_cast<int8_t>(msg->bms_status.batteryPercentage);
  SysStatus.batery_total = static_cast<uint16_t>(msg->bms_status.batteryTotal);
  SysStatus.batery_used = static_cast<uint16_t>(msg->bms_status.batteryUsed);
  SysStatus.batery_capacity = static_cast<uint16_t>(msg->bms_status.batteryCapacity);

  pack_and_send_mavlink_message_t(pack, MainLink);

}

void GCSTransceiver::onGetBoardARM2CallBack(const board_arm2_stamped::ConstPtr &msg)
{
  CUSTOM_MSG_BOARD_ARM2 pack;
  pack.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e-6);

  pack.rudder_load = msg->mx28_status.load;
  pack.rudder_speed = msg->mx28_status.speed;
  pack.rudder_voltage = msg->mx28_status.voltage;
  pack.rudder_position = msg->mx28_status.position;
  pack.rudder_temperature = msg->mx28_status.temperature;
  pack.keller_pa3_pressure = msg->pressure_status.pressure;
  pack.keller_pa3_temperature = msg->pressure_status.temperature;

  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onSetpointCallBack(const Float64MultiArrayStamped::ConstPtr& msg)
{
  POSITION_TARGET_GLOBAL_INT pack;
  pack.time_boot_ms = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e-6);
  pack.coordinate_frame = static_cast<uint8_t>(MAV_FRAME::GLOBAL_INT);
  pack.type_mask = static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::X_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::Y_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::Z_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::VY_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::VZ_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::AX_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::AY_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::AZ_IGNORE) |
                   static_cast<uint16_t>(POSITION_TARGET_TYPEMASK::YAW_RATE_IGNORE);
  pack.vx = static_cast<float>(msg->data[0]);
  pack.yaw = static_cast<float>(msg->data[1]);

  pack_and_send_mavlink_message_t(pack, MainLink);
}

void GCSTransceiver::onThrusterCmdCallBack(const Float64MultiArrayStamped::ConstPtr& msg)
{
  ACTUATOR_CONTROL_TARGET pack;
  pack.time_usec = static_cast<uint32_t>(msg->header.stamp.toNSec() * 1e-3);
  for (size_t i = 0; i < msg->data.size(); i++)
    pack.controls.data()[i] = static_cast<float>(msg->data[i]);

  pack_and_send_mavlink_message_t(pack, MainLink);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gcs_transceiver");
  GCSTransceiver trans;
  ros::spin();
}
