#include "gps_dvl_ins.h"

#define GPS_DVL_INS_DATA_LENGTH 169
#define GPS_DVL_INS_FEEDBACK_DATA_LENGTH 61

static ros::Publisher pub_GPS_DVL_INS;

const std::string gps_dvl_ins_start = "$Viam_Navi,START\r\n";
const std::string gps_dvl_ins_stop = "$Viam_Navi,STOP\r\n";
// Header Calibration
const std::string header_calib_acc_ref = "$PCViam_Calib_acc_ref";
const std::string header_calib_mag_ref = "$PCViam_Calib_mag_ref";
const std::string header_calib_gyro = "$PCViam_Calib_gyro";
const std::string header_calib_sp = "$PCViam_Calib_sp";
const std::string header_calib_sv = "$PCViam_Calib_sv";
const std::string header_calib_sd = "$PCViam_Calib_sd";
const std::string header_calib_qr = "$PCViam_Calib_qr";
const std::string header_calib_qag = "$PCViam_Calib_qag";
// Calibration Feedback
const std::string calib_acc_ref_done = "$PCViam_Calib_acc_ref_OK\r\n";
const std::string calib_mag_ref_done = "$PCViam_Calib_mag_ref_OK\r\n";
const std::string calib_gyro_done = "$PCViam_Calib_gyro_OK\r\n";
const std::string calib_sp_done = "$PCViam_Calib_sp_OK\r\n";
const std::string calib_sv_done = "$PCViam_Calib_sv_OK\r\n";
const std::string calib_sd_done = "$PCViam_Calib_sd_OK\r\n";
const std::string calib_qr_done = "$PCViam_Calib_qr_OK\r\n";
const std::string calib_qag_done = "$PCViam_Calib_qga_OK\r\n";

static serial::Serial GPS_DVL_INS_serial;
static std::ofstream gps_dvl_ins_logfile;
static int gps_dvl_ins_sample_time;
static std::string log_data_direction = "";
static ros::WallTime start_time;
static double last_time;

// Calibration Parameters
static bool calibration_done = false;

//<!-- ACC -->
static float acc_ref_gain[9];
static float acc_ref_offset[3];

//<!-- MAG -->
static float mag_ref_gain[9];
static float mag_ref_offset[3];

//<!-- GYRO -->
static float gyro_offset[3];

//<!-- SP -->
static float sp[3];

//<!-- SV -->
static float sv[3];

//<!-- SD -->
static float sd[3];

//<!-- QR -->
static float qr[4];

//<!-- QAG -->
static float qag[4];

static union
{
  uint8_t _bytes[GPS_DVL_INS_DATA_LENGTH];
  struct
  {
    uint8_t _header[4];
    uint32_t _sample_count;
    float ekf_roll; // degree
    float ekf_pitch; // degree
    float ekf_yaw; // degree
    float ekf_alt; // m
    float ekf_vN; // m/s
    float ekf_vE; // m/s
    float ekf_vD; // m/s
    float ekf_vX; // m/s
    float ekf_vY; // m/s
    float ekf_vZ; // m/s
    float _rad_gyro_X; // rad/s
    float _rad_gyro_Y; // rad/s
    float _rad_gyro_Z; // rad/s
    float _angular_acc_X; // rad/s^2
    float _angular_acc_Y; // rad/s^2
    float _angular_acc_Z; // rad/s^2
    float imu_deg_gyro_X; // deg/s
    float imu_deg_gyro_Y; // deg/s
    float imu_deg_gyro_Z; // deg/s
    float imu_mag_X; // mgauss
    float imu_mag_Y; // mgauss
    float imu_mag_Z; // mgauss
    float imu_acc_X; // m/s^2
    float imu_acc_Y; // m/s^2
    float imu_acc_Z; // m/s^2
    int32_t gps_lat;
    int32_t gps_lon;
    float gps_alt;
    float gps_vN;
    float gps_vE;
    float gps_vD;
    float dvl_vX;
    float dvl_vY;
    float dvl_vZ;
    double ekf_lat; // degree
    double ekf_lon; // degree
    uint16_t _alt_DVL; // dm
    uint8_t _dvl_error_code[2];
    uint8_t _flag_to_check;
    uint8_t checksum;
    uint8_t _end[3];
    uint8_t pad[7];
  }_value_of;
}GPS_DVL_INS_Data;

uint8_t UGDI_Checksum(uint8_t* _data, uint8_t _start, uint32_t _length);
void Convert_Bytes_to_Float(uint8_t* _data_in, float *_data_out);
void Convert_Float_to_Bytes(float _data_in, uint8_t* _data_out);
void UGDI_SendCalibrationParams(std::string _header, const uint8_t* _payload, uint8_t _lengthOfPayload);

GPS_DVL_INS::GPS_DVL_INS()
{
  /// Get local parameters of this node through roslaunch file.
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("gps_dvl_ins_device", gps_dvl_ins_device, "/dev/ttyUSB0");
  private_nh.param<int>("gps_dvl_ins_baurate", gps_dvl_ins_baurate, 115200);
  private_nh.param<bool>("calibration_mode", calibration_mode, false);
  private_nh.param<int>("sample_time", gps_dvl_ins_sample_time, 25);
  private_nh.param<std::string>("log_file_direction", log_data_direction, "/home/thanhhaibk96/GPS_DVL_INS_Calib.txt");

  private_nh.getParam("calibration_done", calibration_done);

  for (uint8_t i = 0; i < 9; i++)
  {
    if(private_nh.hasParam("acc_ref_gain_" + std::to_string(i)))
      private_nh.getParam("acc_ref_gain_" + std::to_string(i), acc_ref_gain[i]);
    else
      ROS_WARN_STREAM("No Parameter: " << "acc_ref_gain_" + std::to_string(i));
  }

  for (uint8_t i = 0; i < 3; i++)
  {
    if(private_nh.hasParam("acc_ref_offset_" + std::to_string(i)))
      private_nh.getParam("acc_ref_offset_" + std::to_string(i), acc_ref_offset[i]);
    else
      ROS_WARN_STREAM("No Parameter: " << "acc_ref_offset" + std::to_string(i));
  }

  for (uint8_t i = 0; i < 9; i++)
  {
    if(private_nh.hasParam("mag_ref_gain_" + std::to_string(i)))
      private_nh.getParam("mag_ref_gain_" + std::to_string(i), mag_ref_gain[i]);
    else
      ROS_WARN_STREAM("No Parameter: " << "mag_ref_gain_" + std::to_string(i));
  }

  for (uint8_t i = 0; i < 3; i++)
  {
    if(private_nh.hasParam("mag_ref_offset_" + std::to_string(i)))
      private_nh.getParam("mag_ref_offset_" + std::to_string(i), mag_ref_offset[i]);
    else
      ROS_WARN_STREAM("No Parameter: " << "mag_ref_offset_" + std::to_string(i));
  }

  for (uint8_t i = 0; i < 3; i++)
  {
    if(private_nh.hasParam("gyro_offset_" + std::to_string(i)))
      private_nh.getParam("gyro_offset_" + std::to_string(i), gyro_offset[i]);
    else
      ROS_WARN_STREAM("No Parameter: " << "gyro_offset_" + std::to_string(i));
  }

  for (uint8_t i = 0; i < 3; i++)
  {
    if(private_nh.hasParam("sp_" + std::to_string(i)))
      private_nh.getParam("sp_" + std::to_string(i), sp[i]);
    else
      ROS_WARN_STREAM("No Parameter: " << "sp_" + std::to_string(i));
  }

  for (uint8_t i = 0; i < 3; i++)
  {
    if(private_nh.hasParam("sv_" + std::to_string(i)))
      private_nh.getParam("sv_" + std::to_string(i), sv[i]);
    else
      ROS_WARN_STREAM("No Parameter: " << "sv_" + std::to_string(i));
  }

  for (uint8_t i = 0; i < 3; i++)
  {
    if(private_nh.hasParam("sd_" + std::to_string(i)))
      private_nh.getParam("sd_" + std::to_string(i), sd[i]);
    else
      ROS_WARN_STREAM("No Parameter: " << "sd_" + std::to_string(i));
  }

  for (uint8_t i = 0; i < 4; i++)
  {
    if(private_nh.hasParam("qr_" + std::to_string(i)))
      private_nh.getParam("qr_" + std::to_string(i), qr[i]);
    else
      ROS_WARN_STREAM("No Parameter: " << "qr_" + std::to_string(i));
  }

  for (uint8_t i = 0; i < 4; i++)
  {
    if(private_nh.hasParam("qag_" + std::to_string(i)))
      private_nh.getParam("qag_" + std::to_string(i), qag[i]);
    else
      ROS_WARN_STREAM("No Parameter: " << "qag_" + std::to_string(i));
  }

  try
  {
     GPS_DVL_INS_serial.setPort(gps_dvl_ins_device);
     GPS_DVL_INS_serial.setBaudrate(static_cast<uint32_t>(gps_dvl_ins_baurate));
     serial::Timeout to = serial::Timeout::simpleTimeout(1000);
     GPS_DVL_INS_serial.setTimeout(to);
     GPS_DVL_INS_serial.open();
  }
  catch (serial::IOException& e)
  {
     ROS_ERROR_STREAM("Unable to open port ");
  }

  if(GPS_DVL_INS_serial.isOpen())
  {
     ROS_INFO_STREAM("Serial Port initialized");
  }
  else
  {
     ROS_INFO_STREAM("Serial Port initialized unsuccessfully");
  }
  ros::Duration(0.5).sleep();

  if(calibration_done == false)
  {
    // Send Calibration Data
    uint8_t _arr_acc_ref[50];
    for (uint8_t i = 0; i < 9; i++)
    {
      Convert_Float_to_Bytes(acc_ref_gain[i], _arr_acc_ref + i*4);
    }
    for (uint8_t i = 0; i < 3; i++)
    {
      Convert_Float_to_Bytes(acc_ref_offset[i], _arr_acc_ref + i*4 + 9*4);
    }
    _arr_acc_ref[48] = '\r';
    _arr_acc_ref[49] = '\n';

    UGDI_SendCalibrationParams(header_calib_acc_ref, _arr_acc_ref, 50);
    while(GPS_DVL_INS_serial.available() < GPS_DVL_INS_FEEDBACK_DATA_LENGTH);
    ROS_INFO_STREAM(GPS_DVL_INS_serial.readline(GPS_DVL_INS_FEEDBACK_DATA_LENGTH, "\r\n"));
    GPS_DVL_INS_serial.flushInput();

    uint8_t _arr_mag_ref[50];
    for (uint8_t i = 0; i < 9; i++)
    {
      Convert_Float_to_Bytes(mag_ref_gain[i], _arr_mag_ref + i*4);
    }
    for (uint8_t i = 0; i < 3; i++)
    {
      Convert_Float_to_Bytes(mag_ref_offset[i], _arr_mag_ref + i*4 + 9*4);
    }
    _arr_mag_ref[48] = '\r';
    _arr_mag_ref[49] = '\n';

    UGDI_SendCalibrationParams(header_calib_mag_ref, _arr_mag_ref, 50);
    while(GPS_DVL_INS_serial.available() < GPS_DVL_INS_FEEDBACK_DATA_LENGTH);
    ROS_INFO_STREAM(GPS_DVL_INS_serial.readline(GPS_DVL_INS_FEEDBACK_DATA_LENGTH, "\r\n"));
    GPS_DVL_INS_serial.flushInput();

    uint8_t _arr_gyro_offset[14];
    for (uint8_t i = 0; i < 3; i++)
    {
      Convert_Float_to_Bytes(gyro_offset[i], _arr_gyro_offset + i*4);
    }
    _arr_gyro_offset[12] = '\r';
    _arr_gyro_offset[13] = '\n';

    UGDI_SendCalibrationParams(header_calib_gyro, _arr_gyro_offset, 14);
    while(GPS_DVL_INS_serial.available() < GPS_DVL_INS_FEEDBACK_DATA_LENGTH);
    ROS_INFO_STREAM(GPS_DVL_INS_serial.readline(GPS_DVL_INS_FEEDBACK_DATA_LENGTH, "\r\n"));
    GPS_DVL_INS_serial.flushInput();

    uint8_t _arr_sp[14];
    for (uint8_t i = 0; i < 3; i++)
    {
      Convert_Float_to_Bytes(sp[i], _arr_sp + i*4);
    }
    _arr_sp[12] = '\r';
    _arr_sp[13] = '\n';

    UGDI_SendCalibrationParams(header_calib_sp, _arr_sp, 14);
    while(GPS_DVL_INS_serial.available() < GPS_DVL_INS_FEEDBACK_DATA_LENGTH);
    ROS_INFO_STREAM(GPS_DVL_INS_serial.readline(GPS_DVL_INS_FEEDBACK_DATA_LENGTH, "\r\n"));
    GPS_DVL_INS_serial.flushInput();

    uint8_t _arr_sv[14];
    for (uint8_t i = 0; i < 3; i++)
    {
      Convert_Float_to_Bytes(sv[i], _arr_sv + i*4);
    }
    _arr_sv[12] = '\r';
    _arr_sv[13] = '\n';

    UGDI_SendCalibrationParams(header_calib_sv, _arr_sv, 14);
    while(GPS_DVL_INS_serial.available() < GPS_DVL_INS_FEEDBACK_DATA_LENGTH);
    ROS_INFO_STREAM(GPS_DVL_INS_serial.readline(GPS_DVL_INS_FEEDBACK_DATA_LENGTH, "\r\n"));
    GPS_DVL_INS_serial.flushInput();

    uint8_t _arr_sd[14];
    for (uint8_t i = 0; i < 3; i++)
    {
      Convert_Float_to_Bytes(sd[i], _arr_sd + i*4);
    }
    _arr_sd[12] = '\r';
    _arr_sd[13] = '\n';

    UGDI_SendCalibrationParams(header_calib_sd, _arr_sd, 14);
    while(GPS_DVL_INS_serial.available() < GPS_DVL_INS_FEEDBACK_DATA_LENGTH);
    ROS_INFO_STREAM(GPS_DVL_INS_serial.readline(GPS_DVL_INS_FEEDBACK_DATA_LENGTH, "\r\n"));
    GPS_DVL_INS_serial.flushInput();

    uint8_t _arr_qr[18];
    for (uint8_t i = 0; i < 4; i++)
    {
      Convert_Float_to_Bytes(qr[i], _arr_qr + i*4);
    }
    _arr_qr[16] = '\r';
    _arr_qr[17] = '\n';

    UGDI_SendCalibrationParams(header_calib_qr, _arr_qr, 18);
    while(GPS_DVL_INS_serial.available() < GPS_DVL_INS_FEEDBACK_DATA_LENGTH);
    ROS_INFO_STREAM(GPS_DVL_INS_serial.readline(GPS_DVL_INS_FEEDBACK_DATA_LENGTH, "\r\n"));
    GPS_DVL_INS_serial.flushInput();

    uint8_t _arr_qag[18];
    for (uint8_t i = 0; i < 4; i++)
    {
      Convert_Float_to_Bytes(qag[i], _arr_qag + i*4);
    }
    _arr_qag[16] = '\r';
    _arr_qag[17] = '\n';

    UGDI_SendCalibrationParams(header_calib_qag, _arr_qag, 18);
    while(GPS_DVL_INS_serial.available() < GPS_DVL_INS_FEEDBACK_DATA_LENGTH);
    ROS_INFO_STREAM(GPS_DVL_INS_serial.readline(GPS_DVL_INS_FEEDBACK_DATA_LENGTH, "\r\n"));
    GPS_DVL_INS_serial.flushInput();
  }

  ros::Duration(5).sleep();
}

int main(int argc, char** argv)
{
  gps_dvl_ins_stamped _gps_dvl_ins_data;

  ros::init(argc, argv, "GPS_DVL_INS");
  ros::NodeHandle nh;
  pub_GPS_DVL_INS = nh.advertise<gps_dvl_ins_stamped>("my_auv2000_sensors/gps_dvl_ins_data", 1);

  GPS_DVL_INS gps_dvl_ins_node;

  // Start GPS_DVL_INS Module
  if(GPS_DVL_INS_serial.isOpen())
  {
    GPS_DVL_INS_serial.write(gps_dvl_ins_start);
    start_time = ros::WallTime::now();
  }

  gps_dvl_ins_logfile.open(log_data_direction);

  ros::Rate loop_rate(gps_dvl_ins_sample_time);

  ROS_INFO_STREAM("GPS-DVL-INS");

  while(ros::ok())
  {
     ros::spinOnce();

     if(GPS_DVL_INS_serial.available())
     {
//       ROS_INFO_STREAM("Read: " << GPS_DVL_INS_serial.available());
        std::string result = "";
        GPS_DVL_INS_serial.readline(result, GPS_DVL_INS_DATA_LENGTH,  "*\r\n");
//        ROS_INFO_STREAM("Read: " << result);
        std::copy(result.begin(), result.end(), GPS_DVL_INS_Data._bytes);

        if((GPS_DVL_INS_Data._bytes[0] == '$') && (GPS_DVL_INS_Data._bytes[1] == 'N') &&
           (GPS_DVL_INS_Data._bytes[2] == 'a') && (GPS_DVL_INS_Data._bytes[3] == 'v') &&
           (GPS_DVL_INS_Data._bytes[165] == UGDI_Checksum(GPS_DVL_INS_Data._bytes, 4, GPS_DVL_INS_DATA_LENGTH - 4)))
        {
          if((gps_dvl_ins_node.calibration_mode) && (gps_dvl_ins_logfile.is_open()))
          {
            std::string _string_per_line = "";
            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of._sample_count);
            _string_per_line += " ";
            _gps_dvl_ins_data.sample_count = GPS_DVL_INS_Data._value_of._sample_count;

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.ekf_roll);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.ekf_pitch);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.ekf_yaw);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.ekf_lat);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.ekf_lon);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.ekf_alt);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.ekf_vN);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.ekf_vE);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.ekf_vD);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.ekf_vX);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.ekf_vY);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.ekf_vZ);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of._rad_gyro_X);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of._rad_gyro_Y);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of._rad_gyro_Z);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of._angular_acc_X);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of._angular_acc_Y);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of._angular_acc_Z);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of._alt_DVL);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of._dvl_error_code[0]);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of._dvl_error_code[1]);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of._flag_to_check);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.imu_deg_gyro_X);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.imu_deg_gyro_Y);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.imu_deg_gyro_Z);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.imu_mag_X);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.imu_mag_Y);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.imu_mag_Z);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.imu_acc_X);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.imu_acc_Y);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.imu_acc_Z);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.gps_lat);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.gps_lon);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.gps_alt);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.gps_vN);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.gps_vE);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.gps_vD);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.dvl_vX);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.dvl_vY);
            _string_per_line += " ";

            _string_per_line +=  std::to_string(GPS_DVL_INS_Data._value_of.dvl_vZ);

            _string_per_line += "\n";
            gps_dvl_ins_logfile << _string_per_line;
          }

            last_time = ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
            _gps_dvl_ins_data.header.stamp = static_cast<ros::Time>(last_time);
            _gps_dvl_ins_data.ekf_roll = GPS_DVL_INS_Data._value_of.ekf_roll;
            _gps_dvl_ins_data.ekf_pitch = GPS_DVL_INS_Data._value_of.ekf_pitch;
            _gps_dvl_ins_data.ekf_yaw = GPS_DVL_INS_Data._value_of.ekf_yaw;
            _gps_dvl_ins_data.ekf_lat = GPS_DVL_INS_Data._value_of.ekf_lat;
            _gps_dvl_ins_data.ekf_lon = GPS_DVL_INS_Data._value_of.ekf_lon;
            _gps_dvl_ins_data.ekf_alt = GPS_DVL_INS_Data._value_of.ekf_alt;
            _gps_dvl_ins_data.ekf_vN = GPS_DVL_INS_Data._value_of.ekf_vN;
            _gps_dvl_ins_data.ekf_vE = GPS_DVL_INS_Data._value_of.ekf_vE;
            _gps_dvl_ins_data.ekf_vD = GPS_DVL_INS_Data._value_of.ekf_vD;
            _gps_dvl_ins_data.ekf_vX = GPS_DVL_INS_Data._value_of.ekf_vX;
            _gps_dvl_ins_data.ekf_vY = GPS_DVL_INS_Data._value_of.ekf_vY;
            _gps_dvl_ins_data.ekf_vZ = GPS_DVL_INS_Data._value_of.ekf_vZ;
            _gps_dvl_ins_data.rad_gyro_X = GPS_DVL_INS_Data._value_of._rad_gyro_X;
            _gps_dvl_ins_data.rad_gyro_Y = GPS_DVL_INS_Data._value_of._rad_gyro_Y;
            _gps_dvl_ins_data.rad_gyro_Z = GPS_DVL_INS_Data._value_of._rad_gyro_Z;
//            _gps_dvl_ins_data.angular_acc_X = GPS_DVL_INS_Data._value_of._angular_acc_X;
//            _gps_dvl_ins_data.angular_acc_Y = GPS_DVL_INS_Data._value_of._angular_acc_Y;
//            _gps_dvl_ins_data.angular_acc_Z = GPS_DVL_INS_Data._value_of._angular_acc_Z;
//            _gps_dvl_ins_data.alt_DVL = GPS_DVL_INS_Data._value_of._alt_DVL;
//            _gps_dvl_ins_data.dvl_error_code[0] = GPS_DVL_INS_Data._value_of._dvl_error_code[0];
//            _gps_dvl_ins_data.dvl_error_code[1] = GPS_DVL_INS_Data._value_of._dvl_error_code[1];
//            _gps_dvl_ins_data.flag_to_check = GPS_DVL_INS_Data._value_of._flag_to_check;
            _gps_dvl_ins_data.imu_deg_gyro_X = GPS_DVL_INS_Data._value_of.imu_deg_gyro_X;
            _gps_dvl_ins_data.imu_deg_gyro_Y = GPS_DVL_INS_Data._value_of.imu_deg_gyro_Y;
            _gps_dvl_ins_data.imu_deg_gyro_Z = GPS_DVL_INS_Data._value_of.imu_deg_gyro_Z;
            _gps_dvl_ins_data.imu_mag_X = GPS_DVL_INS_Data._value_of.imu_mag_X;
            _gps_dvl_ins_data.imu_mag_Y = GPS_DVL_INS_Data._value_of.imu_mag_Y;
            _gps_dvl_ins_data.imu_mag_Z = GPS_DVL_INS_Data._value_of.imu_mag_Z;
            _gps_dvl_ins_data.imu_acc_X = GPS_DVL_INS_Data._value_of.imu_acc_X;
            _gps_dvl_ins_data.imu_acc_Y = GPS_DVL_INS_Data._value_of.imu_acc_Y;
            _gps_dvl_ins_data.imu_acc_Z = GPS_DVL_INS_Data._value_of.imu_acc_Z;
//            _gps_dvl_ins_data.gps_lat = GPS_DVL_INS_Data._value_of.gps_lat;
//            _gps_dvl_ins_data.gps_lon = GPS_DVL_INS_Data._value_of.gps_lon;
//            _gps_dvl_ins_data.gps_alt = GPS_DVL_INS_Data._value_of.gps_alt;
//            _gps_dvl_ins_data.gps_vN = GPS_DVL_INS_Data._value_of.gps_vN;
//            _gps_dvl_ins_data.gps_vE = GPS_DVL_INS_Data._value_of.gps_vE;
//            _gps_dvl_ins_data.gps_vD = GPS_DVL_INS_Data._value_of.gps_vD;
//            _gps_dvl_ins_data.dvl_vX = GPS_DVL_INS_Data._value_of.dvl_vX;
//            _gps_dvl_ins_data.dvl_vY = GPS_DVL_INS_Data._value_of.dvl_vY;
//            _gps_dvl_ins_data.dvl_vZ = GPS_DVL_INS_Data._value_of.dvl_vZ;
            pub_GPS_DVL_INS.publish(_gps_dvl_ins_data);
        }
        loop_rate.sleep();
     }
  }
  GPS_DVL_INS_serial.write(gps_dvl_ins_stop);
  gps_dvl_ins_logfile.close();
}

uint8_t UGDI_Checksum(uint8_t* _data, uint8_t _start, uint32_t _length)
{
  uint8_t _checksum = 0;
  for(uint32_t i = _start; i < _length; i++)
  {
    _checksum ^= _data[i];
  }
  return _checksum;
}

void Convert_Bytes_to_Float(uint8_t* _data_in, float *_data_out)
{
  union
  {
    float _value;
    uint8_t _byte[4];
  }_part;

  _part._byte[0] = _data_in[3];
  _part._byte[1] = _data_in[2];
  _part._byte[2] = _data_in[1];
  _part._byte[3] = _data_in[0];

  *_data_out = _part._value;
}

void Convert_Float_to_Bytes(float _data_in, uint8_t* _data_out)
{
  union
  {
    float _value;
    uint8_t _byte[4];
  }_part;

  _part._value = _data_in;

  _data_out[0] = _part._byte[0];
  _data_out[1] = _part._byte[1];
  _data_out[2] = _part._byte[2];
  _data_out[3] = _part._byte[3];
}

void UGDI_SendCalibrationParams(std::string _header, const uint8_t* _payload, uint8_t _lengthOfPayload)
{
  uint8_t _tmp_arr[_header.length() +  _lengthOfPayload + 1];

//  ROS_INFO_STREAM("GPS-DVL-INS: " << _header.length());
  std::copy(_header.begin(), _header.end(), _tmp_arr);
  _tmp_arr[_header.length()] = '\0';
  for (uint8_t i = 0; i < _lengthOfPayload; i++)
  {
    _tmp_arr[_header.length() + 1 + i] = _payload[i];
  }

  if(GPS_DVL_INS_serial.isOpen())
  {
    GPS_DVL_INS_serial.write(_tmp_arr, _header.length() +  _lengthOfPayload + 1);
  }
}
