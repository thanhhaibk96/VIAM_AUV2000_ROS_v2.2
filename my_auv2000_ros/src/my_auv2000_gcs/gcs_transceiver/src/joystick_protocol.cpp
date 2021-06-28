#include "joystick_protocol.h"
#include "gcs_transceiver.h"

static int MAX_Angle_Rudder = 0;
static int MAX_Thrust = 0;

Joystick_Protocol::Joystick_Protocol(GCSTransceiver* trans, const MAVConnInterface::Ptr link) : link(link), trans(trans)
{
  ros::NodeHandle nh;
  nh.param<int>("max_angle_rudder", MAX_Angle_Rudder, 60);
  nh.param<int>("max_thrust_speed", MAX_Thrust, 1200);

  pubJoystick = nh.advertise<joystick_stamped>("joystick/data", 1);
}

void Joystick_Protocol::handleJoystick(const mavlink_message_t *msg)
{
  switch (msg->msgid)
  {
  case 69: // MANUAL_CONTROL
    publishJoystickData(msg);
    break;
  }
}

void Joystick_Protocol::publishJoystickData(const mavlink_message_t *msg)
{
  MANUAL_CONTROL reqPack;
  unpack_mavlink_message_t(msg, reqPack);

  if(check_for_only_target_id_matching(reqPack, trans->autopilot_sysid))
  {
  //  ROS_INFO_STREAM(reqPack.to_yaml());
    joystick_stamped joystick_data;

    if((reqPack.buttons&0x00000800) == 2048) joystick_data.mass_shifter_up = true;
    else joystick_data.mass_shifter_up = false;

    if((reqPack.buttons&0x00001000) == 4096) joystick_data.mass_shifter_down = true;
    else joystick_data.mass_shifter_down = false;

    if((reqPack.buttons&0x00000008) == 8) joystick_data.pistol_up = true;
    else joystick_data.pistol_up = false;

    if((reqPack.buttons&0x00000001) == 1) joystick_data.pistol_down = true;
    else joystick_data.pistol_down = false;

    joystick_data.rudder =  static_cast<int16_t>(static_cast<float>(reqPack.z)*static_cast<float>(MAX_Angle_Rudder)/1000.0f);
    if(reqPack.y >= 0)
    {
      joystick_data.thruster = static_cast<int16_t>((static_cast<float>((reqPack.y))/1000.0f) * static_cast<float>(MAX_Thrust));
    }

    joystick_data.bhat_lt = (reqPack.lefthat == 1)? true : false;
    joystick_data.bhat_rt = (reqPack.righthat == 1)? true : false;

    joystick_data.bhat_lb = (reqPack.buttons == 512)? true : false;
    joystick_data.bhat_rb = (reqPack.buttons == 1024)? true : false;

    joystick_data.arrow_left = (reqPack.buttons == 8192)? true : false;
    joystick_data.arrow_right = (reqPack.buttons == 16384)? true : false;
    joystick_data.button_x = (reqPack.buttons == 4)? true : false;
    joystick_data.button_b = (reqPack.buttons == 2)? true : false;
    joystick_data.button_back = (reqPack.buttons == 16)? true : false;
    joystick_data.button_start = (reqPack.buttons == 64)? true : false;

    pubJoystick.publish(joystick_data);
  }
}
