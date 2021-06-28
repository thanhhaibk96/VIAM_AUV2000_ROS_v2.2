#ifndef JOYSTICK_PROTOCOL_H
#define JOYSTICK_PROTOCOL_H

#include "interface.h"
#include <ros/ros.h>

#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include "auv_msgs_stamped/joystick_stamped.h"
#include "mavlink.h"

using namespace mavconn;
using namespace mavlink;
using namespace mavlink::common;
using namespace mavlink::common::msg;
using namespace mavlink::viam_auv2000;
using namespace mavlink::viam_auv2000::msg;
using namespace mavros_msgs;
using namespace auv_msgs_stamped;

class GCSTransceiver;

class Joystick_Protocol
{
public:
  Joystick_Protocol();
  Joystick_Protocol(GCSTransceiver* trans, const MAVConnInterface::Ptr link);

  /**
   * @brief Handler for message reception from the parented GCSTransceiver.
   * @param[in] msg Message to handle.
   *
   * This handler serves to branch messages to proper sub-handlers.
   */
  void handleJoystick(const mavlink_message_t* msg);
private:
  /**
   * @brief Link to handle the parameter microservice.
   */
  MAVConnInterface::Ptr link;

  ros::Publisher pubJoystick;

  /**
   * @brief Parented GCSTranceiver.
   *
   * This pointer is needed as the microservice needs to filter the autopilot sysid and compid, in some cases to update
   * the system heartbeat and status.
   */
  GCSTransceiver* trans;

  /**
   * @brief Handler for message sequence for publishing joystick data.
   * @param[in] msg Message to handle.
   */
  void publishJoystickData(const mavlink_message_t* msg);

};

#endif // JOYSTICK_PROTOCOL_H
