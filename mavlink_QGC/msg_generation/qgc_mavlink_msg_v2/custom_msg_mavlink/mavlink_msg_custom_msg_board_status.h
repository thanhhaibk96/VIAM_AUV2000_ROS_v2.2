#pragma once
// MESSAGE CUSTOM_MSG_BOARD_STATUS PACKING

#define MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS 398


typedef struct __mavlink_custom_msg_board_status_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 uint8_t arm1; /*<  ARM1's status in AUV*/
 uint8_t arm2; /*<  ARM2's status in AUV*/
 uint8_t viam_navy; /*<  VIAM_NAVY's status in AUV*/
 uint8_t pistol; /*<  PISTOL's status in AUV*/
 uint8_t mass_shifter; /*<  MASS_SHIFTER's status in AUV*/
 uint8_t thruster; /*<  THRUSTER's status in AUV*/
} mavlink_custom_msg_board_status_t;

#define MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN 10
#define MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_MIN_LEN 10
#define MAVLINK_MSG_ID_398_LEN 10
#define MAVLINK_MSG_ID_398_MIN_LEN 10

#define MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_CRC 1
#define MAVLINK_MSG_ID_398_CRC 1



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CUSTOM_MSG_BOARD_STATUS { \
    398, \
    "CUSTOM_MSG_BOARD_STATUS", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_custom_msg_board_status_t, time_boot_ms) }, \
         { "arm1", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_custom_msg_board_status_t, arm1) }, \
         { "arm2", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_custom_msg_board_status_t, arm2) }, \
         { "viam_navy", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_custom_msg_board_status_t, viam_navy) }, \
         { "pistol", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_custom_msg_board_status_t, pistol) }, \
         { "mass_shifter", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_custom_msg_board_status_t, mass_shifter) }, \
         { "thruster", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_custom_msg_board_status_t, thruster) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CUSTOM_MSG_BOARD_STATUS { \
    "CUSTOM_MSG_BOARD_STATUS", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_custom_msg_board_status_t, time_boot_ms) }, \
         { "arm1", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_custom_msg_board_status_t, arm1) }, \
         { "arm2", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_custom_msg_board_status_t, arm2) }, \
         { "viam_navy", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_custom_msg_board_status_t, viam_navy) }, \
         { "pistol", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_custom_msg_board_status_t, pistol) }, \
         { "mass_shifter", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_custom_msg_board_status_t, mass_shifter) }, \
         { "thruster", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_custom_msg_board_status_t, thruster) }, \
         } \
}
#endif

/**
 * @brief Pack a custom_msg_board_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param arm1  ARM1's status in AUV
 * @param arm2  ARM2's status in AUV
 * @param viam_navy  VIAM_NAVY's status in AUV
 * @param pistol  PISTOL's status in AUV
 * @param mass_shifter  MASS_SHIFTER's status in AUV
 * @param thruster  THRUSTER's status in AUV
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_custom_msg_board_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t arm1, uint8_t arm2, uint8_t viam_navy, uint8_t pistol, uint8_t mass_shifter, uint8_t thruster)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint8_t(buf, 4, arm1);
    _mav_put_uint8_t(buf, 5, arm2);
    _mav_put_uint8_t(buf, 6, viam_navy);
    _mav_put_uint8_t(buf, 7, pistol);
    _mav_put_uint8_t(buf, 8, mass_shifter);
    _mav_put_uint8_t(buf, 9, thruster);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN);
#else
    mavlink_custom_msg_board_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.arm1 = arm1;
    packet.arm2 = arm2;
    packet.viam_navy = viam_navy;
    packet.pistol = pistol;
    packet.mass_shifter = mass_shifter;
    packet.thruster = thruster;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_CRC);
}

/**
 * @brief Pack a custom_msg_board_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param arm1  ARM1's status in AUV
 * @param arm2  ARM2's status in AUV
 * @param viam_navy  VIAM_NAVY's status in AUV
 * @param pistol  PISTOL's status in AUV
 * @param mass_shifter  MASS_SHIFTER's status in AUV
 * @param thruster  THRUSTER's status in AUV
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_custom_msg_board_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t arm1,uint8_t arm2,uint8_t viam_navy,uint8_t pistol,uint8_t mass_shifter,uint8_t thruster)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint8_t(buf, 4, arm1);
    _mav_put_uint8_t(buf, 5, arm2);
    _mav_put_uint8_t(buf, 6, viam_navy);
    _mav_put_uint8_t(buf, 7, pistol);
    _mav_put_uint8_t(buf, 8, mass_shifter);
    _mav_put_uint8_t(buf, 9, thruster);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN);
#else
    mavlink_custom_msg_board_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.arm1 = arm1;
    packet.arm2 = arm2;
    packet.viam_navy = viam_navy;
    packet.pistol = pistol;
    packet.mass_shifter = mass_shifter;
    packet.thruster = thruster;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_CRC);
}

/**
 * @brief Encode a custom_msg_board_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param custom_msg_board_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_custom_msg_board_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_custom_msg_board_status_t* custom_msg_board_status)
{
    return mavlink_msg_custom_msg_board_status_pack(system_id, component_id, msg, custom_msg_board_status->time_boot_ms, custom_msg_board_status->arm1, custom_msg_board_status->arm2, custom_msg_board_status->viam_navy, custom_msg_board_status->pistol, custom_msg_board_status->mass_shifter, custom_msg_board_status->thruster);
}

/**
 * @brief Encode a custom_msg_board_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param custom_msg_board_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_custom_msg_board_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_custom_msg_board_status_t* custom_msg_board_status)
{
    return mavlink_msg_custom_msg_board_status_pack_chan(system_id, component_id, chan, msg, custom_msg_board_status->time_boot_ms, custom_msg_board_status->arm1, custom_msg_board_status->arm2, custom_msg_board_status->viam_navy, custom_msg_board_status->pistol, custom_msg_board_status->mass_shifter, custom_msg_board_status->thruster);
}

/**
 * @brief Send a custom_msg_board_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param arm1  ARM1's status in AUV
 * @param arm2  ARM2's status in AUV
 * @param viam_navy  VIAM_NAVY's status in AUV
 * @param pistol  PISTOL's status in AUV
 * @param mass_shifter  MASS_SHIFTER's status in AUV
 * @param thruster  THRUSTER's status in AUV
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_custom_msg_board_status_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t arm1, uint8_t arm2, uint8_t viam_navy, uint8_t pistol, uint8_t mass_shifter, uint8_t thruster)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint8_t(buf, 4, arm1);
    _mav_put_uint8_t(buf, 5, arm2);
    _mav_put_uint8_t(buf, 6, viam_navy);
    _mav_put_uint8_t(buf, 7, pistol);
    _mav_put_uint8_t(buf, 8, mass_shifter);
    _mav_put_uint8_t(buf, 9, thruster);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS, buf, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_CRC);
#else
    mavlink_custom_msg_board_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.arm1 = arm1;
    packet.arm2 = arm2;
    packet.viam_navy = viam_navy;
    packet.pistol = pistol;
    packet.mass_shifter = mass_shifter;
    packet.thruster = thruster;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS, (const char *)&packet, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_CRC);
#endif
}

/**
 * @brief Send a custom_msg_board_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_custom_msg_board_status_send_struct(mavlink_channel_t chan, const mavlink_custom_msg_board_status_t* custom_msg_board_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_custom_msg_board_status_send(chan, custom_msg_board_status->time_boot_ms, custom_msg_board_status->arm1, custom_msg_board_status->arm2, custom_msg_board_status->viam_navy, custom_msg_board_status->pistol, custom_msg_board_status->mass_shifter, custom_msg_board_status->thruster);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS, (const char *)custom_msg_board_status, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_custom_msg_board_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t arm1, uint8_t arm2, uint8_t viam_navy, uint8_t pistol, uint8_t mass_shifter, uint8_t thruster)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint8_t(buf, 4, arm1);
    _mav_put_uint8_t(buf, 5, arm2);
    _mav_put_uint8_t(buf, 6, viam_navy);
    _mav_put_uint8_t(buf, 7, pistol);
    _mav_put_uint8_t(buf, 8, mass_shifter);
    _mav_put_uint8_t(buf, 9, thruster);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS, buf, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_CRC);
#else
    mavlink_custom_msg_board_status_t *packet = (mavlink_custom_msg_board_status_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->arm1 = arm1;
    packet->arm2 = arm2;
    packet->viam_navy = viam_navy;
    packet->pistol = pistol;
    packet->mass_shifter = mass_shifter;
    packet->thruster = thruster;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS, (const char *)packet, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE CUSTOM_MSG_BOARD_STATUS UNPACKING


/**
 * @brief Get field time_boot_ms from custom_msg_board_status message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_custom_msg_board_status_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field arm1 from custom_msg_board_status message
 *
 * @return  ARM1's status in AUV
 */
static inline uint8_t mavlink_msg_custom_msg_board_status_get_arm1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field arm2 from custom_msg_board_status message
 *
 * @return  ARM2's status in AUV
 */
static inline uint8_t mavlink_msg_custom_msg_board_status_get_arm2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field viam_navy from custom_msg_board_status message
 *
 * @return  VIAM_NAVY's status in AUV
 */
static inline uint8_t mavlink_msg_custom_msg_board_status_get_viam_navy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field pistol from custom_msg_board_status message
 *
 * @return  PISTOL's status in AUV
 */
static inline uint8_t mavlink_msg_custom_msg_board_status_get_pistol(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field mass_shifter from custom_msg_board_status message
 *
 * @return  MASS_SHIFTER's status in AUV
 */
static inline uint8_t mavlink_msg_custom_msg_board_status_get_mass_shifter(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field thruster from custom_msg_board_status message
 *
 * @return  THRUSTER's status in AUV
 */
static inline uint8_t mavlink_msg_custom_msg_board_status_get_thruster(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Decode a custom_msg_board_status message into a struct
 *
 * @param msg The message to decode
 * @param custom_msg_board_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_custom_msg_board_status_decode(const mavlink_message_t* msg, mavlink_custom_msg_board_status_t* custom_msg_board_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    custom_msg_board_status->time_boot_ms = mavlink_msg_custom_msg_board_status_get_time_boot_ms(msg);
    custom_msg_board_status->arm1 = mavlink_msg_custom_msg_board_status_get_arm1(msg);
    custom_msg_board_status->arm2 = mavlink_msg_custom_msg_board_status_get_arm2(msg);
    custom_msg_board_status->viam_navy = mavlink_msg_custom_msg_board_status_get_viam_navy(msg);
    custom_msg_board_status->pistol = mavlink_msg_custom_msg_board_status_get_pistol(msg);
    custom_msg_board_status->mass_shifter = mavlink_msg_custom_msg_board_status_get_mass_shifter(msg);
    custom_msg_board_status->thruster = mavlink_msg_custom_msg_board_status_get_thruster(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN? msg->len : MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN;
        memset(custom_msg_board_status, 0, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_LEN);
    memcpy(custom_msg_board_status, _MAV_PAYLOAD(msg), len);
#endif
}
