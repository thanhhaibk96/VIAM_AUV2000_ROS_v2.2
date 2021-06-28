#pragma once
// MESSAGE CUSTOM_MSG_BOARD_ARM1 PACKING

#define MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1 399


typedef struct __mavlink_custom_msg_board_arm1_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float altimeter_in_metres; /*<  Depth in metres*/
 float altimeter_in_feet; /*<  Depth in feet*/
 float altimeter_in_fathoms; /*<  Depth in fathoms*/
 float roll_motor_angle; /*<  Angle of Motor for Antiing Roll*/
 uint8_t leak_sensor; /*<  Value of Leak Sensors*/
} mavlink_custom_msg_board_arm1_t;

#define MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN 21
#define MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_MIN_LEN 21
#define MAVLINK_MSG_ID_399_LEN 21
#define MAVLINK_MSG_ID_399_MIN_LEN 21

#define MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_CRC 71
#define MAVLINK_MSG_ID_399_CRC 71



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CUSTOM_MSG_BOARD_ARM1 { \
    399, \
    "CUSTOM_MSG_BOARD_ARM1", \
    6, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_custom_msg_board_arm1_t, time_boot_ms) }, \
         { "leak_sensor", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_custom_msg_board_arm1_t, leak_sensor) }, \
         { "altimeter_in_metres", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_custom_msg_board_arm1_t, altimeter_in_metres) }, \
         { "altimeter_in_feet", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_custom_msg_board_arm1_t, altimeter_in_feet) }, \
         { "altimeter_in_fathoms", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_custom_msg_board_arm1_t, altimeter_in_fathoms) }, \
         { "roll_motor_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_custom_msg_board_arm1_t, roll_motor_angle) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CUSTOM_MSG_BOARD_ARM1 { \
    "CUSTOM_MSG_BOARD_ARM1", \
    6, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_custom_msg_board_arm1_t, time_boot_ms) }, \
         { "leak_sensor", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_custom_msg_board_arm1_t, leak_sensor) }, \
         { "altimeter_in_metres", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_custom_msg_board_arm1_t, altimeter_in_metres) }, \
         { "altimeter_in_feet", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_custom_msg_board_arm1_t, altimeter_in_feet) }, \
         { "altimeter_in_fathoms", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_custom_msg_board_arm1_t, altimeter_in_fathoms) }, \
         { "roll_motor_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_custom_msg_board_arm1_t, roll_motor_angle) }, \
         } \
}
#endif

/**
 * @brief Pack a custom_msg_board_arm1 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param leak_sensor  Value of Leak Sensors
 * @param altimeter_in_metres  Depth in metres
 * @param altimeter_in_feet  Depth in feet
 * @param altimeter_in_fathoms  Depth in fathoms
 * @param roll_motor_angle  Angle of Motor for Antiing Roll
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_custom_msg_board_arm1_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t leak_sensor, float altimeter_in_metres, float altimeter_in_feet, float altimeter_in_fathoms, float roll_motor_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, altimeter_in_metres);
    _mav_put_float(buf, 8, altimeter_in_feet);
    _mav_put_float(buf, 12, altimeter_in_fathoms);
    _mav_put_float(buf, 16, roll_motor_angle);
    _mav_put_uint8_t(buf, 20, leak_sensor);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN);
#else
    mavlink_custom_msg_board_arm1_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.altimeter_in_metres = altimeter_in_metres;
    packet.altimeter_in_feet = altimeter_in_feet;
    packet.altimeter_in_fathoms = altimeter_in_fathoms;
    packet.roll_motor_angle = roll_motor_angle;
    packet.leak_sensor = leak_sensor;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_CRC);
}

/**
 * @brief Pack a custom_msg_board_arm1 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param leak_sensor  Value of Leak Sensors
 * @param altimeter_in_metres  Depth in metres
 * @param altimeter_in_feet  Depth in feet
 * @param altimeter_in_fathoms  Depth in fathoms
 * @param roll_motor_angle  Angle of Motor for Antiing Roll
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_custom_msg_board_arm1_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t leak_sensor,float altimeter_in_metres,float altimeter_in_feet,float altimeter_in_fathoms,float roll_motor_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, altimeter_in_metres);
    _mav_put_float(buf, 8, altimeter_in_feet);
    _mav_put_float(buf, 12, altimeter_in_fathoms);
    _mav_put_float(buf, 16, roll_motor_angle);
    _mav_put_uint8_t(buf, 20, leak_sensor);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN);
#else
    mavlink_custom_msg_board_arm1_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.altimeter_in_metres = altimeter_in_metres;
    packet.altimeter_in_feet = altimeter_in_feet;
    packet.altimeter_in_fathoms = altimeter_in_fathoms;
    packet.roll_motor_angle = roll_motor_angle;
    packet.leak_sensor = leak_sensor;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_CRC);
}

/**
 * @brief Encode a custom_msg_board_arm1 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param custom_msg_board_arm1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_custom_msg_board_arm1_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_custom_msg_board_arm1_t* custom_msg_board_arm1)
{
    return mavlink_msg_custom_msg_board_arm1_pack(system_id, component_id, msg, custom_msg_board_arm1->time_boot_ms, custom_msg_board_arm1->leak_sensor, custom_msg_board_arm1->altimeter_in_metres, custom_msg_board_arm1->altimeter_in_feet, custom_msg_board_arm1->altimeter_in_fathoms, custom_msg_board_arm1->roll_motor_angle);
}

/**
 * @brief Encode a custom_msg_board_arm1 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param custom_msg_board_arm1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_custom_msg_board_arm1_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_custom_msg_board_arm1_t* custom_msg_board_arm1)
{
    return mavlink_msg_custom_msg_board_arm1_pack_chan(system_id, component_id, chan, msg, custom_msg_board_arm1->time_boot_ms, custom_msg_board_arm1->leak_sensor, custom_msg_board_arm1->altimeter_in_metres, custom_msg_board_arm1->altimeter_in_feet, custom_msg_board_arm1->altimeter_in_fathoms, custom_msg_board_arm1->roll_motor_angle);
}

/**
 * @brief Send a custom_msg_board_arm1 message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param leak_sensor  Value of Leak Sensors
 * @param altimeter_in_metres  Depth in metres
 * @param altimeter_in_feet  Depth in feet
 * @param altimeter_in_fathoms  Depth in fathoms
 * @param roll_motor_angle  Angle of Motor for Antiing Roll
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_custom_msg_board_arm1_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t leak_sensor, float altimeter_in_metres, float altimeter_in_feet, float altimeter_in_fathoms, float roll_motor_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, altimeter_in_metres);
    _mav_put_float(buf, 8, altimeter_in_feet);
    _mav_put_float(buf, 12, altimeter_in_fathoms);
    _mav_put_float(buf, 16, roll_motor_angle);
    _mav_put_uint8_t(buf, 20, leak_sensor);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1, buf, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_CRC);
#else
    mavlink_custom_msg_board_arm1_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.altimeter_in_metres = altimeter_in_metres;
    packet.altimeter_in_feet = altimeter_in_feet;
    packet.altimeter_in_fathoms = altimeter_in_fathoms;
    packet.roll_motor_angle = roll_motor_angle;
    packet.leak_sensor = leak_sensor;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1, (const char *)&packet, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_CRC);
#endif
}

/**
 * @brief Send a custom_msg_board_arm1 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_custom_msg_board_arm1_send_struct(mavlink_channel_t chan, const mavlink_custom_msg_board_arm1_t* custom_msg_board_arm1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_custom_msg_board_arm1_send(chan, custom_msg_board_arm1->time_boot_ms, custom_msg_board_arm1->leak_sensor, custom_msg_board_arm1->altimeter_in_metres, custom_msg_board_arm1->altimeter_in_feet, custom_msg_board_arm1->altimeter_in_fathoms, custom_msg_board_arm1->roll_motor_angle);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1, (const char *)custom_msg_board_arm1, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_CRC);
#endif
}

#if MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_custom_msg_board_arm1_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t leak_sensor, float altimeter_in_metres, float altimeter_in_feet, float altimeter_in_fathoms, float roll_motor_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, altimeter_in_metres);
    _mav_put_float(buf, 8, altimeter_in_feet);
    _mav_put_float(buf, 12, altimeter_in_fathoms);
    _mav_put_float(buf, 16, roll_motor_angle);
    _mav_put_uint8_t(buf, 20, leak_sensor);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1, buf, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_CRC);
#else
    mavlink_custom_msg_board_arm1_t *packet = (mavlink_custom_msg_board_arm1_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->altimeter_in_metres = altimeter_in_metres;
    packet->altimeter_in_feet = altimeter_in_feet;
    packet->altimeter_in_fathoms = altimeter_in_fathoms;
    packet->roll_motor_angle = roll_motor_angle;
    packet->leak_sensor = leak_sensor;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1, (const char *)packet, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_CRC);
#endif
}
#endif

#endif

// MESSAGE CUSTOM_MSG_BOARD_ARM1 UNPACKING


/**
 * @brief Get field time_boot_ms from custom_msg_board_arm1 message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_custom_msg_board_arm1_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field leak_sensor from custom_msg_board_arm1 message
 *
 * @return  Value of Leak Sensors
 */
static inline uint8_t mavlink_msg_custom_msg_board_arm1_get_leak_sensor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field altimeter_in_metres from custom_msg_board_arm1 message
 *
 * @return  Depth in metres
 */
static inline float mavlink_msg_custom_msg_board_arm1_get_altimeter_in_metres(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field altimeter_in_feet from custom_msg_board_arm1 message
 *
 * @return  Depth in feet
 */
static inline float mavlink_msg_custom_msg_board_arm1_get_altimeter_in_feet(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field altimeter_in_fathoms from custom_msg_board_arm1 message
 *
 * @return  Depth in fathoms
 */
static inline float mavlink_msg_custom_msg_board_arm1_get_altimeter_in_fathoms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field roll_motor_angle from custom_msg_board_arm1 message
 *
 * @return  Angle of Motor for Antiing Roll
 */
static inline float mavlink_msg_custom_msg_board_arm1_get_roll_motor_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a custom_msg_board_arm1 message into a struct
 *
 * @param msg The message to decode
 * @param custom_msg_board_arm1 C-struct to decode the message contents into
 */
static inline void mavlink_msg_custom_msg_board_arm1_decode(const mavlink_message_t* msg, mavlink_custom_msg_board_arm1_t* custom_msg_board_arm1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    custom_msg_board_arm1->time_boot_ms = mavlink_msg_custom_msg_board_arm1_get_time_boot_ms(msg);
    custom_msg_board_arm1->altimeter_in_metres = mavlink_msg_custom_msg_board_arm1_get_altimeter_in_metres(msg);
    custom_msg_board_arm1->altimeter_in_feet = mavlink_msg_custom_msg_board_arm1_get_altimeter_in_feet(msg);
    custom_msg_board_arm1->altimeter_in_fathoms = mavlink_msg_custom_msg_board_arm1_get_altimeter_in_fathoms(msg);
    custom_msg_board_arm1->roll_motor_angle = mavlink_msg_custom_msg_board_arm1_get_roll_motor_angle(msg);
    custom_msg_board_arm1->leak_sensor = mavlink_msg_custom_msg_board_arm1_get_leak_sensor(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN? msg->len : MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN;
        memset(custom_msg_board_arm1, 0, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_LEN);
    memcpy(custom_msg_board_arm1, _MAV_PAYLOAD(msg), len);
#endif
}
