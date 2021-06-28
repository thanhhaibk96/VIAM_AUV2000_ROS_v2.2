#pragma once
// MESSAGE CUSTOM_MSG_THRUSTER PACKING

#define MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER 398


typedef struct __mavlink_custom_msg_thruster_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float motor_duty; /*<  The duty cycle (PWM) of thruster's motor*/
 float motor_temp_on_chip; /*<  The temperature of thruster's motor*/
 float motor_temp_ambient; /*<  The temperature of thruster's motor*/
 float motor_current; /*<  The current of thruster's motor*/
 float motor_rspeed; /*<  The current speed of mthruster's motor*/
 float motor_dspeed; /*<  The desired speed of thruster's motor*/
} mavlink_custom_msg_thruster_t;

#define MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN 28
#define MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_MIN_LEN 28
#define MAVLINK_MSG_ID_398_LEN 28
#define MAVLINK_MSG_ID_398_MIN_LEN 28

#define MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_CRC 49
#define MAVLINK_MSG_ID_398_CRC 49



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CUSTOM_MSG_THRUSTER { \
    398, \
    "CUSTOM_MSG_THRUSTER", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_custom_msg_thruster_t, time_boot_ms) }, \
         { "motor_duty", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_custom_msg_thruster_t, motor_duty) }, \
         { "motor_temp_on_chip", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_custom_msg_thruster_t, motor_temp_on_chip) }, \
         { "motor_temp_ambient", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_custom_msg_thruster_t, motor_temp_ambient) }, \
         { "motor_current", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_custom_msg_thruster_t, motor_current) }, \
         { "motor_rspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_custom_msg_thruster_t, motor_rspeed) }, \
         { "motor_dspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_custom_msg_thruster_t, motor_dspeed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CUSTOM_MSG_THRUSTER { \
    "CUSTOM_MSG_THRUSTER", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_custom_msg_thruster_t, time_boot_ms) }, \
         { "motor_duty", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_custom_msg_thruster_t, motor_duty) }, \
         { "motor_temp_on_chip", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_custom_msg_thruster_t, motor_temp_on_chip) }, \
         { "motor_temp_ambient", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_custom_msg_thruster_t, motor_temp_ambient) }, \
         { "motor_current", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_custom_msg_thruster_t, motor_current) }, \
         { "motor_rspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_custom_msg_thruster_t, motor_rspeed) }, \
         { "motor_dspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_custom_msg_thruster_t, motor_dspeed) }, \
         } \
}
#endif

/**
 * @brief Pack a custom_msg_thruster message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param motor_duty  The duty cycle (PWM) of thruster's motor
 * @param motor_temp_on_chip  The temperature of thruster's motor
 * @param motor_temp_ambient  The temperature of thruster's motor
 * @param motor_current  The current of thruster's motor
 * @param motor_rspeed  The current speed of mthruster's motor
 * @param motor_dspeed  The desired speed of thruster's motor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_custom_msg_thruster_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float motor_duty, float motor_temp_on_chip, float motor_temp_ambient, float motor_current, float motor_rspeed, float motor_dspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, motor_duty);
    _mav_put_float(buf, 8, motor_temp_on_chip);
    _mav_put_float(buf, 12, motor_temp_ambient);
    _mav_put_float(buf, 16, motor_current);
    _mav_put_float(buf, 20, motor_rspeed);
    _mav_put_float(buf, 24, motor_dspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN);
#else
    mavlink_custom_msg_thruster_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.motor_duty = motor_duty;
    packet.motor_temp_on_chip = motor_temp_on_chip;
    packet.motor_temp_ambient = motor_temp_ambient;
    packet.motor_current = motor_current;
    packet.motor_rspeed = motor_rspeed;
    packet.motor_dspeed = motor_dspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_CRC);
}

/**
 * @brief Pack a custom_msg_thruster message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param motor_duty  The duty cycle (PWM) of thruster's motor
 * @param motor_temp_on_chip  The temperature of thruster's motor
 * @param motor_temp_ambient  The temperature of thruster's motor
 * @param motor_current  The current of thruster's motor
 * @param motor_rspeed  The current speed of mthruster's motor
 * @param motor_dspeed  The desired speed of thruster's motor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_custom_msg_thruster_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float motor_duty,float motor_temp_on_chip,float motor_temp_ambient,float motor_current,float motor_rspeed,float motor_dspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, motor_duty);
    _mav_put_float(buf, 8, motor_temp_on_chip);
    _mav_put_float(buf, 12, motor_temp_ambient);
    _mav_put_float(buf, 16, motor_current);
    _mav_put_float(buf, 20, motor_rspeed);
    _mav_put_float(buf, 24, motor_dspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN);
#else
    mavlink_custom_msg_thruster_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.motor_duty = motor_duty;
    packet.motor_temp_on_chip = motor_temp_on_chip;
    packet.motor_temp_ambient = motor_temp_ambient;
    packet.motor_current = motor_current;
    packet.motor_rspeed = motor_rspeed;
    packet.motor_dspeed = motor_dspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_CRC);
}

/**
 * @brief Encode a custom_msg_thruster struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param custom_msg_thruster C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_custom_msg_thruster_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_custom_msg_thruster_t* custom_msg_thruster)
{
    return mavlink_msg_custom_msg_thruster_pack(system_id, component_id, msg, custom_msg_thruster->time_boot_ms, custom_msg_thruster->motor_duty, custom_msg_thruster->motor_temp_on_chip, custom_msg_thruster->motor_temp_ambient, custom_msg_thruster->motor_current, custom_msg_thruster->motor_rspeed, custom_msg_thruster->motor_dspeed);
}

/**
 * @brief Encode a custom_msg_thruster struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param custom_msg_thruster C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_custom_msg_thruster_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_custom_msg_thruster_t* custom_msg_thruster)
{
    return mavlink_msg_custom_msg_thruster_pack_chan(system_id, component_id, chan, msg, custom_msg_thruster->time_boot_ms, custom_msg_thruster->motor_duty, custom_msg_thruster->motor_temp_on_chip, custom_msg_thruster->motor_temp_ambient, custom_msg_thruster->motor_current, custom_msg_thruster->motor_rspeed, custom_msg_thruster->motor_dspeed);
}

/**
 * @brief Send a custom_msg_thruster message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param motor_duty  The duty cycle (PWM) of thruster's motor
 * @param motor_temp_on_chip  The temperature of thruster's motor
 * @param motor_temp_ambient  The temperature of thruster's motor
 * @param motor_current  The current of thruster's motor
 * @param motor_rspeed  The current speed of mthruster's motor
 * @param motor_dspeed  The desired speed of thruster's motor
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_custom_msg_thruster_send(mavlink_channel_t chan, uint32_t time_boot_ms, float motor_duty, float motor_temp_on_chip, float motor_temp_ambient, float motor_current, float motor_rspeed, float motor_dspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, motor_duty);
    _mav_put_float(buf, 8, motor_temp_on_chip);
    _mav_put_float(buf, 12, motor_temp_ambient);
    _mav_put_float(buf, 16, motor_current);
    _mav_put_float(buf, 20, motor_rspeed);
    _mav_put_float(buf, 24, motor_dspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER, buf, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_CRC);
#else
    mavlink_custom_msg_thruster_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.motor_duty = motor_duty;
    packet.motor_temp_on_chip = motor_temp_on_chip;
    packet.motor_temp_ambient = motor_temp_ambient;
    packet.motor_current = motor_current;
    packet.motor_rspeed = motor_rspeed;
    packet.motor_dspeed = motor_dspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER, (const char *)&packet, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_CRC);
#endif
}

/**
 * @brief Send a custom_msg_thruster message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_custom_msg_thruster_send_struct(mavlink_channel_t chan, const mavlink_custom_msg_thruster_t* custom_msg_thruster)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_custom_msg_thruster_send(chan, custom_msg_thruster->time_boot_ms, custom_msg_thruster->motor_duty, custom_msg_thruster->motor_temp_on_chip, custom_msg_thruster->motor_temp_ambient, custom_msg_thruster->motor_current, custom_msg_thruster->motor_rspeed, custom_msg_thruster->motor_dspeed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER, (const char *)custom_msg_thruster, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_CRC);
#endif
}

#if MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_custom_msg_thruster_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float motor_duty, float motor_temp_on_chip, float motor_temp_ambient, float motor_current, float motor_rspeed, float motor_dspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, motor_duty);
    _mav_put_float(buf, 8, motor_temp_on_chip);
    _mav_put_float(buf, 12, motor_temp_ambient);
    _mav_put_float(buf, 16, motor_current);
    _mav_put_float(buf, 20, motor_rspeed);
    _mav_put_float(buf, 24, motor_dspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER, buf, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_CRC);
#else
    mavlink_custom_msg_thruster_t *packet = (mavlink_custom_msg_thruster_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->motor_duty = motor_duty;
    packet->motor_temp_on_chip = motor_temp_on_chip;
    packet->motor_temp_ambient = motor_temp_ambient;
    packet->motor_current = motor_current;
    packet->motor_rspeed = motor_rspeed;
    packet->motor_dspeed = motor_dspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER, (const char *)packet, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_CRC);
#endif
}
#endif

#endif

// MESSAGE CUSTOM_MSG_THRUSTER UNPACKING


/**
 * @brief Get field time_boot_ms from custom_msg_thruster message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_custom_msg_thruster_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field motor_duty from custom_msg_thruster message
 *
 * @return  The duty cycle (PWM) of thruster's motor
 */
static inline float mavlink_msg_custom_msg_thruster_get_motor_duty(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field motor_temp_on_chip from custom_msg_thruster message
 *
 * @return  The temperature of thruster's motor
 */
static inline float mavlink_msg_custom_msg_thruster_get_motor_temp_on_chip(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field motor_temp_ambient from custom_msg_thruster message
 *
 * @return  The temperature of thruster's motor
 */
static inline float mavlink_msg_custom_msg_thruster_get_motor_temp_ambient(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field motor_current from custom_msg_thruster message
 *
 * @return  The current of thruster's motor
 */
static inline float mavlink_msg_custom_msg_thruster_get_motor_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field motor_rspeed from custom_msg_thruster message
 *
 * @return  The current speed of mthruster's motor
 */
static inline float mavlink_msg_custom_msg_thruster_get_motor_rspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field motor_dspeed from custom_msg_thruster message
 *
 * @return  The desired speed of thruster's motor
 */
static inline float mavlink_msg_custom_msg_thruster_get_motor_dspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a custom_msg_thruster message into a struct
 *
 * @param msg The message to decode
 * @param custom_msg_thruster C-struct to decode the message contents into
 */
static inline void mavlink_msg_custom_msg_thruster_decode(const mavlink_message_t* msg, mavlink_custom_msg_thruster_t* custom_msg_thruster)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    custom_msg_thruster->time_boot_ms = mavlink_msg_custom_msg_thruster_get_time_boot_ms(msg);
    custom_msg_thruster->motor_duty = mavlink_msg_custom_msg_thruster_get_motor_duty(msg);
    custom_msg_thruster->motor_temp_on_chip = mavlink_msg_custom_msg_thruster_get_motor_temp_on_chip(msg);
    custom_msg_thruster->motor_temp_ambient = mavlink_msg_custom_msg_thruster_get_motor_temp_ambient(msg);
    custom_msg_thruster->motor_current = mavlink_msg_custom_msg_thruster_get_motor_current(msg);
    custom_msg_thruster->motor_rspeed = mavlink_msg_custom_msg_thruster_get_motor_rspeed(msg);
    custom_msg_thruster->motor_dspeed = mavlink_msg_custom_msg_thruster_get_motor_dspeed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN? msg->len : MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN;
        memset(custom_msg_thruster, 0, MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_LEN);
    memcpy(custom_msg_thruster, _MAV_PAYLOAD(msg), len);
#endif
}
