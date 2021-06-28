#pragma once
// MESSAGE CUSTOM_MSG_BOARD_ARM2 PACKING

#define MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2 402


typedef struct __mavlink_custom_msg_board_arm2_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float rudder_position; /*<  Position of Rudder*/
 float rudder_speed; /*<  Speed of Rudder*/
 float rudder_load; /*<  Load of Rudder*/
 float rudder_voltage; /*<  Voltage of Rudder*/
 float rudder_temperature; /*<  Temperature of Rudder*/
 float keller_pa3_pressure; /*<  Pressure of Keller PA3*/
 float keller_pa3_temperature; /*<  Temperature of Keller PA3*/
} mavlink_custom_msg_board_arm2_t;

#define MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN 32
#define MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_MIN_LEN 32
#define MAVLINK_MSG_ID_402_LEN 32
#define MAVLINK_MSG_ID_402_MIN_LEN 32

#define MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_CRC 237
#define MAVLINK_MSG_ID_402_CRC 237



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CUSTOM_MSG_BOARD_ARM2 { \
    402, \
    "CUSTOM_MSG_BOARD_ARM2", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_custom_msg_board_arm2_t, time_boot_ms) }, \
         { "rudder_position", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_custom_msg_board_arm2_t, rudder_position) }, \
         { "rudder_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_custom_msg_board_arm2_t, rudder_speed) }, \
         { "rudder_load", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_custom_msg_board_arm2_t, rudder_load) }, \
         { "rudder_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_custom_msg_board_arm2_t, rudder_voltage) }, \
         { "rudder_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_custom_msg_board_arm2_t, rudder_temperature) }, \
         { "keller_pa3_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_custom_msg_board_arm2_t, keller_pa3_pressure) }, \
         { "keller_pa3_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_custom_msg_board_arm2_t, keller_pa3_temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CUSTOM_MSG_BOARD_ARM2 { \
    "CUSTOM_MSG_BOARD_ARM2", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_custom_msg_board_arm2_t, time_boot_ms) }, \
         { "rudder_position", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_custom_msg_board_arm2_t, rudder_position) }, \
         { "rudder_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_custom_msg_board_arm2_t, rudder_speed) }, \
         { "rudder_load", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_custom_msg_board_arm2_t, rudder_load) }, \
         { "rudder_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_custom_msg_board_arm2_t, rudder_voltage) }, \
         { "rudder_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_custom_msg_board_arm2_t, rudder_temperature) }, \
         { "keller_pa3_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_custom_msg_board_arm2_t, keller_pa3_pressure) }, \
         { "keller_pa3_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_custom_msg_board_arm2_t, keller_pa3_temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a custom_msg_board_arm2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param rudder_position  Position of Rudder
 * @param rudder_speed  Speed of Rudder
 * @param rudder_load  Load of Rudder
 * @param rudder_voltage  Voltage of Rudder
 * @param rudder_temperature  Temperature of Rudder
 * @param keller_pa3_pressure  Pressure of Keller PA3
 * @param keller_pa3_temperature  Temperature of Keller PA3
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_custom_msg_board_arm2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float rudder_position, float rudder_speed, float rudder_load, float rudder_voltage, float rudder_temperature, float keller_pa3_pressure, float keller_pa3_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, rudder_position);
    _mav_put_float(buf, 8, rudder_speed);
    _mav_put_float(buf, 12, rudder_load);
    _mav_put_float(buf, 16, rudder_voltage);
    _mav_put_float(buf, 20, rudder_temperature);
    _mav_put_float(buf, 24, keller_pa3_pressure);
    _mav_put_float(buf, 28, keller_pa3_temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN);
#else
    mavlink_custom_msg_board_arm2_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.rudder_position = rudder_position;
    packet.rudder_speed = rudder_speed;
    packet.rudder_load = rudder_load;
    packet.rudder_voltage = rudder_voltage;
    packet.rudder_temperature = rudder_temperature;
    packet.keller_pa3_pressure = keller_pa3_pressure;
    packet.keller_pa3_temperature = keller_pa3_temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_CRC);
}

/**
 * @brief Pack a custom_msg_board_arm2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param rudder_position  Position of Rudder
 * @param rudder_speed  Speed of Rudder
 * @param rudder_load  Load of Rudder
 * @param rudder_voltage  Voltage of Rudder
 * @param rudder_temperature  Temperature of Rudder
 * @param keller_pa3_pressure  Pressure of Keller PA3
 * @param keller_pa3_temperature  Temperature of Keller PA3
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_custom_msg_board_arm2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float rudder_position,float rudder_speed,float rudder_load,float rudder_voltage,float rudder_temperature,float keller_pa3_pressure,float keller_pa3_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, rudder_position);
    _mav_put_float(buf, 8, rudder_speed);
    _mav_put_float(buf, 12, rudder_load);
    _mav_put_float(buf, 16, rudder_voltage);
    _mav_put_float(buf, 20, rudder_temperature);
    _mav_put_float(buf, 24, keller_pa3_pressure);
    _mav_put_float(buf, 28, keller_pa3_temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN);
#else
    mavlink_custom_msg_board_arm2_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.rudder_position = rudder_position;
    packet.rudder_speed = rudder_speed;
    packet.rudder_load = rudder_load;
    packet.rudder_voltage = rudder_voltage;
    packet.rudder_temperature = rudder_temperature;
    packet.keller_pa3_pressure = keller_pa3_pressure;
    packet.keller_pa3_temperature = keller_pa3_temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_CRC);
}

/**
 * @brief Encode a custom_msg_board_arm2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param custom_msg_board_arm2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_custom_msg_board_arm2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_custom_msg_board_arm2_t* custom_msg_board_arm2)
{
    return mavlink_msg_custom_msg_board_arm2_pack(system_id, component_id, msg, custom_msg_board_arm2->time_boot_ms, custom_msg_board_arm2->rudder_position, custom_msg_board_arm2->rudder_speed, custom_msg_board_arm2->rudder_load, custom_msg_board_arm2->rudder_voltage, custom_msg_board_arm2->rudder_temperature, custom_msg_board_arm2->keller_pa3_pressure, custom_msg_board_arm2->keller_pa3_temperature);
}

/**
 * @brief Encode a custom_msg_board_arm2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param custom_msg_board_arm2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_custom_msg_board_arm2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_custom_msg_board_arm2_t* custom_msg_board_arm2)
{
    return mavlink_msg_custom_msg_board_arm2_pack_chan(system_id, component_id, chan, msg, custom_msg_board_arm2->time_boot_ms, custom_msg_board_arm2->rudder_position, custom_msg_board_arm2->rudder_speed, custom_msg_board_arm2->rudder_load, custom_msg_board_arm2->rudder_voltage, custom_msg_board_arm2->rudder_temperature, custom_msg_board_arm2->keller_pa3_pressure, custom_msg_board_arm2->keller_pa3_temperature);
}

/**
 * @brief Send a custom_msg_board_arm2 message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param rudder_position  Position of Rudder
 * @param rudder_speed  Speed of Rudder
 * @param rudder_load  Load of Rudder
 * @param rudder_voltage  Voltage of Rudder
 * @param rudder_temperature  Temperature of Rudder
 * @param keller_pa3_pressure  Pressure of Keller PA3
 * @param keller_pa3_temperature  Temperature of Keller PA3
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_custom_msg_board_arm2_send(mavlink_channel_t chan, uint32_t time_boot_ms, float rudder_position, float rudder_speed, float rudder_load, float rudder_voltage, float rudder_temperature, float keller_pa3_pressure, float keller_pa3_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, rudder_position);
    _mav_put_float(buf, 8, rudder_speed);
    _mav_put_float(buf, 12, rudder_load);
    _mav_put_float(buf, 16, rudder_voltage);
    _mav_put_float(buf, 20, rudder_temperature);
    _mav_put_float(buf, 24, keller_pa3_pressure);
    _mav_put_float(buf, 28, keller_pa3_temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2, buf, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_CRC);
#else
    mavlink_custom_msg_board_arm2_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.rudder_position = rudder_position;
    packet.rudder_speed = rudder_speed;
    packet.rudder_load = rudder_load;
    packet.rudder_voltage = rudder_voltage;
    packet.rudder_temperature = rudder_temperature;
    packet.keller_pa3_pressure = keller_pa3_pressure;
    packet.keller_pa3_temperature = keller_pa3_temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2, (const char *)&packet, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_CRC);
#endif
}

/**
 * @brief Send a custom_msg_board_arm2 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_custom_msg_board_arm2_send_struct(mavlink_channel_t chan, const mavlink_custom_msg_board_arm2_t* custom_msg_board_arm2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_custom_msg_board_arm2_send(chan, custom_msg_board_arm2->time_boot_ms, custom_msg_board_arm2->rudder_position, custom_msg_board_arm2->rudder_speed, custom_msg_board_arm2->rudder_load, custom_msg_board_arm2->rudder_voltage, custom_msg_board_arm2->rudder_temperature, custom_msg_board_arm2->keller_pa3_pressure, custom_msg_board_arm2->keller_pa3_temperature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2, (const char *)custom_msg_board_arm2, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_CRC);
#endif
}

#if MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_custom_msg_board_arm2_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float rudder_position, float rudder_speed, float rudder_load, float rudder_voltage, float rudder_temperature, float keller_pa3_pressure, float keller_pa3_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, rudder_position);
    _mav_put_float(buf, 8, rudder_speed);
    _mav_put_float(buf, 12, rudder_load);
    _mav_put_float(buf, 16, rudder_voltage);
    _mav_put_float(buf, 20, rudder_temperature);
    _mav_put_float(buf, 24, keller_pa3_pressure);
    _mav_put_float(buf, 28, keller_pa3_temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2, buf, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_CRC);
#else
    mavlink_custom_msg_board_arm2_t *packet = (mavlink_custom_msg_board_arm2_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->rudder_position = rudder_position;
    packet->rudder_speed = rudder_speed;
    packet->rudder_load = rudder_load;
    packet->rudder_voltage = rudder_voltage;
    packet->rudder_temperature = rudder_temperature;
    packet->keller_pa3_pressure = keller_pa3_pressure;
    packet->keller_pa3_temperature = keller_pa3_temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2, (const char *)packet, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_MIN_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_CRC);
#endif
}
#endif

#endif

// MESSAGE CUSTOM_MSG_BOARD_ARM2 UNPACKING


/**
 * @brief Get field time_boot_ms from custom_msg_board_arm2 message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_custom_msg_board_arm2_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field rudder_position from custom_msg_board_arm2 message
 *
 * @return  Position of Rudder
 */
static inline float mavlink_msg_custom_msg_board_arm2_get_rudder_position(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field rudder_speed from custom_msg_board_arm2 message
 *
 * @return  Speed of Rudder
 */
static inline float mavlink_msg_custom_msg_board_arm2_get_rudder_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field rudder_load from custom_msg_board_arm2 message
 *
 * @return  Load of Rudder
 */
static inline float mavlink_msg_custom_msg_board_arm2_get_rudder_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field rudder_voltage from custom_msg_board_arm2 message
 *
 * @return  Voltage of Rudder
 */
static inline float mavlink_msg_custom_msg_board_arm2_get_rudder_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field rudder_temperature from custom_msg_board_arm2 message
 *
 * @return  Temperature of Rudder
 */
static inline float mavlink_msg_custom_msg_board_arm2_get_rudder_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field keller_pa3_pressure from custom_msg_board_arm2 message
 *
 * @return  Pressure of Keller PA3
 */
static inline float mavlink_msg_custom_msg_board_arm2_get_keller_pa3_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field keller_pa3_temperature from custom_msg_board_arm2 message
 *
 * @return  Temperature of Keller PA3
 */
static inline float mavlink_msg_custom_msg_board_arm2_get_keller_pa3_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a custom_msg_board_arm2 message into a struct
 *
 * @param msg The message to decode
 * @param custom_msg_board_arm2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_custom_msg_board_arm2_decode(const mavlink_message_t* msg, mavlink_custom_msg_board_arm2_t* custom_msg_board_arm2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    custom_msg_board_arm2->time_boot_ms = mavlink_msg_custom_msg_board_arm2_get_time_boot_ms(msg);
    custom_msg_board_arm2->rudder_position = mavlink_msg_custom_msg_board_arm2_get_rudder_position(msg);
    custom_msg_board_arm2->rudder_speed = mavlink_msg_custom_msg_board_arm2_get_rudder_speed(msg);
    custom_msg_board_arm2->rudder_load = mavlink_msg_custom_msg_board_arm2_get_rudder_load(msg);
    custom_msg_board_arm2->rudder_voltage = mavlink_msg_custom_msg_board_arm2_get_rudder_voltage(msg);
    custom_msg_board_arm2->rudder_temperature = mavlink_msg_custom_msg_board_arm2_get_rudder_temperature(msg);
    custom_msg_board_arm2->keller_pa3_pressure = mavlink_msg_custom_msg_board_arm2_get_keller_pa3_pressure(msg);
    custom_msg_board_arm2->keller_pa3_temperature = mavlink_msg_custom_msg_board_arm2_get_keller_pa3_temperature(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN? msg->len : MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN;
        memset(custom_msg_board_arm2, 0, MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_LEN);
    memcpy(custom_msg_board_arm2, _MAV_PAYLOAD(msg), len);
#endif
}
