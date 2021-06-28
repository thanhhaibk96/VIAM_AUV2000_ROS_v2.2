#pragma once
// MESSAGE SYS_STATUS PACKING

#define MAVLINK_MSG_ID_SYS_STATUS 1


typedef struct __mavlink_sys_status_t {
 uint32_t onboard_control_sensors_present; /*<  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.*/
 uint32_t onboard_control_sensors_enabled; /*<  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.*/
 uint32_t onboard_control_sensors_health; /*<  Bitmap showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled.*/
 uint16_t load; /*< [d%] Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000*/
 uint16_t voltage_battery_head; /*< [mV] Battery voltage at head*/
 int16_t current_battery_head; /*< [cA] Battery current at head, -1: autopilot does not measure the current*/
 uint16_t voltage_battery_motor; /*< [mV] Battery voltage at motor*/
 int16_t current_battery_motor; /*< [cA] Battery current at motor, -1: autopilot does not measure the current*/
 uint16_t voltage_battery_tail; /*< [mV] Battery voltage at tail*/
 int16_t current_battery_tail; /*< [cA] Battery current at tail, -1: autopilot does not measure the current*/
 uint16_t drop_rate_comm; /*< [c%] Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
 uint16_t errors_comm; /*<  Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
 uint16_t errors_count1; /*<  Autopilot-specific errors*/
 uint16_t errors_count2; /*<  Autopilot-specific errors*/
 uint16_t errors_count3; /*<  Autopilot-specific errors*/
 uint16_t errors_count4; /*<  Autopilot-specific errors*/
 int8_t battery_remaining_head; /*< [%] Remaining battery energy at head, -1: autopilot estimate the remaining battery*/
 int8_t battery_remaining_motor; /*< [%] Remaining battery energy at motor, -1: autopilot estimate the remaining battery*/
 int8_t battery_remaining_tail; /*< [%] Remaining battery energy at tail, -1: autopilot estimate the remaining battery*/
} mavlink_sys_status_t;

#define MAVLINK_MSG_ID_SYS_STATUS_LEN 41
#define MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN 41
#define MAVLINK_MSG_ID_1_LEN 41
#define MAVLINK_MSG_ID_1_MIN_LEN 41

#define MAVLINK_MSG_ID_SYS_STATUS_CRC 126
#define MAVLINK_MSG_ID_1_CRC 126



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SYS_STATUS { \
    1, \
    "SYS_STATUS", \
    19, \
    {  { "onboard_control_sensors_present", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_sys_status_t, onboard_control_sensors_present) }, \
         { "onboard_control_sensors_enabled", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_sys_status_t, onboard_control_sensors_enabled) }, \
         { "onboard_control_sensors_health", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sys_status_t, onboard_control_sensors_health) }, \
         { "load", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_sys_status_t, load) }, \
         { "voltage_battery_head", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_sys_status_t, voltage_battery_head) }, \
         { "current_battery_head", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_sys_status_t, current_battery_head) }, \
         { "battery_remaining_head", NULL, MAVLINK_TYPE_INT8_T, 0, 38, offsetof(mavlink_sys_status_t, battery_remaining_head) }, \
         { "voltage_battery_motor", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_sys_status_t, voltage_battery_motor) }, \
         { "current_battery_motor", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_sys_status_t, current_battery_motor) }, \
         { "battery_remaining_motor", NULL, MAVLINK_TYPE_INT8_T, 0, 39, offsetof(mavlink_sys_status_t, battery_remaining_motor) }, \
         { "voltage_battery_tail", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_sys_status_t, voltage_battery_tail) }, \
         { "current_battery_tail", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_sys_status_t, current_battery_tail) }, \
         { "battery_remaining_tail", NULL, MAVLINK_TYPE_INT8_T, 0, 40, offsetof(mavlink_sys_status_t, battery_remaining_tail) }, \
         { "drop_rate_comm", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_sys_status_t, drop_rate_comm) }, \
         { "errors_comm", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_sys_status_t, errors_comm) }, \
         { "errors_count1", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_sys_status_t, errors_count1) }, \
         { "errors_count2", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_sys_status_t, errors_count2) }, \
         { "errors_count3", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_sys_status_t, errors_count3) }, \
         { "errors_count4", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_sys_status_t, errors_count4) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SYS_STATUS { \
    "SYS_STATUS", \
    19, \
    {  { "onboard_control_sensors_present", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_sys_status_t, onboard_control_sensors_present) }, \
         { "onboard_control_sensors_enabled", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_sys_status_t, onboard_control_sensors_enabled) }, \
         { "onboard_control_sensors_health", "0x%04x", MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sys_status_t, onboard_control_sensors_health) }, \
         { "load", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_sys_status_t, load) }, \
         { "voltage_battery_head", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_sys_status_t, voltage_battery_head) }, \
         { "current_battery_head", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_sys_status_t, current_battery_head) }, \
         { "battery_remaining_head", NULL, MAVLINK_TYPE_INT8_T, 0, 38, offsetof(mavlink_sys_status_t, battery_remaining_head) }, \
         { "voltage_battery_motor", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_sys_status_t, voltage_battery_motor) }, \
         { "current_battery_motor", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_sys_status_t, current_battery_motor) }, \
         { "battery_remaining_motor", NULL, MAVLINK_TYPE_INT8_T, 0, 39, offsetof(mavlink_sys_status_t, battery_remaining_motor) }, \
         { "voltage_battery_tail", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_sys_status_t, voltage_battery_tail) }, \
         { "current_battery_tail", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_sys_status_t, current_battery_tail) }, \
         { "battery_remaining_tail", NULL, MAVLINK_TYPE_INT8_T, 0, 40, offsetof(mavlink_sys_status_t, battery_remaining_tail) }, \
         { "drop_rate_comm", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_sys_status_t, drop_rate_comm) }, \
         { "errors_comm", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_sys_status_t, errors_comm) }, \
         { "errors_count1", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_sys_status_t, errors_count1) }, \
         { "errors_count2", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_sys_status_t, errors_count2) }, \
         { "errors_count3", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_sys_status_t, errors_count3) }, \
         { "errors_count4", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_sys_status_t, errors_count4) }, \
         } \
}
#endif

/**
 * @brief Pack a sys_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param onboard_control_sensors_present  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
 * @param onboard_control_sensors_enabled  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
 * @param onboard_control_sensors_health  Bitmap showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled.
 * @param load [d%] Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
 * @param voltage_battery_head [mV] Battery voltage at head
 * @param current_battery_head [cA] Battery current at head, -1: autopilot does not measure the current
 * @param battery_remaining_head [%] Remaining battery energy at head, -1: autopilot estimate the remaining battery
 * @param voltage_battery_motor [mV] Battery voltage at motor
 * @param current_battery_motor [cA] Battery current at motor, -1: autopilot does not measure the current
 * @param battery_remaining_motor [%] Remaining battery energy at motor, -1: autopilot estimate the remaining battery
 * @param voltage_battery_tail [mV] Battery voltage at tail
 * @param current_battery_tail [cA] Battery current at tail, -1: autopilot does not measure the current
 * @param battery_remaining_tail [%] Remaining battery energy at tail, -1: autopilot estimate the remaining battery
 * @param drop_rate_comm [c%] Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_comm  Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_count1  Autopilot-specific errors
 * @param errors_count2  Autopilot-specific errors
 * @param errors_count3  Autopilot-specific errors
 * @param errors_count4  Autopilot-specific errors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery_head, int16_t current_battery_head, int8_t battery_remaining_head, uint16_t voltage_battery_motor, int16_t current_battery_motor, int8_t battery_remaining_motor, uint16_t voltage_battery_tail, int16_t current_battery_tail, int8_t battery_remaining_tail, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, onboard_control_sensors_present);
    _mav_put_uint32_t(buf, 4, onboard_control_sensors_enabled);
    _mav_put_uint32_t(buf, 8, onboard_control_sensors_health);
    _mav_put_uint16_t(buf, 12, load);
    _mav_put_uint16_t(buf, 14, voltage_battery_head);
    _mav_put_int16_t(buf, 16, current_battery_head);
    _mav_put_uint16_t(buf, 18, voltage_battery_motor);
    _mav_put_int16_t(buf, 20, current_battery_motor);
    _mav_put_uint16_t(buf, 22, voltage_battery_tail);
    _mav_put_int16_t(buf, 24, current_battery_tail);
    _mav_put_uint16_t(buf, 26, drop_rate_comm);
    _mav_put_uint16_t(buf, 28, errors_comm);
    _mav_put_uint16_t(buf, 30, errors_count1);
    _mav_put_uint16_t(buf, 32, errors_count2);
    _mav_put_uint16_t(buf, 34, errors_count3);
    _mav_put_uint16_t(buf, 36, errors_count4);
    _mav_put_int8_t(buf, 38, battery_remaining_head);
    _mav_put_int8_t(buf, 39, battery_remaining_motor);
    _mav_put_int8_t(buf, 40, battery_remaining_tail);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYS_STATUS_LEN);
#else
    mavlink_sys_status_t packet;
    packet.onboard_control_sensors_present = onboard_control_sensors_present;
    packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    packet.onboard_control_sensors_health = onboard_control_sensors_health;
    packet.load = load;
    packet.voltage_battery_head = voltage_battery_head;
    packet.current_battery_head = current_battery_head;
    packet.voltage_battery_motor = voltage_battery_motor;
    packet.current_battery_motor = current_battery_motor;
    packet.voltage_battery_tail = voltage_battery_tail;
    packet.current_battery_tail = current_battery_tail;
    packet.drop_rate_comm = drop_rate_comm;
    packet.errors_comm = errors_comm;
    packet.errors_count1 = errors_count1;
    packet.errors_count2 = errors_count2;
    packet.errors_count3 = errors_count3;
    packet.errors_count4 = errors_count4;
    packet.battery_remaining_head = battery_remaining_head;
    packet.battery_remaining_motor = battery_remaining_motor;
    packet.battery_remaining_tail = battery_remaining_tail;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYS_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYS_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_LEN, MAVLINK_MSG_ID_SYS_STATUS_CRC);
}

/**
 * @brief Pack a sys_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param onboard_control_sensors_present  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
 * @param onboard_control_sensors_enabled  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
 * @param onboard_control_sensors_health  Bitmap showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled.
 * @param load [d%] Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
 * @param voltage_battery_head [mV] Battery voltage at head
 * @param current_battery_head [cA] Battery current at head, -1: autopilot does not measure the current
 * @param battery_remaining_head [%] Remaining battery energy at head, -1: autopilot estimate the remaining battery
 * @param voltage_battery_motor [mV] Battery voltage at motor
 * @param current_battery_motor [cA] Battery current at motor, -1: autopilot does not measure the current
 * @param battery_remaining_motor [%] Remaining battery energy at motor, -1: autopilot estimate the remaining battery
 * @param voltage_battery_tail [mV] Battery voltage at tail
 * @param current_battery_tail [cA] Battery current at tail, -1: autopilot does not measure the current
 * @param battery_remaining_tail [%] Remaining battery energy at tail, -1: autopilot estimate the remaining battery
 * @param drop_rate_comm [c%] Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_comm  Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_count1  Autopilot-specific errors
 * @param errors_count2  Autopilot-specific errors
 * @param errors_count3  Autopilot-specific errors
 * @param errors_count4  Autopilot-specific errors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t onboard_control_sensors_present,uint32_t onboard_control_sensors_enabled,uint32_t onboard_control_sensors_health,uint16_t load,uint16_t voltage_battery_head,int16_t current_battery_head,int8_t battery_remaining_head,uint16_t voltage_battery_motor,int16_t current_battery_motor,int8_t battery_remaining_motor,uint16_t voltage_battery_tail,int16_t current_battery_tail,int8_t battery_remaining_tail,uint16_t drop_rate_comm,uint16_t errors_comm,uint16_t errors_count1,uint16_t errors_count2,uint16_t errors_count3,uint16_t errors_count4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, onboard_control_sensors_present);
    _mav_put_uint32_t(buf, 4, onboard_control_sensors_enabled);
    _mav_put_uint32_t(buf, 8, onboard_control_sensors_health);
    _mav_put_uint16_t(buf, 12, load);
    _mav_put_uint16_t(buf, 14, voltage_battery_head);
    _mav_put_int16_t(buf, 16, current_battery_head);
    _mav_put_uint16_t(buf, 18, voltage_battery_motor);
    _mav_put_int16_t(buf, 20, current_battery_motor);
    _mav_put_uint16_t(buf, 22, voltage_battery_tail);
    _mav_put_int16_t(buf, 24, current_battery_tail);
    _mav_put_uint16_t(buf, 26, drop_rate_comm);
    _mav_put_uint16_t(buf, 28, errors_comm);
    _mav_put_uint16_t(buf, 30, errors_count1);
    _mav_put_uint16_t(buf, 32, errors_count2);
    _mav_put_uint16_t(buf, 34, errors_count3);
    _mav_put_uint16_t(buf, 36, errors_count4);
    _mav_put_int8_t(buf, 38, battery_remaining_head);
    _mav_put_int8_t(buf, 39, battery_remaining_motor);
    _mav_put_int8_t(buf, 40, battery_remaining_tail);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYS_STATUS_LEN);
#else
    mavlink_sys_status_t packet;
    packet.onboard_control_sensors_present = onboard_control_sensors_present;
    packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    packet.onboard_control_sensors_health = onboard_control_sensors_health;
    packet.load = load;
    packet.voltage_battery_head = voltage_battery_head;
    packet.current_battery_head = current_battery_head;
    packet.voltage_battery_motor = voltage_battery_motor;
    packet.current_battery_motor = current_battery_motor;
    packet.voltage_battery_tail = voltage_battery_tail;
    packet.current_battery_tail = current_battery_tail;
    packet.drop_rate_comm = drop_rate_comm;
    packet.errors_comm = errors_comm;
    packet.errors_count1 = errors_count1;
    packet.errors_count2 = errors_count2;
    packet.errors_count3 = errors_count3;
    packet.errors_count4 = errors_count4;
    packet.battery_remaining_head = battery_remaining_head;
    packet.battery_remaining_motor = battery_remaining_motor;
    packet.battery_remaining_tail = battery_remaining_tail;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYS_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYS_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_LEN, MAVLINK_MSG_ID_SYS_STATUS_CRC);
}

/**
 * @brief Encode a sys_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sys_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sys_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sys_status_t* sys_status)
{
    return mavlink_msg_sys_status_pack(system_id, component_id, msg, sys_status->onboard_control_sensors_present, sys_status->onboard_control_sensors_enabled, sys_status->onboard_control_sensors_health, sys_status->load, sys_status->voltage_battery_head, sys_status->current_battery_head, sys_status->battery_remaining_head, sys_status->voltage_battery_motor, sys_status->current_battery_motor, sys_status->battery_remaining_motor, sys_status->voltage_battery_tail, sys_status->current_battery_tail, sys_status->battery_remaining_tail, sys_status->drop_rate_comm, sys_status->errors_comm, sys_status->errors_count1, sys_status->errors_count2, sys_status->errors_count3, sys_status->errors_count4);
}

/**
 * @brief Encode a sys_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sys_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sys_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sys_status_t* sys_status)
{
    return mavlink_msg_sys_status_pack_chan(system_id, component_id, chan, msg, sys_status->onboard_control_sensors_present, sys_status->onboard_control_sensors_enabled, sys_status->onboard_control_sensors_health, sys_status->load, sys_status->voltage_battery_head, sys_status->current_battery_head, sys_status->battery_remaining_head, sys_status->voltage_battery_motor, sys_status->current_battery_motor, sys_status->battery_remaining_motor, sys_status->voltage_battery_tail, sys_status->current_battery_tail, sys_status->battery_remaining_tail, sys_status->drop_rate_comm, sys_status->errors_comm, sys_status->errors_count1, sys_status->errors_count2, sys_status->errors_count3, sys_status->errors_count4);
}

/**
 * @brief Send a sys_status message
 * @param chan MAVLink channel to send the message
 *
 * @param onboard_control_sensors_present  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
 * @param onboard_control_sensors_enabled  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
 * @param onboard_control_sensors_health  Bitmap showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled.
 * @param load [d%] Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
 * @param voltage_battery_head [mV] Battery voltage at head
 * @param current_battery_head [cA] Battery current at head, -1: autopilot does not measure the current
 * @param battery_remaining_head [%] Remaining battery energy at head, -1: autopilot estimate the remaining battery
 * @param voltage_battery_motor [mV] Battery voltage at motor
 * @param current_battery_motor [cA] Battery current at motor, -1: autopilot does not measure the current
 * @param battery_remaining_motor [%] Remaining battery energy at motor, -1: autopilot estimate the remaining battery
 * @param voltage_battery_tail [mV] Battery voltage at tail
 * @param current_battery_tail [cA] Battery current at tail, -1: autopilot does not measure the current
 * @param battery_remaining_tail [%] Remaining battery energy at tail, -1: autopilot estimate the remaining battery
 * @param drop_rate_comm [c%] Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_comm  Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_count1  Autopilot-specific errors
 * @param errors_count2  Autopilot-specific errors
 * @param errors_count3  Autopilot-specific errors
 * @param errors_count4  Autopilot-specific errors
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sys_status_send(mavlink_channel_t chan, uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery_head, int16_t current_battery_head, int8_t battery_remaining_head, uint16_t voltage_battery_motor, int16_t current_battery_motor, int8_t battery_remaining_motor, uint16_t voltage_battery_tail, int16_t current_battery_tail, int8_t battery_remaining_tail, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYS_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, onboard_control_sensors_present);
    _mav_put_uint32_t(buf, 4, onboard_control_sensors_enabled);
    _mav_put_uint32_t(buf, 8, onboard_control_sensors_health);
    _mav_put_uint16_t(buf, 12, load);
    _mav_put_uint16_t(buf, 14, voltage_battery_head);
    _mav_put_int16_t(buf, 16, current_battery_head);
    _mav_put_uint16_t(buf, 18, voltage_battery_motor);
    _mav_put_int16_t(buf, 20, current_battery_motor);
    _mav_put_uint16_t(buf, 22, voltage_battery_tail);
    _mav_put_int16_t(buf, 24, current_battery_tail);
    _mav_put_uint16_t(buf, 26, drop_rate_comm);
    _mav_put_uint16_t(buf, 28, errors_comm);
    _mav_put_uint16_t(buf, 30, errors_count1);
    _mav_put_uint16_t(buf, 32, errors_count2);
    _mav_put_uint16_t(buf, 34, errors_count3);
    _mav_put_uint16_t(buf, 36, errors_count4);
    _mav_put_int8_t(buf, 38, battery_remaining_head);
    _mav_put_int8_t(buf, 39, battery_remaining_motor);
    _mav_put_int8_t(buf, 40, battery_remaining_tail);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS, buf, MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_LEN, MAVLINK_MSG_ID_SYS_STATUS_CRC);
#else
    mavlink_sys_status_t packet;
    packet.onboard_control_sensors_present = onboard_control_sensors_present;
    packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    packet.onboard_control_sensors_health = onboard_control_sensors_health;
    packet.load = load;
    packet.voltage_battery_head = voltage_battery_head;
    packet.current_battery_head = current_battery_head;
    packet.voltage_battery_motor = voltage_battery_motor;
    packet.current_battery_motor = current_battery_motor;
    packet.voltage_battery_tail = voltage_battery_tail;
    packet.current_battery_tail = current_battery_tail;
    packet.drop_rate_comm = drop_rate_comm;
    packet.errors_comm = errors_comm;
    packet.errors_count1 = errors_count1;
    packet.errors_count2 = errors_count2;
    packet.errors_count3 = errors_count3;
    packet.errors_count4 = errors_count4;
    packet.battery_remaining_head = battery_remaining_head;
    packet.battery_remaining_motor = battery_remaining_motor;
    packet.battery_remaining_tail = battery_remaining_tail;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_LEN, MAVLINK_MSG_ID_SYS_STATUS_CRC);
#endif
}

/**
 * @brief Send a sys_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sys_status_send_struct(mavlink_channel_t chan, const mavlink_sys_status_t* sys_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sys_status_send(chan, sys_status->onboard_control_sensors_present, sys_status->onboard_control_sensors_enabled, sys_status->onboard_control_sensors_health, sys_status->load, sys_status->voltage_battery_head, sys_status->current_battery_head, sys_status->battery_remaining_head, sys_status->voltage_battery_motor, sys_status->current_battery_motor, sys_status->battery_remaining_motor, sys_status->voltage_battery_tail, sys_status->current_battery_tail, sys_status->battery_remaining_tail, sys_status->drop_rate_comm, sys_status->errors_comm, sys_status->errors_count1, sys_status->errors_count2, sys_status->errors_count3, sys_status->errors_count4);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS, (const char *)sys_status, MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_LEN, MAVLINK_MSG_ID_SYS_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SYS_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sys_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery_head, int16_t current_battery_head, int8_t battery_remaining_head, uint16_t voltage_battery_motor, int16_t current_battery_motor, int8_t battery_remaining_motor, uint16_t voltage_battery_tail, int16_t current_battery_tail, int8_t battery_remaining_tail, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, onboard_control_sensors_present);
    _mav_put_uint32_t(buf, 4, onboard_control_sensors_enabled);
    _mav_put_uint32_t(buf, 8, onboard_control_sensors_health);
    _mav_put_uint16_t(buf, 12, load);
    _mav_put_uint16_t(buf, 14, voltage_battery_head);
    _mav_put_int16_t(buf, 16, current_battery_head);
    _mav_put_uint16_t(buf, 18, voltage_battery_motor);
    _mav_put_int16_t(buf, 20, current_battery_motor);
    _mav_put_uint16_t(buf, 22, voltage_battery_tail);
    _mav_put_int16_t(buf, 24, current_battery_tail);
    _mav_put_uint16_t(buf, 26, drop_rate_comm);
    _mav_put_uint16_t(buf, 28, errors_comm);
    _mav_put_uint16_t(buf, 30, errors_count1);
    _mav_put_uint16_t(buf, 32, errors_count2);
    _mav_put_uint16_t(buf, 34, errors_count3);
    _mav_put_uint16_t(buf, 36, errors_count4);
    _mav_put_int8_t(buf, 38, battery_remaining_head);
    _mav_put_int8_t(buf, 39, battery_remaining_motor);
    _mav_put_int8_t(buf, 40, battery_remaining_tail);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS, buf, MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_LEN, MAVLINK_MSG_ID_SYS_STATUS_CRC);
#else
    mavlink_sys_status_t *packet = (mavlink_sys_status_t *)msgbuf;
    packet->onboard_control_sensors_present = onboard_control_sensors_present;
    packet->onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    packet->onboard_control_sensors_health = onboard_control_sensors_health;
    packet->load = load;
    packet->voltage_battery_head = voltage_battery_head;
    packet->current_battery_head = current_battery_head;
    packet->voltage_battery_motor = voltage_battery_motor;
    packet->current_battery_motor = current_battery_motor;
    packet->voltage_battery_tail = voltage_battery_tail;
    packet->current_battery_tail = current_battery_tail;
    packet->drop_rate_comm = drop_rate_comm;
    packet->errors_comm = errors_comm;
    packet->errors_count1 = errors_count1;
    packet->errors_count2 = errors_count2;
    packet->errors_count3 = errors_count3;
    packet->errors_count4 = errors_count4;
    packet->battery_remaining_head = battery_remaining_head;
    packet->battery_remaining_motor = battery_remaining_motor;
    packet->battery_remaining_tail = battery_remaining_tail;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS, (const char *)packet, MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN, MAVLINK_MSG_ID_SYS_STATUS_LEN, MAVLINK_MSG_ID_SYS_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE SYS_STATUS UNPACKING


/**
 * @brief Get field onboard_control_sensors_present from sys_status message
 *
 * @return  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
 */
static inline uint32_t mavlink_msg_sys_status_get_onboard_control_sensors_present(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field onboard_control_sensors_enabled from sys_status message
 *
 * @return  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
 */
static inline uint32_t mavlink_msg_sys_status_get_onboard_control_sensors_enabled(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field onboard_control_sensors_health from sys_status message
 *
 * @return  Bitmap showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled.
 */
static inline uint32_t mavlink_msg_sys_status_get_onboard_control_sensors_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field load from sys_status message
 *
 * @return [d%] Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
 */
static inline uint16_t mavlink_msg_sys_status_get_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field voltage_battery_head from sys_status message
 *
 * @return [mV] Battery voltage at head
 */
static inline uint16_t mavlink_msg_sys_status_get_voltage_battery_head(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field current_battery_head from sys_status message
 *
 * @return [cA] Battery current at head, -1: autopilot does not measure the current
 */
static inline int16_t mavlink_msg_sys_status_get_current_battery_head(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field battery_remaining_head from sys_status message
 *
 * @return [%] Remaining battery energy at head, -1: autopilot estimate the remaining battery
 */
static inline int8_t mavlink_msg_sys_status_get_battery_remaining_head(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  38);
}

/**
 * @brief Get field voltage_battery_motor from sys_status message
 *
 * @return [mV] Battery voltage at motor
 */
static inline uint16_t mavlink_msg_sys_status_get_voltage_battery_motor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field current_battery_motor from sys_status message
 *
 * @return [cA] Battery current at motor, -1: autopilot does not measure the current
 */
static inline int16_t mavlink_msg_sys_status_get_current_battery_motor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field battery_remaining_motor from sys_status message
 *
 * @return [%] Remaining battery energy at motor, -1: autopilot estimate the remaining battery
 */
static inline int8_t mavlink_msg_sys_status_get_battery_remaining_motor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  39);
}

/**
 * @brief Get field voltage_battery_tail from sys_status message
 *
 * @return [mV] Battery voltage at tail
 */
static inline uint16_t mavlink_msg_sys_status_get_voltage_battery_tail(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field current_battery_tail from sys_status message
 *
 * @return [cA] Battery current at tail, -1: autopilot does not measure the current
 */
static inline int16_t mavlink_msg_sys_status_get_current_battery_tail(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field battery_remaining_tail from sys_status message
 *
 * @return [%] Remaining battery energy at tail, -1: autopilot estimate the remaining battery
 */
static inline int8_t mavlink_msg_sys_status_get_battery_remaining_tail(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  40);
}

/**
 * @brief Get field drop_rate_comm from sys_status message
 *
 * @return [c%] Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 */
static inline uint16_t mavlink_msg_sys_status_get_drop_rate_comm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field errors_comm from sys_status message
 *
 * @return  Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 */
static inline uint16_t mavlink_msg_sys_status_get_errors_comm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field errors_count1 from sys_status message
 *
 * @return  Autopilot-specific errors
 */
static inline uint16_t mavlink_msg_sys_status_get_errors_count1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field errors_count2 from sys_status message
 *
 * @return  Autopilot-specific errors
 */
static inline uint16_t mavlink_msg_sys_status_get_errors_count2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field errors_count3 from sys_status message
 *
 * @return  Autopilot-specific errors
 */
static inline uint16_t mavlink_msg_sys_status_get_errors_count3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field errors_count4 from sys_status message
 *
 * @return  Autopilot-specific errors
 */
static inline uint16_t mavlink_msg_sys_status_get_errors_count4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Decode a sys_status message into a struct
 *
 * @param msg The message to decode
 * @param sys_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sys_status->onboard_control_sensors_present = mavlink_msg_sys_status_get_onboard_control_sensors_present(msg);
    sys_status->onboard_control_sensors_enabled = mavlink_msg_sys_status_get_onboard_control_sensors_enabled(msg);
    sys_status->onboard_control_sensors_health = mavlink_msg_sys_status_get_onboard_control_sensors_health(msg);
    sys_status->load = mavlink_msg_sys_status_get_load(msg);
    sys_status->voltage_battery_head = mavlink_msg_sys_status_get_voltage_battery_head(msg);
    sys_status->current_battery_head = mavlink_msg_sys_status_get_current_battery_head(msg);
    sys_status->voltage_battery_motor = mavlink_msg_sys_status_get_voltage_battery_motor(msg);
    sys_status->current_battery_motor = mavlink_msg_sys_status_get_current_battery_motor(msg);
    sys_status->voltage_battery_tail = mavlink_msg_sys_status_get_voltage_battery_tail(msg);
    sys_status->current_battery_tail = mavlink_msg_sys_status_get_current_battery_tail(msg);
    sys_status->drop_rate_comm = mavlink_msg_sys_status_get_drop_rate_comm(msg);
    sys_status->errors_comm = mavlink_msg_sys_status_get_errors_comm(msg);
    sys_status->errors_count1 = mavlink_msg_sys_status_get_errors_count1(msg);
    sys_status->errors_count2 = mavlink_msg_sys_status_get_errors_count2(msg);
    sys_status->errors_count3 = mavlink_msg_sys_status_get_errors_count3(msg);
    sys_status->errors_count4 = mavlink_msg_sys_status_get_errors_count4(msg);
    sys_status->battery_remaining_head = mavlink_msg_sys_status_get_battery_remaining_head(msg);
    sys_status->battery_remaining_motor = mavlink_msg_sys_status_get_battery_remaining_motor(msg);
    sys_status->battery_remaining_tail = mavlink_msg_sys_status_get_battery_remaining_tail(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SYS_STATUS_LEN? msg->len : MAVLINK_MSG_ID_SYS_STATUS_LEN;
        memset(sys_status, 0, MAVLINK_MSG_ID_SYS_STATUS_LEN);
    memcpy(sys_status, _MAV_PAYLOAD(msg), len);
#endif
}
