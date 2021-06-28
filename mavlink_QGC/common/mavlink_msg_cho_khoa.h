#pragma once
// MESSAGE CHO_KHOA PACKING

#define MAVLINK_MSG_ID_CHO_KHOA 396


typedef struct __mavlink_cho_khoa_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 uint32_t ngu_nhu_bo; /*<  The type of metadata being requested.*/
 uint32_t oc_cho; /*<  Unique uid for this metadata which a gcs can use for created cached metadata and understanding whether it's cache it up to date or whether it needs to download new data.*/
} mavlink_cho_khoa_t;

#define MAVLINK_MSG_ID_CHO_KHOA_LEN 12
#define MAVLINK_MSG_ID_CHO_KHOA_MIN_LEN 12
#define MAVLINK_MSG_ID_396_LEN 12
#define MAVLINK_MSG_ID_396_MIN_LEN 12

#define MAVLINK_MSG_ID_CHO_KHOA_CRC 104
#define MAVLINK_MSG_ID_396_CRC 104



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CHO_KHOA { \
    396, \
    "CHO_KHOA", \
    3, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_cho_khoa_t, time_boot_ms) }, \
         { "ngu_nhu_bo", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_cho_khoa_t, ngu_nhu_bo) }, \
         { "oc_cho", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_cho_khoa_t, oc_cho) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CHO_KHOA { \
    "CHO_KHOA", \
    3, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_cho_khoa_t, time_boot_ms) }, \
         { "ngu_nhu_bo", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_cho_khoa_t, ngu_nhu_bo) }, \
         { "oc_cho", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_cho_khoa_t, oc_cho) }, \
         } \
}
#endif

/**
 * @brief Pack a cho_khoa message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param ngu_nhu_bo  The type of metadata being requested.
 * @param oc_cho  Unique uid for this metadata which a gcs can use for created cached metadata and understanding whether it's cache it up to date or whether it needs to download new data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cho_khoa_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint32_t ngu_nhu_bo, uint32_t oc_cho)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHO_KHOA_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, ngu_nhu_bo);
    _mav_put_uint32_t(buf, 8, oc_cho);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHO_KHOA_LEN);
#else
    mavlink_cho_khoa_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.ngu_nhu_bo = ngu_nhu_bo;
    packet.oc_cho = oc_cho;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHO_KHOA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHO_KHOA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHO_KHOA_MIN_LEN, MAVLINK_MSG_ID_CHO_KHOA_LEN, MAVLINK_MSG_ID_CHO_KHOA_CRC);
}

/**
 * @brief Pack a cho_khoa message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param ngu_nhu_bo  The type of metadata being requested.
 * @param oc_cho  Unique uid for this metadata which a gcs can use for created cached metadata and understanding whether it's cache it up to date or whether it needs to download new data.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cho_khoa_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint32_t ngu_nhu_bo,uint32_t oc_cho)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHO_KHOA_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, ngu_nhu_bo);
    _mav_put_uint32_t(buf, 8, oc_cho);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHO_KHOA_LEN);
#else
    mavlink_cho_khoa_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.ngu_nhu_bo = ngu_nhu_bo;
    packet.oc_cho = oc_cho;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHO_KHOA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHO_KHOA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHO_KHOA_MIN_LEN, MAVLINK_MSG_ID_CHO_KHOA_LEN, MAVLINK_MSG_ID_CHO_KHOA_CRC);
}

/**
 * @brief Encode a cho_khoa struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cho_khoa C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cho_khoa_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cho_khoa_t* cho_khoa)
{
    return mavlink_msg_cho_khoa_pack(system_id, component_id, msg, cho_khoa->time_boot_ms, cho_khoa->ngu_nhu_bo, cho_khoa->oc_cho);
}

/**
 * @brief Encode a cho_khoa struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cho_khoa C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cho_khoa_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cho_khoa_t* cho_khoa)
{
    return mavlink_msg_cho_khoa_pack_chan(system_id, component_id, chan, msg, cho_khoa->time_boot_ms, cho_khoa->ngu_nhu_bo, cho_khoa->oc_cho);
}

/**
 * @brief Send a cho_khoa message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param ngu_nhu_bo  The type of metadata being requested.
 * @param oc_cho  Unique uid for this metadata which a gcs can use for created cached metadata and understanding whether it's cache it up to date or whether it needs to download new data.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cho_khoa_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint32_t ngu_nhu_bo, uint32_t oc_cho)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHO_KHOA_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, ngu_nhu_bo);
    _mav_put_uint32_t(buf, 8, oc_cho);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHO_KHOA, buf, MAVLINK_MSG_ID_CHO_KHOA_MIN_LEN, MAVLINK_MSG_ID_CHO_KHOA_LEN, MAVLINK_MSG_ID_CHO_KHOA_CRC);
#else
    mavlink_cho_khoa_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.ngu_nhu_bo = ngu_nhu_bo;
    packet.oc_cho = oc_cho;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHO_KHOA, (const char *)&packet, MAVLINK_MSG_ID_CHO_KHOA_MIN_LEN, MAVLINK_MSG_ID_CHO_KHOA_LEN, MAVLINK_MSG_ID_CHO_KHOA_CRC);
#endif
}

/**
 * @brief Send a cho_khoa message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_cho_khoa_send_struct(mavlink_channel_t chan, const mavlink_cho_khoa_t* cho_khoa)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cho_khoa_send(chan, cho_khoa->time_boot_ms, cho_khoa->ngu_nhu_bo, cho_khoa->oc_cho);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHO_KHOA, (const char *)cho_khoa, MAVLINK_MSG_ID_CHO_KHOA_MIN_LEN, MAVLINK_MSG_ID_CHO_KHOA_LEN, MAVLINK_MSG_ID_CHO_KHOA_CRC);
#endif
}

#if MAVLINK_MSG_ID_CHO_KHOA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cho_khoa_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint32_t ngu_nhu_bo, uint32_t oc_cho)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint32_t(buf, 4, ngu_nhu_bo);
    _mav_put_uint32_t(buf, 8, oc_cho);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHO_KHOA, buf, MAVLINK_MSG_ID_CHO_KHOA_MIN_LEN, MAVLINK_MSG_ID_CHO_KHOA_LEN, MAVLINK_MSG_ID_CHO_KHOA_CRC);
#else
    mavlink_cho_khoa_t *packet = (mavlink_cho_khoa_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->ngu_nhu_bo = ngu_nhu_bo;
    packet->oc_cho = oc_cho;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHO_KHOA, (const char *)packet, MAVLINK_MSG_ID_CHO_KHOA_MIN_LEN, MAVLINK_MSG_ID_CHO_KHOA_LEN, MAVLINK_MSG_ID_CHO_KHOA_CRC);
#endif
}
#endif

#endif

// MESSAGE CHO_KHOA UNPACKING


/**
 * @brief Get field time_boot_ms from cho_khoa message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_cho_khoa_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field ngu_nhu_bo from cho_khoa message
 *
 * @return  The type of metadata being requested.
 */
static inline uint32_t mavlink_msg_cho_khoa_get_ngu_nhu_bo(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field oc_cho from cho_khoa message
 *
 * @return  Unique uid for this metadata which a gcs can use for created cached metadata and understanding whether it's cache it up to date or whether it needs to download new data.
 */
static inline uint32_t mavlink_msg_cho_khoa_get_oc_cho(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Decode a cho_khoa message into a struct
 *
 * @param msg The message to decode
 * @param cho_khoa C-struct to decode the message contents into
 */
static inline void mavlink_msg_cho_khoa_decode(const mavlink_message_t* msg, mavlink_cho_khoa_t* cho_khoa)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    cho_khoa->time_boot_ms = mavlink_msg_cho_khoa_get_time_boot_ms(msg);
    cho_khoa->ngu_nhu_bo = mavlink_msg_cho_khoa_get_ngu_nhu_bo(msg);
    cho_khoa->oc_cho = mavlink_msg_cho_khoa_get_oc_cho(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CHO_KHOA_LEN? msg->len : MAVLINK_MSG_ID_CHO_KHOA_LEN;
        memset(cho_khoa, 0, MAVLINK_MSG_ID_CHO_KHOA_LEN);
    memcpy(cho_khoa, _MAV_PAYLOAD(msg), len);
#endif
}
