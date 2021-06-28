/** @file
 *    @brief MAVLink comm protocol testsuite generated from custom_msg_mavlink.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef CUSTOM_MSG_MAVLINK_TESTSUITE_H
#define CUSTOM_MSG_MAVLINK_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_ardupilotmega(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_uAvionix(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_icarous(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_custom_msg_mavlink(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_ardupilotmega(system_id, component_id, last_msg);
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_uAvionix(system_id, component_id, last_msg);
    mavlink_test_icarous(system_id, component_id, last_msg);
    mavlink_test_custom_msg_mavlink(system_id, component_id, last_msg);
}
#endif

#include "../ardupilotmega/testsuite.h"
#include "../common/testsuite.h"
#include "../uAvionix/testsuite.h"
#include "../icarous/testsuite.h"


static void mavlink_test_custom_msg_pistol(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CUSTOM_MSG_PISTOL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_custom_msg_pistol_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0
    };
    mavlink_custom_msg_pistol_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.position = packet_in.position;
        packet1.motor_duty = packet_in.motor_duty;
        packet1.motor_temp = packet_in.motor_temp;
        packet1.motor_current = packet_in.motor_current;
        packet1.motor_rspeed = packet_in.motor_rspeed;
        packet1.motor_dspeed = packet_in.motor_dspeed;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CUSTOM_MSG_PISTOL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CUSTOM_MSG_PISTOL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_pistol_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_custom_msg_pistol_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_pistol_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_custom_msg_pistol_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_pistol_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_custom_msg_pistol_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_custom_msg_pistol_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_pistol_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_custom_msg_pistol_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_custom_msg_mass_shifter(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CUSTOM_MSG_MASS_SHIFTER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_custom_msg_mass_shifter_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0
    };
    mavlink_custom_msg_mass_shifter_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.position = packet_in.position;
        packet1.motor_duty = packet_in.motor_duty;
        packet1.motor_temp = packet_in.motor_temp;
        packet1.motor_current = packet_in.motor_current;
        packet1.motor_rspeed = packet_in.motor_rspeed;
        packet1.motor_dspeed = packet_in.motor_dspeed;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CUSTOM_MSG_MASS_SHIFTER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CUSTOM_MSG_MASS_SHIFTER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_mass_shifter_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_custom_msg_mass_shifter_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_mass_shifter_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_custom_msg_mass_shifter_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_mass_shifter_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_custom_msg_mass_shifter_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_custom_msg_mass_shifter_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_mass_shifter_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_custom_msg_mass_shifter_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_custom_msg_board_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_custom_msg_board_status_t packet_in = {
        963497464,17,84,151,218,29,96
    };
    mavlink_custom_msg_board_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.arm1 = packet_in.arm1;
        packet1.arm2 = packet_in.arm2;
        packet1.viam_navy = packet_in.viam_navy;
        packet1.pistol = packet_in.pistol;
        packet1.mass_shifter = packet_in.mass_shifter;
        packet1.thruster = packet_in.thruster;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_board_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_custom_msg_board_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_board_status_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.arm1 , packet1.arm2 , packet1.viam_navy , packet1.pistol , packet1.mass_shifter , packet1.thruster );
    mavlink_msg_custom_msg_board_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_board_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.arm1 , packet1.arm2 , packet1.viam_navy , packet1.pistol , packet1.mass_shifter , packet1.thruster );
    mavlink_msg_custom_msg_board_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_custom_msg_board_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_board_status_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.arm1 , packet1.arm2 , packet1.viam_navy , packet1.pistol , packet1.mass_shifter , packet1.thruster );
    mavlink_msg_custom_msg_board_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_custom_msg_mavlink(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_custom_msg_pistol(system_id, component_id, last_msg);
    mavlink_test_custom_msg_mass_shifter(system_id, component_id, last_msg);
    mavlink_test_custom_msg_board_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CUSTOM_MSG_MAVLINK_TESTSUITE_H
