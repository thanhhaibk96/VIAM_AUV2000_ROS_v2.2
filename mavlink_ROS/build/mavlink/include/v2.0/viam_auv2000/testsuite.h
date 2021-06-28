/** @file
 *    @brief MAVLink comm protocol testsuite generated from viam_auv2000.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef VIAM_AUV2000_TESTSUITE_H
#define VIAM_AUV2000_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_ardupilotmega(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_uAvionix(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_icarous(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_viam_auv2000(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_ardupilotmega(system_id, component_id, last_msg);
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_uAvionix(system_id, component_id, last_msg);
    mavlink_test_icarous(system_id, component_id, last_msg);
    mavlink_test_viam_auv2000(system_id, component_id, last_msg);
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
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0
    };
    mavlink_custom_msg_pistol_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.position = packet_in.position;
        packet1.motor_duty = packet_in.motor_duty;
        packet1.motor_temp_on_chip = packet_in.motor_temp_on_chip;
        packet1.motor_temp_ambient = packet_in.motor_temp_ambient;
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
    mavlink_msg_custom_msg_pistol_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_custom_msg_pistol_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_pistol_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
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
    mavlink_msg_custom_msg_pistol_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
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
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0
    };
    mavlink_custom_msg_mass_shifter_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.position = packet_in.position;
        packet1.motor_duty = packet_in.motor_duty;
        packet1.motor_temp_on_chip = packet_in.motor_temp_on_chip;
        packet1.motor_temp_ambient = packet_in.motor_temp_ambient;
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
    mavlink_msg_custom_msg_mass_shifter_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_custom_msg_mass_shifter_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_mass_shifter_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
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
    mavlink_msg_custom_msg_mass_shifter_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.position , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_custom_msg_mass_shifter_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_custom_msg_thruster(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_custom_msg_thruster_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0
    };
    mavlink_custom_msg_thruster_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.motor_duty = packet_in.motor_duty;
        packet1.motor_temp_on_chip = packet_in.motor_temp_on_chip;
        packet1.motor_temp_ambient = packet_in.motor_temp_ambient;
        packet1.motor_current = packet_in.motor_current;
        packet1.motor_rspeed = packet_in.motor_rspeed;
        packet1.motor_dspeed = packet_in.motor_dspeed;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CUSTOM_MSG_THRUSTER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_thruster_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_custom_msg_thruster_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_thruster_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_custom_msg_thruster_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_thruster_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_custom_msg_thruster_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_custom_msg_thruster_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_thruster_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.motor_duty , packet1.motor_temp_on_chip , packet1.motor_temp_ambient , packet1.motor_current , packet1.motor_rspeed , packet1.motor_dspeed );
    mavlink_msg_custom_msg_thruster_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_custom_msg_board_arm1(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_custom_msg_board_arm1_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,65
    };
    mavlink_custom_msg_board_arm1_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.altimeter_in_metres = packet_in.altimeter_in_metres;
        packet1.altimeter_in_feet = packet_in.altimeter_in_feet;
        packet1.altimeter_in_fathoms = packet_in.altimeter_in_fathoms;
        packet1.roll_motor_angle = packet_in.roll_motor_angle;
        packet1.leak_sensor = packet_in.leak_sensor;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM1_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_board_arm1_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_custom_msg_board_arm1_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_board_arm1_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.leak_sensor , packet1.altimeter_in_metres , packet1.altimeter_in_feet , packet1.altimeter_in_fathoms , packet1.roll_motor_angle );
    mavlink_msg_custom_msg_board_arm1_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_board_arm1_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.leak_sensor , packet1.altimeter_in_metres , packet1.altimeter_in_feet , packet1.altimeter_in_fathoms , packet1.roll_motor_angle );
    mavlink_msg_custom_msg_board_arm1_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_custom_msg_board_arm1_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_board_arm1_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.leak_sensor , packet1.altimeter_in_metres , packet1.altimeter_in_feet , packet1.altimeter_in_fathoms , packet1.roll_motor_angle );
    mavlink_msg_custom_msg_board_arm1_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_custom_msg_board_arm2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_custom_msg_board_arm2_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,113
    };
    mavlink_custom_msg_board_arm2_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.rudder_position = packet_in.rudder_position;
        packet1.rudder_speed = packet_in.rudder_speed;
        packet1.rudder_load = packet_in.rudder_load;
        packet1.rudder_voltage = packet_in.rudder_voltage;
        packet1.rudder_temperature = packet_in.rudder_temperature;
        packet1.keller_pa3_pressure = packet_in.keller_pa3_pressure;
        packet1.keller_pa3_temperature = packet_in.keller_pa3_temperature;
        packet1.roll_motor_angle = packet_in.roll_motor_angle;
        packet1.leak_sensor = packet_in.leak_sensor;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CUSTOM_MSG_BOARD_ARM2_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_board_arm2_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_custom_msg_board_arm2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_board_arm2_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.leak_sensor , packet1.rudder_position , packet1.rudder_speed , packet1.rudder_load , packet1.rudder_voltage , packet1.rudder_temperature , packet1.keller_pa3_pressure , packet1.keller_pa3_temperature , packet1.roll_motor_angle );
    mavlink_msg_custom_msg_board_arm2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_board_arm2_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.leak_sensor , packet1.rudder_position , packet1.rudder_speed , packet1.rudder_load , packet1.rudder_voltage , packet1.rudder_temperature , packet1.keller_pa3_pressure , packet1.keller_pa3_temperature , packet1.roll_motor_angle );
    mavlink_msg_custom_msg_board_arm2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_custom_msg_board_arm2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_custom_msg_board_arm2_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.leak_sensor , packet1.rudder_position , packet1.rudder_speed , packet1.rudder_load , packet1.rudder_voltage , packet1.rudder_temperature , packet1.keller_pa3_pressure , packet1.keller_pa3_temperature , packet1.roll_motor_angle );
    mavlink_msg_custom_msg_board_arm2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_viam_auv2000(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_custom_msg_pistol(system_id, component_id, last_msg);
    mavlink_test_custom_msg_mass_shifter(system_id, component_id, last_msg);
    mavlink_test_custom_msg_thruster(system_id, component_id, last_msg);
    mavlink_test_custom_msg_board_arm1(system_id, component_id, last_msg);
    mavlink_test_custom_msg_board_arm2(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // VIAM_AUV2000_TESTSUITE_H
