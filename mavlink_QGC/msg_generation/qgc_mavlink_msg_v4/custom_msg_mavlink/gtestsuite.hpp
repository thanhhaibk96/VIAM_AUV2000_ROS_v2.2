/** @file
 *	@brief MAVLink comm testsuite protocol generated from custom_msg_mavlink.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "custom_msg_mavlink.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(custom_msg_mavlink, CUSTOM_MSG_PISTOL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_PISTOL packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.position = 45.0;
    packet_in.motor_duty = 73.0;
    packet_in.motor_temp_on_chip = 101.0;
    packet_in.motor_temp_ambient = 129.0;
    packet_in.motor_current = 157.0;
    packet_in.motor_rspeed = 185.0;
    packet_in.motor_dspeed = 213.0;

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_PISTOL packet1{};
    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_PISTOL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.position, packet2.position);
    EXPECT_EQ(packet1.motor_duty, packet2.motor_duty);
    EXPECT_EQ(packet1.motor_temp_on_chip, packet2.motor_temp_on_chip);
    EXPECT_EQ(packet1.motor_temp_ambient, packet2.motor_temp_ambient);
    EXPECT_EQ(packet1.motor_current, packet2.motor_current);
    EXPECT_EQ(packet1.motor_rspeed, packet2.motor_rspeed);
    EXPECT_EQ(packet1.motor_dspeed, packet2.motor_dspeed);
}

#ifdef TEST_INTEROP
TEST(custom_msg_mavlink_interop, CUSTOM_MSG_PISTOL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_custom_msg_pistol_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0
    };

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_PISTOL packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.position = 45.0;
    packet_in.motor_duty = 73.0;
    packet_in.motor_temp_on_chip = 101.0;
    packet_in.motor_temp_ambient = 129.0;
    packet_in.motor_current = 157.0;
    packet_in.motor_rspeed = 185.0;
    packet_in.motor_dspeed = 213.0;

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_PISTOL packet2{};

    mavlink_msg_custom_msg_pistol_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.position, packet2.position);
    EXPECT_EQ(packet_in.motor_duty, packet2.motor_duty);
    EXPECT_EQ(packet_in.motor_temp_on_chip, packet2.motor_temp_on_chip);
    EXPECT_EQ(packet_in.motor_temp_ambient, packet2.motor_temp_ambient);
    EXPECT_EQ(packet_in.motor_current, packet2.motor_current);
    EXPECT_EQ(packet_in.motor_rspeed, packet2.motor_rspeed);
    EXPECT_EQ(packet_in.motor_dspeed, packet2.motor_dspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(custom_msg_mavlink, CUSTOM_MSG_MASS_SHIFTER)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_MASS_SHIFTER packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.position = 45.0;
    packet_in.motor_duty = 73.0;
    packet_in.motor_temp_on_chip = 101.0;
    packet_in.motor_temp_ambient = 129.0;
    packet_in.motor_current = 157.0;
    packet_in.motor_rspeed = 185.0;
    packet_in.motor_dspeed = 213.0;

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_MASS_SHIFTER packet1{};
    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_MASS_SHIFTER packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.position, packet2.position);
    EXPECT_EQ(packet1.motor_duty, packet2.motor_duty);
    EXPECT_EQ(packet1.motor_temp_on_chip, packet2.motor_temp_on_chip);
    EXPECT_EQ(packet1.motor_temp_ambient, packet2.motor_temp_ambient);
    EXPECT_EQ(packet1.motor_current, packet2.motor_current);
    EXPECT_EQ(packet1.motor_rspeed, packet2.motor_rspeed);
    EXPECT_EQ(packet1.motor_dspeed, packet2.motor_dspeed);
}

#ifdef TEST_INTEROP
TEST(custom_msg_mavlink_interop, CUSTOM_MSG_MASS_SHIFTER)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_custom_msg_mass_shifter_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0
    };

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_MASS_SHIFTER packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.position = 45.0;
    packet_in.motor_duty = 73.0;
    packet_in.motor_temp_on_chip = 101.0;
    packet_in.motor_temp_ambient = 129.0;
    packet_in.motor_current = 157.0;
    packet_in.motor_rspeed = 185.0;
    packet_in.motor_dspeed = 213.0;

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_MASS_SHIFTER packet2{};

    mavlink_msg_custom_msg_mass_shifter_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.position, packet2.position);
    EXPECT_EQ(packet_in.motor_duty, packet2.motor_duty);
    EXPECT_EQ(packet_in.motor_temp_on_chip, packet2.motor_temp_on_chip);
    EXPECT_EQ(packet_in.motor_temp_ambient, packet2.motor_temp_ambient);
    EXPECT_EQ(packet_in.motor_current, packet2.motor_current);
    EXPECT_EQ(packet_in.motor_rspeed, packet2.motor_rspeed);
    EXPECT_EQ(packet_in.motor_dspeed, packet2.motor_dspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(custom_msg_mavlink, CUSTOM_MSG_THRUSTER)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_THRUSTER packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.motor_duty = 45.0;
    packet_in.motor_temp_on_chip = 73.0;
    packet_in.motor_temp_ambient = 101.0;
    packet_in.motor_current = 129.0;
    packet_in.motor_rspeed = 157.0;
    packet_in.motor_dspeed = 185.0;

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_THRUSTER packet1{};
    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_THRUSTER packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.motor_duty, packet2.motor_duty);
    EXPECT_EQ(packet1.motor_temp_on_chip, packet2.motor_temp_on_chip);
    EXPECT_EQ(packet1.motor_temp_ambient, packet2.motor_temp_ambient);
    EXPECT_EQ(packet1.motor_current, packet2.motor_current);
    EXPECT_EQ(packet1.motor_rspeed, packet2.motor_rspeed);
    EXPECT_EQ(packet1.motor_dspeed, packet2.motor_dspeed);
}

#ifdef TEST_INTEROP
TEST(custom_msg_mavlink_interop, CUSTOM_MSG_THRUSTER)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_custom_msg_thruster_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0
    };

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_THRUSTER packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.motor_duty = 45.0;
    packet_in.motor_temp_on_chip = 73.0;
    packet_in.motor_temp_ambient = 101.0;
    packet_in.motor_current = 129.0;
    packet_in.motor_rspeed = 157.0;
    packet_in.motor_dspeed = 185.0;

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_THRUSTER packet2{};

    mavlink_msg_custom_msg_thruster_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.motor_duty, packet2.motor_duty);
    EXPECT_EQ(packet_in.motor_temp_on_chip, packet2.motor_temp_on_chip);
    EXPECT_EQ(packet_in.motor_temp_ambient, packet2.motor_temp_ambient);
    EXPECT_EQ(packet_in.motor_current, packet2.motor_current);
    EXPECT_EQ(packet_in.motor_rspeed, packet2.motor_rspeed);
    EXPECT_EQ(packet_in.motor_dspeed, packet2.motor_dspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(custom_msg_mavlink, CUSTOM_MSG_BOARD_ARM1)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_BOARD_ARM1 packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.ls_pistol_athead = 53;
    packet_in.ls_pistol_attail = 120;
    packet_in.ls_mass_shifter_athead = 187;
    packet_in.ls_mass_shifter_attail = 254;
    packet_in.altimeter_in_metres = 45.0;
    packet_in.altimeter_in_feet = 73.0;
    packet_in.altimeter_in_fathoms = 101.0;

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_BOARD_ARM1 packet1{};
    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_BOARD_ARM1 packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.ls_pistol_athead, packet2.ls_pistol_athead);
    EXPECT_EQ(packet1.ls_pistol_attail, packet2.ls_pistol_attail);
    EXPECT_EQ(packet1.ls_mass_shifter_athead, packet2.ls_mass_shifter_athead);
    EXPECT_EQ(packet1.ls_mass_shifter_attail, packet2.ls_mass_shifter_attail);
    EXPECT_EQ(packet1.altimeter_in_metres, packet2.altimeter_in_metres);
    EXPECT_EQ(packet1.altimeter_in_feet, packet2.altimeter_in_feet);
    EXPECT_EQ(packet1.altimeter_in_fathoms, packet2.altimeter_in_fathoms);
}

#ifdef TEST_INTEROP
TEST(custom_msg_mavlink_interop, CUSTOM_MSG_BOARD_ARM1)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_custom_msg_board_arm1_t packet_c {
         963497464, 45.0, 73.0, 101.0, 53, 120, 187, 254
    };

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_BOARD_ARM1 packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.ls_pistol_athead = 53;
    packet_in.ls_pistol_attail = 120;
    packet_in.ls_mass_shifter_athead = 187;
    packet_in.ls_mass_shifter_attail = 254;
    packet_in.altimeter_in_metres = 45.0;
    packet_in.altimeter_in_feet = 73.0;
    packet_in.altimeter_in_fathoms = 101.0;

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_BOARD_ARM1 packet2{};

    mavlink_msg_custom_msg_board_arm1_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.ls_pistol_athead, packet2.ls_pistol_athead);
    EXPECT_EQ(packet_in.ls_pistol_attail, packet2.ls_pistol_attail);
    EXPECT_EQ(packet_in.ls_mass_shifter_athead, packet2.ls_mass_shifter_athead);
    EXPECT_EQ(packet_in.ls_mass_shifter_attail, packet2.ls_mass_shifter_attail);
    EXPECT_EQ(packet_in.altimeter_in_metres, packet2.altimeter_in_metres);
    EXPECT_EQ(packet_in.altimeter_in_feet, packet2.altimeter_in_feet);
    EXPECT_EQ(packet_in.altimeter_in_fathoms, packet2.altimeter_in_fathoms);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(custom_msg_mavlink, CUSTOM_MSG_BOARD_ARM2)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_BOARD_ARM2 packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.rudder_position = 45.0;
    packet_in.rudder_speed = 73.0;
    packet_in.rudder_load = 101.0;
    packet_in.rudder_voltage = 129.0;
    packet_in.rudder_temperature = 157.0;
    packet_in.keller_pa3_pressure = 185.0;
    packet_in.keller_pa3_temperature = 213.0;

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_BOARD_ARM2 packet1{};
    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_BOARD_ARM2 packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.rudder_position, packet2.rudder_position);
    EXPECT_EQ(packet1.rudder_speed, packet2.rudder_speed);
    EXPECT_EQ(packet1.rudder_load, packet2.rudder_load);
    EXPECT_EQ(packet1.rudder_voltage, packet2.rudder_voltage);
    EXPECT_EQ(packet1.rudder_temperature, packet2.rudder_temperature);
    EXPECT_EQ(packet1.keller_pa3_pressure, packet2.keller_pa3_pressure);
    EXPECT_EQ(packet1.keller_pa3_temperature, packet2.keller_pa3_temperature);
}

#ifdef TEST_INTEROP
TEST(custom_msg_mavlink_interop, CUSTOM_MSG_BOARD_ARM2)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_custom_msg_board_arm2_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0
    };

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_BOARD_ARM2 packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.rudder_position = 45.0;
    packet_in.rudder_speed = 73.0;
    packet_in.rudder_load = 101.0;
    packet_in.rudder_voltage = 129.0;
    packet_in.rudder_temperature = 157.0;
    packet_in.keller_pa3_pressure = 185.0;
    packet_in.keller_pa3_temperature = 213.0;

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_BOARD_ARM2 packet2{};

    mavlink_msg_custom_msg_board_arm2_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.rudder_position, packet2.rudder_position);
    EXPECT_EQ(packet_in.rudder_speed, packet2.rudder_speed);
    EXPECT_EQ(packet_in.rudder_load, packet2.rudder_load);
    EXPECT_EQ(packet_in.rudder_voltage, packet2.rudder_voltage);
    EXPECT_EQ(packet_in.rudder_temperature, packet2.rudder_temperature);
    EXPECT_EQ(packet_in.keller_pa3_pressure, packet2.keller_pa3_pressure);
    EXPECT_EQ(packet_in.keller_pa3_temperature, packet2.keller_pa3_temperature);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
