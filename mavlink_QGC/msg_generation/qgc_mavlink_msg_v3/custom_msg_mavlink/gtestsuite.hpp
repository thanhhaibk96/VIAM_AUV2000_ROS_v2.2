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
    packet_in.motor_temp = 101.0;
    packet_in.motor_current = 129.0;
    packet_in.motor_rspeed = 157.0;
    packet_in.motor_dspeed = 185.0;

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
    EXPECT_EQ(packet1.motor_temp, packet2.motor_temp);
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
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0
    };

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_PISTOL packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.position = 45.0;
    packet_in.motor_duty = 73.0;
    packet_in.motor_temp = 101.0;
    packet_in.motor_current = 129.0;
    packet_in.motor_rspeed = 157.0;
    packet_in.motor_dspeed = 185.0;

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
    EXPECT_EQ(packet_in.motor_temp, packet2.motor_temp);
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
    packet_in.motor_temp = 101.0;
    packet_in.motor_current = 129.0;
    packet_in.motor_rspeed = 157.0;
    packet_in.motor_dspeed = 185.0;

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
    EXPECT_EQ(packet1.motor_temp, packet2.motor_temp);
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
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0
    };

    mavlink::custom_msg_mavlink::msg::CUSTOM_MSG_MASS_SHIFTER packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.position = 45.0;
    packet_in.motor_duty = 73.0;
    packet_in.motor_temp = 101.0;
    packet_in.motor_current = 129.0;
    packet_in.motor_rspeed = 157.0;
    packet_in.motor_dspeed = 185.0;

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
    EXPECT_EQ(packet_in.motor_temp, packet2.motor_temp);
    EXPECT_EQ(packet_in.motor_current, packet2.motor_current);
    EXPECT_EQ(packet_in.motor_rspeed, packet2.motor_rspeed);
    EXPECT_EQ(packet_in.motor_dspeed, packet2.motor_dspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
