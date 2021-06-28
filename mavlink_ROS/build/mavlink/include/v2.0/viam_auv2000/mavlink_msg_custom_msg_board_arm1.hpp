// MESSAGE CUSTOM_MSG_BOARD_ARM1 support class

#pragma once

namespace mavlink {
namespace viam_auv2000 {
namespace msg {

/**
 * @brief CUSTOM_MSG_BOARD_ARM1 message
 *
 * Information about a board ARM1'status of AUV
 */
struct CUSTOM_MSG_BOARD_ARM1 : mavlink::Message {
    static constexpr msgid_t MSG_ID = 399;
    static constexpr size_t LENGTH = 21;
    static constexpr size_t MIN_LENGTH = 21;
    static constexpr uint8_t CRC_EXTRA = 71;
    static constexpr auto NAME = "CUSTOM_MSG_BOARD_ARM1";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot). */
    uint8_t leak_sensor; /*<  Value of Leak Sensors */
    float altimeter_in_metres; /*<  Depth in metres */
    float altimeter_in_feet; /*<  Depth in feet */
    float altimeter_in_fathoms; /*<  Depth in fathoms */
    float roll_motor_angle; /*<  Angle of Motor for Antiing Roll */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  time_boot_ms: " << time_boot_ms << std::endl;
        ss << "  leak_sensor: " << +leak_sensor << std::endl;
        ss << "  altimeter_in_metres: " << altimeter_in_metres << std::endl;
        ss << "  altimeter_in_feet: " << altimeter_in_feet << std::endl;
        ss << "  altimeter_in_fathoms: " << altimeter_in_fathoms << std::endl;
        ss << "  roll_motor_angle: " << roll_motor_angle << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << altimeter_in_metres;           // offset: 4
        map << altimeter_in_feet;             // offset: 8
        map << altimeter_in_fathoms;          // offset: 12
        map << roll_motor_angle;              // offset: 16
        map << leak_sensor;                   // offset: 20
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> altimeter_in_metres;           // offset: 4
        map >> altimeter_in_feet;             // offset: 8
        map >> altimeter_in_fathoms;          // offset: 12
        map >> roll_motor_angle;              // offset: 16
        map >> leak_sensor;                   // offset: 20
    }
};

} // namespace msg
} // namespace viam_auv2000
} // namespace mavlink
