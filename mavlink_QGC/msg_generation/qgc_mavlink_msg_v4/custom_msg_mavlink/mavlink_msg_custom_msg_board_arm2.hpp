// MESSAGE CUSTOM_MSG_BOARD_ARM2 support class

#pragma once

namespace mavlink {
namespace custom_msg_mavlink {
namespace msg {

/**
 * @brief CUSTOM_MSG_BOARD_ARM2 message
 *
 * Information about a board ARM2'status of AUV
 */
struct CUSTOM_MSG_BOARD_ARM2 : mavlink::Message {
    static constexpr msgid_t MSG_ID = 402;
    static constexpr size_t LENGTH = 32;
    static constexpr size_t MIN_LENGTH = 32;
    static constexpr uint8_t CRC_EXTRA = 237;
    static constexpr auto NAME = "CUSTOM_MSG_BOARD_ARM2";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot). */
    float rudder_position; /*<  Position of Rudder */
    float rudder_speed; /*<  Speed of Rudder */
    float rudder_load; /*<  Load of Rudder */
    float rudder_voltage; /*<  Voltage of Rudder */
    float rudder_temperature; /*<  Temperature of Rudder */
    float keller_pa3_pressure; /*<  Pressure of Keller PA3 */
    float keller_pa3_temperature; /*<  Temperature of Keller PA3 */


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
        ss << "  rudder_position: " << rudder_position << std::endl;
        ss << "  rudder_speed: " << rudder_speed << std::endl;
        ss << "  rudder_load: " << rudder_load << std::endl;
        ss << "  rudder_voltage: " << rudder_voltage << std::endl;
        ss << "  rudder_temperature: " << rudder_temperature << std::endl;
        ss << "  keller_pa3_pressure: " << keller_pa3_pressure << std::endl;
        ss << "  keller_pa3_temperature: " << keller_pa3_temperature << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << rudder_position;               // offset: 4
        map << rudder_speed;                  // offset: 8
        map << rudder_load;                   // offset: 12
        map << rudder_voltage;                // offset: 16
        map << rudder_temperature;            // offset: 20
        map << keller_pa3_pressure;           // offset: 24
        map << keller_pa3_temperature;        // offset: 28
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> rudder_position;               // offset: 4
        map >> rudder_speed;                  // offset: 8
        map >> rudder_load;                   // offset: 12
        map >> rudder_voltage;                // offset: 16
        map >> rudder_temperature;            // offset: 20
        map >> keller_pa3_pressure;           // offset: 24
        map >> keller_pa3_temperature;        // offset: 28
    }
};

} // namespace msg
} // namespace custom_msg_mavlink
} // namespace mavlink
