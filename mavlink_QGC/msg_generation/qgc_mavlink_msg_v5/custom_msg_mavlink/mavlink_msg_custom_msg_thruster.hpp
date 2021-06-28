// MESSAGE CUSTOM_MSG_THRUSTER support class

#pragma once

namespace mavlink {
namespace custom_msg_mavlink {
namespace msg {

/**
 * @brief CUSTOM_MSG_THRUSTER message
 *
 * Information about a thruster'status of AUV
 */
struct CUSTOM_MSG_THRUSTER : mavlink::Message {
    static constexpr msgid_t MSG_ID = 398;
    static constexpr size_t LENGTH = 28;
    static constexpr size_t MIN_LENGTH = 28;
    static constexpr uint8_t CRC_EXTRA = 49;
    static constexpr auto NAME = "CUSTOM_MSG_THRUSTER";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot). */
    float motor_duty; /*<  The duty cycle (PWM) of thruster's motor */
    float motor_temp_on_chip; /*<  The temperature of thruster's motor */
    float motor_temp_ambient; /*<  The temperature of thruster's motor */
    float motor_current; /*<  The current of thruster's motor */
    float motor_rspeed; /*<  The current speed of mthruster's motor */
    float motor_dspeed; /*<  The desired speed of thruster's motor */


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
        ss << "  motor_duty: " << motor_duty << std::endl;
        ss << "  motor_temp_on_chip: " << motor_temp_on_chip << std::endl;
        ss << "  motor_temp_ambient: " << motor_temp_ambient << std::endl;
        ss << "  motor_current: " << motor_current << std::endl;
        ss << "  motor_rspeed: " << motor_rspeed << std::endl;
        ss << "  motor_dspeed: " << motor_dspeed << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << motor_duty;                    // offset: 4
        map << motor_temp_on_chip;            // offset: 8
        map << motor_temp_ambient;            // offset: 12
        map << motor_current;                 // offset: 16
        map << motor_rspeed;                  // offset: 20
        map << motor_dspeed;                  // offset: 24
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> motor_duty;                    // offset: 4
        map >> motor_temp_on_chip;            // offset: 8
        map >> motor_temp_ambient;            // offset: 12
        map >> motor_current;                 // offset: 16
        map >> motor_rspeed;                  // offset: 20
        map >> motor_dspeed;                  // offset: 24
    }
};

} // namespace msg
} // namespace custom_msg_mavlink
} // namespace mavlink
