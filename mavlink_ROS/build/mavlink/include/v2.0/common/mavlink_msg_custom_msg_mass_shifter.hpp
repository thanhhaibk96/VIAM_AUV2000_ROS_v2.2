// MESSAGE CUSTOM_MSG_MASS_SHIFTER support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief CUSTOM_MSG_MASS_SHIFTER message
 *
 * Information about a mass shifter'status of AUV
 */
struct CUSTOM_MSG_MASS_SHIFTER : mavlink::Message {
    static constexpr msgid_t MSG_ID = 397;
    static constexpr size_t LENGTH = 32;
    static constexpr size_t MIN_LENGTH = 32;
    static constexpr uint8_t CRC_EXTRA = 218;
    static constexpr auto NAME = "CUSTOM_MSG_MASS_SHIFTER";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot). */
    float position; /*<  The position of mass shifter in AUV */
    float motor_duty; /*<  The duty cycle (PWM) of mass shifter's motor */
    float motor_temp_on_chip; /*<  The temperature of mass shifter's motor */
    float motor_temp_ambient; /*<  The temperature of mass shifter's motor */
    float motor_current; /*<  The current of mass shifter's motor */
    float motor_rspeed; /*<  The current speed of mass shifter's motor */
    float motor_dspeed; /*<  The desired speed of mass shifter's motor */


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
        ss << "  position: " << position << std::endl;
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
        map << position;                      // offset: 4
        map << motor_duty;                    // offset: 8
        map << motor_temp_on_chip;            // offset: 12
        map << motor_temp_ambient;            // offset: 16
        map << motor_current;                 // offset: 20
        map << motor_rspeed;                  // offset: 24
        map << motor_dspeed;                  // offset: 28
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> position;                      // offset: 4
        map >> motor_duty;                    // offset: 8
        map >> motor_temp_on_chip;            // offset: 12
        map >> motor_temp_ambient;            // offset: 16
        map >> motor_current;                 // offset: 20
        map >> motor_rspeed;                  // offset: 24
        map >> motor_dspeed;                  // offset: 28
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
