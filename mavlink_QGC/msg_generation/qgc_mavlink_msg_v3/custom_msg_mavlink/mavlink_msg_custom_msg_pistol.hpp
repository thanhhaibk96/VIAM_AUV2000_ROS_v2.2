// MESSAGE CUSTOM_MSG_PISTOL support class

#pragma once

namespace mavlink {
namespace custom_msg_mavlink {
namespace msg {

/**
 * @brief CUSTOM_MSG_PISTOL message
 *
 * Information about a pistol'status of AUV
 */
struct CUSTOM_MSG_PISTOL : mavlink::Message {
    static constexpr msgid_t MSG_ID = 396;
    static constexpr size_t LENGTH = 28;
    static constexpr size_t MIN_LENGTH = 28;
    static constexpr uint8_t CRC_EXTRA = 185;
    static constexpr auto NAME = "CUSTOM_MSG_PISTOL";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot). */
    float position; /*<  The position of Pistol in AUV */
    float motor_duty; /*<  The duty cycle (PWM) of Pistol's motor */
    float motor_temp; /*<  The temperature of Pistol's motor */
    float motor_current; /*<  The current of Pistol's motor */
    float motor_rspeed; /*<  The current speed of Pistol's motor */
    float motor_dspeed; /*<  The desired speed of Pistol's motor */


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
        ss << "  motor_temp: " << motor_temp << std::endl;
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
        map << motor_temp;                    // offset: 12
        map << motor_current;                 // offset: 16
        map << motor_rspeed;                  // offset: 20
        map << motor_dspeed;                  // offset: 24
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> position;                      // offset: 4
        map >> motor_duty;                    // offset: 8
        map >> motor_temp;                    // offset: 12
        map >> motor_current;                 // offset: 16
        map >> motor_rspeed;                  // offset: 20
        map >> motor_dspeed;                  // offset: 24
    }
};

} // namespace msg
} // namespace custom_msg_mavlink
} // namespace mavlink
