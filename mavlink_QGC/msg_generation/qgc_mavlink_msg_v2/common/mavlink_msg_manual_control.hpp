// MESSAGE MANUAL_CONTROL support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief MANUAL_CONTROL message
 *
 * This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their 
 */
struct MANUAL_CONTROL : mavlink::Message {
    static constexpr msgid_t MSG_ID = 69;
    static constexpr size_t LENGTH = 11;
    static constexpr size_t MIN_LENGTH = 11;
    static constexpr uint8_t CRC_EXTRA = 60;
    static constexpr auto NAME = "MANUAL_CONTROL";


    uint8_t target; /*<  The system to be controlled. */
    int16_t massshifter; /*<  Mass-Shifter-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle. */
    int16_t pistol; /*<  Pistol-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to down(-1000)-up(1000) movement on a joystick and the roll of a vehicle. */
    int16_t rudder; /*<  Rudder-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the yaw of a vehicle. */
    int16_t thruster; /*<  Thruster-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust. */
    uint16_t buttons; /*<  A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1. */


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
        ss << "  target: " << target << std::endl;
        ss << "  massshifter: " << massshifter << std::endl;
        ss << "  pistol: " << pistol << std::endl;
        ss << "  rudder: " << rudder << std::endl;
        ss << "  thruster: " << thruster << std::endl;
        ss << "  buttons: " << buttons << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << massshifter;                   // offset: 0
        map << pistol;                        // offset: 2
        map << rudder;                        // offset: 4
        map << thruster;                      // offset: 6
        map << buttons;                       // offset: 8
        map << target;                        // offset: 10
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> massshifter;                   // offset: 0
        map >> pistol;                        // offset: 2
        map >> rudder;                        // offset: 4
        map >> thruster;                      // offset: 6
        map >> buttons;                       // offset: 8
        map >> target;                        // offset: 10
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
