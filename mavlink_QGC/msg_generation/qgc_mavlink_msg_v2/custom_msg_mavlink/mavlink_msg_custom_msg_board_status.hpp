// MESSAGE CUSTOM_MSG_BOARD_STATUS support class

#pragma once

namespace mavlink {
namespace custom_msg_mavlink {
namespace msg {

/**
 * @brief CUSTOM_MSG_BOARD_STATUS message
 *
 * Check boards's status of AUV
 */
struct CUSTOM_MSG_BOARD_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 398;
    static constexpr size_t LENGTH = 10;
    static constexpr size_t MIN_LENGTH = 10;
    static constexpr uint8_t CRC_EXTRA = 1;
    static constexpr auto NAME = "CUSTOM_MSG_BOARD_STATUS";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot). */
    uint8_t arm1; /*<  ARM1's status in AUV */
    uint8_t arm2; /*<  ARM2's status in AUV */
    uint8_t viam_navy; /*<  VIAM_NAVY's status in AUV */
    uint8_t pistol; /*<  PISTOL's status in AUV */
    uint8_t mass_shifter; /*<  MASS_SHIFTER's status in AUV */
    uint8_t thruster; /*<  THRUSTER's status in AUV */


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
        ss << "  arm1: " << +arm1 << std::endl;
        ss << "  arm2: " << +arm2 << std::endl;
        ss << "  viam_navy: " << +viam_navy << std::endl;
        ss << "  pistol: " << +pistol << std::endl;
        ss << "  mass_shifter: " << +mass_shifter << std::endl;
        ss << "  thruster: " << +thruster << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << arm1;                          // offset: 4
        map << arm2;                          // offset: 5
        map << viam_navy;                     // offset: 6
        map << pistol;                        // offset: 7
        map << mass_shifter;                  // offset: 8
        map << thruster;                      // offset: 9
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> arm1;                          // offset: 4
        map >> arm2;                          // offset: 5
        map >> viam_navy;                     // offset: 6
        map >> pistol;                        // offset: 7
        map >> mass_shifter;                  // offset: 8
        map >> thruster;                      // offset: 9
    }
};

} // namespace msg
} // namespace custom_msg_mavlink
} // namespace mavlink
