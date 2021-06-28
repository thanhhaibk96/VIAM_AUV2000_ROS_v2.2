// MESSAGE CUSTOM_MSG_BOARD_ARM1 support class

#pragma once

namespace mavlink {
namespace custom_msg_mavlink {
namespace msg {

/**
 * @brief CUSTOM_MSG_BOARD_ARM1 message
 *
 * Information about a board ARM1'status of AUV
 */
struct CUSTOM_MSG_BOARD_ARM1 : mavlink::Message {
    static constexpr msgid_t MSG_ID = 399;
    static constexpr size_t LENGTH = 20;
    static constexpr size_t MIN_LENGTH = 20;
    static constexpr uint8_t CRC_EXTRA = 220;
    static constexpr auto NAME = "CUSTOM_MSG_BOARD_ARM1";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot). */
    uint8_t ls_pistol_athead; /*<  Limit switch of Pistol at head */
    uint8_t ls_pistol_attail; /*<  Limit switch of Pistol at tail */
    uint8_t ls_mass_shifter_athead; /*<  Limit switch of Mass-Shifter at head */
    uint8_t ls_mass_shifter_attail; /*<  Limit switch of Mass-Shifter at tail */
    float altimeter_in_metres; /*<  Depth in metres */
    float altimeter_in_feet; /*<  Depth in feet */
    float altimeter_in_fathoms; /*<  Depth in fathoms */


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
        ss << "  ls_pistol_athead: " << +ls_pistol_athead << std::endl;
        ss << "  ls_pistol_attail: " << +ls_pistol_attail << std::endl;
        ss << "  ls_mass_shifter_athead: " << +ls_mass_shifter_athead << std::endl;
        ss << "  ls_mass_shifter_attail: " << +ls_mass_shifter_attail << std::endl;
        ss << "  altimeter_in_metres: " << altimeter_in_metres << std::endl;
        ss << "  altimeter_in_feet: " << altimeter_in_feet << std::endl;
        ss << "  altimeter_in_fathoms: " << altimeter_in_fathoms << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << altimeter_in_metres;           // offset: 4
        map << altimeter_in_feet;             // offset: 8
        map << altimeter_in_fathoms;          // offset: 12
        map << ls_pistol_athead;              // offset: 16
        map << ls_pistol_attail;              // offset: 17
        map << ls_mass_shifter_athead;        // offset: 18
        map << ls_mass_shifter_attail;        // offset: 19
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> altimeter_in_metres;           // offset: 4
        map >> altimeter_in_feet;             // offset: 8
        map >> altimeter_in_fathoms;          // offset: 12
        map >> ls_pistol_athead;              // offset: 16
        map >> ls_pistol_attail;              // offset: 17
        map >> ls_mass_shifter_athead;        // offset: 18
        map >> ls_mass_shifter_attail;        // offset: 19
    }
};

} // namespace msg
} // namespace custom_msg_mavlink
} // namespace mavlink
