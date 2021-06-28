// MESSAGE CHO_KHOA support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief CHO_KHOA message
 *
 * Information about a component. For camera components instead use CAMERA_INFORMATION, and for autopilots use AUTOPILOT_VERSION. Components including GCSes should consider supporting requests of this message via MAV_CMD_REQUEST_MESSAGE.
 */
struct CHO_KHOA : mavlink::Message {
    static constexpr msgid_t MSG_ID = 396;
    static constexpr size_t LENGTH = 12;
    static constexpr size_t MIN_LENGTH = 12;
    static constexpr uint8_t CRC_EXTRA = 104;
    static constexpr auto NAME = "CHO_KHOA";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot). */
    uint32_t ngu_nhu_bo; /*<  The type of metadata being requested. */
    uint32_t oc_cho; /*<  Unique uid for this metadata which a gcs can use for created cached metadata and understanding whether it's cache it up to date or whether it needs to download new data. */


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
        ss << "  ngu_nhu_bo: " << ngu_nhu_bo << std::endl;
        ss << "  oc_cho: " << oc_cho << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << ngu_nhu_bo;                    // offset: 4
        map << oc_cho;                        // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> ngu_nhu_bo;                    // offset: 4
        map >> oc_cho;                        // offset: 8
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
