// MESSAGE SYS_STATUS support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief SYS_STATUS message
 *
 * The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows whether the system is currently active or not and if an emergency occurred. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occurred it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
 */
struct SYS_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 1;
    static constexpr size_t LENGTH = 48;
    static constexpr size_t MIN_LENGTH = 48;
    static constexpr uint8_t CRC_EXTRA = 173;
    static constexpr auto NAME = "SYS_STATUS";


    uint32_t onboard_control_sensors_present; /*<  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. */
    uint32_t onboard_control_sensors_enabled; /*<  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. */
    uint32_t onboard_control_sensors_health; /*<  Bitmap showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. */
    uint16_t load; /*< [d%] Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000 */
    uint16_t voltage_battery_head; /*< [mV] Battery voltage at head */
    int16_t current_battery_head; /*< [cA] Battery current at head, -1: autopilot does not measure the current */
    int8_t battery_remaining_head; /*< [%] Remaining battery energy at head, -1: autopilot estimate the remaining battery */
    uint16_t batery_total_head; /*< [mAh] Total Capacity of Battery at head, -1: autopilot does not measure the current */
    uint16_t batery_capacity_head; /*< [mAh] Current Capacity of Battery at head, -1: autopilot does not measure the current */
    uint16_t batery_used_head; /*< [mAh] Used Capacity of Battery at head, -1: autopilot does not measure the current */
    uint16_t voltage_battery_tail; /*< [mV] Battery voltage at tail */
    int16_t current_battery_tail; /*< [cA] Battery current at tail, -1: autopilot does not measure the current */
    int8_t battery_remaining_tail; /*< [%] Remaining battery energy at tail, -1: autopilot estimate the remaining battery */
    uint16_t batery_total_tail; /*< [mAh] Total Capacity of Battery at tail, -1: autopilot does not measure the current */
    uint16_t batery_capacity_tail; /*< [mAh] Current Capacity of Battery at tail, -1: autopilot does not measure the current */
    uint16_t batery_used_tail; /*< [mAh] Used Capacity of Battery at tail, -1: autopilot does not measure the current */
    uint16_t drop_rate_comm; /*< [c%] Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV) */
    uint16_t errors_comm; /*<  Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV) */
    uint16_t errors_count1; /*<  Autopilot-specific errors */
    uint16_t errors_count2; /*<  Autopilot-specific errors */
    uint16_t errors_count3; /*<  Autopilot-specific errors */
    uint16_t errors_count4; /*<  Autopilot-specific errors */


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
        ss << "  onboard_control_sensors_present: " << onboard_control_sensors_present << std::endl;
        ss << "  onboard_control_sensors_enabled: " << onboard_control_sensors_enabled << std::endl;
        ss << "  onboard_control_sensors_health: " << onboard_control_sensors_health << std::endl;
        ss << "  load: " << load << std::endl;
        ss << "  voltage_battery_head: " << voltage_battery_head << std::endl;
        ss << "  current_battery_head: " << current_battery_head << std::endl;
        ss << "  battery_remaining_head: " << +battery_remaining_head << std::endl;
        ss << "  batery_total_head: " << batery_total_head << std::endl;
        ss << "  batery_capacity_head: " << batery_capacity_head << std::endl;
        ss << "  batery_used_head: " << batery_used_head << std::endl;
        ss << "  voltage_battery_tail: " << voltage_battery_tail << std::endl;
        ss << "  current_battery_tail: " << current_battery_tail << std::endl;
        ss << "  battery_remaining_tail: " << +battery_remaining_tail << std::endl;
        ss << "  batery_total_tail: " << batery_total_tail << std::endl;
        ss << "  batery_capacity_tail: " << batery_capacity_tail << std::endl;
        ss << "  batery_used_tail: " << batery_used_tail << std::endl;
        ss << "  drop_rate_comm: " << drop_rate_comm << std::endl;
        ss << "  errors_comm: " << errors_comm << std::endl;
        ss << "  errors_count1: " << errors_count1 << std::endl;
        ss << "  errors_count2: " << errors_count2 << std::endl;
        ss << "  errors_count3: " << errors_count3 << std::endl;
        ss << "  errors_count4: " << errors_count4 << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << onboard_control_sensors_present; // offset: 0
        map << onboard_control_sensors_enabled; // offset: 4
        map << onboard_control_sensors_health; // offset: 8
        map << load;                          // offset: 12
        map << voltage_battery_head;          // offset: 14
        map << current_battery_head;          // offset: 16
        map << batery_total_head;             // offset: 18
        map << batery_capacity_head;          // offset: 20
        map << batery_used_head;              // offset: 22
        map << voltage_battery_tail;          // offset: 24
        map << current_battery_tail;          // offset: 26
        map << batery_total_tail;             // offset: 28
        map << batery_capacity_tail;          // offset: 30
        map << batery_used_tail;              // offset: 32
        map << drop_rate_comm;                // offset: 34
        map << errors_comm;                   // offset: 36
        map << errors_count1;                 // offset: 38
        map << errors_count2;                 // offset: 40
        map << errors_count3;                 // offset: 42
        map << errors_count4;                 // offset: 44
        map << battery_remaining_head;        // offset: 46
        map << battery_remaining_tail;        // offset: 47
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> onboard_control_sensors_present; // offset: 0
        map >> onboard_control_sensors_enabled; // offset: 4
        map >> onboard_control_sensors_health; // offset: 8
        map >> load;                          // offset: 12
        map >> voltage_battery_head;          // offset: 14
        map >> current_battery_head;          // offset: 16
        map >> batery_total_head;             // offset: 18
        map >> batery_capacity_head;          // offset: 20
        map >> batery_used_head;              // offset: 22
        map >> voltage_battery_tail;          // offset: 24
        map >> current_battery_tail;          // offset: 26
        map >> batery_total_tail;             // offset: 28
        map >> batery_capacity_tail;          // offset: 30
        map >> batery_used_tail;              // offset: 32
        map >> drop_rate_comm;                // offset: 34
        map >> errors_comm;                   // offset: 36
        map >> errors_count1;                 // offset: 38
        map >> errors_count2;                 // offset: 40
        map >> errors_count3;                 // offset: 42
        map >> errors_count4;                 // offset: 44
        map >> battery_remaining_head;        // offset: 46
        map >> battery_remaining_tail;        // offset: 47
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
