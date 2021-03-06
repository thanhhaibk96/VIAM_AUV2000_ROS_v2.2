/** @file
 *	@brief MAVLink comm protocol generated from ardupilotmega.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace ardupilotmega {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (trought @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 60> MESSAGE_ENTRIES {{ {150, 134, 42, 42, 0, 0, 0}, {151, 219, 8, 8, 3, 6, 7}, {152, 208, 4, 8, 0, 0, 0}, {153, 188, 12, 12, 0, 0, 0}, {154, 84, 15, 15, 3, 6, 7}, {155, 22, 13, 13, 3, 4, 5}, {156, 19, 6, 6, 3, 0, 1}, {157, 21, 15, 15, 3, 12, 13}, {158, 134, 14, 14, 3, 12, 13}, {160, 78, 12, 12, 3, 8, 9}, {161, 68, 3, 3, 3, 0, 1}, {162, 189, 8, 8, 0, 0, 0}, {163, 127, 28, 28, 0, 0, 0}, {164, 154, 44, 44, 0, 0, 0}, {165, 21, 3, 3, 0, 0, 0}, {166, 21, 9, 9, 0, 0, 0}, {167, 144, 22, 22, 0, 0, 0}, {168, 1, 12, 12, 0, 0, 0}, {169, 234, 18, 18, 0, 0, 0}, {170, 73, 34, 34, 0, 0, 0}, {171, 181, 66, 66, 0, 0, 0}, {172, 22, 98, 98, 0, 0, 0}, {173, 83, 8, 8, 0, 0, 0}, {174, 167, 48, 48, 0, 0, 0}, {175, 138, 19, 19, 3, 14, 15}, {176, 234, 3, 3, 3, 0, 1}, {177, 240, 20, 20, 0, 0, 0}, {178, 47, 24, 24, 0, 0, 0}, {179, 189, 29, 29, 1, 26, 0}, {180, 52, 45, 47, 1, 42, 0}, {181, 174, 4, 4, 0, 0, 0}, {182, 229, 40, 40, 0, 0, 0}, {183, 85, 2, 2, 3, 0, 1}, {184, 159, 206, 206, 3, 4, 5}, {185, 186, 7, 7, 3, 4, 5}, {186, 72, 29, 29, 3, 0, 1}, {191, 92, 27, 27, 0, 0, 0}, {192, 36, 44, 50, 0, 0, 0}, {193, 71, 22, 26, 0, 0, 0}, {194, 98, 25, 25, 0, 0, 0}, {195, 120, 37, 37, 0, 0, 0}, {200, 134, 42, 42, 3, 40, 41}, {201, 205, 14, 14, 3, 12, 13}, {214, 69, 8, 8, 3, 6, 7}, {215, 101, 3, 3, 0, 0, 0}, {216, 50, 3, 3, 3, 0, 1}, {217, 202, 6, 6, 0, 0, 0}, {218, 17, 7, 7, 3, 0, 1}, {219, 162, 2, 2, 0, 0, 0}, {226, 207, 8, 8, 0, 0, 0}, {11000, 134, 51, 51, 3, 4, 5}, {11001, 15, 135, 135, 0, 0, 0}, {11002, 234, 179, 179, 3, 4, 5}, {11003, 64, 5, 5, 0, 0, 0}, {11010, 46, 49, 49, 0, 0, 0}, {11011, 106, 44, 44, 0, 0, 0}, {11020, 205, 16, 16, 0, 0, 0}, {11030, 144, 44, 44, 0, 0, 0}, {11031, 133, 44, 44, 0, 0, 0}, {11032, 85, 44, 44, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 2;


// ENUM DEFINITIONS


/** @brief  */
enum class ACCELCAL_VEHICLE_POS
{
    LEVEL=1, /*  | */
    LEFT=2, /*  | */
    RIGHT=3, /*  | */
    NOSEDOWN=4, /*  | */
    NOSEUP=5, /*  | */
    BACK=6, /*  | */
    SUCCESS=16777215, /*  | */
    FAILED=16777216, /*  | */
};

//! ACCELCAL_VEHICLE_POS ENUM_END
constexpr auto ACCELCAL_VEHICLE_POS_ENUM_END = 16777217;

/** @brief Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. */
enum class MAV_CMD
{
    NAV_WAYPOINT=16, /* Navigate to waypoint. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the waypoint counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at waypoint (rotary wing). NaN for unchanged.| Latitude| Longitude| Altitude|  */
    NAV_LOITER_UNLIM=17, /* Loiter around this waypoint an unlimited amount of time |Empty| Empty| Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
    NAV_LOITER_TURNS=18, /* Loiter around this waypoint for X turns |Turns| Empty| Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
    NAV_LOITER_TIME=19, /* Loiter around this waypoint for X seconds |Seconds (decimal)| Empty| Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
    NAV_RETURN_TO_LAUNCH=20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    NAV_LAND=21, /* Land at location. |Minimum target altitude if landing is aborted (0 = undefined/use system default).| Precision land mode.| Empty.| Desired yaw angle. NaN for unchanged.| Latitude.| Longitude.| Landing altitude (ground level in current frame).|  */
    NAV_TAKEOFF=22, /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.| Latitude| Longitude| Altitude|  */
    NAV_LAND_LOCAL=23, /* Land at local position (local frame only) |Landing target number (if available)| Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land| Landing descend rate [ms^-1]| Desired yaw angle [rad]| Y-axis position [m]| X-axis position [m]| Z-axis / ground level position [m]|  */
    NAV_TAKEOFF_LOCAL=24, /* Takeoff from local position (local frame only) |Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]| Empty| Takeoff ascend rate [ms^-1]| Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these| Y-axis position [m]| X-axis position [m]| Z-axis position [m]|  */
    NAV_FOLLOW=25, /* Vehicle following, i.e. this waypoint represents the position of a moving vehicle |Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation| Ground speed of vehicle to be followed| Radius around waypoint, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
    NAV_CONTINUE_AND_CHANGE_ALT=30, /* Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.| Empty| Empty| Empty| Empty| Empty| Desired altitude in meters|  */
    NAV_LOITER_TO_ALT=31, /* Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint. |Heading Required (0 = False)| Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.| Empty| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location| Latitude| Longitude| Altitude|  */
    DO_FOLLOW=32, /* Being following a target |System ID (the system ID of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode| RESERVED| RESERVED| altitude flag: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home| altitude| RESERVED| TTL in seconds in which the MAV should go to the default position hold mode after a message rx timeout|  */
    DO_FOLLOW_REPOSITION=33, /* Reposition the MAV after a follow target command has been sent |Camera q1 (where 0 is on the ray from the camera to the tracking device)| Camera q2| Camera q3| Camera q4| altitude offset from target (m)| X offset from target (m)| Y offset from target (m)|  */
    DO_ORBIT=34, /* Start orbiting on the circumference of a circle defined by the parameters. Setting any value NaN results in using defaults. |Radius of the circle in meters. positive: Orbit clockwise. negative: Orbit counter-clockwise.| Velocity tangential in m/s. NaN: Vehicle configuration default.| Yaw behavior of the vehicle. 0: vehicle front points to the center (default). 1: Hold last heading. 2: Leave yaw uncontrolled.| Reserved (e.g. for dynamic center beacon options)| Center point latitude (if no MAV_FRAME specified) / X coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.| Center point longitude (if no MAV_FRAME specified) / Y coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.| Center point altitude (MSL) (if no MAV_FRAME specified) / Z coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.|  */
    NAV_ROI=80, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
    NAV_PATHPLANNING=81, /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
    NAV_SPLINE_WAYPOINT=82, /* Navigate to waypoint using a spline path. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at waypoint for rotary wing)| Empty| Empty| Empty| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
    NAV_ALTITUDE_WAIT=83, /* Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up. |Altitude (m).| Descent speed (m/s).| Wiggle Time (s).| Empty.| Empty.| Empty.| Empty.|  */
    NAV_VTOL_TAKEOFF=84, /* Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. |Empty| Front transition heading.| Empty| Yaw angle in degrees. NaN for unchanged.| Latitude| Longitude| Altitude|  */
    NAV_VTOL_LAND=85, /* Land using VTOL mode |Empty| Empty| Approach altitude (with the same reference as the Altitude field). NaN if unspecified.| Yaw angle in degrees. NaN for unchanged.| Latitude| Longitude| Altitude (ground level)|  */
    NAV_GUIDED_ENABLE=92, /* hand control over to an external controller |On / Off (> 0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    NAV_DELAY=93, /* Delay the next navigation command a number of seconds or until a specified time |Delay in seconds (decimal, -1 to enable time-of-day fields)| hour (24h format, UTC, -1 to ignore)| minute (24h format, UTC, -1 to ignore)| second (24h format, UTC)| Empty| Empty| Empty|  */
    NAV_PAYLOAD_PLACE=94, /* Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload. |Maximum distance to descend.| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
    NAV_LAST=95, /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    CONDITION_DELAY=112, /* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    CONDITION_CHANGE_ALT=113, /* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */
    CONDITION_DISTANCE=114, /* Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    CONDITION_YAW=115, /* Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  */
    CONDITION_LAST=159, /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_MODE=176, /* Set system mode. |Mode, as defined by ENUM MAV_MODE| Custom mode - this is system specific, please refer to the individual autopilot specifications for details.| Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.| Empty| Empty| Empty| Empty|  */
    DO_JUMP=177, /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
    DO_CHANGE_SPEED=178, /* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| absolute or relative [0,1]| Empty| Empty| Empty|  */
    DO_SET_HOME=179, /* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
    DO_SET_PARAMETER=180, /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_RELAY=181, /* Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
    DO_REPEAT_RELAY=182, /* Cycle a relay on and off for a desired number of cycles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  */
    DO_SET_SERVO=183, /* Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  */
    DO_REPEAT_SERVO=184, /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  */
    DO_FLIGHTTERMINATION=185, /* Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_CHANGE_ALTITUDE=186, /* Change altitude set point. |Altitude in meters| Mav frame of new altitude (see MAV_FRAME)| Empty| Empty| Empty| Empty| Empty|  */
    DO_LAND_START=189, /* Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence. |Empty| Empty| Empty| Empty| Latitude| Longitude| Empty|  */
    DO_RALLY_LAND=190, /* Mission command to perform a landing from a rally point. |Break altitude (meters)| Landing speed (m/s)| Empty| Empty| Empty| Empty| Empty|  */
    DO_GO_AROUND=191, /* Mission command to safely abort an autonomous landing. |Altitude (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_REPOSITION=192, /* Reposition the vehicle to a specific WGS84 global position. |Ground speed, less than 0 (-1) for default| Bitmask of option flags, see the MAV_DO_REPOSITION_FLAGS enum.| Reserved| Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)| Latitude (deg * 1E7)| Longitude (deg * 1E7)| Altitude (meters)|  */
    DO_PAUSE_CONTINUE=193, /* If in a GPS controlled position mode, hold the current position or continue. |0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    DO_SET_REVERSE=194, /* Set moving direction to forward or reverse. |Direction (0=Forward, 1=Reverse)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_ROI_LOCATION=195, /* Sets the region of interest (ROI) to a location. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
    DO_SET_ROI_WPNEXT_OFFSET=196, /* Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| pitch offset from next waypoint| roll offset from next waypoint| yaw offset from next waypoint|  */
    DO_SET_ROI_NONE=197, /* Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_CONTROL_VIDEO=200, /* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
    DO_SET_ROI=201, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude| MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude| MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude|  */
    DO_DIGICAM_CONFIGURE=202, /* Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|  */
    DO_DIGICAM_CONTROL=203, /* Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.|  */
    DO_MOUNT_CONFIGURE=204, /* Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)| pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)| yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)|  */
    DO_MOUNT_CONTROL=205, /* Mission command to control a camera or antenna mount |pitch depending on mount mode (degrees or degrees/second depending on pitch input).| roll depending on mount mode (degrees or degrees/second depending on roll input).| yaw depending on mount mode (degrees or degrees/second depending on yaw input).| alt in meters depending on mount mode.| latitude in degrees * 1E7, set if appropriate mount mode.| longitude in degrees * 1E7, set if appropriate mount mode.| MAV_MOUNT_MODE enum value|  */
    DO_SET_CAM_TRIGG_DIST=206, /* Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera. |Camera trigger distance (meters). 0 to stop triggering.| Camera shutter integration time (milliseconds). -1 or 0 to ignore| Trigger camera once immediately. (0 = no trigger, 1 = trigger)| Empty| Empty| Empty| Empty|  */
    DO_FENCE_ENABLE=207, /* Mission command to enable the geofence |enable? (0=disable, 1=enable, 2=disable_floor_only)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_PARACHUTE=208, /* Mission command to trigger a parachute |action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_MOTOR_TEST=209, /* Mission command to perform motor test |motor number (a number from 1 to max number of motors on the vehicle)| throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)| throttle| timeout (in seconds)| motor count (number of motors to test to test in sequence, waiting for the timeout above between them; 0=1 motor, 1=1 motor, 2=2 motors...)| motor test order (See MOTOR_TEST_ORDER enum)| Empty|  */
    DO_INVERTED_FLIGHT=210, /* Change to/from inverted flight |inverted (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_GRIPPER=211, /* Mission command to operate EPM gripper. |Gripper number (a number from 1 to max number of grippers on the vehicle).| Gripper action (0=release, 1=grab. See GRIPPER_ACTIONS enum).| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    DO_AUTOTUNE_ENABLE=212, /* Enable/disable autotune. |Enable (1: enable, 0:disable).| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    NAV_SET_YAW_SPEED=213, /* Sets a desired vehicle turn angle and speed change |yaw angle to adjust steering by in centidegress| speed - normalized to 0 .. 1| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_CAM_TRIGG_INTERVAL=214, /* Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera. |Camera trigger cycle time (milliseconds). -1 or 0 to ignore.| Camera shutter integration time (milliseconds). Should be less than trigger cycle time. -1 or 0 to ignore.| Empty| Empty| Empty| Empty| Empty|  */
    DO_MOUNT_CONTROL_QUAT=220, /* Mission command to control a camera or antenna mount, using a quaternion as reference. |q1 - quaternion param #1, w (1 in null-rotation)| q2 - quaternion param #2, x (0 in null-rotation)| q3 - quaternion param #3, y (0 in null-rotation)| q4 - quaternion param #4, z (0 in null-rotation)| Empty| Empty| Empty|  */
    DO_GUIDED_MASTER=221, /* set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  */
    DO_GUIDED_LIMITS=222, /* Set limits for external control |Timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout.| Altitude (MSL) min, in meters - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit.| Altitude (MSL) max, in meters - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit.| Horizontal move limit, in meters - if vehicle moves more than this distance from its location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal move limit.| Empty| Empty| Empty|  */
    DO_ENGINE_CONTROL=223, /* Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines |0: Stop engine, 1:Start Engine| 0: Warm start, 1:Cold start. Controls use of choke where applicable| Height delay (meters). This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.| Empty| Empty| Empty| Empty|  */
    DO_SET_MISSION_CURRENT=224, /* Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between). |Mission sequence value to set| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_LAST=240, /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero. |1: gyro calibration, 3: gyro temperature calibration| 1: magnetometer calibration| 1: ground pressure calibration| 1: radio RC calibration, 2: RC trim calibration| 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration| 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration| 1: ESC calibration, 3: barometer temperature calibration|  */
    PREFLIGHT_SET_SENSOR_OFFSETS=242, /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
    PREFLIGHT_UAVCAN=243, /* Trigger UAVCAN config. This command will be only accepted if in pre-flight mode. |1: Trigger actuator ID assignment and direction mapping.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    PREFLIGHT_STORAGE=245, /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz logging)| Reserved| Empty| Empty| Empty|  */
    PREFLIGHT_REBOOT_SHUTDOWN=246, /* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.| WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded| WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded| Reserved, send 0| Reserved, send 0| WIP: ID (e.g. camera ID -1 for all IDs)|  */
    OVERRIDE_GOTO=252, /* Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  */
    MISSION_START=300, /* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    COMPONENT_ARM_DISARM=400, /* Arms / Disarms a component |1 to arm, 0 to disarm| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    GET_HOME_POSITION=410, /* Request the home position from the vehicle. |Reserved| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    START_RX_PAIR=500, /* Starts receiver pairing |0:Spektrum| RC type (see RC_TYPE enum)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    GET_MESSAGE_INTERVAL=510, /* Request the interval between messages for a particular MAVLink message ID |The MAVLink message ID| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    SET_MESSAGE_INTERVAL=511, /* Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM |The MAVLink message ID| The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_MESSAGE=512, /* Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL). |The MAVLink message ID of the requested message.| Index id (if appropriate). The use of this parameter (if any), must be defined in the requested message.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_PROTOCOL_VERSION=519, /* Request MAVLink protocol version compatibility |1: Request supported protocol versions by all nodes on the network| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_AUTOPILOT_CAPABILITIES=520, /* Request autopilot capabilities |1: Request autopilot version| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_CAMERA_INFORMATION=521, /* Request camera information (CAMERA_INFORMATION). |0: No action 1: Request camera capabilities| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_CAMERA_SETTINGS=522, /* Request camera settings (CAMERA_SETTINGS). |0: No Action 1: Request camera settings| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_STORAGE_INFORMATION=525, /* Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage. |Storage ID (0 for all, 1 for first, 2 for second, etc.)| 0: No Action 1: Request storage information| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    STORAGE_FORMAT=526, /* Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage. |Storage ID (1 for first, 2 for second, etc.)| 0: No action 1: Format storage| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_CAMERA_CAPTURE_STATUS=527, /* Request camera capture status (CAMERA_CAPTURE_STATUS) |0: No Action 1: Request camera capture status| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_FLIGHT_INFORMATION=528, /* Request flight information (FLIGHT_INFORMATION) |1: Request flight information| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    RESET_CAMERA_SETTINGS=529, /* Reset all camera settings to Factory Default |0: No Action 1: Reset all settings| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    SET_CAMERA_MODE=530, /* Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming. |Reserved (Set to 0)| Camera mode| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    SET_CAMERA_ZOOM=531, /* Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success). Use NaN for reserved values. |Zoom type| Zoom value. The range of valid values depend on the zoom type.| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    SET_CAMERA_FOCUS=532, /* Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success). Use NaN for reserved values. |Focus type| Focus value| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    SET_AUV_CONTROLLER=533, /* Set AUV Controller:
                      HEADING_PID : 1
                      HEADING_SM : 2
                      PITCH_PID : 3
                      PITCH_SM : 4
                      DEPTH_PID : 5
                      DEPTH_SM : 6
                      LOS_PID : 7
                      LOS_SM : 8
                      SPEED_PID : 9
                      SPEED_SM : 10
                      ROLL_PID : 11
                      ROLL_SM : 12 |Select type of Controller| 0: Disable, 1: Enable| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    ENABLE_AUV_LIGHT=534, /* Enable/Disable Light Of AUV |0: Disable, 1: Enable| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    JUMP_TAG=600, /* Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG. |Tag.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    DO_JUMP_TAG=601, /* Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number. |Target tag to jump to.| Repeat count| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    IMAGE_START_CAPTURE=2000, /* Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values. |Reserved (Set to 0)| Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on hardware (typically greater than 2 seconds).| Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE.| Capture sequence number starting from 1. This is only valid for single-capture (param3 == 1). Increment the capture ID for each capture command to prevent double captures when a command is re-transmitted. Use 0 to ignore it.| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)|  */
    IMAGE_STOP_CAPTURE=2001, /* Stop image capture sequence Use NaN for reserved values. |Reserved (Set to 0)| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_CAMERA_IMAGE_CAPTURE=2002, /* Re-request a CAMERA_IMAGE_CAPTURE message. Use NaN for reserved values. |Sequence number for missing CAMERA_IMAGE_CAPTURE message| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    DO_TRIGGER_CONTROL=2003, /* Enable or disable on-board camera triggering system. |Trigger enable/disable (0 for disable, 1 for start), -1 to ignore| 1 to reset the trigger sequence, -1 or 0 to ignore| 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    VIDEO_START_CAPTURE=2500, /* Starts video capture (recording). Use NaN for reserved values. |Video Stream ID (0 for all streams)| Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency in Hz)| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    VIDEO_STOP_CAPTURE=2501, /* Stop the current video capture (recording). Use NaN for reserved values. |Video Stream ID (0 for all streams)| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    VIDEO_START_STREAMING=2502, /* Start video streaming |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Reserved| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    VIDEO_STOP_STREAMING=2503, /* Stop the given video stream |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Reserved| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_VIDEO_STREAM_INFORMATION=2504, /* Request video stream information (VIDEO_STREAM_INFORMATION) |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_VIDEO_STREAM_STATUS=2505, /* Request video stream status (VIDEO_STREAM_STATUS) |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    LOGGING_START=2510, /* Request to start streaming logging data over MAVLink (see also LOGGING_DATA message) |Format: 0: ULog| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
    LOGGING_STOP=2511, /* Request to stop streaming log data over MAVLink |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
    AIRFRAME_CONFIGURATION=2520, /*  |Landing gear ID (default: 0, -1 for all)| Landing gear position (Down: 0, Up: 1, NaN for no change)| Reserved, set to NaN| Reserved, set to NaN| Reserved, set to NaN| Reserved, set to NaN| Reserved, set to NaN|  */
    CONTROL_HIGH_LATENCY=2600, /* Request to start/stop transmitting over the high latency telemetry |Control transmission over high latency telemetry (0: stop, 1: start)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    PANORAMA_CREATE=2800, /* Create a panorama at the current position |Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)| Viewing angle vertical of panorama (in degrees)| Speed of the horizontal rotation (in degrees per second)| Speed of the vertical rotation (in degrees per second)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    DO_VTOL_TRANSITION=3000, /* Request VTOL transition |The target VTOL state, as defined by ENUM MAV_VTOL_STATE. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    ARM_AUTHORIZATION_REQUEST=3001, /* Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
         |Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    SET_GUIDED_SUBMODE_STANDARD=4000, /* This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.
                   |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    SET_GUIDED_SUBMODE_CIRCLE=4001, /* This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
                   |Radius of desired circle in CIRCLE_MODE| User defined| User defined| User defined| Unscaled target latitude of center of circle in CIRCLE_MODE| Unscaled target longitude of center of circle in CIRCLE_MODE| Reserved (default:0)|  */
    CONDITION_GATE=4501, /* Delay mission state machine until gate has been reached. |Geometry: 0: orthogonal to path between previous and next waypoint.| Altitude: 0: ignore altitude| Empty| Empty| Latitude| Longitude| Altitude|  */
    NAV_FENCE_RETURN_POINT=5000, /* Fence return point. There can only be one fence return point.
         |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  */
    NAV_FENCE_POLYGON_VERTEX_INCLUSION=5001, /* Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
         |Polygon vertex count| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
    NAV_FENCE_POLYGON_VERTEX_EXCLUSION=5002, /* Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
         |Polygon vertex count| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
    NAV_FENCE_CIRCLE_INCLUSION=5003, /* Circular fence area. The vehicle must stay inside this area.
         |radius in meters| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
    NAV_FENCE_CIRCLE_EXCLUSION=5004, /* Circular fence area. The vehicle must stay outside this area.
         |radius in meters| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
    NAV_RALLY_POINT=5100, /* Rally point. You can have multiple rally points defined.
         |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  */
    UAVCAN_GET_NODE_INFO=5200, /* Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages. |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
    PAYLOAD_PREPARE_DEPLOY=30001, /* Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.| Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.| Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Altitude (MSL), in meters|  */
    PAYLOAD_CONTROL_DEPLOY=30002, /* Control the payload deployment. |Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deployment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    WAYPOINT_USER_1=31000, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
    WAYPOINT_USER_2=31001, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
    WAYPOINT_USER_3=31002, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
    WAYPOINT_USER_4=31003, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
    WAYPOINT_USER_5=31004, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
    SPATIAL_USER_1=31005, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
    SPATIAL_USER_2=31006, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
    SPATIAL_USER_3=31007, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
    SPATIAL_USER_4=31008, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
    SPATIAL_USER_5=31009, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL), in meters|  */
    USER_1=31010, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_2=31011, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_3=31012, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_4=31013, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_5=31014, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    POWER_OFF_INITIATED=42000, /* A system wide power-off event has been initiated. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    SOLO_BTN_FLY_CLICK=42001, /* FLY button has been clicked. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    SOLO_BTN_FLY_HOLD=42002, /* FLY button has been held for 1.5 seconds. |Takeoff altitude.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    SOLO_BTN_PAUSE_CLICK=42003, /* PAUSE button has been clicked. |1 if Solo is in a shot mode, 0 otherwise.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    FIXED_MAG_CAL=42004, /* Magnetometer calibration based on fixed position
        in earth field given by inclination, declination and intensity. |MagDeclinationDegrees.| MagInclinationDegrees.| MagIntensityMilliGauss.| YawDegrees.| Empty.| Empty.| Empty.|  */
    FIXED_MAG_CAL_FIELD=42005, /* Magnetometer calibration based on fixed expected field values in milliGauss. |FieldX.| FieldY.| FieldZ.| Empty.| Empty.| Empty.| Empty.|  */
    DO_START_MAG_CAL=42424, /* Initiate a magnetometer calibration. |uint8_t bitmask of magnetometers (0 means all).| Automatically retry on failure (0=no retry, 1=retry).| Save without user input (0=require input, 1=autosave).| Delay (seconds).| Autoreboot (0=user reboot, 1=autoreboot).| Empty.| Empty.|  */
    DO_ACCEPT_MAG_CAL=42425, /* Initiate a magnetometer calibration. |uint8_t bitmask of magnetometers (0 means all).| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    DO_CANCEL_MAG_CAL=42426, /* Cancel a running magnetometer calibration. |uint8_t bitmask of magnetometers (0 means all).| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    SET_FACTORY_TEST_MODE=42427, /* Command autopilot to get into factory test/diagnostic mode. |0 means get out of test mode, 1 means get into test mode.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    DO_SEND_BANNER=42428, /* Reply with the version banner. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    ACCELCAL_VEHICLE_POS=42429, /* Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the vehicle in. When sent to the vehicle says what position the vehicle is in. |Position, one of the ACCELCAL_VEHICLE_POS enum values.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    GIMBAL_RESET=42501, /* Causes the gimbal to reset and boot as if it was just powered on. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    GIMBAL_AXIS_CALIBRATION_STATUS=42502, /* Reports progress and success or failure of gimbal axis calibration procedure. |Gimbal axis we're reporting calibration progress for.| Current calibration progress for this axis, 0x64=100%.| Status of the calibration.| Empty.| Empty.| Empty.| Empty.|  */
    GIMBAL_REQUEST_AXIS_CALIBRATION=42503, /* Starts commutation calibration on the gimbal. |Empty.| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    GIMBAL_FULL_RESET=42505, /* Erases gimbal application and parameters. |Magic number.| Magic number.| Magic number.| Magic number.| Magic number.| Magic number.| Magic number.|  */
    DO_WINCH=42600, /* Command to operate winch. |Winch number (0 for the default winch, otherwise a number from 1 to max number of winches on the vehicle).| Action (0=relax, 1=relative length control, 2=rate control. See WINCH_ACTIONS enum.).| Release length (cable distance to unwind in meters, negative numbers to wind in cable).| Release rate (meters/second).| Empty.| Empty.| Empty.|  */
    FLASH_BOOTLOADER=42650, /* Update the bootloader |Empty| Empty| Empty| Empty| Magic number - set to 290876 to actually flash| Empty| Empty|  */
};

//! MAV_CMD ENUM_END
constexpr auto MAV_CMD_ENUM_END = 42651;

/** @brief  */
enum class LIMITS_STATE : uint8_t
{
    INIT=0, /* Pre-initialization. | */
    DISABLED=1, /* Disabled. | */
    ENABLED=2, /* Checking limits. | */
    TRIGGERED=3, /* A limit has been breached. | */
    RECOVERING=4, /* Taking action e.g. Return/RTL. | */
    RECOVERED=5, /* We're no longer in breach of a limit. | */
};

//! LIMITS_STATE ENUM_END
constexpr auto LIMITS_STATE_ENUM_END = 6;

/** @brief  */
enum class LIMIT_MODULE : uint8_t
{
    GPSLOCK=1, /* Pre-initialization. | */
    GEOFENCE=2, /* Disabled. | */
    ALTITUDE=4, /* Checking limits. | */
};

//! LIMIT_MODULE ENUM_END
constexpr auto LIMIT_MODULE_ENUM_END = 5;

/** @brief Flags in RALLY_POINT message. */
enum class RALLY_FLAGS : uint8_t
{
    FAVORABLE_WIND=1, /* Flag set when requiring favorable winds for landing. | */
    LAND_IMMEDIATELY=2, /* Flag set when plane is to immediately descend to break altitude and land without GCS intervention. Flag not set when plane is to loiter at Rally point until commanded to land. | */
};

//! RALLY_FLAGS ENUM_END
constexpr auto RALLY_FLAGS_ENUM_END = 3;

/** @brief  */
enum class PARACHUTE_ACTION
{
    DISABLE=0, /* Disable parachute release. | */
    ENABLE=1, /* Enable parachute release. | */
    RELEASE=2, /* Release parachute. | */
};

//! PARACHUTE_ACTION ENUM_END
constexpr auto PARACHUTE_ACTION_ENUM_END = 3;

/** @brief Gripper actions. */
enum class GRIPPER_ACTIONS
{
    ACTION_RELEASE=0, /* Gripper release cargo. | */
    ACTION_GRAB=1, /* Gripper grab onto cargo. | */
};

//! GRIPPER_ACTIONS ENUM_END
constexpr auto GRIPPER_ACTIONS_ENUM_END = 2;

/** @brief Winch actions. */
enum class WINCH_ACTIONS
{
    RELAXED=0, /* Relax winch. | */
    RELATIVE_LENGTH_CONTROL=1, /* Winch unwinds or winds specified length of cable optionally using specified rate. | */
    RATE_CONTROL=2, /* Winch unwinds or winds cable at specified rate in meters/seconds. | */
};

//! WINCH_ACTIONS ENUM_END
constexpr auto WINCH_ACTIONS_ENUM_END = 3;

/** @brief  */
enum class CAMERA_STATUS_TYPES : uint8_t
{
    TYPE_HEARTBEAT=0, /* Camera heartbeat, announce camera component ID at 1Hz. | */
    TYPE_TRIGGER=1, /* Camera image triggered. | */
    TYPE_DISCONNECT=2, /* Camera connection lost. | */
    TYPE_ERROR=3, /* Camera unknown error. | */
    TYPE_LOWBATT=4, /* Camera battery low. Parameter p1 shows reported voltage. | */
    TYPE_LOWSTORE=5, /* Camera storage low. Parameter p1 shows reported shots remaining. | */
    TYPE_LOWSTOREV=6, /* Camera storage low. Parameter p1 shows reported video minutes remaining. | */
};

//! CAMERA_STATUS_TYPES ENUM_END
constexpr auto CAMERA_STATUS_TYPES_ENUM_END = 7;

/** @brief  */
enum class CAMERA_FEEDBACK_FLAGS : uint8_t
{
    PHOTO=0, /* Shooting photos, not video. | */
    VIDEO=1, /* Shooting video, not stills. | */
    BADEXPOSURE=2, /* Unable to achieve requested exposure (e.g. shutter speed too low). | */
    CLOSEDLOOP=3, /* Closed loop feedback from camera, we know for sure it has successfully taken a picture. | */
    OPENLOOP=4, /* Open loop camera, an image trigger has been requested but we can't know for sure it has successfully taken a picture. | */
};

//! CAMERA_FEEDBACK_FLAGS ENUM_END
constexpr auto CAMERA_FEEDBACK_FLAGS_ENUM_END = 5;

/** @brief  */
enum class MAV_MODE_GIMBAL
{
    UNINITIALIZED=0, /* Gimbal is powered on but has not started initializing yet. | */
    CALIBRATING_PITCH=1, /* Gimbal is currently running calibration on the pitch axis. | */
    CALIBRATING_ROLL=2, /* Gimbal is currently running calibration on the roll axis. | */
    CALIBRATING_YAW=3, /* Gimbal is currently running calibration on the yaw axis. | */
    INITIALIZED=4, /* Gimbal has finished calibrating and initializing, but is relaxed pending reception of first rate command from copter. | */
    ACTIVE=5, /* Gimbal is actively stabilizing. | */
    RATE_CMD_TIMEOUT=6, /* Gimbal is relaxed because it missed more than 10 expected rate command messages in a row. Gimbal will move back to active mode when it receives a new rate command. | */
};

//! MAV_MODE_GIMBAL ENUM_END
constexpr auto MAV_MODE_GIMBAL_ENUM_END = 7;

/** @brief  */
enum class GIMBAL_AXIS
{
    YAW=0, /* Gimbal yaw axis. | */
    PITCH=1, /* Gimbal pitch axis. | */
    ROLL=2, /* Gimbal roll axis. | */
};

//! GIMBAL_AXIS ENUM_END
constexpr auto GIMBAL_AXIS_ENUM_END = 3;

/** @brief  */
enum class GIMBAL_AXIS_CALIBRATION_STATUS
{
    IN_PROGRESS=0, /* Axis calibration is in progress. | */
    SUCCEEDED=1, /* Axis calibration succeeded. | */
    FAILED=2, /* Axis calibration failed. | */
};

//! GIMBAL_AXIS_CALIBRATION_STATUS ENUM_END
constexpr auto GIMBAL_AXIS_CALIBRATION_STATUS_ENUM_END = 3;

/** @brief  */
enum class GIMBAL_AXIS_CALIBRATION_REQUIRED
{
    UNKNOWN=0, /* Whether or not this axis requires calibration is unknown at this time. | */
    TRUE=1, /* This axis requires calibration. | */
    FALSE=2, /* This axis does not require calibration. | */
};

//! GIMBAL_AXIS_CALIBRATION_REQUIRED ENUM_END
constexpr auto GIMBAL_AXIS_CALIBRATION_REQUIRED_ENUM_END = 3;

/** @brief  */
enum class GOPRO_HEARTBEAT_STATUS : uint8_t
{
    DISCONNECTED=0, /* No GoPro connected. | */
    INCOMPATIBLE=1, /* The detected GoPro is not HeroBus compatible. | */
    CONNECTED=2, /* A HeroBus compatible GoPro is connected. | */
    ERROR=3, /* An unrecoverable error was encountered with the connected GoPro, it may require a power cycle. | */
};

//! GOPRO_HEARTBEAT_STATUS ENUM_END
constexpr auto GOPRO_HEARTBEAT_STATUS_ENUM_END = 4;

/** @brief  */
enum class GOPRO_HEARTBEAT_FLAGS : uint8_t
{
    FLAG_RECORDING=1, /* GoPro is currently recording. | */
};

//! GOPRO_HEARTBEAT_FLAGS ENUM_END
constexpr auto GOPRO_HEARTBEAT_FLAGS_ENUM_END = 2;

/** @brief  */
enum class GOPRO_REQUEST_STATUS : uint8_t
{
    SUCCESS=0, /* The write message with ID indicated succeeded. | */
    FAILED=1, /* The write message with ID indicated failed. | */
};

//! GOPRO_REQUEST_STATUS ENUM_END
constexpr auto GOPRO_REQUEST_STATUS_ENUM_END = 2;

/** @brief  */
enum class GOPRO_COMMAND : uint8_t
{
    POWER=0, /* (Get/Set). | */
    CAPTURE_MODE=1, /* (Get/Set). | */
    SHUTTER=2, /* (___/Set). | */
    BATTERY=3, /* (Get/___). | */
    MODEL=4, /* (Get/___). | */
    VIDEO_SETTINGS=5, /* (Get/Set). | */
    LOW_LIGHT=6, /* (Get/Set). | */
    PHOTO_RESOLUTION=7, /* (Get/Set). | */
    PHOTO_BURST_RATE=8, /* (Get/Set). | */
    PROTUNE=9, /* (Get/Set). | */
    PROTUNE_WHITE_BALANCE=10, /* (Get/Set) Hero 3+ Only. | */
    PROTUNE_COLOUR=11, /* (Get/Set) Hero 3+ Only. | */
    PROTUNE_GAIN=12, /* (Get/Set) Hero 3+ Only. | */
    PROTUNE_SHARPNESS=13, /* (Get/Set) Hero 3+ Only. | */
    PROTUNE_EXPOSURE=14, /* (Get/Set) Hero 3+ Only. | */
    TIME=15, /* (Get/Set). | */
    CHARGING=16, /* (Get/Set). | */
};

//! GOPRO_COMMAND ENUM_END
constexpr auto GOPRO_COMMAND_ENUM_END = 17;

/** @brief  */
enum class GOPRO_CAPTURE_MODE : uint8_t
{
    VIDEO=0, /* Video mode. | */
    PHOTO=1, /* Photo mode. | */
    BURST=2, /* Burst mode, Hero 3+ only. | */
    TIME_LAPSE=3, /* Time lapse mode, Hero 3+ only. | */
    MULTI_SHOT=4, /* Multi shot mode, Hero 4 only. | */
    PLAYBACK=5, /* Playback mode, Hero 4 only, silver only except when LCD or HDMI is connected to black. | */
    SETUP=6, /* Playback mode, Hero 4 only. | */
    UNKNOWN=255, /* Mode not yet known. | */
};

//! GOPRO_CAPTURE_MODE ENUM_END
constexpr auto GOPRO_CAPTURE_MODE_ENUM_END = 256;

/** @brief  */
enum class GOPRO_RESOLUTION
{
    RESOLUTION_480p=0, /* 848 x 480 (480p). | */
    RESOLUTION_720p=1, /* 1280 x 720 (720p). | */
    RESOLUTION_960p=2, /* 1280 x 960 (960p). | */
    RESOLUTION_1080p=3, /* 1920 x 1080 (1080p). | */
    RESOLUTION_1440p=4, /* 1920 x 1440 (1440p). | */
    RESOLUTION_2_7k_17_9=5, /* 2704 x 1440 (2.7k-17:9). | */
    RESOLUTION_2_7k_16_9=6, /* 2704 x 1524 (2.7k-16:9). | */
    RESOLUTION_2_7k_4_3=7, /* 2704 x 2028 (2.7k-4:3). | */
    RESOLUTION_4k_16_9=8, /* 3840 x 2160 (4k-16:9). | */
    RESOLUTION_4k_17_9=9, /* 4096 x 2160 (4k-17:9). | */
    RESOLUTION_720p_SUPERVIEW=10, /* 1280 x 720 (720p-SuperView). | */
    RESOLUTION_1080p_SUPERVIEW=11, /* 1920 x 1080 (1080p-SuperView). | */
    RESOLUTION_2_7k_SUPERVIEW=12, /* 2704 x 1520 (2.7k-SuperView). | */
    RESOLUTION_4k_SUPERVIEW=13, /* 3840 x 2160 (4k-SuperView). | */
};

//! GOPRO_RESOLUTION ENUM_END
constexpr auto GOPRO_RESOLUTION_ENUM_END = 14;

/** @brief  */
enum class GOPRO_FRAME_RATE
{
    RATE_12=0, /* 12 FPS. | */
    RATE_15=1, /* 15 FPS. | */
    RATE_24=2, /* 24 FPS. | */
    RATE_25=3, /* 25 FPS. | */
    RATE_30=4, /* 30 FPS. | */
    RATE_48=5, /* 48 FPS. | */
    RATE_50=6, /* 50 FPS. | */
    RATE_60=7, /* 60 FPS. | */
    RATE_80=8, /* 80 FPS. | */
    RATE_90=9, /* 90 FPS. | */
    RATE_100=10, /* 100 FPS. | */
    RATE_120=11, /* 120 FPS. | */
    RATE_240=12, /* 240 FPS. | */
    RATE_12_5=13, /* 12.5 FPS. | */
};

//! GOPRO_FRAME_RATE ENUM_END
constexpr auto GOPRO_FRAME_RATE_ENUM_END = 14;

/** @brief  */
enum class GOPRO_FIELD_OF_VIEW
{
    WIDE=0, /* 0x00: Wide. | */
    MEDIUM=1, /* 0x01: Medium. | */
    NARROW=2, /* 0x02: Narrow. | */
};

//! GOPRO_FIELD_OF_VIEW ENUM_END
constexpr auto GOPRO_FIELD_OF_VIEW_ENUM_END = 3;

/** @brief  */
enum class GOPRO_VIDEO_SETTINGS_FLAGS
{
    TV_MODE=1, /* 0=NTSC, 1=PAL. | */
};

//! GOPRO_VIDEO_SETTINGS_FLAGS ENUM_END
constexpr auto GOPRO_VIDEO_SETTINGS_FLAGS_ENUM_END = 2;

/** @brief  */
enum class GOPRO_PHOTO_RESOLUTION
{
    RESOLUTION_5MP_MEDIUM=0, /* 5MP Medium. | */
    RESOLUTION_7MP_MEDIUM=1, /* 7MP Medium. | */
    RESOLUTION_7MP_WIDE=2, /* 7MP Wide. | */
    RESOLUTION_10MP_WIDE=3, /* 10MP Wide. | */
    RESOLUTION_12MP_WIDE=4, /* 12MP Wide. | */
};

//! GOPRO_PHOTO_RESOLUTION ENUM_END
constexpr auto GOPRO_PHOTO_RESOLUTION_ENUM_END = 5;

/** @brief  */
enum class GOPRO_PROTUNE_WHITE_BALANCE
{
    AUTO=0, /* Auto. | */
    BALANCE_3000K=1, /* 3000K. | */
    BALANCE_5500K=2, /* 5500K. | */
    BALANCE_6500K=3, /* 6500K. | */
    RAW=4, /* Camera Raw. | */
};

//! GOPRO_PROTUNE_WHITE_BALANCE ENUM_END
constexpr auto GOPRO_PROTUNE_WHITE_BALANCE_ENUM_END = 5;

/** @brief  */
enum class GOPRO_PROTUNE_COLOUR
{
    STANDARD=0, /* Auto. | */
    NEUTRAL=1, /* Neutral. | */
};

//! GOPRO_PROTUNE_COLOUR ENUM_END
constexpr auto GOPRO_PROTUNE_COLOUR_ENUM_END = 2;

/** @brief  */
enum class GOPRO_PROTUNE_GAIN
{
    GAIN_400=0, /* ISO 400. | */
    GAIN_800=1, /* ISO 800 (Only Hero 4). | */
    GAIN_1600=2, /* ISO 1600. | */
    GAIN_3200=3, /* ISO 3200 (Only Hero 4). | */
    GAIN_6400=4, /* ISO 6400. | */
};

//! GOPRO_PROTUNE_GAIN ENUM_END
constexpr auto GOPRO_PROTUNE_GAIN_ENUM_END = 5;

/** @brief  */
enum class GOPRO_PROTUNE_SHARPNESS
{
    LOW=0, /* Low Sharpness. | */
    MEDIUM=1, /* Medium Sharpness. | */
    HIGH=2, /* High Sharpness. | */
};

//! GOPRO_PROTUNE_SHARPNESS ENUM_END
constexpr auto GOPRO_PROTUNE_SHARPNESS_ENUM_END = 3;

/** @brief  */
enum class GOPRO_PROTUNE_EXPOSURE
{
    NEG_5_0=0, /* -5.0 EV (Hero 3+ Only). | */
    NEG_4_5=1, /* -4.5 EV (Hero 3+ Only). | */
    NEG_4_0=2, /* -4.0 EV (Hero 3+ Only). | */
    NEG_3_5=3, /* -3.5 EV (Hero 3+ Only). | */
    NEG_3_0=4, /* -3.0 EV (Hero 3+ Only). | */
    NEG_2_5=5, /* -2.5 EV (Hero 3+ Only). | */
    NEG_2_0=6, /* -2.0 EV. | */
    NEG_1_5=7, /* -1.5 EV. | */
    NEG_1_0=8, /* -1.0 EV. | */
    NEG_0_5=9, /* -0.5 EV. | */
    ZERO=10, /* 0.0 EV. | */
    POS_0_5=11, /* +0.5 EV. | */
    POS_1_0=12, /* +1.0 EV. | */
    POS_1_5=13, /* +1.5 EV. | */
    POS_2_0=14, /* +2.0 EV. | */
    POS_2_5=15, /* +2.5 EV (Hero 3+ Only). | */
    POS_3_0=16, /* +3.0 EV (Hero 3+ Only). | */
    POS_3_5=17, /* +3.5 EV (Hero 3+ Only). | */
    POS_4_0=18, /* +4.0 EV (Hero 3+ Only). | */
    POS_4_5=19, /* +4.5 EV (Hero 3+ Only). | */
    POS_5_0=20, /* +5.0 EV (Hero 3+ Only). | */
};

//! GOPRO_PROTUNE_EXPOSURE ENUM_END
constexpr auto GOPRO_PROTUNE_EXPOSURE_ENUM_END = 21;

/** @brief  */
enum class GOPRO_CHARGING
{
    DISABLED=0, /* Charging disabled. | */
    ENABLED=1, /* Charging enabled. | */
};

//! GOPRO_CHARGING ENUM_END
constexpr auto GOPRO_CHARGING_ENUM_END = 2;

/** @brief  */
enum class GOPRO_MODEL
{
    UNKNOWN=0, /* Unknown gopro model. | */
    HERO_3_PLUS_SILVER=1, /* Hero 3+ Silver (HeroBus not supported by GoPro). | */
    HERO_3_PLUS_BLACK=2, /* Hero 3+ Black. | */
    HERO_4_SILVER=3, /* Hero 4 Silver. | */
    HERO_4_BLACK=4, /* Hero 4 Black. | */
};

//! GOPRO_MODEL ENUM_END
constexpr auto GOPRO_MODEL_ENUM_END = 5;

/** @brief  */
enum class GOPRO_BURST_RATE
{
    RATE_3_IN_1_SECOND=0, /* 3 Shots / 1 Second. | */
    RATE_5_IN_1_SECOND=1, /* 5 Shots / 1 Second. | */
    RATE_10_IN_1_SECOND=2, /* 10 Shots / 1 Second. | */
    RATE_10_IN_2_SECOND=3, /* 10 Shots / 2 Second. | */
    RATE_10_IN_3_SECOND=4, /* 10 Shots / 3 Second (Hero 4 Only). | */
    RATE_30_IN_1_SECOND=5, /* 30 Shots / 1 Second. | */
    RATE_30_IN_2_SECOND=6, /* 30 Shots / 2 Second. | */
    RATE_30_IN_3_SECOND=7, /* 30 Shots / 3 Second. | */
    RATE_30_IN_6_SECOND=8, /* 30 Shots / 6 Second. | */
};

//! GOPRO_BURST_RATE ENUM_END
constexpr auto GOPRO_BURST_RATE_ENUM_END = 9;

/** @brief  */
enum class LED_CONTROL_PATTERN
{
    OFF=0, /* LED patterns off (return control to regular vehicle control). | */
    FIRMWAREUPDATE=1, /* LEDs show pattern during firmware update. | */
    CUSTOM=255, /* Custom Pattern using custom bytes fields. | */
};

//! LED_CONTROL_PATTERN ENUM_END
constexpr auto LED_CONTROL_PATTERN_ENUM_END = 256;

/** @brief Flags in EKF_STATUS message. */
enum class EKF_STATUS_FLAGS : uint16_t
{
    ATTITUDE=1, /* Set if EKF's attitude estimate is good. | */
    VELOCITY_HORIZ=2, /* Set if EKF's horizontal velocity estimate is good. | */
    VELOCITY_VERT=4, /* Set if EKF's vertical velocity estimate is good. | */
    POS_HORIZ_REL=8, /* Set if EKF's horizontal position (relative) estimate is good. | */
    POS_HORIZ_ABS=16, /* Set if EKF's horizontal position (absolute) estimate is good. | */
    POS_VERT_ABS=32, /* Set if EKF's vertical position (absolute) estimate is good. | */
    POS_VERT_AGL=64, /* Set if EKF's vertical position (above ground) estimate is good. | */
    CONST_POS_MODE=128, /* EKF is in constant position mode and does not know it's absolute or relative position. | */
    PRED_POS_HORIZ_REL=256, /* Set if EKF's predicted horizontal position (relative) estimate is good. | */
    PRED_POS_HORIZ_ABS=512, /* Set if EKF's predicted horizontal position (absolute) estimate is good. | */
};

//! EKF_STATUS_FLAGS ENUM_END
constexpr auto EKF_STATUS_FLAGS_ENUM_END = 513;

/** @brief  */
enum class PID_TUNING_AXIS : uint8_t
{
    ROLL=1, /*  | */
    PITCH=2, /*  | */
    YAW=3, /*  | */
    ACCZ=4, /*  | */
    STEER=5, /*  | */
    LANDING=6, /*  | */
};

//! PID_TUNING_AXIS ENUM_END
constexpr auto PID_TUNING_AXIS_ENUM_END = 7;

/** @brief  */
enum class MAG_CAL_STATUS : uint8_t
{
    NOT_STARTED=0, /*  | */
    WAITING_TO_START=1, /*  | */
    RUNNING_STEP_ONE=2, /*  | */
    RUNNING_STEP_TWO=3, /*  | */
    SUCCESS=4, /*  | */
    FAILED=5, /*  | */
    BAD_ORIENTATION=6, /*  | */
};

//! MAG_CAL_STATUS ENUM_END
constexpr auto MAG_CAL_STATUS_ENUM_END = 7;

/** @brief Special ACK block numbers control activation of dataflash log streaming. */
enum class MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS : uint32_t
{
    STOP=2147483645, /* UAV to stop sending DataFlash blocks. | */
    START=2147483646, /* UAV to start sending DataFlash blocks. | */
};

//! MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS ENUM_END
constexpr auto MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_ENUM_END = 2147483647;

/** @brief Possible remote log data block statuses. */
enum class MAV_REMOTE_LOG_DATA_BLOCK_STATUSES : uint8_t
{
    NACK=0, /* This block has NOT been received. | */
    ACK=1, /* This block has been received. | */
};

//! MAV_REMOTE_LOG_DATA_BLOCK_STATUSES ENUM_END
constexpr auto MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_ENUM_END = 2;

/** @brief Bus types for device operations. */
enum class DEVICE_OP_BUSTYPE : uint8_t
{
    I2C=0, /* I2C Device operation. | */
    SPI=1, /* SPI Device operation. | */
};

//! DEVICE_OP_BUSTYPE ENUM_END
constexpr auto DEVICE_OP_BUSTYPE_ENUM_END = 2;

/** @brief Deepstall flight stage. */
enum class DEEPSTALL_STAGE : uint8_t
{
    FLY_TO_LANDING=0, /* Flying to the landing point. | */
    ESTIMATE_WIND=1, /* Building an estimate of the wind. | */
    WAIT_FOR_BREAKOUT=2, /* Waiting to breakout of the loiter to fly the approach. | */
    FLY_TO_ARC=3, /* Flying to the first arc point to turn around to the landing point. | */
    ARC=4, /* Turning around back to the deepstall landing point. | */
    APPROACH=5, /* Approaching the landing point. | */
    LAND=6, /* Stalling and steering towards the land point. | */
};

//! DEEPSTALL_STAGE ENUM_END
constexpr auto DEEPSTALL_STAGE_ENUM_END = 7;

/** @brief A mapping of plane flight modes for custom_mode field of heartbeat. */
enum class PLANE_MODE
{
    MANUAL=0, /*  | */
    CIRCLE=1, /*  | */
    STABILIZE=2, /*  | */
    TRAINING=3, /*  | */
    ACRO=4, /*  | */
    FLY_BY_WIRE_A=5, /*  | */
    FLY_BY_WIRE_B=6, /*  | */
    CRUISE=7, /*  | */
    AUTOTUNE=8, /*  | */
    AUTO=10, /*  | */
    RTL=11, /*  | */
    LOITER=12, /*  | */
    AVOID_ADSB=14, /*  | */
    GUIDED=15, /*  | */
    INITIALIZING=16, /*  | */
    QSTABILIZE=17, /*  | */
    QHOVER=18, /*  | */
    QLOITER=19, /*  | */
    QLAND=20, /*  | */
    QRTL=21, /*  | */
};

//! PLANE_MODE ENUM_END
constexpr auto PLANE_MODE_ENUM_END = 22;

/** @brief A mapping of copter flight modes for custom_mode field of heartbeat. */
enum class COPTER_MODE
{
    STABILIZE=0, /*  | */
    ACRO=1, /*  | */
    ALT_HOLD=2, /*  | */
    AUTO=3, /*  | */
    GUIDED=4, /*  | */
    LOITER=5, /*  | */
    RTL=6, /*  | */
    CIRCLE=7, /*  | */
    LAND=9, /*  | */
    DRIFT=11, /*  | */
    SPORT=13, /*  | */
    FLIP=14, /*  | */
    AUTOTUNE=15, /*  | */
    POSHOLD=16, /*  | */
    BRAKE=17, /*  | */
    THROW=18, /*  | */
    AVOID_ADSB=19, /*  | */
    GUIDED_NOGPS=20, /*  | */
    SMART_RTL=21, /*  | */
};

//! COPTER_MODE ENUM_END
constexpr auto COPTER_MODE_ENUM_END = 22;

/** @brief A mapping of sub flight modes for custom_mode field of heartbeat. */
enum class SUB_MODE
{
    STABILIZE=0, /*  | */
    ACRO=1, /*  | */
    ALT_HOLD=2, /*  | */
    AUTO=3, /*  | */
    GUIDED=4, /*  | */
    CIRCLE=7, /*  | */
    SURFACE=9, /*  | */
    POSHOLD=16, /*  | */
    MANUAL=19, /*  | */
};

//! SUB_MODE ENUM_END
constexpr auto SUB_MODE_ENUM_END = 20;

/** @brief A mapping of rover flight modes for custom_mode field of heartbeat. */
enum class ROVER_MODE
{
    MANUAL=0, /*  | */
    ACRO=1, /*  | */
    STEERING=3, /*  | */
    HOLD=4, /*  | */
    LOITER=5, /*  | */
    AUTO=10, /*  | */
    RTL=11, /*  | */
    SMART_RTL=12, /*  | */
    GUIDED=15, /*  | */
    INITIALIZING=16, /*  | */
};

//! ROVER_MODE ENUM_END
constexpr auto ROVER_MODE_ENUM_END = 17;

/** @brief A mapping of antenna tracker flight modes for custom_mode field of heartbeat. */
enum class TRACKER_MODE
{
    MANUAL=0, /*  | */
    STOP=1, /*  | */
    SCAN=2, /*  | */
    SERVO_TEST=3, /*  | */
    AUTO=10, /*  | */
    INITIALIZING=16, /*  | */
};

//! TRACKER_MODE ENUM_END
constexpr auto TRACKER_MODE_ENUM_END = 17;


} // namespace ardupilotmega
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_sensor_offsets.hpp"
#include "./mavlink_msg_set_mag_offsets.hpp"
#include "./mavlink_msg_meminfo.hpp"
#include "./mavlink_msg_ap_adc.hpp"
#include "./mavlink_msg_digicam_configure.hpp"
#include "./mavlink_msg_digicam_control.hpp"
#include "./mavlink_msg_mount_configure.hpp"
#include "./mavlink_msg_mount_control.hpp"
#include "./mavlink_msg_mount_status.hpp"
#include "./mavlink_msg_fence_point.hpp"
#include "./mavlink_msg_fence_fetch_point.hpp"
#include "./mavlink_msg_fence_status.hpp"
#include "./mavlink_msg_ahrs.hpp"
#include "./mavlink_msg_simstate.hpp"
#include "./mavlink_msg_hwstatus.hpp"
#include "./mavlink_msg_radio.hpp"
#include "./mavlink_msg_limits_status.hpp"
#include "./mavlink_msg_wind.hpp"
#include "./mavlink_msg_data16.hpp"
#include "./mavlink_msg_data32.hpp"
#include "./mavlink_msg_data64.hpp"
#include "./mavlink_msg_data96.hpp"
#include "./mavlink_msg_rangefinder.hpp"
#include "./mavlink_msg_airspeed_autocal.hpp"
#include "./mavlink_msg_rally_point.hpp"
#include "./mavlink_msg_rally_fetch_point.hpp"
#include "./mavlink_msg_compassmot_status.hpp"
#include "./mavlink_msg_ahrs2.hpp"
#include "./mavlink_msg_camera_status.hpp"
#include "./mavlink_msg_camera_feedback.hpp"
#include "./mavlink_msg_battery2.hpp"
#include "./mavlink_msg_ahrs3.hpp"
#include "./mavlink_msg_autopilot_version_request.hpp"
#include "./mavlink_msg_remote_log_data_block.hpp"
#include "./mavlink_msg_remote_log_block_status.hpp"
#include "./mavlink_msg_led_control.hpp"
#include "./mavlink_msg_mag_cal_progress.hpp"
#include "./mavlink_msg_mag_cal_report.hpp"
#include "./mavlink_msg_ekf_status_report.hpp"
#include "./mavlink_msg_pid_tuning.hpp"
#include "./mavlink_msg_deepstall.hpp"
#include "./mavlink_msg_gimbal_report.hpp"
#include "./mavlink_msg_gimbal_control.hpp"
#include "./mavlink_msg_gimbal_torque_cmd_report.hpp"
#include "./mavlink_msg_gopro_heartbeat.hpp"
#include "./mavlink_msg_gopro_get_request.hpp"
#include "./mavlink_msg_gopro_get_response.hpp"
#include "./mavlink_msg_gopro_set_request.hpp"
#include "./mavlink_msg_gopro_set_response.hpp"
#include "./mavlink_msg_rpm.hpp"
#include "./mavlink_msg_device_op_read.hpp"
#include "./mavlink_msg_device_op_read_reply.hpp"
#include "./mavlink_msg_device_op_write.hpp"
#include "./mavlink_msg_device_op_write_reply.hpp"
#include "./mavlink_msg_adap_tuning.hpp"
#include "./mavlink_msg_vision_position_delta.hpp"
#include "./mavlink_msg_aoa_ssa.hpp"
#include "./mavlink_msg_esc_telemetry_1_to_4.hpp"
#include "./mavlink_msg_esc_telemetry_5_to_8.hpp"
#include "./mavlink_msg_esc_telemetry_9_to_12.hpp"

// base include
#include "../common/common.hpp"
#include "../uAvionix/uAvionix.hpp"
#include "../icarous/icarous.hpp"
