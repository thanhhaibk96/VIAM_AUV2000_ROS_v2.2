cmake_minimum_required(VERSION 2.8.3)
project(utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    message_generation

    std_msgs
    geometry_msgs
)

add_message_files(
  FILES
    Odometry.msg
    Setpoint.msg
    Error.msg
    ThrustCommand.msg
    KeyboardCommand.msg
    Waypoint.msg
    WaypointList.msg
    ParamValue.msg
    gps_dvl_ins_stamped.msg
    motor_stamped.msg
    altimeter_stamped.msg
    bms_stamped.msg
    keller_pa3_stamped.msg
    limit_switch_stamped.msg
    mx28_stamped.msg
    board_arm1_stamped.msg
    board_arm2_stamped.msg
    joystick_stamped.msg
    anti_rolling_stamped.msg
    BoolStamped.msg
    ByteStamped.msg
    ByteMultiArrayStamped.msg
    CharStamped.msg
    ColorRGBAStamped.msg
    DurationStamped.msg
    Float32MultiArrayStamped.msg
    Float32Stamped.msg
    Float64MultiArrayStamped.msg
    Float64Stamped.msg
    Int8MultiArrayStamped.msg
    Int8Stamped.msg
    Int16MultiArrayStamped.msg
    Int16Stamped.msg
    Int32MultiArrayStamped.msg
    Int32Stamped.msg
    Int64MultiArrayStamped.msg
    Int64Stamped.msg
    StringStamped.msg
    TimeStamped.msg
    UInt8MultiArrayStamped.msg
    UInt8Stamped.msg
    UInt16MultiArrayStamped.msg
    UInt16Stamped.msg
    UInt32MultiArrayStamped.msg
    UInt32Stamped.msg
    UInt64MultiArrayStamped.msg
    UInt64Stamped.msg
)

add_service_files(
  FILES
    CommandInt.srv
    CommandLong.srv
    ParamGet.srv
    ParamSet.srv
    SetMode.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include)

include_directories(
    include
    ${catkin_INCLUDE_DIRS})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

