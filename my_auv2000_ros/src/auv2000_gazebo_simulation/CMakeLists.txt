cmake_minimum_required(VERSION 2.8.3)
project(auv2000_gazebo_simulation)

add_compile_options(-std=c++11)



find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    message_filters
    uuv_assistants
    sensor_msgs
    gazebo_ros
    utils
    urdf
    controller_manager
    joint_state_controller
    robot_state_publisher
)

find_package(gazebo REQUIRED)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS})

add_library(dynamics_plugin
    src/dynamics_plugin.cc)
target_link_libraries(dynamics_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(thrusters_plugin
    src/thrusters_plugin.cc)
target_link_libraries(thrusters_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(ahrs_filter
    src/ahrs_filter.cc)
target_link_libraries(ahrs_filter
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(gps-ins-dvl_filter
    src/gps-ins-dvl_filter.cc)
target_link_libraries(gps-ins-dvl_filter
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(gps_plugin
    src/gps_plugin.cc)
target_link_libraries(gps_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(imu_plugin
    src/imu_plugin.cc)
target_link_libraries(imu_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(magnet_plugin
    src/magnet_plugin.cc)
target_link_libraries(magnet_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(dvl_plugin
    src/dvl_plugin.cc)
target_link_libraries(dvl_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(p3d_plugin
    src/p3d_plugin.cc)
target_link_libraries(p3d_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(rudder_plugin
    src/rudder_plugin.cc)
target_link_libraries(rudder_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(mass_shifter_plugin
    src/mass_shifter_plugin.cc)
target_link_libraries(mass_shifter_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})


add_library(gps_ins_dvl
    src/gps_ins_dvl.cc)
target_link_libraries(gps_ins_dvl
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(pressure_sensor_plugin
    src/pressure_sensor_plugin.cc)
target_link_libraries(pressure_sensor_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(arm2_plugin
    src/arm2_plugin.cc)
target_link_libraries(arm2_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(arm1_plugin
    src/arm1_plugin.cc)
target_link_libraries(arm1_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(irm1_plugin
    src/irm1_plugin.cc)
target_link_libraries(irm1_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(irm2_plugin
    src/irm2_plugin.cc)
target_link_libraries(irm2_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(piston_plugin
    src/piston_plugin.cc)
target_link_libraries(piston_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})

add_library(sonar_plugin
    src/sonar_plugin.cc)
target_link_libraries(sonar_plugin
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES})




