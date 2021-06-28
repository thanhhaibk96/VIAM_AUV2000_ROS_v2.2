cmake_minimum_required(VERSION 2.8.3)
project(navigator)

add_compile_options(-std=c++11)
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    utils
)

find_package(Qt5 REQUIRED
  COMPONENTS
    Core
    SerialPort
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS})

add_executable(gps-ahrs_receiver
    src/gps-ahrs_receiver.cpp
    include/gps-ahrs_receiver.h)
target_link_libraries(gps-ahrs_receiver
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::SerialPort)

add_executable(gps-ahrs2_receiver
    src/gps-ahrs2_receiver.cpp
    include/gps-ahrs2_receiver.h)
target_link_libraries(gps-ahrs2_receiver
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::SerialPort)

add_executable(gps-ins-dvl_receiver
    src/gps-ins-dvl_receiver.cpp
    include/gps-ins-dvl_receiver.h)
target_link_libraries(gps-ins-dvl_receiver
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::SerialPort)

