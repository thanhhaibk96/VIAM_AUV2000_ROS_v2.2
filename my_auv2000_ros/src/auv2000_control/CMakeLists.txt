cmake_minimum_required(VERSION 3.0.2)
project(auv2000_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    message_filters
    utils
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS other non-ROS libs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS})

# add_executable(auv2000_control_exe
#     src/auv_controller.cpp
#     src/pid.cpp
#     src/heading_smc.cpp
#     src/straight_los.cpp
#     src/base_los.cpp)

add_library(${PROJECT_NAME}
    src/pid.cpp
    src/heading_smc.cpp
    src/straight_los.cpp
    src/base_los.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} utils_generate_messages_cpp)
# install(
#   TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
