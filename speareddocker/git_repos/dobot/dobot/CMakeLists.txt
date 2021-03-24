cmake_minimum_required(VERSION 2.8.12)
project(dobot)
set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

## Add support for C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -fpic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  tf_helper
  geometry_msgs
  dobot_msgs
  detection_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    include_directories(
            ${catkin_INCLUDE_DIRS}
            src/DobotDll_x64
    )

    link_directories(
            src/DobotDll_x64
    )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i386")
    include_directories(
            ${catkin_INCLUDE_DIRS}
            src/DobotDll_x86
    )

    link_directories(
            src/DobotDll_x86
    )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    include_directories(
            ${catkin_INCLUDE_DIRS}
            src/DobotDll_aarch64
    )

    link_directories(
            src/DobotDll_aarch64
    )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    include_directories(
            ${catkin_INCLUDE_DIRS}
            src/DobotDll_arm
    )

    link_directories(
            src/DobotDll_arm
    )
endif()

set(LIB_NAME Dobot)

catkin_package(
        CATKIN_DEPENDS std_msgs dobot_msgs detection_msgs
        INCLUDE_DIRS include
        LIBRARIES ${LIB_NAME}
)

add_library(${LIB_NAME} src/dobot.cpp)
target_link_libraries(${LIB_NAME} ${catkin_LIBRARIES})
add_dependencies(${LIB_NAME} std_msgs tf2 tf2_ros ${catkin_EXPORTED_TARGETS} detection_msgs_generate_messages_cpp
        detection_msgs_generate_messages_cpp)

add_executable(DobotServer src/DobotServer.cpp)
target_link_libraries(DobotServer ${catkin_LIBRARIES} DobotDll)
add_dependencies(DobotServer std_msgs tf2 tf2_ros ${catkin_EXPORTED_TARGETS} detection_msgs_generate_messages_cpp
        detection_msgs_generate_messages_cpp)

add_executable(DobotNameChooser src/dobot_name_chooser.cpp)
target_link_libraries(DobotNameChooser DobotDll)
add_dependencies(DobotNameChooser std_msgs tf2 tf2_ros ${catkin_EXPORTED_TARGETS} detection_msgs_generate_messages_cpp
        detection_msgs_generate_messages_cpp)

add_executable(DobotClient_JOG src/DobotClient_JOG.cpp)
target_link_libraries(DobotClient_JOG ${catkin_LIBRARIES})
add_dependencies(DobotClient_JOG std_msgs tf2 tf2_ros ${catkin_EXPORTED_TARGETS} detection_msgs_generate_messages_cpp
        detection_msgs_generate_messages_cpp)

add_executable(DobotClient_PTP src/DobotClient_PTP.cpp)
target_link_libraries(DobotClient_PTP ${catkin_LIBRARIES})
add_dependencies(DobotClient_PTP std_msgs tf2 tf2_ros ${catkin_EXPORTED_TARGETS} detection_msgs_generate_messages_cpp
        detection_msgs_generate_messages_cpp)

add_executable(DobotClient_topic src/DobotClient_topic.cpp)
target_link_libraries(DobotClient_topic ${catkin_LIBRARIES})
add_dependencies(DobotClient_topic std_msgs tf2 tf2_ros ${catkin_EXPORTED_TARGETS} detection_msgs_generate_messages_cpp
        detection_msgs_generate_messages_cpp)

install(TARGETS ${LIB_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})