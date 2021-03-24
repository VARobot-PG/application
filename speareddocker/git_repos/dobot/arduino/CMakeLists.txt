cmake_minimum_required(VERSION 2.8.12)
project(arduino)

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

add_compile_options (-std=c++11 -fpic)

find_package (catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        tf
        arduino_msgs
)

set(LIB_NAME Arduino)

catkin_package(
        CATKIN_DEPENDS std_msgs arduino_msgs
        INCLUDE_DIRS include
        LIBRARIES ${LIB_NAME}
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(${LIB_NAME} src/arduino.cpp)

target_link_libraries(${LIB_NAME} ${catkin_LIBRARIES})

add_dependencies(${LIB_NAME} ${catkin_EXPORTED_TARGETS} arduino_msgs_generate_messages_cpp std_msgs)

install(TARGETS ${LIB_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

