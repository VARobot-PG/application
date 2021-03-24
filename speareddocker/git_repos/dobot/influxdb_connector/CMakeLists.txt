cmake_minimum_required(VERSION 2.8.12)
project(influxdb_connector)

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

add_subdirectory(lib/influxdb_cpp)


add_compile_options (-std=c++14 -fpic)

find_package(catkin REQUIRED COMPONENTS
		rospy
		roscpp
		sensor_msgs
		std_msgs
		geometry_msgs
		tf
		dobot_msgs
		arduino_msgs
		dobot
		detection_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(

)

add_executable(influxdb_connector src/connector.cpp src/main.cpp)

target_link_libraries(influxdb_connector ${catkin_LIBRARIES} influxdb_cpp)

add_dependencies(influxdb_connector ${catkin_EXPORTED_TARGETS} dobot_msgs_generate_messages_cpp
		         arduino_msgs_generate_messages_cpp std_msgs sensor_msgs dobot detection_msgs)






