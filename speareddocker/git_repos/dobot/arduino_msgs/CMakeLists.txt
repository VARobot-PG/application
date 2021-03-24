cmake_minimum_required(VERSION 2.8.3)
project(arduino_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
	COMPONENTS message_generation std_msgs geometry_msgs
)

add_message_files(FILES
		rgb_color.msg
		distance.msg
		get_velocity.msg
		rfid.msg
)

add_service_files(FILES
		SetConveyorbeltSpeed.srv
		rfiduid.srv
		)

generate_messages(DEPENDENCIES 
	std_msgs geometry_msgs 
)

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
