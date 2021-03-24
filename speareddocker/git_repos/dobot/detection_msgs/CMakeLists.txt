cmake_minimum_required(VERSION 2.8.3)
project(detection_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS 
	message_generation 
	std_msgs
	sensor_msgs
	geometry_msgs
)

add_message_files(FILES
	DetectedObject.msg
	DetectedObjects.msg
	DetectedGrid.msg
	DetectedDomino.msg
	DetectedDominos.msg
)

add_service_files(FILES
  AddFrame.srv
  AddDetectedObject.srv
  GetDetectedObject.srv
  GetFrames.srv
  SetFrame.srv
  SetPointcloud.srv
  SetString.srv
)


generate_messages(DEPENDENCIES 
	std_msgs
	geometry_msgs
	sensor_msgs
)

catkin_package(
	CATKIN_DEPENDS 
	message_runtime 
	std_msgs
	sensor_msgs
)
