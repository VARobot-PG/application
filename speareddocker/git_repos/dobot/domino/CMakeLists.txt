cmake_minimum_required(VERSION 2.8.12)
project(domino)

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

add_compile_options (-std=c++14 -fpic)

find_package (catkin REQUIRED COMPONENTS
	roscpp
	rospy
  	sensor_msgs
  	std_msgs
	geometry_msgs
	tf2_geometry_msgs
	tf2
	tf2_ros
	dobot_msgs
	detection_msgs
	dobot
	detection
)


include_directories(${catkin_INCLUDE_DIRS} include)

catkin_package(
  
)

add_executable(domino src/domino.cpp src/domino_game.cpp src/domino_player.cpp src/domino_field.cpp src/domino_pool.cpp)

target_link_libraries(domino ${catkin_LIBRARIES})

add_dependencies(domino dobot)
#add_dependencies(detectionBlockLoader ${catkin_EXPORTED_TARGETS} detection_msgs_generate_messages_cpp dobot arduino markerHelper tfHelper)
#add_dependencies(DobotRailActionServer ${catkin_EXPORTED_TARGETS} dobot_msgs_generate_messages_cpp dobot arduino)
#add_dependencies(pickupClient ${catkin_EXPORTED_TARGETS} dobot_msgs_generate_messages_cpp dobot)
#add_dependencies(pictureDrawer ${catkin_EXPORTED_TARGETS} tf_helper arduino dobot detection dobot_msgs_generate_message_cpp detection_msgs_generate_messages_cpp)
#add_dependencies(transportBelt arduino dobot markerHelper detection_msgs_generate_messages_cpp tfHelper std_msgs)
#add_dependencies(rfid_test ${catkin_EXPORTED_TARGETS} arduino tfHelper dobot_msgs_generate_messages_cpp detection_msgs_generate_messages_cpp dobot)

