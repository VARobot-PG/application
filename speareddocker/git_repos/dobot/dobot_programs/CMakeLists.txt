cmake_minimum_required(VERSION 2.8.12)
project(dobot_programs)

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
	arduino_msgs
	dobot
	arduino
	marker_helper
	tf_helper
	detection
)


include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  
)

find_package(OpenCV 3.4.6 QUIET)

if(NOT OpenCV_FOUND)
	find_package(OpenCV REQUIRED)
	include_directories(/usr/local/opencv4)
endif()

add_executable(blockLoader src/blockLoader.cpp)
add_executable(detectionBlockLoader src/detectionBlockLoader.cpp)
add_executable(detectionGraspTest src/detectionGraspTest.cpp)
add_executable(transportBelt src/transportBelt.cpp)
add_executable(DobotRailActionServer src/DobotRailActionServer.cpp src/Action.cpp)
add_executable(pickupClient src/pickupClient.cpp)
add_executable(BlockRotationDetector src/block_rotation_detector.cpp)
add_executable(MeasureTransportbeltSpeed src/measure_transportbelt_speed.cpp)
add_executable(pictureDrawer src/pictureDrawer.cpp src/picture.cpp src/pictureParser.cpp)
add_executable(truckScenario src/truckScenario.cpp)
add_executable(gridAngle src/grid_angle.cpp)

target_link_libraries(blockLoader ${catkin_LIBRARIES})
target_link_libraries(detectionBlockLoader ${catkin_LIBRARIES})
target_link_libraries(detectionGraspTest ${catkin_LIBRARIES})
target_link_libraries(transportBelt ${catkin_LIBRARIES})
target_link_libraries(DobotRailActionServer ${catkin_LIBRARIES})
target_link_libraries(pickupClient ${catkin_LIBRARIES})
target_link_libraries(BlockRotationDetector ${catkin_LIBRARIES})
target_link_libraries(MeasureTransportbeltSpeed ${catkin_LIBRARIES})
target_link_libraries(pictureDrawer ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(truckScenario ${catkin_LIBRARIES})
target_link_libraries(gridAngle ${catkin_LIBRARIES})

add_dependencies(blockLoader dobot)
add_dependencies(detectionBlockLoader ${catkin_EXPORTED_TARGETS} detection_msgs_generate_messages_cpp dobot arduino markerHelper tfHelper)
add_dependencies(detectionGraspTest ${catkin_EXPORTED_TARGETS} detection_msgs_generate_messages_cpp dobot arduino markerHelper tfHelper)
add_dependencies(DobotRailActionServer ${catkin_EXPORTED_TARGETS} dobot_msgs_generate_messages_cpp dobot arduino)
add_dependencies(pickupClient ${catkin_EXPORTED_TARGETS} dobot_msgs_generate_messages_cpp dobot)
add_dependencies(pictureDrawer ${catkin_EXPORTED_TARGETS} tf_helper arduino dobot detection dobot_msgs_generate_message_cpp detection_msgs_generate_messages_cpp)
add_dependencies(transportBelt arduino dobot markerHelper detection_msgs_generate_messages_cpp tfHelper std_msgs)
add_dependencies(truckScenario ${catkin_EXPORTED_TARGETS} arduino tfHelper dobot_msgs_generate_messages_cpp detection_msgs_generate_messages_cpp dobot)
add_dependencies(gridAngle ${catkin_EXPORTED_TARGETS} arduino tfHelper dobot_msgs_generate_messages_cpp detection_msgs_generate_messages_cpp dobot)

