cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)
project(detection)

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

add_compile_options (-Wall -std=c++14 -fopenmp)
set(CMAKE_CXX_STANDARD 14)

link_libraries("-fopenmp")

find_package(PCL 1.7 REQUIRED)

find_package (catkin REQUIRED COMPONENTS
	roscpp
  	sensor_msgs
  	pcl_ros
  	std_msgs
  	std_srvs
	geometry_msgs
	tf2
	tf2_ros
	tf2_msgs
	tf2_sensor_msgs
	tf2_eigen
	tf_helper
	tf2_geometry_msgs
	dobot_msgs
	detection_msgs
	dobot
	marker_helper
)

set(LIB_NAME Detection)

catkin_package(
		CATKIN_DEPENDS std_msgs detection_msgs
		INCLUDE_DIRS include
		LIBRARIES ${LIB_NAME}
)

find_package(OpenCV 3.4.6 QUIET)

if(NOT OpenCV_FOUND)
	find_package(OpenCV REQUIRED)
	include_directories(/usr/local/opencv4)
endif()

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_library(cloud
	src/cloud.cpp
	src/medianFilter.cpp
)
target_link_libraries(cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (graspPointDetection src/graspPointDetection.cpp)
target_link_libraries (graspPointDetection 
		${catkin_LIBRARIES} 
		${PCL_LIBRARIES}
		cloud
)

add_executable (dominoDetection src/dominoDetection.cpp)
target_link_libraries (dominoDetection 
		${catkin_LIBRARIES} 
		${PCL_LIBRARIES}
		cloud
)

add_executable (gridDetection src/gridDetection.cpp)
target_link_libraries (gridDetection
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
		cloud
)

add_executable (graspPointBuffer src/graspPointBuffer.cpp)
target_link_libraries (graspPointBuffer 
		${catkin_LIBRARIES} 
		${PCL_LIBRARIES}
		cloud
)

add_executable (tfPublisherClient src/tfPublisherClient.cpp)
target_link_libraries (tfPublisherClient 
		${catkin_LIBRARIES} 
		${PCL_LIBRARIES}
		cloud
)

add_executable (tfPublisher src/tfPublisher.cpp)
target_link_libraries (tfPublisher 
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
		cloud
)

add_executable (dobotCalibration src/dobotCalibration.cpp)
target_link_libraries (dobotCalibration 
		${catkin_LIBRARIES} 
		${PCL_LIBRARIES}
		cloud
)

add_executable (medianFilterServer src/medianFilterServer.cpp)
target_link_libraries (medianFilterServer 
		${catkin_LIBRARIES} 
		${PCL_LIBRARIES}
		cloud
)


add_executable (rotationDetection src/rotationDetection.cpp)
target_link_libraries (rotationDetection ${catkin_LIBRARIES} ${PCL_LIBRARIES} cloud)

add_library(${LIB_NAME} src/detection.cpp)
target_link_libraries(${LIB_NAME} ${catkin_LIBRARIES})

add_executable (kinectCalibration src/kinectCalibration.cpp)
target_link_libraries (kinectCalibration
		${catkin_LIBRARIES} 
		${PCL_LIBRARIES}
		cloud
)


add_dependencies(graspPointDetection ${catkin_EXPORTED_TARGETS} detection_msgs_generate_messages_cpp markerHelper)
add_dependencies(graspPointBuffer ${catkin_EXPORTED_TARGETS} detection_msgs_generate_messages_cpp dobot markerHelper)
add_dependencies(dobotCalibration dobot)
add_dependencies(${LIB_NAME} ${catkin_EXPORTED_TARGETS} detection_msgs_generate_messages_cpp std_msgs)
add_dependencies(rotationDetection ${catkin_EXPORTED_TARGETS} detection_msgs_generate_messages_cpp markerHelper)

install(TARGETS ${LIB_NAME}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})