cmake_minimum_required(VERSION 2.8.12)
project(dobot_description)

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

add_compile_options (-std=c++14 -fpic)

find_package(gazebo REQUIRED)

find_package (catkin REQUIRED COMPONENTS
	roscpp
	gazebo_ros
	rospy
	control_msgs
	dobot_msgs
	trajectory_msgs
	geometry_msgs
	marker_helper
	tf2
	tf2_ros
	actionlib
	gazebo_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)


include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  
)
find_package(Gazebo 9 QUIET)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(dobotSim src/dobotSim.cpp)
add_dependencies(dobotSim dobot_msgs)
target_link_libraries(dobotSim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(dobotTest src/dobotTest.cpp)
target_link_libraries(dobotTest ${catkin_LIBRARIES})