cmake_minimum_required(VERSION 2.8.12)
project(tf_helper)

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

add_compile_options (-std=c++14 -fpic)

find_package (catkin REQUIRED COMPONENTS
	roscpp
	tf2
	tf2_ros
)

set(LIB_NAME TfHelper)

catkin_package(
		CATKIN_DEPENDS tf2 tf2_ros
		INCLUDE_DIRS include
		LIBRARIES ${LIB_NAME}
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(${LIB_NAME} src/tfHelper.cpp)

target_link_libraries(${LIB_NAME} ${catkin_LIBRARIES})

add_dependencies(${LIB_NAME} tf2 tf2_ros)

install(TARGETS ${LIB_NAME}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

