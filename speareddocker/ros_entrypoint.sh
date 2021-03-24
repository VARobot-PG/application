#!/bin/bash -v
set -e
# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
cp -r /root/git/dobot/* /root/catkin_ws/src/
cd ~/catkin_ws # && /opt/ros/kinetic/bin/catkin_make .

## update links

cd  /root/catkin_ws/src/dobot/src/DobotDll_x64/
ln -s -f libQt5Network.so.5.6.0 libQt5Network.so
ln -s -f libQt5Core.so.5.6.0 libQt5Core.so
ln -s -f libQt5Core.so.5.6.0 libQt5Core.so.5
ln -s -f libQt5Core.so.5.6.0 libQt5Core.so.5.6
ln -s -f libQt5Network.so.5.6.0 libQt5Network.so.5
ln -s -f libQt5Network.so.5.6.0 libQt5Network.so.5.6
ln -s -f libQt5SerialPort.so.5.6.0 libQt5SerialPort.so.5.6
ln -s -f libQt5SerialPort.so.5.6.0 libQt5SerialPort.so.5
ln -s -f libQt5SerialPort.so.5.6.0 libQt5SerialPort.so
ln -s -f libDobotDll.so.1.0.0 libDobotDll.so.1.0
ln -s -f libDobotDll.so.1.0.0 libDobotDll.so.1
ln -s -f libDobotDll.so.1.0.0 libDobotDll.so
ln -s -f libicudata.so.56.1 libicudata.so.56
ln -s -f libicui18n.so.56.1 libicui18n.so.56
ln -s -f libicuuc.so.56.1 libicuuc.so.56
# removed compile errors
#cp /root/kinectCalibration.cpp /root/catkin_ws/src/detection/src/kinectCalibration.cpp
#cp /root/truckScenario.cpp  /root/catkin_ws/src/dobot_programs/src/truckScenario.cpp
cd ~/catkin_ws/src
catkin_init_workspace
cd /root/catkin_ws
cp /root/dobotSim.cpp /root/catkin_ws/src/dobot_description/src/dobotSim.cpp
cp /root/dobotSim.h /root/catkin_ws/src/dobot_description/include/dobotSim.h
#cp /root/dobotCalibration.cpp /root/catkin_ws/src/detection/src/dobotCalibration.cpp
# removed ki4as reference
#cp /root/dobot_functions.xacro /root/catkin_ws/src/dobot_description/urdf/dobot_functions.xacro
#cp /root/dobot_rviz.xacro /root/catkin_ws/src/dobot_description/urdf/dobot_rviz.xacro
#cp /root/dobot_parameters.xacro /root/catkin_ws/src/dobot_description/urdf/dobot_parameters.xacro
#vi +228 /root/catkin_ws/src/detection/src/kinectCalibration.cpp
#vi +294 /root/catkin_ws/src/detection/src/kinectCalibration.cpp
#vi +168 /root/catkin_ws/src/detection/src/dobotCalibration.cpp
#catkin build
#cd ~/catkin_ws  && /opt/ros/kinetic/bin/catkin_make .
#source ~/catkin_ws/devel/setup.bash

###############TEST##################
#cp -r ~/ros-sharp/ROS/file_server ~/catkin_ws/src/
#####################################i
#echo "Starting turtlebot2.launch in background in 3seconds"
#sleep 3s
#roslaunch file_server publish_description_turtlebot2.launch & 
#cd /
#exec "$@"

