# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images/create_ros_image.Dockerfile.em
FROM osrf/ros:melodic-desktop-bionic

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full=1.4.1-0*
RUN apt-get update
RUN apt-get -y install ros-melodic-rosbridge-server
RUN apt-get install -y  vim wget
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `/usr/bin/lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update
RUN apt-get install -y ros-melodic-controller-manager ros-melodic-joint-state-controller ros-melodic-joint-trajectory-controller ros-melodic-rqt ros-melodic-rqt-controller-manager ros-melodic-rqt-joint-trajectory-controller ros-melodic-ros-control ros-melodic-rqt-gui
RUN apt-get install -y ros-melodic-gazebo-ros ros-melodic-kdl-conversions ros-melodic-kdl-parser ros-melodic-forward-command-controller ros-melodic-tf-conversions ros-melodic-xacro ros-melodic-joint-state-publisher ros-melodic-robot-state-publisher ros-melodic-gazebo-ros-control
RUN apt-get -y install python-xlib
RUN apt-get -y install silversearcher-ag
RUN apt-get -y install libwebsocketpp-dev
RUN apt-get -y install python-catkin-tools
RUN apt-get -y install nano
RUN apt-get -y install openssh-server htop
RUN apt-get -y --fix-missing install ros-melodic-geometry2 ros-melodic-pr2-controllers
# install tmux
RUN apt-get -y install tmux 
RUN apt-get -y install dos2unix
RUN apt-get -y install idle

RUN mkdir /var/run/sshd

RUN echo 'root:123321' | chpasswd
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile
RUN export TURTLEBOT_BASE=roomba && export TURTLEBOT_STACKS=circles &&  export TURTLEBOT_3D_SENSOR=kinect

RUN cd ~ && mkdir ~/catkin_ws
RUN mkdir -p /root/git
RUN cd /root/git
RUN mkdir dobot

# Step 1.8

RUN mkdir ~/catkin_ws/src
RUN cd ~
# add ssh keys 
RUN mkdir ~/.ssh

# COPY and make things
RUN cd ~ && git clone https://github.com/siemens/ros-sharp.git
RUN  cp -r ~/ros-sharp/ROS/unity_simulation_scene ~/catkin_ws/src/
RUN cp -r ~/ros-sharp/ROS/file_server ~/catkin_ws/src/
# Step 1.7
RUN cp -r ~/ros-sharp/ROS/gazebo_simulation_scene ~/catkin_ws/src/
# copy  Dobot Uni Porject into catkin workspace
COPY ./git_repos/dobot/ /root/git/dobot/
COPY ./dobotCalibration.cpp /root/dobotCalibration.cpp
COPY ./kinectCalibration.cpp /root/kinectCalibration.cpp
COPY ./truckScenario.cpp  /root/truckScenario.cpp
COPY ./dobotSim.cpp /root/dobotSim.cpp
COPY ./dobotSim.h /root/dobotSim.h
#COPY ./dobot_functions.xacro /root/dobot_functions.xacro 
#COPY ./dobot_rviz.xacro /root/dobot_rviz.xacro 
#COPY ./dobot_parameters.xacro /root/dobot_parameters.xacro 
COPY ./publish_description_dobot.launch /root/catkin_ws/src/file_server/launch/publish_description_dobot.launch
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
COPY ./mouse_to_joy.py /root/catkin_ws/src/unity_simulation_scene/scripts/mouse_to_joy.py
COPY ./dobot_simulation_scene.launch /root/catkin_ws/src/gazebo_simulation_scene/launch/dobot_simulation_scene.launch
COPY ./ptp_cmd.py /root/catkin_ws/src/gazebo_simulation_scene/scripts/ptp_cmd.py
COPY ./add_detected_object_service.py /root/catkin_ws/src/gazebo_simulation_scene/scripts/add_detected_object_service.py
COPY ./publisher_subscriber_suction.py /root/catkin_ws/src/gazebo_simulation_scene/scripts/publisher_subscriber_suction.py
COPY ./publisher_subscriber.py /root/catkin_ws/src/gazebo_simulation_scene/scripts/publisher_subscriber.py
RUN mkdir -p /root/catkin_ws/src/gazebo_simulation_scene/sdf/
COPY ./box.sdf /root/catkin_ws/src/gazebo_simulation_scene/sdf/box.sdf
# set executable files
RUN chmod u+x ./ros_entrypoint.sh
RUN chmod u+x /root/catkin_ws/src/unity_simulation_scene/scripts/mouse_to_joy.py
RUN chmod u+x /root/catkin_ws/src/gazebo_simulation_scene/scripts/joy_to_twist.py
RUN chmod u+x /root/catkin_ws/src/gazebo_simulation_scene/scripts/ptp_cmd.py
RUN chmod u+x /root/catkin_ws/src/gazebo_simulation_scene/scripts/publisher_subscriber_suction.py
RUN chmod u+x /root/catkin_ws/src/gazebo_simulation_scene/scripts/publisher_subscriber.py

# set bash rc
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /root/catkin_ws/devel/setup.bash ]; then" >> ~/.bashrc
RUN echo "  source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "fi" >> ~/.bashrc
# update sshd
RUN echo "PermitRootLogin Yes" >> /etc/ssh/sshd_config
RUN echo "X11UseLocalhost no" >> /etc/ssh/sshd_config
# set bashrc and entrypoint executable
RUN dos2unix ~/.bashrc
RUN dos2unix ./ros_entrypoint.sh
RUN dos2unix /root/catkin_ws/src/gazebo_simulation_scene/scripts/ptp_cmd.py
RUN dos2unix /root/catkin_ws/src/gazebo_simulation_scene/scripts/add_detected_object_service.py
RUN dos2unix /root/catkin_ws/src/gazebo_simulation_scene/scripts/publisher_subscriber.py
# add required python packages
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python get-pip.py
RUN pip install tornado
RUN pip install bson
RUN pip install pymongo

# set Display Settings
ENV DISPLAY 172.17.0.1:0.0
# add keys for ssh
COPY ./authorized_keys /root/.ssh/authorized_keys
# fix XAuthority File
COPY ./.Xauthority /root/.Xauthority

#define Entrypoint
ENTRYPOINT ["/bin/bash"]
