# Clone repos
Clone DoBot Repo   repo into folder `git_repos`
```
cd git_repos
git clone https://oauth2:fagrR2exDuRwsGA9oCQx@git.cs.upb.de/ki4as/dobot.git 
```
check out simulator2 branch
```
cd git_repos/dobot
git checkout feature/simulator2
```
# Build docker
Build the dockerfile - hint the repos and the other files (.cpp, .xacro etc.) are copied into it

`docker build -t rosdock .`

# run docker

`docker run -it -p9090:9090 -p 32775:22 rosdock`


# container setup
## preparation
```
/usr/sbin/sshd
./ros_entrypoint.sh
cd /root/catkin_ws/src/
catkin_init_workspace
cd /root/catkin_ws/
catkin_make
```
note: sometimes catkin_make has to be called up to three times.  It can take a while (30 minutes is not unusual!!)
# start simulation
## connect via ssh
Note: You *need* to use an X-Server because the start of gazebo and rqt will not work otherwise!
```
ssh -X -p 32775 root@127.0.0.1 
```
## run gazebo simulation scene
```
source ~/.bashrc
roslaunch gazebo_simulation_scene dobot_simulation_scene.launch
```
## run dobot_control.launch
```
source ~/.bashrc
roslaunch dobot_description dobot_control.launch
```

Now you can connect via e.g. ROS# (Ros-Sharp) to it. E.g. under Unity via ws://127.0.0.1:9090 or via LAN e.g. ws://192.168.2.232 (your ip-adress where the docker container is running). You can even host it on a server and connect to it via ws://server-name:9090.

Furthermore you can manually test the ros services etc. via rqt and service caller or whatever your preference is.