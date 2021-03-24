This package is used to simulate the scenario inside the gazebo simulator. (Gazebo 9.0)
#To Start:
1. compile the package!
2. `roslaunch dobot_description dobot.launch`
3. `roslaunch dobot_description dobot_control.launch`

#Available Services:
```
/Dobot_Loader/SetEndEffectorSuctionCup
/Dobot_Loader/SetPTPCmd
/Dobot_Rail/SetEndEffectorSuctionCup
/Dobot_Rail/SetPTPWithLCmd
/Dobot_Rail/SetPTPCmd
```
#Available Topics:
```
/Dobot_Loader/arm/Pose
/Dobot_Loader/idle
/Dobot_Loader/joint_states
/Dobot_Rail/arm/Pose
/Dobot_Rail/idle
/Dobot_Rail/joint_states
```
#DO NOT USE THIS:
```
/gazebo/*
/Dobot_Loader/controller_manager/*
/Dobot_Loader/joint_trajectory_controller/*
/Dobot_Loader/robot_state_publisher/*
/Dobot_Rail/controller_manager/*
/Dobot_Rail/joint_trajectory_controller/*
/Dobot_Rail/robot_state_publisher/*
```

#How Does It Work?
The robot model (joints, links) is described inside the `urdf/dobot.xacro` file. Meshes are placed inside the `meshes` folder and referenced by the `urdf/dobot.xacro` file. When the dobot.launch is started, it converts the .xacro file into .urdf, which is loaded onto the parameter server and THE file format do describe a robot in ROS. Now e.g. rviz can read this an knows how the model looks like. In case of gazebo, the model spawner converts it into an .sdf file (model format used by gazebo) and spawns it inside the simulation. The .xacro file is ment to only describe a single robot arm, and can deal with parameters, if, else... The launch file sets the parameters and makes the individual robots spawn. 

Static thinks like the camera, transportbelt, blocks, misc (expecially with no joints) should be directly modeld in an .sdf file and not spawned by the launch file. Instead, you can open the gazebo simulator (just type in gazebo in the console) and place them directly in the world to construct the scenario and save the world as a .world file. Inside the dobot.launch file you can tell gazebo to load your custom .world file, instead of `empty.world`, and change the robots position by using the available parameters.

The main logic of the robot arm is controlled by the `src/DobotSim.cpp` plugin. It is referenced by the `urdf/dobot.xacro` file (plugins can be also referenced in a .sdf file) and recieves initial parameters from it. The plugin offers all the services and topics, has access to the simulator and can read out the joints/links position, can access the physic engine, can see collisions and can calculate waypoints which the arm has to follow to move like the real robot. A plugin is basically inbetween ROS and Gazebo, and used to simulate sensors, drives... The waypoints are send to the joint\_trajectory\_controller, started with the dobot_control.launch, which directly controls and simulates the motors.

#What are the upcoming steps?
1. The `src/DobotSim.cpp` should be extended to support all services and topics, which are required by our scenario (provided by the dobots itself).
2. Blocks, truck, camera, transportbelt, [...] have to be modeled in blender, put into an .sdf file. 
3. Gazebo already provides a plugin to simulate a depth camera, so this can be used in the camera's .sdf file. See: http://gazebosim.org/tutorials?tut=ros_gzplugins
4. The transportbelt can be simulated by using my old transportbelt, from the `simulator` package used by the makeblock demonstrator. It is in `simulator/ki4as_plugins/conveyorBeltPlugin.cc`. Watch out for the CMakeList.txt because it needs the gazebo source files to be able to compile, there is no other way... Just use the plugin as in the `simulator/ki4as_description/urdf/ki4as_functions.xacro` file inside the create_transportbelt macro.
5. Next, we need the Infrared Sensor. I dont know if there is a better way, but the laser scanner plugin from gazebo could be extended/used as a start. See the `Mobile_Robot/racecar_plugins/RacecarLaserscanner.cpp` and the other plugins as an example, on how to overwrite/extend gazebo sensor plugins. I created that plugin by copying the official `gazebo_ros_gpu_laser.cpp` file from google and rewrote it as i needed. You can see which other plugins are available at: http://gazebosim.org/tutorials?tut=ros_gzplugins. Typically, gazebo provides a sensor which does the trick, and the plugin gets informed if a new measurement is ready and does the ROS stuff.
6. The color sensor could done by extending the gazebo camera plugin, only using a low resolution and field of view, then just publishing the average color value. 
7. Finally, all sensors have to be placed in the .world file (open gazebo, place them and use the save function) and the dobots have to be spawned at the right position. PS. you can teleport objects in gazebo by using `/gazebo/setModel_state` service or topic. This can be used to reset the scenario at runtime.
