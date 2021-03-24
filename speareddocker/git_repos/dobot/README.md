# Running the tutorial_package:
To start gazebo simulation use:
Do not forget to press 'play' inside gazebo!
```
roslaunch racecar_description racecar.launch 
```
Note: By default test_arena is launched. To launch another world from 'tutorial_package/worlds/' use:
```
roslaunch racecar_description racecar.launch current_world:=YOUR_FILE.world
```

## Running Kinect Cameras
Execute the following on jetson:
```
roslaunch detection kinectJetson.launch
```
Execute the following on Xavier:
```
roslaunch detection kinectXavier.launch
```

## connect to dobot-server
Make sure jetson is turned on.
Make sure both dobots are turned on.
Ssh into jetson:
```
ssh nvidia@131.234.120.91
```
Run dobot server:
```
roslaunch dobot dobot.launch
```
## To visualize sensors:
Open `rviz`.
In options->load configuration load `tutorial_package/tutorial.rviz`.

## To visualize cameras:
Open 'rqt'.
Open plugins->visualization->image_view
Select a camera topic.

OR:
```
rqt_image_view YOUR_CAMERA_TOPIC_NAME
```

## To manually publish messages:
Open a new terminal and open 'rqt'.
Open plugins->topics->message_publisher.
Select a topic, type will be filled automatically. The set a frequency e.g. 30HZ. Then click on the green plus sign.
Expand your new created topic and enter a value into the expression field.
Klick into the checkbox to publish the data.

## Console commands:
Run your node:
```
rosrun PACKAGE_NAME EXECUTABLE
```
Show all Topics:
```
rostopic list
```
Publish a message:
After you wrote the topic name, use tab for autocompletion!
```
rostopic pub --once /ki4as/cmd_vel YOUR_MESSAGE
```
Subscribe a topic:
```
rostopic echo /ki4as/cmd_vel YOUR_MESSAGE
```
List publishers and subscribers for a specified topic:
```
rostopic info /ki4as/cmd_vel
```
List all ros nodes:
```
rosnode list
```
Show all Topics used by a node:
```
rosnode info YOUR_NODE_NAME
```

## Run Picture Draw Scenario:
```
Udo: roslaunch dobot scenario.launch
Jetson: roslaunch detection kinectJetson.launch
Jetson: roslaunch detection graspDetection.launch
Xavier: roslaunch detection kinectXavier.launch
You: rosrun dobot_programs transportBelt
You: rosrun dobot_programs rightDobotActionServer
You: rosrun dobot_programs pictureDrawer ~/ros/Sweden.png
You: rosrun dobot_programs detectionBlockLoader
```
