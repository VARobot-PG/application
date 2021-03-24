##HOW TO CALIBRATE:
#1. Start, required for all steps!
	a. udo: roslaunch dobot scenario.launch
	c. jetson: roslaunch detection kinectJetson.launch
	d. xavier: roslaunch detection kinectXavier.launch (only required if you do step 4)

#2. Calibrate the left kinect camera, which is above the Dobot_Loader:
	a. roslaunch detection calibrationKinect1Client.launch
	b. This will start the joystick node (please connect a joystick) and the calibration tool.
		Wait a minute, until the pointclouds have been recorded and send to the GUI. 
	c. The calibration tool allows to move the world coordinate frame with the joystick.
		It is manily used to move the world coordinate frame, make sure that it is placed correctly into the world marker using the nobs.
		Make sure that the left and right anchor frames are perfectly in the pointcloud plane and paralell to the long line on the pink paper. 
		If not, the world coordinate frame has to be rotated around the x and z axis. 
		Do not move the anchor frames! Use the arrow keys to the left on the joystick to pitch and roll the world frame, until both anchor points look good.
		Press A to apply the transform, press back to go back to the last applied transform.
		You can also add new coordinate frames by using the provided ros services (/tfPublisher/*), press the Y button to update the GUI and select
		them with the joystick to move them around (LB + B or LB + X).
		If you are happy, press start to save the configuration.

#3. Calibrate the dobot frame:
	a. Place stickers which can be reached by the robot arm.
	b. rosrun detection dobotCalibration Dobot_Loader Dobot_Loader kinect2_jetson true
	c. Select a sticker in the point cloud viewer (shift + left mouse).
	d. Move the robot arm onto the sticker, then use the joystick to finetune the position. Press A to accept.
	e. The blue points are in robot coordinates, the green points in camera coordinates. The yellow points are the
		transformed robot points. If they reliably lay in the green points, the calibration is good. Press start
		to save the calibration.
	f. Use for dobot_rail: rosrun detection dobotCalibration Dobot_Rail Dobot_Rail_rail kinect2_jetson true
	   If you replace kinect2_jetson with kinect2_xavier, the second camera is used as a reference.
	g. If you later on see, that the robot always grasps too low, too high, too much in y, you can use the
		/tfPublisherrviz
	h. In another console, run the save service, to save the changes: '/tfPublisher/save'

#4. Calibrate the second kinect camera:
	a. Place markers in the field of view of the left camera. For each marker, place a corresponding marker in the right field of view.
		The right markers should have a distance of exact 1m on the y axis to its parent marker on the left side.
		roslaunch detection calibrationKinect2Client.launch
	b. roslaunch detection calibrationKinect2Client.launch
	c. Wait some seconds for the pointcloud to show up.
	d. Select a marker on the left pointcloud (shift + left mouse). Select the corresponding marker on the right pointcloud.
	   Green is the selected point, yellow is the projected point.
	e. If enough points are there, you will see the combined pointcloud in the third window. Press start to accept the result.
		
