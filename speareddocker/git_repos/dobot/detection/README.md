# Info
Detection is a package which is used to process pointclouds. The messages used are saved in detection_msgs.
#tfPublisher
This node publishes a list of static tf frames e.g. the world frame. It is neccecary for all kinect operations.
#tfPubisherClient
You have to connect a joystick. The user can now manually select a coordinate frame using the LB + X or LB + B buttons. The selected frame is shown in yellow. By using the axis buttons, the frame can be moved. By pressing the left axis button, the frame is snapped into the nearest point. By pressing the right axis button, the frame will snap and align to the nearest surface. By pressing A, the frame is stored as a fallback. To recover the last fallback, the B button can be pressed. To flip the Z-axis, the X button can be pressed. Pressing Y will record the scene again. To save all settings, the start button can be pressed. This setting will now be loaded on every startup.
#graspPointDetection
This node will find graspable surfaces inside a rectangular region (currently placed next to the left dobot). The surfaces information will be published and will be subscribed by the detectionBlockLoader application. Parameters can be dynamically changed using the ros parameter server.
#graspPointBuffer
This node will listen to detected grasp points. If a grasp point occures multiple times, it will be published as a save/approved grasp point.
#dominoDetection
Detects domino blocks, their color and rotation.
#Cloud
Cloud is a library of functions which might be usefull for processing pointclouds.
#dobotCalibration
To setup the dobot frame, calibration is needed. Run this to calibrate the dobot with the camera. The frames will be afterwards publisher by tfPublisher.
#medianFilter
This is a class used to record multiple samples of a sceen and smooth the output for noise reduction.
#kinectCalibration
This aligns the kinect frames together and stores it for the tfPublisher.

