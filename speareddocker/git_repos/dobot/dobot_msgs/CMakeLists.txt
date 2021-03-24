cmake_minimum_required(VERSION 2.8.3)
project(dobot_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
	COMPONENTS message_generation std_msgs geometry_msgs actionlib actionlib_msgs
)

add_action_files(
  DIRECTORY action
  FILES Pickup.action PlaceInStorage.action ScanColor.action ScanRFID.action TruckBuild.action TruckStoragePlace.action TruckStorageTake.action
)

add_message_files(FILES
		block_rotation.msg
)

add_service_files(FILES
  SetCmdTimeout.srv

  GetDeviceSN.srv
  SetDeviceName.srv
  GetDeviceName.srv
  GetDeviceVersion.srv

  GetPose.srv
		GetPoseL.srv

  GetAlarmsState.srv
  ClearAllAlarmsState.srv

  SetHOMEParams.srv
  GetHOMEParams.srv
  SetHOMECmd.srv

  SetEndEffectorParams.srv
  GetEndEffectorParams.srv
  SetEndEffectorLaser.srv
  GetEndEffectorLaser.srv
  SetEndEffectorSuctionCup.srv
  GetEndEffectorSuctionCup.srv
  SetEndEffectorGripper.srv
  GetEndEffectorGripper.srv

  SetJOGJointParams.srv
  GetJOGJointParams.srv
  SetJOGCoordinateParams.srv
  GetJOGCoordinateParams.srv
  SetJOGCommonParams.srv
  GetJOGCommonParams.srv
  SetJOGCmd.srv

  SetPTPJointParams.srv
  GetPTPJointParams.srv
  SetPTPCoordinateParams.srv
  GetPTPCoordinateParams.srv
  SetPTPJumpParams.srv
  GetPTPJumpParams.srv
  SetPTPCommonParams.srv
  GetPTPCommonParams.srv
  SetPTPCmd.srv
  SetPTPWithLCmd.srv
  SetPTPLParams.srv

  SetCPParams.srv
  GetCPParams.srv
  SetCPCmd.srv

  SetARCParams.srv
  GetARCParams.srv
  SetARCCmd.srv

  SetWAITCmd.srv

  SetTRIGCmd.srv

  SetIOMultiplexing.srv
  GetIOMultiplexing.srv
  SetIODO.srv
  GetIODO.srv
  SetIOPWM.srv
  GetIOPWM.srv
  GetIODI.srv
  GetIOADC.srv
  SetEMotor.srv
  SetEMotorS.srv

  SetQueuedCmdStartExec.srv
  SetQueuedCmdStopExec.srv
  SetQueuedCmdForceStopExec.srv
  SetQueuedCmdClear.srv
  
  SetTransportbeltSpeed.srv
  SetColorSensor.srv
  GetColorSensor.srv
  SetInfraredSensor.srv
  GetInfraredSensor.srv
  
  Vector3.srv
)


generate_messages(DEPENDENCIES 
	std_msgs geometry_msgs actionlib_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs geometry_msgs actionlib_msgs
)
