/*
 * detectionBlockLoader.cpp
 *
 * Controls the left dobot. Moves to grasp points and takes the objects onto the conveyor belt. Will check if conveyor belt is free.
 *
 *  Created on: 22 April, 2019
 *      Author: philipf
 */

#include <ros/ros.h>
#include <string>
#include <cmath>
#include <memory>
#include <algorithm>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>
#include <cstdlib>
#include "std_msgs/Bool.h"

#include <detection_msgs/DetectedObject.h>
#include <detection_msgs/DetectedObjects.h>
#include <detection_msgs/AddDetectedObject.h>
#include <visualization_msgs/Marker.h>

#include <marker_helper/markerHelper.h>
#include <tf_helper/tfHelper.h>
#include <dobot/dobot.h>

enum State {
	INIT, MOVE_TO_LOAD, LOAD, MOVE_TO_UNLOAD, UNLOAD, WAIT
};

//allowed positioning accuracy error
const double ALLOWED_ERROR = 1.0;
//Additional z value in m
const double ADDITIONAL_Z_VALUE = -3;

const int TRANSPORT_BELT_MESSAGES_NEEDED = 120;

// The robot's name, used when publishing its states and information
const std::string ROBOT_NAME = dobot::dobot_names::DOBOT_LOADER;//dobot::dobot_names::DOBOT_LOADER;

//Subscriber that listens to the robot's state
ros::Subscriber robotStateSub;
ros::ServiceClient addObjectToBeltService;

//Stores the latest seen objects and since when the arm is out of the camera view
detection_msgs::DetectedObjects latestObjects;
bool dirtyLatestObjects;
ros::Time robotArmOutOfViewTime;

//Coordinates in m
Object takenObject;
tf2::Vector3 graspPoint;
tf2::Vector3 unloadPosition;

//Coordinates in mm and offset from endeffector to tool included
tf2::Transform graspPointCorrected;

bool skippedABlock = false;
bool robotIsIdle;
double secondsInCurrentState = 0.0;
double blockRotation = 0.0;
uint8_t taskExecuted = true;
bool transportBeltIsMoving = false;
int transportBeltIsClear = TRANSPORT_BELT_MESSAGES_NEEDED;

int errorOnMoveCounter = 0;

State current_State = INIT;

dobot::Dobot arm(ROBOT_NAME);
std::unique_ptr<TfHelper> tfHelper;
std::unique_ptr<MarkerHelper> markerManager;

//Is the robot moving or idle?
void robotStateCallback(const std_msgs::Bool::ConstPtr& msg){
	robotIsIdle = msg->data;

	//If taskExecuted == 0, robot is finished.
	if(robotIsIdle && (taskExecuted > 0)){
		taskExecuted--;
	}
}

/*Callback function, which is called when the transport belt's state changed
 * updates variable, indicating state of the transport belt*/
void transportBeltCallback(const std_msgs::Bool::ConstPtr &msg)
{
	transportBeltIsMoving = !msg->data;
	if(transportBeltIsMoving && transportBeltIsClear > 0)
	{
		transportBeltIsClear--;
	}
}

void objectsCallback(const detection_msgs::DetectedObjects::ConstPtr& obj)
{
	latestObjects = *obj;
	dirtyLatestObjects = true;
}

bool isRotationOkay(tf2::Transform frameToDobot, detection_msgs::DetectedObject obj)
{
	tf2::Vector3 object(obj.graspPoint.x,obj.graspPoint.y,obj.graspPoint.z);
	//transform into dobot frame
	object = frameToDobot.inverse() * object;

	object.normalize();
	//This calculates the angle which the dobot has for grasping.
	tf2::Vector3 forwardVectorGlobal(1,0,0);
	tf2::Vector3 rotAxis = forwardVectorGlobal.cross(object);
	tf2::Quaternion rot;
	rot.setX(rotAxis.x());
	rot.setY(rotAxis.y());
	rot.setZ(rotAxis.z());
	rot.setW(std::sqrt(object.length2() * forwardVectorGlobal.length2()) + object.dot(forwardVectorGlobal));
	rot.normalize();

	ROS_ERROR("TODO!!");
	return true;
}

bool decideForObject(detection_msgs::DetectedObject& obj, ros::Time latest)
{
	tf2::Transform transform;
	if(!tfHelper->getTransform(ROBOT_NAME + "_tool",latestObjects.frame_id,ros::Time(0),transform)) return false;

	double smallestDistance = 0.0;
	int bestPoint = -1;
	for(size_t i = 0; i < latestObjects.objects.size(); i++)
	{
		if(latestObjects.objects.at(i).timestamp < latest)
		{
			continue;
		}

		if(latestObjects.objects.at(i).probability > 0.9)
		{
			tf2::Vector3 gP(latestObjects.objects.at(i).graspPoint.x,latestObjects.objects.at(i).graspPoint.y,latestObjects.objects.at(i).graspPoint.z);
			//transform into dobot frame
			gP = transform.inverse() * gP;
			double distance = gP.distance2(tf2::Vector3(0,0,0));
			if((bestPoint == -1) || (smallestDistance > distance))
			{
				smallestDistance = distance;
				bestPoint = i;
			}

		}
	}
	if(bestPoint != -1)
	{
		std::cout << "Best object: [" << latestObjects.objects.at(bestPoint).graspPoint.x << ", " << latestObjects.objects.at(bestPoint).graspPoint.y << ", " << latestObjects.objects.at(bestPoint).graspPoint.z << std::endl;
		obj = latestObjects.objects.at(bestPoint);
		return true;
	}
	std::cout << "Found no good grasp point!\n";
	return false;
}

/* Performs a state transition, given a target state and a reason for leaving the old one.
 * Input:   State to enter
 *          Reason why old state should be left*/
void doStateTransition(State new_State, std::string reason){

	//Leave old state:
	switch (current_State) {
	case INIT: {
		ROS_INFO("Leaving INIT: %s",reason.c_str());
		break;
	}
	case MOVE_TO_LOAD: {
		ROS_INFO("Leaving MOVE_TO_LOAD: %s",reason.c_str());
		break;
	}
	case LOAD: {
		break;
	}
	case MOVE_TO_UNLOAD: {
		ROS_INFO("Leaving MOVE_TO_UNLOAD: %s",reason.c_str());
		break;
	}
	case UNLOAD: {
		transportBeltIsClear = TRANSPORT_BELT_MESSAGES_NEEDED;
		ROS_INFO("Leaving UNLOAD: %s",reason.c_str());
		break;
	}
	case WAIT: {
		ROS_INFO("Leaving WAIT: %s",reason.c_str());
		break;
	}
	default:
		ROS_ERROR("UNKNOWN CURRENT STATE!");
		break;
	}
	current_State = new_State;
	secondsInCurrentState = 0.0;

	//Init new state
	switch (current_State) {
	case INIT: {
		ROS_INFO("->INIT");
		arm.enableSuctionCup(false);
		break;
	}
	case MOVE_TO_LOAD: {
		ROS_INFO("->MOVE_TO_LOAD");
		errorOnMoveCounter = 0;
		break;
	}
	case LOAD: {
		ROS_INFO("->LOAD");
		markerManager->publishMarker(takenObject.pose.getOrigin(),latestObjects.frame_id,0,0,255,0,"",0);
		arm.enableSuctionCup(true);
		break;
	}
	case MOVE_TO_UNLOAD: {
		ROS_INFO("->MOVE_TO_UNLOAD");
		arm.enableSuctionCup(true);
		double objectHeight = std::fabs(takenObject.pose.getOrigin().z() * 1000.0);
		std::cout << "Object height: " << objectHeight << std::endl;
		unloadPosition = dobot::env::L_DROP_POSITION + tf2::Vector3(0,0,objectHeight);
		break;
	}
	case UNLOAD: {
		ROS_INFO("->UNLOAD");
		arm.enableSuctionCup(false);

		//Add object to transport belt queue!
		detection_msgs::AddDetectedObject srv;
		srv.request.frame_id = ROBOT_NAME + "_tool";
		takenObject.pose.setOrigin(tf2::Vector3(0,0,0));
		srv.request.obj = takenObject.toDetectedObject();
		srv.request.obj.timestamp = ros::Time::now();
		srv.request.timestamp = ros::Time::now();
		addObjectToBeltService.call(srv.request,srv.response);
		break;
	}
	case WAIT: {
		ROS_INFO("->WAIT");
		robotArmOutOfViewTime = ros::Time::now(); //+ros::Duration(0.5)
		break;
	}
	default:
		ROS_ERROR("UNKNOWN NEW STATE!");
		break;
	}
}
/* Decides which new state to enter based on current state, time in this state
 * Input:  Double, indicating the time since the state was changed
 */
void updateState(double deltaTime){
	secondsInCurrentState += deltaTime;

	switch (current_State) {
	case INIT: {
		if(taskExecuted == 0){
			std::cout << "Checking arm pose!\n";
			if(arm.isAtPosition(tf2::Vector3(dobot::env::L_DROP_POSITION.x(),dobot::env::L_DROP_POSITION.y(),30.0))){
				doStateTransition(WAIT,"Init done.");
			}else{
				arm.moveArmToPosition(tf2::Vector3(dobot::env::L_DROP_POSITION.x(),dobot::env::L_DROP_POSITION.y(),30.0), 0.0);
				taskExecuted = 3;
			}
		}
		break;
	}
	case MOVE_TO_LOAD: {
		if(taskExecuted == 0){
			std::cout << "Checking arm pose!\n";
			if(arm.isAtPosition(graspPointCorrected.getOrigin())){
				if(transportBeltIsClear <= 0 || !addObjectToBeltService.exists()){ //Transportbelt should move, so that unload position is free.
					doStateTransition(LOAD,"Is at loading position.");
				}else{
					std::cout << "Wait for transportbelt. " << 100.0 * double(TRANSPORT_BELT_MESSAGES_NEEDED - transportBeltIsClear) / double(TRANSPORT_BELT_MESSAGES_NEEDED) << "%\n";
				}
			}else{
				std::cout << "Executed but not at LOAD_POSITION!" << std::endl;
				if(errorOnMoveCounter < 3)
				{
					arm.moveArmToPosition(graspPointCorrected.getOrigin(), blockRotation);
					markerManager->publishMarker(takenObject.pose.getOrigin(),latestObjects.frame_id,0,0,255,255,"",0);
					ROS_INFO("Commanding: [%f,%f,%f]",graspPoint.x(),graspPoint.y(),graspPoint.z());
					taskExecuted = 3;
					errorOnMoveCounter++;
				}else{
					doStateTransition(INIT,"Unable to move to load position. Choosing new object!");
					skippedABlock = true;
				}
			}
		}
		break;
	}
	case LOAD: {
		if (secondsInCurrentState > 0.25) {
			doStateTransition(MOVE_TO_UNLOAD,"Time limit reached for load.");
		}
		break;
	}
	case MOVE_TO_UNLOAD: {
		if (taskExecuted == 0) {
			if (arm.isAtPosition(unloadPosition)) {
				doStateTransition(UNLOAD,"Is at drop position.");
			} else {
				std::cout << "Executed but not at DROP_POSITION!" << std::endl;
				arm.moveArmToPosition(unloadPosition, 90.0);
				taskExecuted = 3;
			}
		}
		break;
	}
	case UNLOAD: {
		if(secondsInCurrentState > 0.5){
			doStateTransition(WAIT,"Finished with unload.");
		}
		break;
	}
	case WAIT: {
		if(dirtyLatestObjects)
		{
			std::cout << "Objects size: " << latestObjects.objects.size() << std::endl;
			if(latestObjects.objects.size() > 0)
			{
				detection_msgs::DetectedObject o;
				if(decideForObject(o,robotArmOutOfViewTime)) //Ignore all detected objects when the robot arm was next to the grasp location.
				{
					takenObject = Object(o);

					if(arm.convertToDobotPose(takenObject.pose,graspPointCorrected,latestObjects.frame_id))//Converts from any frame to dobot coordinate frame. Converts from meters to mm.
					{
						graspPoint = graspPointCorrected.getOrigin() + tf2::Vector3(0,0,ADDITIONAL_Z_VALUE);
						tf2::Matrix3x3 rotMat = graspPointCorrected.getBasis();
						double roll,pitch,yaw;
						rotMat.getEulerYPR(yaw,pitch,roll);

						blockRotation = 180.0*yaw/M_PI;

						markerManager->publishMarker(takenObject.pose.getOrigin(),latestObjects.frame_id,0,255,0,0,"",0);
						ROS_INFO("Choosing: [%f,%f,%f]",graspPoint.x(),graspPoint.y(),graspPoint.z());
						doStateTransition(MOVE_TO_LOAD,"Has found new graps point.");
					}

				}
			}else{
				ROS_WARN("No objects for grasping!");
			}
			dirtyLatestObjects = false;
		}
		break;
	}
	default:
		break;
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "blockLoader");
	std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
	tfHelper = std::make_unique<TfHelper>();
	markerManager = std::make_unique<MarkerHelper>(nh,ROBOT_NAME + "/marker","");

	ros::Rate r(4);
	arm.initArm(nh,ROBOT_NAME);

	robotStateSub = nh->subscribe<std_msgs::Bool>(ROBOT_NAME + "/idle", 1, robotStateCallback);
	ros::Subscriber transportBeltSub = nh->subscribe<std_msgs::Bool>("/arduino_conveyor/GetInfraredSensorBlocked", 30, transportBeltCallback);
	ros::Subscriber cameraSub = nh->subscribe<detection_msgs::DetectedObjects>("/LoadGraspDetection/objectsSave", 1, objectsCallback);
	addObjectToBeltService = nh->serviceClient<detection_msgs::AddDetectedObject>("/transportbelt/addObject");

	doStateTransition(INIT,"Start.");
	ROS_INFO("Starting blockLoader loop.");
	while (ros::ok()){
		updateState(r.expectedCycleTime().toSec());
		ros::spinOnce();
		r.sleep();
	}

	ROS_INFO("Done!");
	return 0;
}



