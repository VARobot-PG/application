/*
 * detectionGraspTest.cpp
 *
 * Tries to touch each block which is detected.
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
	INIT, MOVE_TO_LOAD, WAIT
};

//allowed positioning accuracy error
const double ALLOWED_ERROR = 1.0;
//Additional z value in m
const double ADDITIONAL_Z_VALUE = -3;

// The robot's name, used when publishing its states and information
const std::string ROBOT_NAME = dobot::dobot_names::DOBOT_LOADER;//dobot::dobot_names::DOBOT_LOADER;

//Subscriber that listens to the robot's state
ros::Subscriber robotStateSub;

//Stores the latest seen objects and since when the arm is out of the camera view
detection_msgs::DetectedObjects latestObjects;
bool dirtyLatestObjects;
ros::Time robotArmOutOfViewTime;

//Coordinates in m
std::vector<Object> takenObjects;
std::vector<tf2::Vector3> graspPoints;
//Coordinates in mm and offset from endeffector to tool included
std::vector<tf2::Transform> graspPointsCorrected;
std::vector<double> blockRotations;
size_t currentObject = 0;

bool robotIsIdle;
double secondsInCurrentState = 0.0;
uint8_t taskExecuted = 3;

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

bool decideForObjects(std::vector<detection_msgs::DetectedObject> &outputObjects, ros::Time latest)
{
	outputObjects.resize(0);
	tf2::Transform transform;
	if(!tfHelper->getTransform(ROBOT_NAME + "_tool",latestObjects.frame_id,ros::Time(0),transform)) return false;
	if(latestObjects.objects.size() == 0) return false;
	if(latestObjects.objects.at(0).timestamp < latest) return false;

	std::vector<detection_msgs::DetectedObject> objs;
	for(size_t i = 0; i < latestObjects.objects.size(); i++)
	{
		objs.push_back(latestObjects.objects.at(i));
	}

	while(objs.size() > 0)
	{
		//Find remaining point with smallest distance to arm
		double smallestDistance = 0.0;
		int bestPoint = -1;
		for(size_t i = 0; i < objs.size(); i++)
		{

			if(objs.at(i).probability > 0.9)
			{
				tf2::Vector3 gP(objs.at(i).graspPoint.x,objs.at(i).graspPoint.y,objs.at(i).graspPoint.z);
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
		//Have we found an object? If yes, remember it.
		if(bestPoint != -1)
		{
			outputObjects.push_back(objs.at(bestPoint));
			objs.erase(objs.begin() + bestPoint);
		}
		if(bestPoint == -1)
		{
			return false;
		}
	}
	return true;
}

bool moveArmToGoal(tf2::Vector3 goal, double rotation, ros::Rate r)
{
	errorOnMoveCounter = 0;
	while(ros::ok())
	{
		if(taskExecuted == 0 && arm.isIdle())
		{
			if(arm.isAtPosition(goal))
			{
				return true;
			}else{
				std::cout << "Not at goal position. Sending command!" << std::endl;
				if(errorOnMoveCounter < 3)
				{
					arm.moveArmToPosition(goal, rotation);
					ROS_INFO("Commanding: [%f,%f,%f]",graspPoints.at(currentObject).x(),graspPoints.at(currentObject).y(),graspPoints.at(currentObject).z());
					taskExecuted = 3;
					errorOnMoveCounter++;
				}else{
					return false;
				}
			}
		}
		ros::spinOnce();
		r.sleep();
	}
	return false;
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "blockLoader");
	std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
	tfHelper = std::make_unique<TfHelper>();
	markerManager = std::make_unique<MarkerHelper>(nh,ROBOT_NAME + "/marker","");

	ros::Rate r(4);
	arm.initArm(nh,ROBOT_NAME,50);

	robotStateSub = nh->subscribe<std_msgs::Bool>(ROBOT_NAME + "/idle", 1, robotStateCallback);
	ros::Subscriber cameraSub = nh->subscribe<detection_msgs::DetectedObjects>("/LoadGraspDetection/objectsSave", 1, objectsCallback);

	ROS_INFO("Starting.");
	ros::spinOnce();

	bool done = false;
	//Move to wait position!
	while(ros::ok() && !done)
	{
		if(arm.isIdle() && taskExecuted == 0)
		{
			if(arm.isAtPosition(tf2::Vector3(dobot::env::L_DROP_POSITION.x(),dobot::env::L_DROP_POSITION.y(),70.0))){
				done = true;
			}else{
				arm.moveArmToPosition(tf2::Vector3(dobot::env::L_DROP_POSITION.x(),dobot::env::L_DROP_POSITION.y(),70.0), 0.0);
				taskExecuted = 3;
			}
		}
		r.sleep();
		ros::spinOnce();
	}
	robotArmOutOfViewTime = ros::Time::now() + ros::Duration(4);
	done = false;
	size_t maxSize = 0;
	//Creates a grasp queue
	while(ros::ok() && !done)
	{
		//Only if we have recieved a new message
		if(dirtyLatestObjects)
		{
			std::cout << "Objects size: " << latestObjects.objects.size() << std::endl;
			if(latestObjects.objects.size() > 0) //Do we have some objects?
			{
				bool goodDetection = false;
				if(maxSize <= latestObjects.objects.size())
				{
					std::cout << "Good detection!\n";
					goodDetection = true;
					maxSize = latestObjects.objects.size();
				}else{
					std::cout << "Wait futher!\n";
					robotArmOutOfViewTime = ros::Time::now();
					maxSize = 0;
				}
				//Create a list of objects to grasp
				std::vector<detection_msgs::DetectedObject> o;
				if(decideForObjects(o,robotArmOutOfViewTime) && goodDetection)
				{
					for(size_t i = 0; i < o.size(); i++) //For each object, get the grasp point
					{
						Object takenObject = Object(o.at(i));
						tf2::Transform graspCorrected;
						tf2::Vector3 graspPoint;
						if(arm.convertToDobotPose(takenObject.pose,graspCorrected,latestObjects.frame_id))//Converts from any frame to dobot coordinate frame. Converts from meters to mm.
						{
							graspPoints.push_back(graspCorrected.getOrigin() + tf2::Vector3(0,0,ADDITIONAL_Z_VALUE));
							tf2::Matrix3x3 rotMat = graspCorrected.getBasis();
							double roll,pitch,yaw;
							rotMat.getEulerYPR(yaw,pitch,roll);

							blockRotations.push_back(180.0*yaw/M_PI);
							markerManager->publishMarker(takenObject.pose.getOrigin(),latestObjects.frame_id,0,255,0,0,"",0);
							graspPointsCorrected.push_back(graspCorrected);
							takenObjects.push_back(o.at(i));

						}
					}
					done = true;
				}else{
					std::cout << "Objects are not ready!\n";
				}
			}else{
				ROS_WARN("No objects for grasping!");
			}
			dirtyLatestObjects = false;
		}
		ros::spinOnce();
		r.sleep();
	}

	done = false;
	errorOnMoveCounter = 0;
	while(ros::ok() && !done)
	{
		if(moveArmToGoal(graspPointsCorrected.at(currentObject).getOrigin(), blockRotations.at(currentObject),r))
		{
			std::cout << "Reached goal position!";
			markerManager->publishMarker(takenObjects.at(currentObject).pose.getOrigin(),latestObjects.frame_id,currentObject,0,255,0,"",0);
		}else
		{
			std::cout << "Unable to reach goal position!";
			markerManager->publishMarker(takenObjects.at(currentObject).pose.getOrigin(),latestObjects.frame_id,currentObject,255,0,0,"",0);
		}
		if(!moveArmToGoal(tf2::Vector3(0,190,70), 0.0,r))
		{
			std::cout << "Unable to leave object position!";
			markerManager->publishMarker(takenObjects.at(currentObject).pose.getOrigin(),latestObjects.frame_id,currentObject,255,255,0,"",0);
		}

		currentObject++;
		if(currentObject >= graspPoints.size())
		{
			done = true;
		}
	}

	ROS_INFO("Done! Press CTRL + C to quit!");
	while(ros::ok());
	return 0;
}



