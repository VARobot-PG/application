/*
 * transportation.cpp
 *
 *  Created on: Oct 25, 2018
 *      Author: philipf
 */

#include <ros/ros.h>
#include <string>
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Transform.h>
#include <iostream>
#include <cstdlib>
#include <memory>
#include "std_msgs/Bool.h"

#include <dobot/dobot.h>


//TODO: fix strange up and down movements of arm on pickup and drop positions
//TODO: fix wrong next pickup coordinate

//position at which the first block can be picked up
//const tf2::Vector3 LOAD_POSITION(16.0,239.0,-46.0); //160.0,173.0,-45.0 // old: 160.0, 187.0, -46.0
//position at which blocks should be dropped
//const tf2::Vector3 DROP_POSITION(182.0,25.0,20.0); // 213.0,58.0,30.0
//size of the blocks to be loaded
//const tf2::Vector3 BLOCK_SIZE(25.0,25.0,25.0); // old: all values 25.33
//allowed positioning accuracy error
//const double ALLOWED_ERROR = 1.0;

/* Underlying state machine of the dobot:
 * INIT - Starting state in which initialization is performed
 * MOVE_TO_LOAD - Moves the arm to the location of the next block to be picked up
 * LOAD - Enables suction cup to load the block
 * MOVE_TO_UNLOAD - Moves the arm to the location where the block should be dropped
 * UNLOAD - Disables suction cup to drop the block */
enum State {
	INIT, MOVE_TO_LOAD, LOAD, MOVE_TO_UNLOAD, UNLOAD
};

// The robot's name, used when publishing its states and information
const std::string ROBOT_NAME = dobot::dobot_names::DOBOT_LOADER;
dobot::Dobot left_dobot(ROBOT_NAME);

double secondsInCurrentState = 0.0;

bool transportBeltIsMoving = false;
bool onTrack = false;
//Setting the starting state
State current_State = INIT;
/* ServiceClients used to interact with the services dobot provides:
 * ptp_init_client - currently not used
 * ptp_cmd_client - used to call service that moves arm from point to point
 * end_effector_client - used to call service that enables and disables suction cup */
//ros::ServiceClient ptp_init_client;


/*Callback function, which is called when the transport belt's state changed
 * updates variable, indicating state of the transport belt*/
void transportBeltCallback(const std_msgs::Bool::ConstPtr &msg)
{
	transportBeltIsMoving = !msg->data;
}


/* Computes the position, at which the next block can be picked up, given the number already processed blocks
 * Input: Integer, indicating the number of blocks that have been processed
 * Returns: Position of the next block to be processed*/
tf2::Vector3 getBlockPosition(int counter){
    //ROS_INFO("Current counter: %i", counter);
	tf2::Vector3 offset = tf2::Vector3((int)(counter/3),counter%3,0); //Offset measured in number of blocks
	//ROS_INFO("Current offset: row: %i , column: %i ", (int)(counter/3), (counter%3));
	offset = offset * dobot::env::L_BLOCK_SIZE;							 //Convert offset from number of blocks to millimeter
	return (dobot::env::L_PICKUP_POSITION - offset); 						 //Convert offset to LOAD_POSITION coordinates
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
		ROS_INFO("Leaving UNLOAD: %s",reason.c_str());
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
		left_dobot.enableSuctionCup(false);
		break;
	}
	case MOVE_TO_LOAD: {
		ROS_INFO("->MOVE_TO_LOAD");
		break;
	}
	case LOAD: {
		ROS_INFO("->LOAD");
		left_dobot.enableSuctionCup(true);
		break;
	}
	case MOVE_TO_UNLOAD: {
		ROS_INFO("->MOVE_TO_UNLOAD");
		left_dobot.enableSuctionCup(true);
		break;
	}
	case UNLOAD: {
		ROS_INFO("->UNLOAD");
		left_dobot.enableSuctionCup(false);
		break;
	}
	default:
		ROS_ERROR("UNKNOWN NEW STATE!");
		break;
	}
}
/* Decides which new state to enter based on current state, time in this state
 * Input:  Double, indicating the time since the state was changed
 *         Integer, indicating the amount of blocks that have yet been processed
 * Returns: Integer, indicating adjusted amount of processed blocks*/
int updateState(double deltaTime, int blockCounter){
	secondsInCurrentState += deltaTime;

	switch (current_State) {
	case INIT: {
		doStateTransition(MOVE_TO_LOAD,"Init done.");
		break;
	}
	case MOVE_TO_LOAD: {
		if(left_dobot.isIdle()){
			if(left_dobot.isAtPosition(getBlockPosition(blockCounter))){
				if(transportBeltIsMoving){ //Transportbelt should move, so that unload position is free.
					doStateTransition(LOAD,"Is at loading position.");
					onTrack = false;
				}
			}else{

				if(!onTrack){
					onTrack = true;
					std::cout << "Executed but not at LOAD_POSITION!" << std::endl;
					left_dobot.moveArmToPosition(getBlockPosition(blockCounter), 0.0);
				}

			}
		}
		break;
	}
	case LOAD: {
		if (secondsInCurrentState > 0.5) {
			doStateTransition(MOVE_TO_UNLOAD,"Time limit reached for load.");
		}
		break;
	}
	case MOVE_TO_UNLOAD: {
		if (left_dobot.isIdle()) {
			if (left_dobot.isAtPosition(dobot::env::L_DROP_POSITION)) {
				doStateTransition(UNLOAD,"Is at drop position.");
				onTrack = false;
			} else {

				if(!onTrack){
					onTrack = true;
					std::cout << "Executed but not at DROP_POSITION!" << std::endl;
					left_dobot.moveArmToPosition(dobot::env::L_DROP_POSITION, 0.0);
				}

			}
		}
		break;
	}
	case UNLOAD: {
		if(secondsInCurrentState > 0.5){
			doStateTransition(MOVE_TO_LOAD,"Time limit reached for unload.");
			blockCounter++;
		}
		break;
	}
	default:
		break;
	}
	return blockCounter;
}

int main(int argc, char* argv[]) {
    // Initialization of amount of processed blocks
	int blockCounter = 0;
	ros::init(argc, argv, "blockLoader");
	ros::NodeHandle nh;
	std::shared_ptr<ros::NodeHandle> nh_ptr = std::make_shared<ros::NodeHandle>(nh);

	ros::Rate r(20); //60hz
	ros::Duration(1.0).sleep();

	ros::Subscriber transportBeltSub = nh_ptr->subscribe<std_msgs::Bool>("/arduino_conveyor/GetInfraredSensorBlocked", 1, transportBeltCallback);

	left_dobot.initArm(nh_ptr, ROBOT_NAME);

	if(!left_dobot.isInitialized())
		return -1;

	doStateTransition(INIT,"Start.");
	ROS_INFO("Starting blockLoader loop.");
	while (ros::ok() && blockCounter < 9) {
		blockCounter = updateState(r.expectedCycleTime().toSec(),blockCounter);
		ros::spinOnce();
		r.sleep();
	}

	ROS_INFO("Done!");
	return 0;
}



