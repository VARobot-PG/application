#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include "std_msgs/Bool.h"
#include <memory>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arduino/arduino.h>
#include <dobot/dobot.h>
#include <dobot_msgs/PickupAction.h>
#include <dobot_msgs/ScanColorAction.h>
#include <dobot_msgs/PlaceInStorageAction.h>

const tf2::Vector3 RED_POSITION = tf2::Vector3(134.0,-150.0,-107.0); 	// Positions from 134 upto 135 + (4 * 25) can be reached
const tf2::Vector3 GREEN_POSITION = tf2::Vector3(134.0,-450.0,-107.0); // Positions from 134 upto 135 + (3 * 25) can be reached
const tf2::Vector3 BLUE_POSITION = tf2::Vector3(134.0,-800.0,-107.0);  // Positions from 134 upto 135 + (4 * 25) can be reached
const tf2::Vector3 YELLOW_POSITION = tf2::Vector3(134.0,-600.0,-107.0);  // Positions from 134 upto 135 + (4 * 25) can be reached

//const tf2::Vector3 BLOCK_SIZE(32,32,25);

std::unique_ptr<arduino::ConveyorNode> if_sensor;

std::vector<Storage> storages = {Storage(RED_POSITION,1,2,"RED"),
								 Storage(GREEN_POSITION,1,2,"GREEN"),
								 Storage(BLUE_POSITION,1,2,"BLUE"),
								 Storage(YELLOW_POSITION,1,2,"YELLOW")};

void pickupDoneCb(const actionlib::SimpleClientGoalState& state,
            const dobot_msgs::PickupResultConstPtr& result)
{
	ROS_INFO("Dobot %s pickup action!", result->success?"succeded":"failed");
}

void pickupActiveCb()
{
  ROS_INFO("Right Dobot just went active");
}

void pickupFeedbackCb(const dobot_msgs::PickupFeedbackConstPtr& feedback)
{
  ROS_INFO("Progress: %f Comment: %s",feedback->progress ,feedback->comment.data.c_str());
}

bool pickupBlock()
{
	// create the action client
	actionlib::SimpleActionClient<dobot_msgs::PickupAction> ac("pickup");

	ROS_INFO("Waiting for action Pickup.");
	ac.waitForServer(); //will wait for infinite time
	ROS_INFO("Pickup Action server started, sending goal.");

	dobot_msgs::PickupGoal goal;
	goal.position.x = dobot::env::R_PICKUP_POSITION.x();
	goal.position.y = dobot::env::R_PICKUP_POSITION.y();
	goal.position.z = dobot::env::R_PICKUP_POSITION.z();
	ac.sendGoal(goal,&pickupDoneCb,&pickupActiveCb,&pickupFeedbackCb);

	//wait for the action to return
	bool timeout = !ac.waitForResult(ros::Duration(20.0));
	if(timeout)
	{
		ROS_ERROR("Pickup Action timeout!");
		return false;
	}

	actionlib::SimpleClientGoalState state = ac.getState();
	ROS_INFO("Pickup Action finished: %s",state.toString().c_str());
	return true;
}

bool scanColor(uint8_t &r, uint8_t &g, uint8_t &b)
{
	// create the action client
	actionlib::SimpleActionClient<dobot_msgs::ScanColorAction> ac("scanColor");

	ROS_INFO("Waiting for action ScanColor.");
	ac.waitForServer(); //will wait for infinite time
	ROS_INFO("ScanColor Action server started, sending goal.");

	dobot_msgs::ScanColorGoal goal;
	ac.sendGoal(goal);

	//wait for the action to return
	bool timeout = !ac.waitForResult(ros::Duration(10.0));
	if(timeout)
	{
		ROS_ERROR("ScanColor Action timeout!");
		return false;
	}

	actionlib::SimpleClientGoalState state = ac.getState();
	ROS_INFO("ScanColor Action finished: %s",state.toString().c_str());

	auto res = ac.getResult();
	r = res->r;
	g = res->g;
	b = res->b;
	return true;
}

bool storeBlock(tf2::Vector3 location)
{
	// create the action client
	actionlib::SimpleActionClient<dobot_msgs::PlaceInStorageAction> ac("place");

	ROS_INFO("Waiting for action PlaceInStorage.");
	ac.waitForServer(); //will wait for infinite time
	ROS_INFO("PlaceInStorage Action server started, sending goal.");

	dobot_msgs::PlaceInStorageGoal goal;
	goal.position.x = location.x();
	goal.position.y = location.y();
	goal.position.z = location.z();
	goal.rotation = 0.0;
	ac.sendGoal(goal);

	//wait for the action to return
	bool timeout = !ac.waitForResult(ros::Duration(20.0));
	if(timeout)
	{
		ROS_ERROR("PlaceInStorage Action timeout!");
		return false;
	}

	actionlib::SimpleClientGoalState state = ac.getState();
	ROS_INFO("PlaceInStorage Action finished: %s",state.toString().c_str());
	return true;
}

Storage* getStorage(uint8_t r, uint8_t g, uint8_t b)
{
	std::cout << "msg: r: " << (int)r << " g: " << (int)g << " b: "<< (int)b << std::endl;

	switch(dobot::convert_rgb_to_block_color(r,g,b))
    {
        case dobot::BlockColor::RED:
            ROS_INFO("RED");
            return &storages.at(0);

        case dobot::BlockColor::GREEN:
            ROS_INFO("GREEN");
            return &storages.at(1);

        case dobot::BlockColor::BLUE:
            ROS_INFO("BLUE");
            return &storages.at(2);

        case dobot::BlockColor::YELLOW:
            ROS_INFO("YELLOW");
            return &storages.at(3);

        case dobot::BlockColor::MAGENTA:
            ROS_INFO("MAGENTA");
            //TODO: add magenta
        case dobot::BlockColor::CYAN:
            ROS_INFO("CYAN");
            //TODO: add cyan
    }

	return &storages.at(0);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "pickupClient");
    ros::NodeHandle nh;
    std::shared_ptr<ros::NodeHandle> nh_ptr = std::make_shared<ros::NodeHandle>(nh);

    if_sensor = std::make_unique<arduino::ConveyorNode>(nh_ptr);

    ROS_INFO("Starting loop!");
    while(ros::ok())
    {
    	ROS_INFO("Waiting for block!");
    	ros::spinOnce();
    	while(!if_sensor->is_if_sensor_blocked() && ros::ok()) //Wait until blocks have reached infrared sensor
    	{
    		ros::spinOnce();
    		ros::Duration(0.25).sleep();
    	}
    	if(if_sensor->is_if_sensor_blocked())
    	{
			ROS_INFO("Found waiting block. Starting operation!");
			pickupBlock();

			uint8_t r = 0; uint8_t g = 0; uint8_t b = 0;
			scanColor(r,g,b);

			Storage* storage = getStorage(r,g,b); //Choose the right storage depending on the block color. ScanColor currently does not work, because the hardware is not ready.
			if(storage)
			{
				tf2::Vector3 location = storage->getItemPosition(storage->itemCount, dobot::env::R_BLOCK_SIZE);
				ROS_INFO("Choosing storage: %s. Block id: %i. Block location: [%f, %f, %f]",storage->name.c_str(),storage->itemCount,location.x(),location.y(),location.z());
				storage->itemCount++;
				storeBlock(location);
			}else
			{
				ROS_ERROR("Storages array is invalid!");
			}
			//Move back into idle position
			ros::spinOnce();
			if(!if_sensor->is_if_sensor_blocked())
			{
				storeBlock(dobot::env::R_PICKUP_POSITION + tf2::Vector3(0,0,50));
			}
    	}
    }
    ros::shutdown();
    //exit
    return 0;

}
