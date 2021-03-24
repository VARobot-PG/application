//
// Created by moritzt on 22.11.18.
//
#include <ros/ros.h>
#include <string>
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Transform.h>
#include <iostream>
#include <cstdlib>
#include <memory>
#include <detection_msgs/DetectedObjects.h>
#include <detection_msgs/AddDetectedObject.h>
#include <detection_msgs/GetDetectedObject.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include <marker_helper/markerHelper.h>
#include <tf_helper/tfHelper.h>
#include <arduino/arduino.h>
#include <dobot/dobot.h>

enum State {
    MOVE, STOP
};

State currentState = STOP;
double secondsInCurrentState = 0.0;
std::unique_ptr<arduino::ConveyorNode> conveyor_node;
std::unique_ptr<MarkerHelper> markerManager;
std::vector<Object> objectQueue;
std::unique_ptr<TfHelper> tfHelper;

bool addObjectToQueue(detection_msgs::AddDetectedObject::Request &req, detection_msgs::AddDetectedObject::Response &res)
{
	if(req.obj.timestamp == ros::Time(0)) req.obj.timestamp = ros::Time::now();
	Object obj(req.obj);

	tf2::Transform trans;
	if(tfHelper->getTransform(req.frame_id,"transportBelt",req.obj.timestamp,trans))
	{
		obj.pose = trans.inverse() * obj.pose;

		objectQueue.push_back(obj);
		ROS_INFO("Added new object to queue.");
		res.success = true;
		return true;
	}
	ROS_ERROR("Failed to add object!");
	res.success = false;
	return false;
}

bool popObjectFromQueue(detection_msgs::GetDetectedObject::Request &req, detection_msgs::GetDetectedObject::Response &res)
{
	if(currentState == STOP)
	{
		if(objectQueue.size() > 0)
		{
			res.obj = objectQueue.at(0).toDetectedObject();
			res.frame_id = "transportBelt";
			res.timestamp = objectQueue.at(0).timestamp;
			res.success = true;

			//Remove first element and reoder queue.
			objectQueue.erase(objectQueue.begin());
			return true;
		}
	}

	res.success = false;
	return true;
}

void updateObjectQueue(bool beltWasActive)
{
	ros::Time currentTime = ros::Time::now();

	for(int k = 0; k < objectQueue.size(); k++)
	{
		Object &obj = objectQueue.at(k);
		if(beltWasActive)//Has the position changed over time?
		{
			tf2::Vector3 position = obj.pose.getOrigin();
			double delta = (currentTime - obj.timestamp).toSec();
			position += tf2::Vector3(1.0,0,0) * ((dobot::env::CONVEYORBELT_SPEED  / 27.25) * double(dobot::env::CONVEYORBELT_SPEED_IN_PERCENT)) * 0.001 * delta;
			obj.pose.setOrigin(position);
		}
		markerManager->publishArrow(tf2::Transform(obj.pose.getRotation(),obj.pose.getOrigin()),"transportBelt",10+k,0.05,1,1,1,"rotation",2.0);
		markerManager->publishBox(tf2::Transform(obj.pose.getRotation(),obj.pose.getOrigin() - tf2::Vector3(0,0,obj.height*0.5)),
					"transportBelt",tf2::Vector3(obj.length,obj.width,obj.height),k,obj.color[0],obj.color[1],obj.color[2]);
		obj.timestamp = currentTime;
	}
	//Throw blocks which are out of bounds
	if(objectQueue.size() > 0)
	{
		if(objectQueue.at(0).pose.getOrigin().length() > 1.0)
		{
			objectQueue.erase(objectQueue.begin());
		}
	}
}

void doStateTransition(State newState)
{
    int8_t speed = 0.0;

    if(currentState!=newState)
    {
        currentState = newState;
        switch (currentState)
        {
            case MOVE: {
                ROS_INFO("->MOVE");
                speed = dobot::env::CONVEYORBELT_SPEED_IN_PERCENT;
                conveyor_node->set_transportbelt_speed(speed);
                updateObjectQueue(false);//Transportbelt was stopped until now. So do not move objects, but update timestamp.
                break;
            }
            case STOP: {
                ROS_INFO("->STOP");
                speed = 0.0;
                conveyor_node->set_transportbelt_speed(speed);
                updateObjectQueue(true);//Transportbelt is stopped now. Calculate the new objects position!
                break;
            }
            default:
                ROS_ERROR("UNKNOWN NEW STATE!");
                break;
        }
    }
}

void if_callback(const std_msgs::BoolConstPtr &msg)
{
	if(msg->data == false)
	{
		if(currentState == STOP)
		{
			doStateTransition(MOVE);
		}
	}else
	{
		if(currentState == MOVE)
		{
			doStateTransition(STOP);
		}
	}
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "transportBelt");
    std::shared_ptr<ros::NodeHandle> nh_ptr = std::make_shared<ros::NodeHandle>();

    conveyor_node = std::make_unique<arduino::ConveyorNode>(nh_ptr);
    conveyor_node->register_if_callback(if_callback);
    tfHelper = std::make_unique<TfHelper>();
    markerManager = std::make_unique<MarkerHelper>(nh_ptr,"/transportbelt/marker","objects");

    ros::ServiceServer addObjectsServer = nh_ptr->advertiseService("/transportbelt/addObject",addObjectToQueue);
    ros::ServiceServer popObjectServer = nh_ptr->advertiseService("/transportbelt/popObject",popObjectFromQueue);

    ros::Rate r(4);
    ros::Duration(1.0).sleep();

    doStateTransition(MOVE);
    ROS_INFO("Starting transportBelt loop.");
    while (ros::ok())
    {
		updateObjectQueue(currentState == MOVE);
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO("Done!");
    return 0;
}
