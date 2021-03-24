#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <actionlib/server/simple_action_server.h>
#include <dobot_msgs/PickupAction.h>
#include <dobot_msgs/PickupActionFeedback.h>
#include <dobot_msgs/PickupActionResult.h>
#include <boost/bind.hpp>
#include <dobot/dobot.h>

class PickupAction
{
protected:

    actionlib::SimpleActionServer<dobot_msgs::PickupAction> ps_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    const std::string action_name_;
    std::shared_ptr<ros::NodeHandle> nh;
    dobot::Dobot &dobot1;
    // create messages that are used to publish feedback/result
    dobot_msgs::PickupFeedback feedback_;
    dobot_msgs::PickupResult result_;

    tf2::Vector3 PICKUP_POSITION = tf2::Vector3(217.0,67.0,-71.0);
    double PICKUP_LINEARRAIL_POS = 0.0;

public:

    PickupAction(std::string name,std::shared_ptr<ros::NodeHandle> nh, dobot::Dobot &d) :
            ps_(*nh, name, boost::bind(&PickupAction::pickupBlockCB, this, _1), false),
            action_name_(name),
            dobot1(d)
    {
    	std::cout << "PickupAction\n";
    	ps_.registerPreemptCallback(boost::bind(&PickupAction::preemtCB,this));
        ps_.start();
    }

    ~PickupAction(void)
    {

    }


    void preemtCB()
    {
    	ROS_INFO("%s: Preempted", action_name_.c_str());
    	// set the action state to preempted
    	ps_.setPreempted();
    }

    void pickupBlockCB(const dobot_msgs::PickupGoalConstPtr &goal)
    {
    	dobot1.enableSuctionCup(false);
    	geometry_msgs::Point point = goal->position;
    	PICKUP_LINEARRAIL_POS = goal->railPos;
    	PICKUP_POSITION = tf2::Vector3(point.x,point.y,point.z);
        // helper variables
        bool success = true;
        // publish info to the console for the user
        ROS_INFO("Executing, picking up block at position [%f,%f,%f,%f]", PICKUP_POSITION.x(),PICKUP_POSITION.y(),PICKUP_POSITION.z(),PICKUP_LINEARRAIL_POS);
        // start executing the action
        feedback_.comment.data = "Moving to Pickup";
        feedback_.progress = 0.1;
        ps_.publishFeedback(feedback_);

        /* move arm to pickup position */
        while(!dobot1.isAtPosition(PICKUP_POSITION) && ros::ok()){ //TODO stop if position can not be reached //TODO Calculate distance to target position and return as progress
        	if(dobot1.isIdle()){
                dobot1.moveArmToPositionWithL(PICKUP_POSITION, goal->rotation, PICKUP_LINEARRAIL_POS);
        	}
        	ros::Duration(1).sleep();
        }
        feedback_.comment.data = "Picking up";
        feedback_.progress = 0.9;
        ps_.publishFeedback(feedback_);
        dobot1.enableSuctionCup(true);
        ros::Duration(0.5).sleep();

        if(success)
        {
            result_.success = true;
            feedback_.comment.data = "Done";
            feedback_.progress = 1.0;
            ps_.publishFeedback(feedback_);
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            ps_.setSucceeded(result_); // set the action state to succeeded
        }
    }
};
