#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <actionlib/server/simple_action_server.h>
#include <dobot_msgs/PlaceInStorageAction.h>
#include <dobot_msgs/PlaceInStorageActionFeedback.h>
#include <dobot_msgs/PlaceInStorageActionResult.h>
#include <boost/bind.hpp>
#include <dobot/dobot.h>

class PlaceInStorageAction
{
protected:

    actionlib::SimpleActionServer<dobot_msgs::PlaceInStorageAction> ps_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    const std::string action_name_;
    std::shared_ptr<ros::NodeHandle> nh;
    dobot::Dobot &dobot1;
    // create messages that are used to publish feedback/result
    dobot_msgs::PlaceInStorageFeedback feedback_;
    dobot_msgs::PlaceInStorageResult result_;

    tf2::Vector3 PLACE_POSITION = tf2::Vector3(200.0,0.0,50.0);
    double PLACE_LINEARRAIL_POS = 0.0;

public:

    PlaceInStorageAction(std::string name,std::shared_ptr<ros::NodeHandle> nh, dobot::Dobot &gDobot) :
            ps_(*nh, name, boost::bind(&PlaceInStorageAction::pickupBlockCB, this, _1), false),
            action_name_(name),
    		dobot1(gDobot)
    {
    	std::cout << "PlaceInStorageAction\n";
    	ps_.registerPreemptCallback(boost::bind(&PlaceInStorageAction::preemtCB,this));
        ps_.start();
    }

    ~PlaceInStorageAction(void)
    {

    }

    void preemtCB()
    {
    	ROS_INFO("%s: Preempted", action_name_.c_str());
    	// set the action state to preempted
    	ps_.setPreempted();
    }

    void pickupBlockCB(const dobot_msgs::PlaceInStorageGoalConstPtr &goal)
    {
    	geometry_msgs::Point point = goal->position;
    	dobot1.positionToPositionWithL(tf2::Vector3(point.x,point.y,point.z),PLACE_POSITION,PLACE_LINEARRAIL_POS);
        // helper variables
        bool success = true;
        // publish info to the console for the user
        ROS_INFO("Executing, picking up block at position [%f,%f,%f,%f]", PLACE_POSITION.x(),PLACE_POSITION.y(),PLACE_POSITION.z(),PLACE_LINEARRAIL_POS);
        // start executing the action
        feedback_.comment.data = "Moving to Pickup";
        feedback_.progress = 0.1;
        ps_.publishFeedback(feedback_);

        /* move arm to pickup position */
        /*while(!dobot1.isAtPosition(PLACE_POSITION) && ros::ok()){ //TODO stop if position can not be reached //TODO Calculate distance to target position and return as progress
        	if(dobot1.isIdle()){
                dobot1.moveArmToPositionWithL(PLACE_POSITION, goal->rotation, PLACE_LINEARRAIL_POS);
        	}
        	ros::Duration(1).sleep();
        }*/
        dobot1.moveToPositionWithLBlocking(PLACE_POSITION, goal->rotation, PLACE_LINEARRAIL_POS);
        feedback_.comment.data = "Releasing block!";
        feedback_.progress = 0.9;
        ps_.publishFeedback(feedback_);
        dobot1.enableSuctionCup(false);
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
